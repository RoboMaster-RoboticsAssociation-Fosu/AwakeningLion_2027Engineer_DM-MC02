/**
 ******************************************************************************
 * @file    custom_controller_system.c
 * @brief   UART7自定义控制器接收owner实现。
 ******************************************************************************
 */

#include "custom_controller_system.h"

#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"

#include <stddef.h>

#define CUSTOM_CONTROLLER_RX_CHUNK_LENGTH  64U
#define CUSTOM_CONTROLLER_RX_RING_LENGTH  256U
#define CUSTOM_CONTROLLER_OFFLINE_MS       500U
#define CUSTOM_CONTROLLER_RECOVERY_MS        5U

typedef struct
{
    CustomControllerParser_t parser;
    CustomControllerSnapshot_t snapshot;
    uint32_t last_valid_frame_ms;
    uint32_t next_recovery_ms;
    volatile uint16_t ring_write_index;
    volatile uint16_t ring_read_index;
    volatile bool restart_pending;
    uint8_t rx_chunk[CUSTOM_CONTROLLER_RX_CHUNK_LENGTH];
    uint8_t rx_ring[CUSTOM_CONTROLLER_RX_RING_LENGTH];
} CustomControllerSystem_t;

static CustomControllerSystem_t g_custom_controller_system = {
    .restart_pending = true,
};

static HAL_StatusTypeDef custom_controller_start_receive(void)
{
    HAL_StatusTypeDef status;

    if (huart7.RxState != HAL_UART_STATE_READY)
    {
        if (HAL_UART_AbortReceive(&huart7) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    status = HAL_UARTEx_ReceiveToIdle_IT(
        &huart7,
        g_custom_controller_system.rx_chunk,
        sizeof(g_custom_controller_system.rx_chunk));
    if (status == HAL_OK)
    {
        g_custom_controller_system.restart_pending = false;
        g_custom_controller_system.next_recovery_ms = 0U;
    }
    return status;
}

static bool custom_controller_ring_take(uint8_t *byte)
{
    uint16_t read_index;

    if (byte == NULL)
    {
        return false;
    }
    read_index = g_custom_controller_system.ring_read_index;
    if (read_index == g_custom_controller_system.ring_write_index)
    {
        return false;
    }

    *byte = g_custom_controller_system.rx_ring[read_index];
    g_custom_controller_system.ring_read_index =
        (uint16_t)((read_index + 1U) % CUSTOM_CONTROLLER_RX_RING_LENGTH);
    return true;
}

void CustomControllerSystem_Init(void)
{
    CustomControllerProtocol_Init(&g_custom_controller_system.parser);
    g_custom_controller_system.snapshot.online = false;
    g_custom_controller_system.last_valid_frame_ms = 0U;
    (void)custom_controller_start_receive();
}

void CustomControllerSystem_Process(void)
{
    CustomControllerPayload_t payload;
    uint32_t now_ms = HAL_GetTick();
    uint8_t byte;

    if (g_custom_controller_system.restart_pending &&
        ((int32_t)(now_ms -
                   g_custom_controller_system.next_recovery_ms) >= 0))
    {
        if (custom_controller_start_receive() != HAL_OK)
        {
            g_custom_controller_system.next_recovery_ms =
                now_ms + CUSTOM_CONTROLLER_RECOVERY_MS;
        }
    }

    while (custom_controller_ring_take(&byte))
    {
        if (CustomControllerProtocol_AcceptByte(
                &g_custom_controller_system.parser,
                byte,
                &payload))
        {
            uint32_t joint;

            taskENTER_CRITICAL();
            g_custom_controller_system.snapshot.work_mode = payload.work_mode;
            for (joint = 0U;
                 joint < CUSTOM_CONTROLLER_JOINT_COUNT;
                 joint++)
            {
                g_custom_controller_system.snapshot.joint_target_rad[joint] =
                    payload.joint_target_rad[joint];
            }
            g_custom_controller_system.snapshot.button_pressed =
                payload.button_pressed;
            g_custom_controller_system.snapshot.valid_frame_sequence++;
            g_custom_controller_system.snapshot.online = true;
            g_custom_controller_system.last_valid_frame_ms = now_ms;
            taskEXIT_CRITICAL();
        }
    }

    if (g_custom_controller_system.snapshot.online &&
        ((uint32_t)(now_ms -
                    g_custom_controller_system.last_valid_frame_ms) >
         CUSTOM_CONTROLLER_OFFLINE_MS))
    {
        taskENTER_CRITICAL();
        g_custom_controller_system.snapshot.online = false;
        taskEXIT_CRITICAL();
    }
}

void CustomControllerSystem_CopySnapshot(
    CustomControllerSnapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }
    taskENTER_CRITICAL();
    *snapshot = g_custom_controller_system.snapshot;
    taskEXIT_CRITICAL();
}

void CustomControllerSystem_UartRxEvent(uint16_t size)
{
    uint16_t byte_index;

    if (size > CUSTOM_CONTROLLER_RX_CHUNK_LENGTH)
    {
        size = CUSTOM_CONTROLLER_RX_CHUNK_LENGTH;
    }
    for (byte_index = 0U; byte_index < size; byte_index++)
    {
        uint16_t write_index = g_custom_controller_system.ring_write_index;
        uint16_t next_index = (uint16_t)(
            (write_index + 1U) % CUSTOM_CONTROLLER_RX_RING_LENGTH);

        if (next_index == g_custom_controller_system.ring_read_index)
        {
            break;
        }
        g_custom_controller_system.rx_ring[write_index] =
            g_custom_controller_system.rx_chunk[byte_index];
        g_custom_controller_system.ring_write_index = next_index;
    }
    g_custom_controller_system.restart_pending = true;
}

void CustomControllerSystem_UartError(void)
{
    g_custom_controller_system.restart_pending = true;
}
