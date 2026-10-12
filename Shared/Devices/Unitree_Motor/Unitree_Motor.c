/**
 ******************************************************************************
 * @file    Unitree_Motor.c
 * @brief   GO8010 帧编解码与非阻塞 DMA 发送。
 ******************************************************************************
 */

#include "Unitree_Motor.h"

#include "CRC.h"
#include "bsp_dwt.h"

#include <stddef.h>

#define UNITREE_RX_HEAD             0xFDEEU
#define UNITREE_CALIBRATION_COUNT   5U
#define UNITREE_TWO_PI              6.28318530717958647692f

static void unitree_update_offset(Unitree_Motor_Info_t *motor);

bool Unitree_Motor_Info_Update(Unitree_Motor_Info_t *motor,
                               const uint8_t *frame,
                               uint16_t frame_len)
{
    Unitree_RxInfo_t feedback;

    if ((motor == NULL) || (frame == NULL) ||
        (frame_len != UNITREE_RX_BUF_LEN))
    {
        return false;
    }

    feedback.HEAD = ((uint16_t)frame[0] << 8U) | frame[1];
    if ((feedback.HEAD != UNITREE_RX_HEAD) ||
        !Verify_CRC16_Check_Sum((uint8_t *)frame, UNITREE_RX_BUF_LEN))
    {
        return false;
    }

    feedback.Mode_Info.raw = frame[2];
    if (feedback.Mode_Info.bits.ID != motor->ID_Set.Rx_ID)
    {
        return false;
    }

    feedback.T_fbk = (int16_t)((uint16_t)frame[3] |
                               ((uint16_t)frame[4] << 8U));
    feedback.W_fbk = (int16_t)((uint16_t)frame[5] |
                               ((uint16_t)frame[6] << 8U));
    feedback.Theta_fbk = (int32_t)((uint32_t)frame[7] |
                                   ((uint32_t)frame[8] << 8U) |
                                   ((uint32_t)frame[9] << 16U) |
                                   ((uint32_t)frame[10] << 24U));
    feedback.Temp = (int8_t)frame[11];
    feedback.Status.raw = (uint16_t)frame[12] |
                          ((uint16_t)frame[13] << 8U);
    feedback.CRC16 = (uint16_t)frame[14] |
                     ((uint16_t)frame[15] << 8U);

    motor->Data.Rx_ID = feedback.Mode_Info.bits.ID;
    motor->Data.Error_Type =
        (MotorError_Type_e)feedback.Status.bits.MERROR;
    motor->Data.Mode_Status =
        (MotorMode_Status_e)feedback.Mode_Info.bits.STATUS;
    motor->Data.Tor = (float)feedback.T_fbk /
                      256.0f * UNITREE_8010_RATIO;
    motor->Data.Vel = (float)feedback.W_fbk * UNITREE_TWO_PI /
                      256.0f / UNITREE_8010_RATIO;
    motor->Data.Pos = (float)feedback.Theta_fbk * UNITREE_TWO_PI /
                      32768.0f / UNITREE_8010_RATIO;
    motor->Data.Temp = (float)feedback.Temp;
    motor->Data.feedback_valid = true;
    motor->Data.last_feedback_time_ms = DWT_GetTimeMs();
    unitree_update_offset(motor);

    return true;
}

void Unitree_Motor_Cmd(Unitree_Motor_Info_t *motor,
                       MotorMode_Status_e mode,
                       float torque_ff_nm,
                       float velocity_rad_s,
                       float position_rad,
                       float kp,
                       float kd)
{
    if (motor == NULL)
    {
        return;
    }

    motor->Cmd.Tx_ID = motor->ID_Set.Tx_ID & 0x0FU;
    motor->Cmd.Mode = (MotorMode_Status_e)((uint8_t)mode & 0x07U);
    motor->Cmd.T_set = (int16_t)(torque_ff_nm * 256.0f);
    motor->Cmd.W_set = (int16_t)(velocity_rad_s * 256.0f /
                                 UNITREE_TWO_PI * UNITREE_8010_RATIO);
    motor->Cmd.Pos_set = (int32_t)(position_rad * 32768.0f /
                                   UNITREE_TWO_PI * UNITREE_8010_RATIO);
    motor->Cmd.K_pos = (int16_t)(kp * 1280.0f);
    motor->Cmd.K_spd = (int16_t)(kd * 1280.0f);
}

uint8_t Unitree_Motor_Ctrl(UART_HandleTypeDef *huart,
                           Unitree_Motor_Info_t *motor)
{
    uint8_t *data;

    if ((huart == NULL) || (motor == NULL))
    {
        return (uint8_t)HAL_ERROR;
    }
    if (huart->gState != HAL_UART_STATE_READY)
    {
        return (uint8_t)HAL_BUSY;
    }

    data = motor->tx_buffer;
    data[0] = 0xFEU;
    data[1] = 0xEEU;
    data[2] = (uint8_t)(((uint8_t)motor->Cmd.Mode & 0x07U) << 4U) |
              (motor->Cmd.Tx_ID & 0x0FU);
    data[3] = (uint8_t)((uint16_t)motor->Cmd.T_set & 0xFFU);
    data[4] = (uint8_t)(((uint16_t)motor->Cmd.T_set >> 8U) & 0xFFU);
    data[5] = (uint8_t)((uint16_t)motor->Cmd.W_set & 0xFFU);
    data[6] = (uint8_t)(((uint16_t)motor->Cmd.W_set >> 8U) & 0xFFU);
    data[7] = (uint8_t)((uint32_t)motor->Cmd.Pos_set & 0xFFU);
    data[8] = (uint8_t)(((uint32_t)motor->Cmd.Pos_set >> 8U) & 0xFFU);
    data[9] = (uint8_t)(((uint32_t)motor->Cmd.Pos_set >> 16U) & 0xFFU);
    data[10] = (uint8_t)(((uint32_t)motor->Cmd.Pos_set >> 24U) & 0xFFU);
    data[11] = (uint8_t)((uint16_t)motor->Cmd.K_pos & 0xFFU);
    data[12] = (uint8_t)(((uint16_t)motor->Cmd.K_pos >> 8U) & 0xFFU);
    data[13] = (uint8_t)((uint16_t)motor->Cmd.K_spd & 0xFFU);
    data[14] = (uint8_t)(((uint16_t)motor->Cmd.K_spd >> 8U) & 0xFFU);

    motor->Cmd.CRC16 = Get_CRC16_Check_Sum(
        data,
        UNITREE_TX_BUF_LEN - 2U,
        0x0000U);
    data[15] = (uint8_t)(motor->Cmd.CRC16 & 0xFFU);
    data[16] = (uint8_t)((motor->Cmd.CRC16 >> 8U) & 0xFFU);

    return (uint8_t)HAL_UART_Transmit_DMA(
        huart,
        data,
        UNITREE_TX_BUF_LEN);
}

static void unitree_update_offset(Unitree_Motor_Info_t *motor)
{
    if (motor->Data.cnt < UNITREE_CALIBRATION_COUNT)
    {
        motor->Data.pos_sum += motor->Data.Pos;
        motor->Data.cnt++;
        if (motor->Data.cnt == UNITREE_CALIBRATION_COUNT)
        {
            motor->Data.Pos_Offset = motor->Data.pos_sum /
                                     (float)UNITREE_CALIBRATION_COUNT;
        }
        return;
    }

    motor->Data.Pos_Out = motor->Data.Pos - motor->Data.Pos_Offset;
}
