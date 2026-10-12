/**
 ******************************************************************************
 * @file    Unitree_Motor.h
 * @brief   GO8010 串口电机驱动接口。
 ******************************************************************************
 */

#ifndef UNITREE_MOTOR_H
#define UNITREE_MOTOR_H

#include "stm32h7xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#define UNITREE_RX_BUF_LEN       16U
#define UNITREE_TX_BUF_LEN       17U
#define UNITREE_8010_RATIO       6.33f

typedef enum
{
    UNITREE_GO8010 = 0,
} Unitree_Motor_Type_e;

typedef enum
{
    UNITREE_MODE_LOCK = 0,
    UNITREE_MODE_FOC = 1,
    UNITREE_MODE_CALIB = 2,
} MotorMode_Status_e;

typedef enum
{
    MOTOR_ERROR_NORMAL = 0,
    MOTOR_ERROR_OVERHEAT = 1,
    MOTOR_ERROR_OVERCURRENT = 2,
    MOTOR_ERROR_OVERVOLTAGE = 3,
    MOTOR_ERROR_ENCODER = 4,
    MOTOR_ERROR_RESERVED_5 = 5,
    MOTOR_ERROR_RESERVED_6 = 6,
    MOTOR_ERROR_RESERVED_7 = 7,
} MotorError_Type_e;

#pragma pack(push, 1)
typedef struct
{
    uint16_t HEAD;
    union
    {
        struct
        {
            uint8_t ID : 4;
            uint8_t STATUS : 3;
            uint8_t RESV1 : 1;
        } bits;
        uint8_t raw;
    } Mode_Info;
    int16_t T_fbk;
    int16_t W_fbk;
    int32_t Theta_fbk;
    int8_t Temp;
    union
    {
        struct
        {
            uint16_t MERROR : 3;
            uint16_t FORCE : 12;
            uint16_t RESV2 : 1;
        } bits;
        uint16_t raw;
    } Status;
    uint16_t CRC16;
} Unitree_RxInfo_t;
#pragma pack(pop)

typedef struct
{
    uint8_t Rx_ID;
    float Tor;
    float Vel;
    float Pos;
    float Pos_Offset;
    float Pos_Out;
    float Temp;
    MotorMode_Status_e Mode_Status;
    MotorError_Type_e Error_Type;
    uint8_t cnt;
    float pos_sum;
    bool feedback_valid;            /* 已收到过合法反馈，不表示当前在线 */
    uint32_t last_feedback_time_ms; /* 最近合法反馈时间 */
} Unitree_Motor_Data_t;

typedef struct
{
    uint8_t Tx_ID;
    uint8_t Rx_ID;
} Unitree_Motor_ID_t;

typedef struct
{
    uint8_t Tx_ID;
    MotorMode_Status_e Mode;
    int16_t T_set;
    int16_t W_set;
    int32_t Pos_set;
    int16_t K_pos;
    int16_t K_spd;
    uint16_t CRC16;
} Unitree_Motor_Cmd_t;

typedef struct
{
    Unitree_Motor_Type_e Motor_Type;
    Unitree_Motor_ID_t ID_Set;
    Unitree_Motor_Data_t Data;
    Unitree_Motor_Cmd_t Cmd;
    uint8_t tx_buffer[UNITREE_TX_BUF_LEN];
} Unitree_Motor_Info_t;

/**
 * @brief 静态创建一个 GO8010 实例。
 * @note 固定 ID 和上电锁定状态在创建时完成，不需要运行期 Init。
 */
#define UNITREE_MOTOR_CREATE(motor_type, tx_id, rx_id) \
    {                                                   \
        .Motor_Type = (motor_type),                    \
        .ID_Set = {                                    \
            .Tx_ID = (tx_id),                          \
            .Rx_ID = (rx_id),                          \
        },                                             \
        .Cmd = {                                       \
            .Tx_ID = (tx_id),                          \
            .Mode = UNITREE_MODE_LOCK,                 \
        },                                             \
    }

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 校验并写入一帧 GO8010 反馈。
 * @return true 表示帧头、长度、CRC 和电机 ID 均有效。
 */
bool Unitree_Motor_Info_Update(Unitree_Motor_Info_t *motor,
                               const uint8_t *frame,
                               uint16_t frame_len);

/** @brief 更新 GO8010 指令缓存，不执行串口发送。 */
void Unitree_Motor_Cmd(Unitree_Motor_Info_t *motor,
                       MotorMode_Status_e mode,
                       float torque_ff_nm,
                       float velocity_rad_s,
                       float position_rad,
                       float kp,
                       float kd);

/**
 * @brief 通过 DMA 非阻塞发送当前指令缓存。
 * @return HAL 状态值，HAL_OK 为成功入队，HAL_BUSY 表示上一帧未发完。
 */
uint8_t Unitree_Motor_Ctrl(UART_HandleTypeDef *huart,
                           Unitree_Motor_Info_t *motor);

#ifdef __cplusplus
}
#endif

#endif /* UNITREE_MOTOR_H */
