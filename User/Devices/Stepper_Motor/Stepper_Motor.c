/**
 ******************************************************************************
 * @file    Stepper_Motor.c
 * @version V1.0.0
 * @date    2026.04.11
 * @brief   步进电机驱动函数
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Includes ---------------------------------------------------------------- */
#include "Stepper_Motor.h"
#include "bsp_uart.h"
#include "cmsis_os.h"

/* Defines ----------------------------------------------------------------- */

/* Global variable --------------------------------------------------------- */
__attribute__((section (".AXI_SRAM"))) uint8_t Stepper_MultiRx_Buf[2][STEPPER_MOTOR_RX_BUF_LEN];
Stepper_Motor_RxInfo_t Stepper_Rx_Info;
static Stepper_Motor_Info_t g_stepper_motor[4];

/* Static Fun -------------------------------------------------------------- */
static void Stepper_Motor_Call_RxInfo(Stepper_Motor_Info_t *Motor, uint32_t Delay_Time);
static void Stepper_Motor_Call_Zero_RxInfo(Stepper_Motor_Info_t *Motor, uint32_t Delay_Time);
static void Stepper_Turn_sign(Stepper_Motor_RxInfo_t *RxInfo, Stepper_Motor_Info_t *Motor);
static void Stepper_Motor_Info_Update(Stepper_Motor_Info_t *Motor, Stepper_Motor_RxInfo_t *RxInfo, uint8_t code);
static void Stepper_ID_Select(uint8_t ID, Stepper_Motor_RxInfo_t *Stepper_RxInfo, uint8_t code);

/* Functions --------------------------------------------------------------- */
/**
 * @brief	步进电机校准指令
 * @param	Motor: 步进电机信息结构体指针
 * @param	Delay_Time: 延时时间
 * @return	无
 * @note	无
 */
void Stepper_Motor_Call_Cal(Stepper_Motor_Info_t *Motor, uint32_t Delay_Time)
{
    static uint8_t Tx_Data[4] = {0}; 
    Tx_Data[0] = Motor->Set.motor_Addr;
    Tx_Data[1] = STEPPER_CAL_CODE;
    Tx_Data[2] = 0x45;
    Tx_Data[3] = STEPPER_CHECK_CODE;

    HAL_UART_Transmit(&huart10, Tx_Data, sizeof(Tx_Data), 10);
    osDelay(Delay_Time);
}
/**
 * @brief	步进电机位置停止指令
 * @param	Motor: 步进电机信息结构体指针
 * @param	Delay_Time: 延时时间
 * @return	无
 * @note	无
 */
void Stepper_Motor_Set_Pos_Stop(Stepper_Motor_Info_t *Motor, uint32_t Delay_Time)
{
    static uint8_t Tx_Data[5] = {0};
    Tx_Data[0] = Motor->Set.motor_Addr;
    Tx_Data[1] = STEPPER_POS_STOP_CODE;
    Tx_Data[2] = 0x98;
    Tx_Data[3] = 0x00;				/* 同步标志位(0:立即执行,1:先缓存当前命令) */
    Tx_Data[4] = STEPPER_CHECK_CODE;

    HAL_UART_Transmit(&huart10, Tx_Data, sizeof(Tx_Data), 10);
    osDelay(Delay_Time);
}

/**
 * @brief	步进电机回零指令
 * @param	Motor: 步进电机信息结构体指针
 * @param	Zero_Mode: 回零模式
 * @param	Delay_Time: 延时时间
 * @return	无
 * @note	无
 */
void Stepper_Motor_Set_Zero_Cmd(Stepper_Motor_Info_t *Motor,
                                Stepper_Motor_Zero_Mode_e Zero_Mode, 
                                uint32_t Delay_Time)
{
    static uint8_t Tx_Data[5] = {0};
    Tx_Data[0] = Motor->Set.motor_Addr;
    Tx_Data[1] = STEPPER_ZERO_CMD_CODE;
    Tx_Data[2] = Zero_Mode;
    Tx_Data[3] = 0x00;				/* 同步标志位(0:立即执行,1:先缓存当前命令) */
    Tx_Data[4] = STEPPER_CHECK_CODE;

    HAL_UART_Transmit(&huart10, Tx_Data, sizeof(Tx_Data), 10);
    osDelay(Delay_Time);
}

/**
 * @brief	步进电机设置回零参数
 * @param	Motor: 步进电机信息结构体指针
 * @param	Delay_Time: 延时时间
 * @return	无
 * @note	无
 */
void Stepper_Motor_Set_Zero(Stepper_Motor_Info_t *Motor, uint32_t Delay_Time)
{
    static uint8_t Tx_Data[20] = {0};
    Tx_Data[0] = Motor->Set.motor_Addr;
    Tx_Data[1] = STEPPER_ZERO_SET_CODE;
    Tx_Data[2] = 0xAE;
    Tx_Data[3] = Motor->Set.Zero_Set.zero_save;
    Tx_Data[4] = Motor->Set.Zero_Set.zero_mode;
    Tx_Data[5] = Motor->Set.Zero_Set.zero_dir;
    Tx_Data[6] = (Motor->Set.Zero_Set.zero_speed >> 8) & 0xFF;             
    Tx_Data[7] = Motor->Set.Zero_Set.zero_speed & 0xFF;
    Tx_Data[8] = (Motor->Set.Zero_Set.zero_out_time >> 24) & 0xFF;
    Tx_Data[9] = (Motor->Set.Zero_Set.zero_out_time >> 16) & 0xFF;
    Tx_Data[10] = (Motor->Set.Zero_Set.zero_out_time >> 8) & 0xFF;
    Tx_Data[11] = (Motor->Set.Zero_Set.zero_out_time & 0xFF);
    Tx_Data[12] = (Motor->Set.Zero_Set.zero_detect_speed >> 8) & 0xFF;
    Tx_Data[13] = Motor->Set.Zero_Set.zero_detect_speed & 0xFF;
    Tx_Data[14] = (Motor->Set.Zero_Set.zero_detect_current >> 8) & 0xFF;
    Tx_Data[15] = (Motor->Set.Zero_Set.zero_detect_current & 0xFF);
    Tx_Data[16] = (Motor->Set.Zero_Set.zero_detect_time >> 8) & 0xFF;
    Tx_Data[17] = (Motor->Set.Zero_Set.zero_detect_time & 0xFF);
    Tx_Data[18] = Motor->Set.Zero_Set.zero_POT_En;
    Tx_Data[19] = STEPPER_CHECK_CODE;

    HAL_UART_Transmit(&huart10, Tx_Data, sizeof(Tx_Data), 10);
    osDelay(Delay_Time);
}

/**
 * @brief	步进电机使能控制指令
 * @param	Motor: 步进电机信息结构体指针
 * @param	Cmd: 使能控制命令
 * @param	Delay_Time: 延时时间
 * @return	无
 * @note	无
 */
void Stepper_Motor_Set_Cmd(Stepper_Motor_Info_t *Motor, 
                            Stepper_Motor_Cmd_e Cmd,
                            uint32_t Delay_Time)
{
    static uint8_t Tx_Data[6] = {0}; 
    Tx_Data[0] = Motor->Set.motor_Addr;
    Tx_Data[1] = STEPPER_CMD_CODE;
    Tx_Data[2] = 0xAB;
    Tx_Data[3] = Cmd;   
    Tx_Data[4] = 0x00;				/* 同步标志位(0:立即执行,1:先缓存当前命令) */
    Tx_Data[5] = STEPPER_CHECK_CODE;
    
    HAL_UART_Transmit(&huart10, Tx_Data, sizeof(Tx_Data), 10);
    osDelay(Delay_Time);
}

/**
 * @brief	步进电机位置控制指令
 * @param	Motor: 步进电机信息结构体指针
 * @param	Speed: 目标速度
 * @param	Ac_speed: 加速度
 * @param	Target_pos: 目标位置(角度)
 * @param	Pose_Mode: 位置模式
 * @param	Delay_Time: 延时时间
 * @return	无
 * @note	无
 */
void Stepper_Motor_Set_Pos(Stepper_Motor_Info_t *Motor,
                           uint16_t Speed,
                           uint8_t Ac_speed, 
                           float Target_pos, 
                           Stepper_Motor_Pos_Mode_e Pose_Mode,
                           uint32_t Delay_Time)
{
    static uint8_t Tx_Data[13] = {0}; 
    int32_t signed_pulse = (int32_t)(Target_pos / 0.1125f + 0.5f);
    Stepper_Motor_Dir_e Dir = Stepper_CW;

    if (signed_pulse < 0)
    {
        Dir = Stepper_CCW;
        signed_pulse = -signed_pulse;
    }

    uint32_t Pulse = (uint32_t)signed_pulse;
    Tx_Data[0] = Motor->Set.motor_Addr;
    Tx_Data[1] = STEPPER_POS_CODE;
    Tx_Data[2] = Dir;
    Tx_Data[3] = (Speed >> 8) & 0xFF;             
    Tx_Data[4] = Speed & 0xFF;
    Tx_Data[5] = Ac_speed;  
    Tx_Data[6] = (Pulse >> 24) & 0xFF;
    Tx_Data[7] = (Pulse >> 16) & 0xFF;
    Tx_Data[8] = (Pulse >> 8) & 0xFF;
    Tx_Data[9] = Pulse & 0xFF;
    Tx_Data[10] = Pose_Mode;
    Tx_Data[11] = 0x00;
    Tx_Data[12] = STEPPER_CHECK_CODE;

    HAL_UART_Transmit(&huart10, Tx_Data, 13, 10);
    osDelay(Delay_Time);
}

/**
 * @brief	步进电机限流位置控制指令
 * @param	Motor: 步进电机信息结构体指针
 * @param	Speed: 目标速度 (0-30000, 对应 0-3000.0 RPM)
 * @param	Target_pos: 目标位置(角度, 单位: °)
 * @param	Pose_Mode: 位置模式
 * @param	Max_Current: 最大电流限制 (0-5000, 单位: mA)
 * @param	Delay_Time: 延时时间
 * @return	无
 * @note	功能码 0xCB, 位置以 0.1° 为单位编码
 */
void Stepper_Motor_Set_Pos_Current(Stepper_Motor_Info_t *Motor,
                                   uint16_t Speed,
                                   float Target_pos,
                                   Stepper_Motor_Pos_Mode_e Pose_Mode,
                                   uint16_t Max_Current,
                                   uint32_t Delay_Time)
{
    static uint8_t Tx_Data[14] = {0};
    int32_t pos_raw;
    Stepper_Motor_Dir_e Dir = Stepper_CW;

    pos_raw = (int32_t)(Target_pos * 10.0f + 0.5f);

    if (pos_raw < 0)
    {
        Dir = Stepper_CCW;
        pos_raw = -pos_raw;
    }

    Tx_Data[0] = Motor->Set.motor_Addr;
    Tx_Data[1] = STEPPER_POS_CURRENT_CODE;
    Tx_Data[2] = Dir;
    Tx_Data[3] = (Speed >> 8) & 0xFF;
    Tx_Data[4] = Speed & 0xFF;
    Tx_Data[5] = (pos_raw >> 24) & 0xFF;
    Tx_Data[6] = (pos_raw >> 16) & 0xFF;
    Tx_Data[7] = (pos_raw >> 8) & 0xFF;
    Tx_Data[8] = pos_raw & 0xFF;
    Tx_Data[9] = Pose_Mode;
    Tx_Data[10] = 0x00;
    Tx_Data[11] = (Max_Current >> 8) & 0xFF;
    Tx_Data[12] = Max_Current & 0xFF;
    Tx_Data[13] = STEPPER_CHECK_CODE;

    HAL_UART_Transmit(&huart10, Tx_Data, 14, 10);
    osDelay(Delay_Time);
}

/**
 * @brief	步进电机速度模式控制指令
 * @param	Motor: 步进电机信息结构体指针
 * @param	Speed: 目标速度 (-3000 ~ +3000, 单位: RPM, 正=CW 负=CCW)
 * @param	Accel: 加速度档位 (0-255, 0=无加减速, 值越大加速越快)
 * @param	Delay_Time: 延时时间
 * @return	无
 * @note	功能码 0xF6, 协议 5.3.7 速度模式控制（Emm）
 */
void Stepper_Motor_Set_Speed(Stepper_Motor_Info_t *Motor,
                             int16_t Speed,
                             uint8_t Accel,
                             uint32_t Delay_Time)
{
    static uint8_t Tx_Data[8] = {0};
    uint16_t abs_speed;
    uint8_t dir;

    if (Speed >= 0)
    {
        dir = 0x00;
        abs_speed = (uint16_t)Speed;
    }
    else
    {
        dir = 0x01;
        abs_speed = (uint16_t)(-Speed);
    }

    Tx_Data[0] = Motor->Set.motor_Addr;
    Tx_Data[1] = STEPPER_SPEED_CODE;
    Tx_Data[2] = dir;
    Tx_Data[3] = (abs_speed >> 8) & 0xFF;
    Tx_Data[4] = abs_speed & 0xFF;
    Tx_Data[5] = Accel;
    Tx_Data[6] = 0x00;
    Tx_Data[7] = STEPPER_CHECK_CODE;

    HAL_UART_Transmit(&huart10, Tx_Data, 8, 10);
    osDelay(Delay_Time);
}

/**
 * @brief	步进电机状态信息查询
 * @param	Motor: 步进电机信息结构体指针
 * @param	Delay_Time: 延时时间
 * @return	无
 * @note	调用读取数据和读取回零参数，并更新状态标志
 */
void Stepper_Motor_Call_Info(Stepper_Motor_Info_t *Motor, uint32_t Delay_Time)
{
    Stepper_Motor_Call_RxInfo(Motor, Delay_Time);
    Stepper_Motor_Call_Zero_RxInfo(Motor, Delay_Time);
    if (Motor->Data.Rx_Addr == Motor->Set.motor_Addr)
        Motor->Motor_Update_Flag = Motor_Update_Success;
    else
        Motor->Motor_Update_Flag = Motor_Update_Fail;

}

/* Private functions ------------------------------------------------------- */
/**
 * @brief	步进电机读取运行数据
 * @param	Motor: 步进电机信息结构体指针
 * @param	Delay_Time: 延时时间
 * @return	无
 * @note	静态函数，由 Call_Info 调用
 */
static void Stepper_Motor_Call_RxInfo(Stepper_Motor_Info_t *Motor, uint32_t Delay_Time)
{
    static uint8_t Tx_Data[4] = {0}; 
    Tx_Data[0] = Motor->Set.motor_Addr;
    Tx_Data[1] = STEPPER_RX_CODE;
    Tx_Data[2] = 0x7A;
    Tx_Data[3] = STEPPER_CHECK_CODE;

    HAL_UART_Transmit(&huart10, Tx_Data, sizeof(Tx_Data), 10);
    osDelay(Delay_Time);
}
/**
 * @brief	步进电机读取回零参数
 * @param	Motor: 步进电机信息结构体指针
 * @param	Delay_Time: 延时时间
 * @return	无
 * @note	静态函数，由 Call_Info 调用
 */
static void Stepper_Motor_Call_Zero_RxInfo(Stepper_Motor_Info_t *Motor, uint32_t Delay_Time)
{
    static uint8_t Tx_Data[3] = {0};
    Tx_Data[0] = Motor->Set.motor_Addr;
    Tx_Data[1] = STEPPER_ZERO_RX_CODE;
    Tx_Data[2] = STEPPER_CHECK_CODE;

    HAL_UART_Transmit(&huart10, Tx_Data, sizeof(Tx_Data), 10);
    osDelay(Delay_Time);
}
/**
 * @brief	步进电机符号位转换
 * @param	RxInfo: 接收数据结构体指针
 * @param	Motor: 步进电机信息结构体指针
 * @return	无
 * @note	根据通信协议符号位转换数据正负
 */
static void Stepper_Turn_sign(Stepper_Motor_RxInfo_t *RxInfo, 
                              Stepper_Motor_Info_t *Motor)
{
    if(RxInfo->sign1 == 1) Motor->Data.target_pos = -Motor->Data.target_pos;
    if(RxInfo->sign2 == 1) Motor->Data.speed      = -Motor->Data.speed;
    if(RxInfo->sign3 == 1) Motor->Data.pos        = -Motor->Data.pos;
    if(RxInfo->sign4 == 1) Motor->Data.pos_error  = -Motor->Data.pos_error;
}

/**
 * @brief	步进电机接收数据解析
 * @param	Motor: 步进电机信息结构体指针
 * @param	RxInfo: 接收数据结构体指针
 * @param	code: 指令码
 * @return	无
 * @note	根据指令码解析不同数据帧
 */
static void Stepper_Motor_Info_Update(Stepper_Motor_Info_t *Motor, 
                                      Stepper_Motor_RxInfo_t *RxInfo, 
                                      uint8_t code)
{
    switch(code)
    {
        case STEPPER_RX_CODE:
        {
            Motor->Data.Rx_Addr = RxInfo->motor_Addr;
            Motor->Data.bus_voltage = (float)RxInfo->bus_v_u16;
            Motor->Data.phase_current = (float)RxInfo->phase_c_u16;
            Motor->Data.line_encoder_value = ((float)RxInfo->line_encoder_u16 * 360.0f) / 65536.0f;
            Motor->Data.target_pos = ((float)RxInfo->target_pos_u32 * 360.0f) / 65536.0f ;
            Motor->Data.speed = (float)RxInfo->speed_u16 ;
            Motor->Data.pos = ((float)RxInfo->pos_u32 * 360.0f) / 65536.0f ;
            Motor->Data.pos_error = ((float)RxInfo->pos_error_u32 * 360.0f) / 65536.0f ;
            Stepper_Turn_sign(RxInfo,Motor);
            Motor->Data.zero_status.byte = RxInfo->zero_s;
            Motor->Data.motor_status.byte = RxInfo->motor_s;
        } break;

        case STEPPER_CMD_CODE:
        {
            Motor->Data.return_status.Motor_Cmd = (Stepper_Motor_Return_e)RxInfo->Return_state_Cmd;
        } break;

        case STEPPER_SPEED_CODE:
        {
            Motor->Data.return_status.Motor_Cmd = (Stepper_Motor_Return_e)RxInfo->Return_state_Cmd;
        } break;

        case STEPPER_CAL_CODE:
        {
            Motor->Data.return_status.Motor_Cal = (Stepper_Motor_Return_e)RxInfo->Return_state_Cal;
        } break;
        
        case STEPPER_POS_CODE:
        {
            Motor->Data.return_status.Motor_Pos = (Stepper_Motor_Return_e)RxInfo->Return_state_Pos;
        } break;
        
        case STEPPER_ZERO_CMD_CODE:
        {
            Motor->Data.return_status.Motor_Zero_Cmd = (Stepper_Motor_Return_e)RxInfo->Return_state_Zero_Cmd;
        } break;

        case STEPPER_ZERO_RX_CODE:
        {
            Motor->Zero_Data.zero_mode = (Stepper_Motor_Zero_Mode_e)RxInfo->zero_mode;
            Motor->Zero_Data.zero_dir = (Stepper_Motor_Dir_e)RxInfo->zero_dir;
            Motor->Zero_Data.zero_out_time = (float)RxInfo->zero_out_time_u32;
            Motor->Zero_Data.zero_speed = (float)RxInfo->zero_speed_u16 ;
            Motor->Zero_Data.zero_detect_speed = (float)RxInfo->zero_detect_speed_u16 ;
            Motor->Zero_Data.zero_detect_current = (float)RxInfo->zero_detect_current_u16;
            Motor->Zero_Data.zero_detect_time = (float)RxInfo->zero_detect_time_u16;
            Motor->Zero_Data.zero_POT_En = RxInfo->zero_POT_En;
        } break;
        
        case STEPPER_ZERO_SET_CODE:
        {
            Motor->Data.return_status.Motor_Zero_Set = (Stepper_Motor_Return_e)RxInfo->Return_state_Zero_Set;
        } break;
        
        case STEPPER_POS_STOP_CODE:
        {
            Motor->Data.return_status.Motor_Pos_Stop = (Stepper_Motor_Return_e)RxInfo->Return_state_Pos_Stop;
        } break;
        
        default:
        break;
    }
}

/**
 * @brief	步进电机ID选择与数据分发
 * @param	ID: 电机地址ID
 * @param	Stepper_RxInfo: 接收数据结构体指针
 * @param	code: 指令码
 * @return	无
 * @note	根据ID调用对应的电机数据更新函数
 */
static void Stepper_ID_Select(uint8_t ID, 
                              Stepper_Motor_RxInfo_t *Stepper_RxInfo, 
                              uint8_t code)
{
    switch(ID)
    {
        case 1: Stepper_Motor_Info_Update(&g_stepper_motor[0], Stepper_RxInfo, code); break;
        case 2: Stepper_Motor_Info_Update(&g_stepper_motor[1], Stepper_RxInfo, code); break;
        case 3: Stepper_Motor_Info_Update(&g_stepper_motor[2], Stepper_RxInfo, code); break;
        case 4: Stepper_Motor_Info_Update(&g_stepper_motor[3], Stepper_RxInfo, code); break;
        default: break;
    }
}

/**
 * @brief	步进电机接收数据解包
 * @param	Rx_Buf: 接收数据缓冲区
 * @param	RxInfo: 接收数据结构体指针
 * @return	无
 * @note	无
 */
void Stepper_RxInfo_Unpack(uint8_t *Rx_Buf, Stepper_Motor_RxInfo_t *RxInfo)
{
    RxInfo->motor_Addr = Rx_Buf[0];
    switch(Rx_Buf[1])
    {
        case STEPPER_RX_CODE:
        {
            if (Rx_Buf[2] == STEPPER_MOTOR_RX_BUF_LEN && Rx_Buf[3] == 9)
            {
                if (Rx_Buf[30] == STEPPER_CHECK_CODE)
                {
                    RxInfo->bus_v_u16 = (Rx_Buf[4] << 8) | Rx_Buf[5];
                    RxInfo->phase_c_u16 = (Rx_Buf[6] << 8) | Rx_Buf[7];
                    RxInfo->line_encoder_u16 = (Rx_Buf[8] << 8) | Rx_Buf[9];
                    RxInfo->sign1 = Rx_Buf[10];
                    RxInfo->target_pos_u32 = ((uint32_t)Rx_Buf[11] << 24) | 
                                         ((uint32_t)Rx_Buf[12] << 16) | 
                                         ((uint32_t)Rx_Buf[13] << 8) | 
                                         ((uint32_t)Rx_Buf[14]);
                    RxInfo->sign2 = Rx_Buf[15];
                    RxInfo->speed_u16 = (Rx_Buf[16] << 8) | Rx_Buf[17];
                    RxInfo->sign3 = Rx_Buf[18];
                    RxInfo->pos_u32 = ((uint32_t)Rx_Buf[19] << 24) | 
                                      ((uint32_t)Rx_Buf[20] << 16) | 
                                      ((uint32_t)Rx_Buf[21] << 8) | 
                                      ((uint32_t)Rx_Buf[22]);
                    RxInfo->sign4 = Rx_Buf[23];
                    RxInfo->pos_error_u32 = ((uint32_t)Rx_Buf[24] << 24) | 
                                            ((uint32_t)Rx_Buf[25] << 16) | 
                                            ((uint32_t)Rx_Buf[26] << 8) | 
                                            ((uint32_t)Rx_Buf[27]);
                    RxInfo->zero_s = Rx_Buf[28];
                    RxInfo->motor_s = Rx_Buf[29];
	        		Stepper_ID_Select(RxInfo->motor_Addr, RxInfo, STEPPER_RX_CODE);
                }
           }
        } break;

        case STEPPER_CMD_CODE:
        {
            if(Rx_Buf[3] == STEPPER_CHECK_CODE)
            {
                RxInfo->Return_state_Cmd = Rx_Buf[2];
                Stepper_ID_Select(RxInfo->motor_Addr, RxInfo, STEPPER_CMD_CODE);
            }
        } break;

        case STEPPER_CAL_CODE:
        {
            if(Rx_Buf[3] == STEPPER_CHECK_CODE)
            {
                RxInfo->Return_state_Cal = Rx_Buf[2];
                Stepper_ID_Select(RxInfo->motor_Addr, RxInfo, STEPPER_CAL_CODE);
            }
        } break;

        case STEPPER_POS_CODE:
        {
            if(Rx_Buf[3] == STEPPER_CHECK_CODE)
            {
                RxInfo->Return_state_Pos = Rx_Buf[2];
                Stepper_ID_Select(RxInfo->motor_Addr, RxInfo, STEPPER_POS_CODE);
            }
        } break;

        case STEPPER_ZERO_CMD_CODE:
        {
            if(Rx_Buf[3] == STEPPER_CHECK_CODE)
            {
                RxInfo->Return_state_Zero_Cmd = Rx_Buf[2];
                Stepper_ID_Select(RxInfo->motor_Addr, RxInfo, STEPPER_ZERO_CMD_CODE);
            }
        } break;

        case STEPPER_ZERO_RX_CODE:
        {
            if(Rx_Buf[17] == STEPPER_CHECK_CODE)
            {
                RxInfo->zero_mode = Rx_Buf[2];
                RxInfo->zero_dir = Rx_Buf[3];
                RxInfo->zero_speed_u16 = (Rx_Buf[4] << 8) | Rx_Buf[5];
                RxInfo->zero_out_time_u32 = ((uint32_t)Rx_Buf[6] << 24) | 
                                            ((uint32_t)Rx_Buf[7] << 16) | 
                                            ((uint32_t)Rx_Buf[8] << 8) | 
                                            ((uint32_t)Rx_Buf[9]);
                RxInfo->zero_detect_speed_u16 = (Rx_Buf[10] << 8) | Rx_Buf[11];
                RxInfo->zero_detect_current_u16 = (Rx_Buf[12] << 8) | Rx_Buf[13];
                RxInfo->zero_detect_time_u16 = (Rx_Buf[14] << 8) | Rx_Buf[15];
                RxInfo->zero_POT_En = Rx_Buf[16];
                Stepper_ID_Select(RxInfo->motor_Addr, RxInfo, STEPPER_ZERO_RX_CODE);
            }
        } break;

        case STEPPER_ZERO_SET_CODE:
        {
            if(Rx_Buf[3] == STEPPER_CHECK_CODE)
            {
                RxInfo->Return_state_Zero_Set = Rx_Buf[2];
                Stepper_ID_Select(RxInfo->motor_Addr, RxInfo, STEPPER_ZERO_SET_CODE);
            }
        } break;

        case STEPPER_POS_STOP_CODE:
        {
            if(Rx_Buf[3] == STEPPER_CHECK_CODE)
            {
                RxInfo->Return_state_Pos_Stop = Rx_Buf[2];
                Stepper_ID_Select(RxInfo->motor_Addr, RxInfo, STEPPER_POS_STOP_CODE);
            }
        } break;

        default:
        break;
    }
}

/* Interrupt functions ----------------------------------------------------- */

/* ------------------------------------------------------------------------- */
