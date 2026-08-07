/**
 ******************************************************************************
 * @file    DM_Motor.c
 * @version V1.0.0
 * @date    2026.03.04
 * @brief   DM系列电机驱动函数
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Includes ---------------------------------------------------------------- */
#include "DM_Motor.h"
#include "bsp_dwt.h"
#include "fdcan.h"
#include "arm_math.h"
#include "cmsis_os2.h"

/* Defines ----------------------------------------------------------------- */

/* Global variable --------------------------------------------------------- */

/* Static Fun -------------------------------------------------------------- */

/* Functions --------------------------------------------------------------- */
/**
 * @brief  十六进制转浮点数
 * @param  Byte: 十六进制数组指针
 * @param  num: 字节数
 * @return 浮点数
 * @note   无
 */
float Hex_To_Float(uint32_t *Byte,int num)
{
	return *((float*)Byte);
}

/**
 * @brief  浮点数转十六进制
 * @param  HEX: 浮点数
 * @return 十六进制
 * @note   无
 */
uint32_t FloatTohex(float HEX)
{
	return *( uint32_t *)&HEX;
}

/**
 * @brief  浮点数转无符号整型
 * @param  x_float: 要转换的浮点数
 * @param  x_min: 范围最小值
 * @param  x_max: 范围最大值
 * @param  bits: 目标无符号整数的位数
 * @return 无符号整数
 * @note   将浮点数x在指定范围[x_min, x_max]内进行线性映射，映射为指定位数的一个无符号整数
 */
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

/**
 * @brief  无符号整型转浮点数
 * @param  x_int: 要转换的无符号整数
 * @param  x_min: 范围最小值
 * @param  x_max: 范围最大值
 * @param  bits: 无符号整数的位数
 * @return 浮点数
 * @note   将无符号整数x_int在指定范围[x_min, x_max]内进行线性映射，映射为一个浮点数
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @brief  DM电机反馈数据处理
 * @param  motor: 电机信息结构体指针
 * @param  rx_data: 接收数据指针
 * @param  data_len: 数据长度
 * @return 无
 * @note   从接收到的CAN数据中提取DM电机反馈信息，包括ID、状态、位置、速度、扭矩和温度
 */
void DM_Motor_Info_Update(DM_Motor_Info_Typedef *motor, uint8_t *rx_data,uint32_t data_len)
{ 
	if(data_len==FDCAN_DLC_BYTES_8)
	{
	  motor->Data.id = (rx_data[0])&0x0F;
	  motor->Data.state = (rx_data[0])>>4;
	  motor->Data.p_int=(rx_data[1]<<8)|rx_data[2];
	  motor->Data.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	  motor->Data.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	  motor->feedback_valid = true;
	  motor->last_feedback_time_ms = DWT_GetTimeMs();
	  switch(motor->Motor_Type)
	  {
		case DM_J4310:
	  	{	
		 	motor->Data.pos = uint_to_float(motor->Data.p_int, J4310_P_MIN, J4310_P_MAX, 16);
		 	motor->Data.vel = uint_to_float(motor->Data.v_int, J4310_V_MIN, J4310_V_MAX, 12);
	 	 	motor->Data.tor = uint_to_float(motor->Data.t_int, J4310_T_MIN, J4310_T_MAX, 12);
	  	}break;
	  	case DM_J4340:
	  	{
		 	motor->Data.pos = uint_to_float(motor->Data.p_int, J4340_P_MIN, J4340_P_MAX, 16);
		 	motor->Data.vel = uint_to_float(motor->Data.v_int, J4340_V_MIN, J4340_V_MAX, 12);
	 	 	motor->Data.tor = uint_to_float(motor->Data.t_int, J4340_T_MIN, J4340_T_MAX, 12);
	  	}break;
	  	case DM_J8006:
	  	{
		 	
		}break;
	  	default:
	  	{
		 	break;
	  	}		
	  }
	  motor->Data.Tmos = (float)(rx_data[6]);
	  motor->Data.Tcoil = (float)(rx_data[7]);
	}
}

/**
 * @brief  使能电机模式
 * @param  hcan: CAN句柄
 * @param  motor_id: 电机ID
 * @param  mode_id: 模式ID
 * @param  delay_time: 延时时间
 * @return 0 表示CAN发送成功，非零表示发送失败
 * @note   delay_time为0时不阻塞当前任务
 */
uint8_t DM_Enable_Motor(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id, uint8_t delay_time)
{
	uint8_t data[8];
	uint8_t send_status;
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
	
	send_status = canx_send_data(hcan, id, data, 8);
	if(delay_time > 0U)
	{
		osDelay(delay_time);
	}
	return send_status;
}

/**
 * @brief  保存电机零点
 * @param  hcan: CAN句柄
 * @param  motor_id: 电机ID
 * @param  mode_id: 模式ID
 * @param  delay_time: 延时时间
 * @return 0 表示CAN发送成功，非零表示发送失败
 * @note   delay_time为0时不阻塞当前任务
 */
uint8_t DM_Save_Motor_Zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id, uint8_t delay_time)
{
	uint8_t data[8];
	uint8_t send_status;
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFE;
	
	send_status = canx_send_data(hcan, id, data, 8);
	if(delay_time > 0U)
	{
		osDelay(delay_time);
	}
	return send_status;
}

/**
 * @brief  失能电机模式
 * @param  hcan: CAN句柄
 * @param  motor_id: 电机ID
 * @param  mode_id: 模式ID
 * @param  delay_time: 延时时间
 * @return 0 表示CAN发送成功，非零表示发送失败
 * @note   delay_time为0时不阻塞当前任务
 */
uint8_t DM_Disable_Motor(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id, uint8_t delay_time)
{
	uint8_t data[8];
	uint8_t send_status;
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;
	
	send_status = canx_send_data(hcan, id, data, 8);
	if(delay_time > 0U)
	{
		osDelay(delay_time);
	}
	return send_status;
}

/**
 * @brief  DM电机控制
 * @param  hcan: CAN句柄
 * @param  motor: 电机信息结构体指针
 * @param  pos: 位置
 * @param  vel: 速度
 * @param  kp: KP系数
 * @param  kd: KD系数
 * @param  torq: 扭矩
 * @param  delay_time: 延时时间
 * @return 0 表示CAN发送成功，非零表示模式无效或发送失败
 * @note   delay_time为0时不阻塞当前任务
 */
uint8_t DM_Motor_Ctrl(hcan_t *hcan, volatile DM_Motor_Info_Typedef *motor, float pos, float vel, float kp, float kd, float torq, uint8_t delay_time)
{
	uint8_t send_status = 1U;

    switch (motor->Mode)
    {
        case Mit_mode:
        {
            uint8_t data_mit[8];
            uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

            uint16_t id = motor->ID_Set.TxIdentifier + MIT_MODE;

            pos_tmp = float_to_uint(pos, J4310_P_MIN, J4310_P_MAX, 16);
            vel_tmp = float_to_uint(vel, J4310_V_MIN, J4310_V_MAX, 12);
            kp_tmp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
            kd_tmp = float_to_uint(kd, KD_MIN, KD_MAX, 12);
            tor_tmp = float_to_uint(torq, J4310_T_MIN, J4310_T_MAX, 12);

            data_mit[0] = (pos_tmp >> 8);
            data_mit[1] = pos_tmp;
            data_mit[2] = (vel_tmp >> 4);
            data_mit[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
            data_mit[4] = kp_tmp;
            data_mit[5] = (kd_tmp >> 4);
            data_mit[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
            data_mit[7] = tor_tmp;

            send_status = canx_send_data(hcan, id, data_mit, 8);

            break;
        }
        case Pos_mode:
        {
            uint8_t data_pos_spd[8];
            uint8_t *pbuf, *vbuf;

            uint16_t id = motor->ID_Set.TxIdentifier + POS_MODE;

            pbuf = (uint8_t *)&pos;
            vbuf = (uint8_t *)&vel;

            data_pos_spd[0] = *pbuf;
            data_pos_spd[1] = *(pbuf + 1);
            data_pos_spd[2] = *(pbuf + 2);
            data_pos_spd[3] = *(pbuf + 3);

            data_pos_spd[4] = *vbuf;
            data_pos_spd[5] = *(vbuf + 1);
            data_pos_spd[6] = *(vbuf + 2);
            data_pos_spd[7] = *(vbuf + 3);

            send_status = canx_send_data(hcan, id, data_pos_spd, 8);

            break;
        }
        case Spd_mode:
        {
            uint8_t data_spd[4];
            uint8_t *vbuf;

            uint16_t id = motor->ID_Set.TxIdentifier + SPEED_MODE;

            vbuf = (uint8_t *)&vel;

            data_spd[0] = *vbuf;
            data_spd[1] = *(vbuf + 1);
            data_spd[2] = *(vbuf + 2);
            data_spd[3] = *(vbuf + 3);

            send_status = canx_send_data(hcan, id, data_spd, 4);

            break;
        }
    }
    if(delay_time > 0U)
    {
        osDelay(delay_time);
    }
    return send_status;
}



/* Private functions ------------------------------------------------------- */

/* Interrupt functions ----------------------------------------------------- */

/* ------------------------------------------------------------------------- */
