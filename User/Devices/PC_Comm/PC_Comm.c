/**
 ******************************************************************************
 * @file    PC_Comm.c
 * @version V1.0.0
 * @date    2026.04.11
 * @brief   上位机通信驱动实现
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 模板示例 未运用到实际工程里面
 ******************************************************************************
 */

/* Includes ---------------------------------------------------------------- */
#include "PC_Comm.h"
#include <string.h>

/* Defines ----------------------------------------------------------------- */

/* Global variable --------------------------------------------------------- */
PC_Info_Typedef PC_RxInfo = {
    .online_cnt = 0xFAU,
    .pc_lost = true,
};

__attribute__((section (".AXI_SRAM"))) uint8_t PC_MultiRx_Buf[PC_RX_DATA_LEN];

/* Static Fun -------------------------------------------------------------- */

/* Functions --------------------------------------------------------------- */
/**
 * @brief	上位机数据解包
 * @param	buff: 接收数据缓冲区指针
 * @param	PC_Info: 上位机信息结构体指针
 * @return	无
 * @note	无
 */
void PC_Info_Update(uint8_t *buff,PC_Info_Typedef *PC_Info)
{
    // PC_Info->HEAD = buff[0];
    // if(PC_Info->HEAD == PC_RxHEAD)
    // {
    //     PC_Info->Data_Len = buff[1];
    //     if (PC_Info->Data_Len == PC_RX_DATA_LEN)
    //     {
    //         PC_Info->Vx.uval = buff[2]  | buff[3] << 8 | buff[4] << 16 | buff[5] << 24;
    //         PC_Info->Vy.uval = buff[6]  | buff[7] << 8 | buff[8] << 16 | buff[9] << 24;
    //         PC_Info->Vz.uval = buff[10] | buff[11] << 8 | buff[12] << 16 | buff[13] << 24;
    //         PC_Info->Reserved[0] = buff[14];
    //         PC_Info->Reserved[1] = buff[15];
    //         PC_Info->Reserved[2] = buff[16];
    //         PC_Info->Reserved[3] = buff[17];
	// 		   PC_Info->Checksum = buff[18];
    //         PC_Info->pc_lost = false;
    //         PC_Info->online_cnt = 0xFAU;
    //     }
    // }
}

/**
 * @brief	上位机数据上传
 * @param	无
 * @return	无
 * @note	无
 */
void PC_Info_Upload()
{

}

/**
 * @brief	上位机离线检测
 * @param	PC_Info: 上位机信息结构体指针
 * @return	无
 * @note	在线计数值低于阈值时判定为离线
 */
void PC_Offline_Detect(PC_Info_Typedef *PC_Info)
{
    if(PC_Info->online_cnt <= 0x32U)
    {
        memset(PC_Info,0,sizeof(PC_Info_Typedef));
        PC_Info->pc_lost = true;
    }
    else if(PC_Info->online_cnt > 0)
    {
        PC_Info->online_cnt--;
    }
}

/* Private functions ------------------------------------------------------- */

/* Interrupt functions ----------------------------------------------------- */

/* ------------------------------------------------------------------------- */