/**
 ******************************************************************************
 * @file    Remote_Control.c
 * @version V1.0.0
 * @date    2026.03.04
 * @brief   遥控器数据处理函数
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 
 ******************************************************************************
 */

/* Includes ---------------------------------------------------------------- */
#include "Remote_Control.h"
#include "FreeRTOS.h"
#include "bsp_dwt.h"
#include "stm32h7xx.h"
#include "string.h"
#include "task.h"

/* Defines ----------------------------------------------------------------- */

enum
{
    RC_SWITCH_EDGE_BIT_COUNT = 3U,
    RC_IW_EDGE_BIT_COUNT = 4U,
};

/* Global variable --------------------------------------------------------- */
Remote_Info_Typedef remote_ctrl={
	.online_cnt = 0xFAU,
	.rc_lost = true,
};
KeyBoard_Info_Typedef KeyBoard_Info;

/* Private variables ------------------------------------------------------- */
/**
 * @brief 全局发布序号及每种边沿最后一次出现的序号。
 * @note UART5 DMA ISR 是唯一写入者；任务通过临界区读取，避免 64 位撕裂。
 */
static volatile uint64_t g_remote_edge_sequence = 0U;
static volatile uint64_t
    g_remote_switch_edge_sequence[REMOTE_SWITCH_COUNT]
                                 [RC_SWITCH_EDGE_BIT_COUNT] = {{0U}};
static volatile uint64_t
    g_remote_iw_edge_sequence[RC_IW_EDGE_BIT_COUNT] = {0U};

/* Static Fun -------------------------------------------------------------- */
static void Key_Status_Update(KeyBoard_Info_Typedef *KeyInfo,bool KeyBoard_Status);

/* Functions --------------------------------------------------------------- */
/**
 * @brief  SBUS数据解析至遥控器
 * @param  sbus_buf: SBUS数据缓冲区指针
 * @param  remote_ctrl: 遥控器信息结构体指针
 * @return 无
 * @note   无
 */
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef  *remote_ctrl)
{
    if (sbus_buf == NULL || remote_ctrl == NULL) return;

    remote_ctrl->rc.ch[0] = (  sbus_buf[0]       | (sbus_buf[1] << 8 ) ) & 0x07ff;                        
    remote_ctrl->rc.ch[1] = ( (sbus_buf[1] >> 3) | (sbus_buf[2] << 5 ) ) & 0x07ff;                        
    remote_ctrl->rc.ch[2] = ( (sbus_buf[2] >> 6) | (sbus_buf[3] << 2 ) | (sbus_buf[4] << 10) ) & 0x07ff;  
    remote_ctrl->rc.ch[3] = ( (sbus_buf[4] >> 1) | (sbus_buf[5] << 7 ) ) & 0x07ff;                        
    remote_ctrl->rc.iw = (int16_t)(
        ((uint16_t)sbus_buf[16] | ((uint16_t)sbus_buf[17] << 8U)) &
        0x07FFU);

    remote_ctrl->rc.s[REMOTE_SWITCH_SW1] =
        ((sbus_buf[5] >> 4) & 0x000C) >> 2;
    remote_ctrl->rc.s[REMOTE_SWITCH_SW2] =
        ((sbus_buf[5] >> 4) & 0x0003);

    remote_ctrl->mouse.x = sbus_buf[6]  | (sbus_buf[7] << 8);               
    remote_ctrl->mouse.y = sbus_buf[8]  | (sbus_buf[9] << 8);               
    remote_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);              

    remote_ctrl->mouse.press_l = sbus_buf[12];                              
    remote_ctrl->mouse.press_r = sbus_buf[13];                              

    remote_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                

    remote_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.iw -= (int16_t)RC_CH_VALUE_OFFSET;
    
	remote_ctrl->online_cnt = 0xFAU;
	remote_ctrl->rc_lost = false;
}

/**
 * @brief  遥控器离线检测
 * @param  remote_ctrl: 遥控器信息结构体指针
 * @return 无
 * @note   无
 */
void Remote_Offline_Detect( Remote_Info_Typedef  *remote_ctrl)
{
    if(remote_ctrl->online_cnt <= 0x32U)
    {
        memset(remote_ctrl,0,sizeof(Remote_Info_Typedef));
        remote_ctrl->rc_lost = true;
    }
    else if(remote_ctrl->online_cnt > 0)
    {
        remote_ctrl->online_cnt--;
    }
}

/**
 * @brief  遥控器活动检测
 * @param  remote_ctrl: 遥控器信息结构体指针
 * @return 无
 * @note   无
 */
void Remote_Active_Detect( Remote_Info_Typedef  *remote_ctrl)
{
    for(uint8_t i = 0; i < RC_CHANNEL_COUNT; i++)
    {
        if(abs(remote_ctrl->rc.ch[i]) >= RC_CH_DEADZONE) {
            remote_ctrl->rc_active[i] = true;
            remote_ctrl->Last_Remote_Active_Time[i] = DWT_GetTimeMs();
        }
        else remote_ctrl->rc_active[i] = false;
    }
}

void Remote_EdgeCursor_Init(RC_EdgeCursor_t *cursor)
{
    if (cursor == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    cursor->seen_sequence = g_remote_edge_sequence;
    cursor->switch_edges[REMOTE_SWITCH_SW1] =
        RC_SWITCH_EDGE_NONE;
    cursor->switch_edges[REMOTE_SWITCH_SW2] =
        RC_SWITCH_EDGE_NONE;
    cursor->iw_edges = RC_IW_EDGE_NONE;
    taskEXIT_CRITICAL();
}

void Remote_EdgeCursor_Poll(RC_EdgeCursor_t *cursor)
{
    uint64_t published_sequence;
    uint8_t switch_index;
    uint8_t edge_index;

    if (cursor == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    cursor->switch_edges[REMOTE_SWITCH_SW1] =
        RC_SWITCH_EDGE_NONE;
    cursor->switch_edges[REMOTE_SWITCH_SW2] =
        RC_SWITCH_EDGE_NONE;
    cursor->iw_edges = RC_IW_EDGE_NONE;
    published_sequence = g_remote_edge_sequence;
    for (switch_index = 0U;
         switch_index < (uint8_t)REMOTE_SWITCH_COUNT;
         ++switch_index)
    {
        for (edge_index = 0U;
             edge_index < RC_SWITCH_EDGE_BIT_COUNT;
             ++edge_index)
        {
            if (g_remote_switch_edge_sequence[switch_index]
                                                     [edge_index] >
                cursor->seen_sequence)
            {
                cursor->switch_edges[switch_index] |=
                    (1UL << edge_index);
            }
        }
    }

    for (edge_index = 0U;
         edge_index < RC_IW_EDGE_BIT_COUNT;
         ++edge_index)
    {
        if (g_remote_iw_edge_sequence[edge_index] >
            cursor->seen_sequence)
        {
            cursor->iw_edges |= (1UL << edge_index);
        }
    }

    cursor->seen_sequence = published_sequence;
    taskEXIT_CRITICAL();
}

void Remote_Edge_PublishFromISR(uint32_t sw1_edges,
                                uint32_t sw2_edges,
                                uint32_t iw_edges)
{
    uint32_t switch_edges[REMOTE_SWITCH_COUNT];
    uint64_t published_sequence;
    uint8_t switch_index;
    uint8_t edge_index;

    switch_edges[REMOTE_SWITCH_SW1] =
        sw1_edges &
        ((1UL << RC_SWITCH_EDGE_BIT_COUNT) - 1UL);
    switch_edges[REMOTE_SWITCH_SW2] =
        sw2_edges &
        ((1UL << RC_SWITCH_EDGE_BIT_COUNT) - 1UL);
    iw_edges &= ((1UL << RC_IW_EDGE_BIT_COUNT) - 1UL);
    if ((switch_edges[REMOTE_SWITCH_SW1] == RC_SWITCH_EDGE_NONE) &&
        (switch_edges[REMOTE_SWITCH_SW2] == RC_SWITCH_EDGE_NONE) &&
        (iw_edges == RC_IW_EDGE_NONE))
    {
        return;
    }

    published_sequence = g_remote_edge_sequence + 1U;
    g_remote_edge_sequence = published_sequence;
    for (switch_index = 0U;
         switch_index < (uint8_t)REMOTE_SWITCH_COUNT;
         ++switch_index)
    {
        for (edge_index = 0U;
             edge_index < RC_SWITCH_EDGE_BIT_COUNT;
             ++edge_index)
        {
            if ((switch_edges[switch_index] &
                 (1UL << edge_index)) != 0U)
            {
                g_remote_switch_edge_sequence[switch_index]
                                                     [edge_index] =
                    published_sequence;
            }
        }
    }

    for (edge_index = 0U;
         edge_index < RC_IW_EDGE_BIT_COUNT;
         ++edge_index)
    {
        if ((iw_edges & (1UL << edge_index)) != 0U)
        {
            g_remote_iw_edge_sequence[edge_index] =
                published_sequence;
        }
    }
}

void Remote_Edge_Reset(void)
{
    uint8_t switch_index;
    uint8_t edge_index;

    taskENTER_CRITICAL();
    for (switch_index = 0U;
         switch_index < (uint8_t)REMOTE_SWITCH_COUNT;
         ++switch_index)
    {
        for (edge_index = 0U;
             edge_index < RC_SWITCH_EDGE_BIT_COUNT;
             ++edge_index)
        {
            g_remote_switch_edge_sequence[switch_index]
                                                 [edge_index] = 0U;
        }
    }

    for (edge_index = 0U;
         edge_index < RC_IW_EDGE_BIT_COUNT;
         ++edge_index)
    {
        g_remote_iw_edge_sequence[edge_index] = 0U;
    }
    taskEXIT_CRITICAL();
}

/* Private functions ------------------------------------------------------- */

/* Interrupt functions ----------------------------------------------------- */

/* ------------------------------------------------------------------------- */
