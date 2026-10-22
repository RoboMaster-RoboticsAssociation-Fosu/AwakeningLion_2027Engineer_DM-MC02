/**
 * @file input_task.c
 * @brief 遥控输入 → 本板许可快照；USART10 收发始终由本任务调度。
 */
#include "input_task.h"
#include "board_link_system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx_hal.h"
#include "chassis_task.h"

static InputTask_t g_input_task = {.locked = true};

void InputTask_Run(void)
{
    TickType_t wake_time = xTaskGetTickCount();

    for (;;)
    {
        uint8_t sw2;
        bool ready;

        BoardLinkSystem_Process(HAL_GetTick());
        ready = g_board_link_system.remote_valid && !remote_ctrl.rc_lost;
        sw2 = remote_ctrl.rc.s[REMOTE_SWITCH_SW2];
        if (ready)
        {
            if (g_input_task.switch_history_valid &&
                (sw2 != g_input_task.previous_sw2))
            {
                if (sw2 == RC_SW_UP) { g_input_task.locked = false; }
                if (sw2 == RC_SW_DOWN) { g_input_task.locked = true; }
            }
            g_input_task.previous_sw2 = sw2;
            g_input_task.switch_history_valid = true;
        }
        /* 沿用已有锁定/rc_lost 门控；板间 50 ms 超时和重连策略留待后续确认。 */
        ChassisTask_Notify(ready && !g_input_task.locked);
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(1U));
    }
}
