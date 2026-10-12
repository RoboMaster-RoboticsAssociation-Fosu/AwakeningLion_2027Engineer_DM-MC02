/**
 ******************************************************************************
 * @file    bsp_dwt.c
 * @version V2.0.0
 * @date    2026.07.20
 * @brief   Cortex-M DWT 单调时间与精确延时实现
 * @encoding UTF-8
 ******************************************************************************
 */

#include "bsp_dwt.h"

#include "stm32h7xx.h"
#include <limits.h>

/* Private variables ------------------------------------------------------- */
static uint32_t dwt_cpu_frequency_hz;
static volatile uint32_t dwt_last_cycle;
static volatile uint64_t dwt_accumulated_cycles;
static volatile bool dwt_initialized;

/* Private function prototypes --------------------------------------------- */
static uint64_t dwt_capture_cycles_locked(void);
static void dwt_delay_cycles(uint64_t delay_cycles);

bool DWT_Init(uint32_t cpu_frequency_hz)
{
    uint32_t primask;

    if (cpu_frequency_hz == 0U)
    {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    __DMB();

    dwt_initialized = false;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    dwt_cpu_frequency_hz = cpu_frequency_hz;
    dwt_last_cycle = DWT->CYCCNT;
    dwt_accumulated_cycles = 0ULL;
    dwt_initialized =
        ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U);
    if (!dwt_initialized)
    {
        dwt_cpu_frequency_hz = 0U;
    }

    __DMB();
    if (primask == 0U)
    {
        __enable_irq();
    }

    return dwt_initialized;
}

void DWT_Update(void)
{
    uint32_t primask;

    if (!dwt_initialized)
    {
        return;
    }

    /*
     * 本层需要在调度器启动前、任务和 ISR 中工作，因此使用最短的 PRIMASK
     * 临界区原子更新 64 位累计值，不依赖 FreeRTOS 调度器状态。
     */
    primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    (void)dwt_capture_cycles_locked();
    __DMB();
    if (primask == 0U)
    {
        __enable_irq();
    }
}

uint64_t DWT_GetCycleCount64(void)
{
    uint32_t primask;
    uint64_t cycles;

    if (!dwt_initialized)
    {
        return 0ULL;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    cycles = dwt_capture_cycles_locked();
    __DMB();
    if (primask == 0U)
    {
        __enable_irq();
    }

    return cycles;
}

uint64_t DWT_GetTimeUs(void)
{
    const uint64_t cycles = DWT_GetCycleCount64();
    uint64_t whole_seconds;
    uint64_t remaining_cycles;

    if (!dwt_initialized)
    {
        return 0ULL;
    }

    /* 先除后乘，避免 cycles * 1000000ULL 在长时间运行后溢出。 */
    whole_seconds = cycles / (uint64_t)dwt_cpu_frequency_hz;
    remaining_cycles = cycles % (uint64_t)dwt_cpu_frequency_hz;
    return whole_seconds * 1000000ULL +
           (remaining_cycles * 1000000ULL) /
               (uint64_t)dwt_cpu_frequency_hz;
}

uint32_t DWT_GetTimeMs(void)
{
    return (uint32_t)(DWT_GetTimeUs() / 1000ULL);
}

float DWT_GetDeltaT(uint32_t *last_cycle)
{
    uint32_t now_cycle;
    uint32_t elapsed_cycles;

    if ((last_cycle == NULL) || !dwt_initialized)
    {
        return 0.0F;
    }

    now_cycle = DWT->CYCCNT;
    elapsed_cycles = (uint32_t)(now_cycle - *last_cycle);
    *last_cycle = now_cycle;
    return (float)elapsed_cycles / (float)dwt_cpu_frequency_hz;
}

double DWT_GetDeltaT64(uint32_t *last_cycle)
{
    uint32_t now_cycle;
    uint32_t elapsed_cycles;

    if ((last_cycle == NULL) || !dwt_initialized)
    {
        return 0.0;
    }

    now_cycle = DWT->CYCCNT;
    elapsed_cycles = (uint32_t)(now_cycle - *last_cycle);
    *last_cycle = now_cycle;
    return (double)elapsed_cycles / (double)dwt_cpu_frequency_hz;
}

void DWT_Delay(float delay_s)
{
    uint64_t delay_cycles;

    if ((delay_s <= 0.0F) || !dwt_initialized)
    {
        return;
    }

    delay_cycles = (uint64_t)((double)delay_s *
                              (double)dwt_cpu_frequency_hz);
    dwt_delay_cycles(delay_cycles);
}

void DWT_DelayUs(uint32_t delay_us)
{
    uint64_t delay_cycles;

    if ((delay_us == 0U) || !dwt_initialized)
    {
        return;
    }

    delay_cycles =
        ((uint64_t)(dwt_cpu_frequency_hz / 1000000U) * delay_us) +
        (((uint64_t)(dwt_cpu_frequency_hz % 1000000U) * delay_us) /
         1000000ULL);
    dwt_delay_cycles(delay_cycles);
}

static uint64_t dwt_capture_cycles_locked(void)
{
    const uint32_t now_cycle = DWT->CYCCNT;
    const uint32_t elapsed_cycles =
        (uint32_t)(now_cycle - dwt_last_cycle);

    dwt_accumulated_cycles += (uint64_t)elapsed_cycles;
    dwt_last_cycle = now_cycle;
    return dwt_accumulated_cycles;
}

static void dwt_delay_cycles(uint64_t delay_cycles)
{
    while (delay_cycles > 0ULL)
    {
        const uint32_t chunk =
            (delay_cycles > (uint64_t)(UINT32_MAX / 2U))
                ? (UINT32_MAX / 2U)
                : (uint32_t)delay_cycles;
        const uint32_t start_cycle = DWT->CYCCNT;

        while ((uint32_t)(DWT->CYCCNT - start_cycle) < chunk)
        {
        }
        delay_cycles -= (uint64_t)chunk;
    }
}
