/**
 * @file test_rc_feedback_time.c
 * @brief Host regression for the actual RC offline check and edge-reset bodies.
 * @note run_rc_time_tests.py includes the production bodies verbatim; UART/DMA
 *       startup is built by the native firmware compiler, not emulated here.
 */
#include "rc_system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_dwt.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static RcSystem_t g_rc_system;
static uint32_t test_time_ms;
static uint32_t critical_depth;
static bool next_ms_interrupt_pending;

/* BEGIN host-only time/interrupt boundary. Deliver one valid frame immediately
 * before the outermost critical section; never link this boundary into firmware.
 * Remove it together with these regression tests if they are retired.
 */
uint32_t DWT_GetTimeMs(void)
{
    return test_time_ms;
}

void vPortEnterCritical(void)
{
    if ((critical_depth == 0U) && next_ms_interrupt_pending)
    {
        next_ms_interrupt_pending = false;
        ++test_time_ms;
        g_rc_system.receive.last_valid_time_ms = test_time_ms;
        g_rc_system.receive.has_valid_frame = 1U;
        g_rc_system.receive.ready = 1U;
        remote_ctrl.rc.ch[0] = 150;
        remote_ctrl.rc_lost = false;
    }
    ++critical_depth;
}

void vPortExitCritical(void)
{
    assert(critical_depth != 0U);
    --critical_depth;
}
/* END host-only time/interrupt boundary. */

static void RC_ResetEdgeState(void);
#include "rc_time_production.inc"

/** @brief Give each test an already valid, idle receiver and a known input snapshot. */
static void reset_fixture(uint32_t now_ms, uint32_t received_ms)
{
    memset(&g_rc_system, 0, sizeof(g_rc_system));
    memset(&remote_ctrl, 0, sizeof(remote_ctrl));
    test_time_ms = now_ms;
    critical_depth = 0U;
    next_ms_interrupt_pending = false;
    g_rc_system.receive.has_valid_frame = 1U;
    g_rc_system.receive.ready = 1U;
    g_rc_system.receive.last_valid_time_ms = received_ms;
    g_rc_system.edge.valid = 1U;
    remote_ctrl.rc.ch[0] = 150;
    Remote_Edge_Reset();
}

/** @brief The ISR may publish at now+1 before the task enters its critical section. */
static void test_interrupt_after_task_time(void)
{
    reset_fixture(100U, 100U);
    next_ms_interrupt_pending = true;
    const uint32_t checked_time_ms = RC_UpdateOfflineState();
    assert(!next_ms_interrupt_pending);
    assert(checked_time_ms == 101U);
    assert(!remote_ctrl.rc_lost);
    assert(remote_ctrl.rc.ch[0] == 150);
    assert(g_rc_system.receive.ready == 1U);
    assert(g_rc_system.edge.valid == 1U);
    assert(critical_depth == 0U);

    /* Reproduce the same ordering at the initial 0 -> 1 ms boundary. */
    reset_fixture(0U, 0U);
    g_rc_system.receive.has_valid_frame = 0U;
    next_ms_interrupt_pending = true;
    (void)RC_UpdateOfflineState();
    assert(!remote_ctrl.rc_lost && g_rc_system.receive.ready == 1U);
}

/** @brief Preserve the real offline threshold and initial no-frame behavior. */
static void test_offline_boundary(void)
{
    reset_fixture(300U, 100U);
    (void)RC_UpdateOfflineState();
    assert(!remote_ctrl.rc_lost);
    assert(remote_ctrl.rc.ch[0] == 150);
    test_time_ms = 301U;
    (void)RC_UpdateOfflineState();
    assert(remote_ctrl.rc_lost);
    assert(remote_ctrl.rc.ch[0] == 0);
    assert(g_rc_system.receive.ready == 0U && g_rc_system.edge.valid == 0U);
    assert(critical_depth == 0U);

    reset_fixture(0U, 0U);
    g_rc_system.receive.has_valid_frame = 0U;
    (void)RC_UpdateOfflineState();
    assert(remote_ctrl.rc_lost && g_rc_system.receive.ready == 0U);
}

/** @brief Unsigned subtraction still measures elapsed time across uint32 rollover. */
static void test_counter_wrap(void)
{
    reset_fixture(20U, UINT32_MAX - 10U);
    (void)RC_UpdateOfflineState();
    assert(!remote_ctrl.rc_lost); /* 31 ms elapsed. */
    test_time_ms = 201U;
    (void)RC_UpdateOfflineState();
    assert(remote_ctrl.rc_lost); /* 212 ms elapsed. */
}

int main(void)
{
    test_interrupt_after_task_time();
    test_offline_boundary();
    test_counter_wrap();
    puts("PASS: RC next-ms ISR, startup, true timeout, threshold and uint32 wrap");
    return 0;
}
