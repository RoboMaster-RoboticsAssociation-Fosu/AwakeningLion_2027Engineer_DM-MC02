/**
 * @file test_motor_control.c
 * @brief Offline behavior checks for the production chassis task and CAN paths.
 * @note No test implementation is linked into firmware. Static task steps are
 *       included here to exercise their real lifecycle without exporting a test API.
 */
#include "../User/APP/chassis_task.c"
#include "Remote_Control.h"
#include <assert.h>
#include <float.h>
#include <stdio.h>
#include <string.h>

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;
Remote_Info_t remote_ctrl;

typedef struct
{
    FDCAN_HandleTypeDef *bus;
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[8];
    HAL_StatusTypeDef result;
} CapturedFrame_t;

static ChassisTask_t initial_chassis;
static CapturedFrame_t frames[2048];
static uint32_t frame_count;
static uint32_t test_time_ms;
static uint32_t critical_depth;
static FDCAN_HandleTypeDef *failed_bus;
static uint32_t failed_id;
static uint32_t failures_left;
static FDCAN_HandleTypeDef *rx_bus;
static uint32_t rx_fifo;
static bool rx_pending;
static FDCAN_RxHeaderTypeDef rx_header;
static uint8_t rx_data[8];
static void (*before_critical_interrupt)(void);
static uint32_t interrupt_motor;
static uint8_t interrupt_leg_state;

/* BEGIN host-only hardware boundary: deterministic time, interrupt exclusion,
 * CAN receive queues and transmitted wire frames. Scope is this test executable;
 * never compile it into firmware. Remove with this host test if it is retired.
 */
uint32_t DWT_GetTimeMs(void)
{
    return test_time_ms;
}

void vPortEnterCritical(void)
{
    /* Deliver one pending ISR immediately before the outermost mask is raised. */
    if ((critical_depth == 0U) && (before_critical_interrupt != NULL))
    {
        void (*interrupt)(void) = before_critical_interrupt;
        before_critical_interrupt = NULL;
        interrupt();
    }
    ++critical_depth;
}

void vPortExitCritical(void)
{
    assert(critical_depth > 0U);
    --critical_depth;
}

osStatus_t osDelay(uint32_t ticks)
{
    test_time_ms += ticks;
    return osOK;
}

HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *bus,
                                               const FDCAN_TxHeaderTypeDef *header,
                                               const uint8_t *data)
{
    CapturedFrame_t *frame;
    assert(frame_count < (uint32_t)(sizeof(frames) / sizeof(frames[0])));
    assert(header->DataLength == FDCAN_DLC_BYTES_8);
    assert(header->IdType == FDCAN_STANDARD_ID);
    assert(header->FDFormat == FDCAN_CLASSIC_CAN);
    assert(header->TxFrameType == FDCAN_DATA_FRAME);
    assert(header->BitRateSwitch == FDCAN_BRS_OFF);
    frame = &frames[frame_count++];
    frame->bus = bus;
    frame->header = *header;
    memcpy(frame->data, data, sizeof(frame->data));
    frame->result = HAL_OK;
    if ((bus == failed_bus) && (header->Identifier == failed_id) &&
        (failures_left != 0U))
    {
        --failures_left;
        frame->result = HAL_ERROR;
    }
    return frame->result;
}

uint32_t HAL_FDCAN_GetRxFifoFillLevel(const FDCAN_HandleTypeDef *bus, uint32_t fifo)
{
    return rx_pending && (bus == rx_bus) && (fifo == rx_fifo) ? 1U : 0U;
}

HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *bus, uint32_t fifo,
                                        FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{
    assert(rx_pending && (bus == rx_bus) && (fifo == rx_fifo));
    *header = rx_header;
    memcpy(data, rx_data, sizeof(rx_data));
    rx_pending = false;
    return HAL_OK;
}
/* END host-only hardware boundary. */

/** @brief Reset the real static instances, retaining their production initializers. */
static void reset_fixture(void)
{
    g_chassis = initial_chassis;
    memset(&g_chassis_test, 0, sizeof(g_chassis_test));
    memset(&remote_ctrl, 0, sizeof(remote_ctrl));
    frame_count = 0U;
    test_time_ms = 100U;
    failed_bus = NULL;
    failures_left = 0U;
    rx_pending = false;
    critical_depth = 0U;
    before_critical_interrupt = NULL;
}

/** @brief One production control tick with the same externally supplied permission. */
static void step(bool enabled)
{
    chassis_task_step(enabled);
    assert(critical_depth == 0U);
    ++test_time_ms;
}

/** @brief Submit through the actual FIFO callbacks, including their frame filters. */
static void receive(FDCAN_HandleTypeDef *bus, uint32_t id, const uint8_t data[8],
                    uint32_t id_type, uint32_t format, uint32_t length)
{
    assert(!rx_pending);
    rx_bus = bus;
    rx_fifo = bus == &hfdcan2 ? FDCAN_RX_FIFO1 : FDCAN_RX_FIFO0;
    memset(&rx_header, 0, sizeof(rx_header));
    rx_header.Identifier = id;
    rx_header.IdType = id_type;
    rx_header.RxFrameType = FDCAN_DATA_FRAME;
    rx_header.DataLength = length;
    rx_header.FDFormat = format;
    memcpy(rx_data, data, sizeof(rx_data));
    rx_pending = true;
    if (bus == &hfdcan2)
    {
        HAL_FDCAN_RxFifo1Callback(bus, FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
    }
    else
    {
        HAL_FDCAN_RxFifo0Callback(bus, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    }
    assert(!rx_pending);
}

/** @brief Encode an independent DJI feedback vector; speed is pre-reduction rpm. */
static void dji_feedback(FDCAN_HandleTypeDef *bus, uint32_t slot, int16_t speed)
{
    uint8_t data[8] = {0U};
    data[2] = (uint8_t)((uint16_t)speed >> 8U);
    data[3] = (uint8_t)speed;
    receive(bus, 0x201U + slot, data, FDCAN_STANDARD_ID,
            FDCAN_CLASSIC_CAN, FDCAN_DLC_BYTES_8);
}

/** @brief Encode a leg feedback vector independent of production quantization code. */
static void leg_feedback(uint32_t leg, uint8_t state, uint16_t position, uint16_t velocity)
{
    uint8_t data[8] = {0U};
    data[0] = (uint8_t)((uint32_t)state * 16U + leg + 1U);
    data[1] = (uint8_t)(position >> 8U);
    data[2] = (uint8_t)position;
    data[3] = (uint8_t)(velocity >> 4U);
    data[4] = (uint8_t)(((uint32_t)velocity & 15U) * 16U + 7U);
    data[5] = 0xFFU;
    receive(&hfdcan2, 0x11U + leg, data, FDCAN_STANDARD_ID,
            FDCAN_CLASSIC_CAN, FDCAN_DLC_BYTES_8);
}

/** @brief Inject a real CAN receive callback one millisecond after task-start time. */
static void next_ms_feedback_interrupt(void)
{
    assert(critical_depth == 0U);
    ++test_time_ms;
    if (interrupt_motor <= (uint32_t)CHASSIS_TEST_WHEEL_RIGHT_BACK)
    {
        dji_feedback(&hfdcan1, interrupt_motor - 1U, 0);
    }
    else if (interrupt_motor < (uint32_t)CHASSIS_TEST_LEG_RIGHT_FRONT)
    {
        dji_feedback(&hfdcan3,
                     interrupt_motor - (uint32_t)CHASSIS_TEST_TRACK_LEFT, 0);
    }
    else
    {
        leg_feedback(interrupt_motor - (uint32_t)CHASSIS_TEST_LEG_RIGHT_FRONT,
                     interrupt_leg_state, 35000U, 2500U);
    }
}

/** @brief Inspect a successfully submitted frame, not an unsuccessful attempt. */
static const CapturedFrame_t *last_frame(FDCAN_HandleTypeDef *bus, uint32_t id)
{
    uint32_t index = frame_count;
    while (index > 0U)
    {
        --index;
        if ((frames[index].bus == bus) && (frames[index].header.Identifier == id) &&
            (frames[index].result == HAL_OK))
        {
            return &frames[index];
        }
    }
    assert(0 && "expected transmitted frame is missing");
    return NULL;
}

/** @brief All unused slots and the other DJI bus must carry zero current. */
static void check_dji_currents(FDCAN_HandleTypeDef *selected_bus,
                               uint32_t selected_slot, int16_t expected)
{
    FDCAN_HandleTypeDef *buses[2] = {&hfdcan1, &hfdcan3};
    uint32_t bus;
    uint32_t slot;
    for (bus = 0U; bus < 2U; ++bus)
    {
        const uint8_t *data = last_frame(buses[bus], 0x200U)->data;
        for (slot = 0U; slot < 4U; ++slot)
        {
            const uint16_t value = (uint16_t)((uint32_t)data[slot * 2U] * 256U +
                                             data[slot * 2U + 1U]);
            const int16_t wanted = ((buses[bus] == selected_bus) &&
                                     (slot == selected_slot)) ? expected : 0;
            assert(value == (uint16_t)wanted);
        }
    }
}

/** @brief Select while stopped, allowing the production selection edge to clear run. */
static void select_motor(uint32_t motor)
{
    g_chassis.command.motor = motor;
    g_chassis.command.run = 0U;
    step(true);
    assert(g_chassis.command.run == 0U);
}

/** @brief Complete an actual request/fresh-feedback enable handshake for one leg. */
static void start_leg(uint32_t leg)
{
    select_motor((uint32_t)CHASSIS_TEST_LEG_RIGHT_FRONT + leg);
    leg_feedback(leg, 0U, 35000U, 2500U);
    g_chassis.command.position_deg = -90.0F; /* Must not be replayed at enable. */
    g_chassis.command.run = 1U;
    step(true);
    assert(g_chassis.test_status == CHASSIS_TEST_ENABLING);
    assert(last_frame(&hfdcan2, leg + 1U)->data[7] == 0xFCU);
    step(true); /* No new feedback: queueing enable alone is not confirmation. */
    assert(g_chassis.test_status == CHASSIS_TEST_ENABLING);
    leg_feedback(leg, 1U, 35000U, 2500U);
    step(true);
    assert(g_chassis.test_status == CHASSIS_TEST_RUNNING);
    assert(fabsf(g_chassis.command.position_deg -
                 g_chassis.leg_motor[leg].Data.pos * 57.2957795F) < 0.0001F);
}

/** @brief The default firmware cannot enter wheel motion or enable a leg. */
static void test_boot_and_management_rate(void)
{
    uint32_t tick;
    uint32_t management = 0U;
    reset_fixture();
    assert(g_chassis.command.motor == CHASSIS_TEST_MOTOR_NONE);
    assert(g_chassis.command.run == 0U);
    for (tick = 0U; tick < 100U; ++tick)
    {
        step(false);
    }
    for (tick = 0U; tick < frame_count; ++tick)
    {
        if (frames[tick].bus == &hfdcan2)
        {
            assert(frames[tick].data[7] == 0xFDU);
            ++management;
        }
    }
    assert(management == 20U); /* Four disables per 20 ms, not four per 1 ms. */
    check_dji_currents(NULL, 0U, 0);
}

/** @brief Same DJI IDs on CAN1 and CAN3 stay distinct; invalid frames are ignored. */
static void test_can_dispatch(void)
{
    uint8_t data[8] = {0x11U, 0x80U, 0U, 0x7FU, 0xF7U, 0xFFU, 25U, 26U};
    uint32_t leg;
    reset_fixture();
    dji_feedback(&hfdcan1, 0U, 123);
    dji_feedback(&hfdcan3, 0U, -456);
    assert(g_chassis.wheel_motor[0].Data.Velocity == 123);
    assert(g_chassis.track_motor[0].Data.Velocity == -456);
    assert(!g_chassis.wheel_motor[1].feedback_valid);
    for (leg = 0U; leg < 4U; ++leg)
    {
        leg_feedback(leg, 0U, 32768U, 2047U);
        assert(g_chassis.leg_motor[leg].feedback_sequence == 1U);
    }
    receive(&hfdcan2, 0x12U, data, FDCAN_STANDARD_ID, FDCAN_CLASSIC_CAN, FDCAN_DLC_BYTES_8);
    receive(&hfdcan2, 0x11U, data, FDCAN_EXTENDED_ID, FDCAN_CLASSIC_CAN, FDCAN_DLC_BYTES_8);
    receive(&hfdcan2, 0x11U, data, FDCAN_STANDARD_ID, FDCAN_FD_CAN, FDCAN_DLC_BYTES_8);
    receive(&hfdcan2, 0x11U, data, FDCAN_STANDARD_ID, FDCAN_CLASSIC_CAN, FDCAN_DLC_BYTES_6);
    ChassisTask_OnCan2Rx(0x11U, NULL, FDCAN_DLC_BYTES_8);
    ChassisTask_OnCan3Rx(0x201U, NULL, FDCAN_DLC_BYTES_8);
    assert(g_chassis.leg_motor[0].feedback_sequence == 1U);
    assert(g_chassis.leg_motor[1].feedback_sequence == 1U);
}

/** @brief All six speed loops work with only the selected feedback present. */
static void test_all_speed_motors(void)
{
    uint32_t motor;
    for (motor = 1U; motor <= 6U; ++motor)
    {
        FDCAN_HandleTypeDef *bus = motor <= 4U ? &hfdcan1 : &hfdcan3;
        const uint32_t slot = motor <= 4U ? motor - 1U : motor - 5U;
        reset_fixture();
        select_motor(motor);
        dji_feedback(bus, slot, 20);
        g_chassis.command.speed_rpm = 100.0F;
        g_chassis.command.run = 1U;
        step(true);
        assert(g_chassis.test_status == CHASSIS_TEST_RUNNING);
        check_dji_currents(bus, slot, 800);
        g_chassis.command.speed_rpm = -100.0F;
        step(true);
        check_dji_currents(bus, slot, -1200);
        g_chassis.command.speed_rpm = 100000.0F;
        step(true);
        check_dji_currents(bus, slot, DJI_C620_CURRENT_COMMAND_MAX);
        step(false);
        assert(g_chassis.command.run == 0U);
        check_dji_currents(NULL, 0U, 0);
        step(true);
        check_dji_currents(NULL, 0U, 0); /* Remote recovery cannot replay run. */
    }
}

/** @brief Inspect H7 PD, unit conversion, pure MIT torque fields and lack of angle cap. */
static void test_all_leg_pd(void)
{
    uint32_t leg;
    for (leg = 0U; leg < 4U; ++leg)
    {
        const CapturedFrame_t *frame;
        float expected;
        float wire_torque;
        uint32_t torque_code;
        reset_fixture();
        start_leg(leg);
        g_chassis.command.position_deg += 10.0F;
        step(true);
        expected = 5.0F * 10.0F * 0.0174532925F -
                   0.2F * g_chassis.leg_motor[leg].Data.vel;
        assert(fabsf(g_chassis.leg_submitted_torque_nm[leg] - expected) < 0.00001F);
        frame = last_frame(&hfdcan2, leg + 1U);
        assert(frame->data[0] == 0x7FU && frame->data[1] == 0xFFU);
        assert(frame->data[2] == 0x7FU && frame->data[3] == 0xF0U);
        assert(frame->data[4] == 0U && frame->data[5] == 0U);
        assert((frame->data[6] >> 4U) == 0U); /* MIT kp=kd=0. */
        torque_code = (uint32_t)(frame->data[6] & 15U) * 256U + frame->data[7];
        wire_torque = (float)torque_code * 56.0F / 4095.0F - 28.0F;
        assert(fabsf(wire_torque - expected) <= 56.0F / 4095.0F);
        check_dji_currents(NULL, 0U, 0);
        g_chassis.command.position_deg = 10000.0F;
        step(true);
        assert(g_chassis.command.position_deg == 10000.0F);
        assert(g_chassis.leg_submitted_torque_nm[leg] == 28.0F);
        g_chassis.command.position_deg = -10000.0F;
        step(true);
        assert(g_chassis.command.position_deg == -10000.0F);
        assert(g_chassis.leg_submitted_torque_nm[leg] == -28.0F);
    }
}

/** @brief A stopped leg must acknowledge disable before another motor can start. */
static void test_stop_and_selection(void)
{
    reset_fixture();
    start_leg(0U);
    g_chassis.command.run = 0U;
    step(true);
    assert(g_chassis.test_status == CHASSIS_TEST_STOPPING);
    assert(g_chassis.active_test_motor == CHASSIS_TEST_LEG_RIGHT_FRONT);
    assert(last_frame(&hfdcan2, 1U)->data[7] == 0xFDU);
    g_chassis.command.motor = CHASSIS_TEST_TRACK_LEFT;
    step(true);
    assert(g_chassis.command.run == 0U);
    g_chassis.command.run = 1U;
    g_chassis.command.speed_rpm = 100.0F;
    dji_feedback(&hfdcan3, 0U, 0);
    step(true);
    check_dji_currents(NULL, 0U, 0);
    /* Old/stale state cannot acknowledge the just-sent disable request. */
    g_chassis.leg_motor[0].Data.state = DM_MOTOR_STATE_DISABLED;
    step(true);
    check_dji_currents(NULL, 0U, 0);
    leg_feedback(0U, 0U, 35000U, 2047U);
    step(true);
    assert(g_chassis.active_test_motor == CHASSIS_TEST_TRACK_LEFT);
    check_dji_currents(&hfdcan3, 0U, 1000);
}

/** @brief Feedback timeout latches run off, handles tick wrap, and ignores other motors. */
static void test_feedback_and_wrap(void)
{
    reset_fixture();
    select_motor(CHASSIS_TEST_WHEEL_RIGHT_FRONT);
    g_chassis.command.run = 1U;
    step(true);
    assert(g_chassis.test_status == CHASSIS_TEST_WAIT_FEEDBACK);
    test_time_ms = UINT32_MAX - 10U;
    dji_feedback(&hfdcan1, 0U, 0);
    test_time_ms = 20U;
    g_chassis.command.speed_rpm = 100.0F;
    step(true);
    assert(g_chassis.test_status == CHASSIS_TEST_RUNNING);
    test_time_ms = 60U;
    step(true);
    assert(g_chassis.test_status == CHASSIS_TEST_FEEDBACK_LOST);
    assert(g_chassis.command.run == 0U);
    check_dji_currents(NULL, 0U, 0);
    dji_feedback(&hfdcan1, 0U, 0);
    step(true);
    check_dji_currents(NULL, 0U, 0);

    reset_fixture();
    start_leg(0U);
    test_time_ms += 51U;
    step(true);
    assert(g_chassis.test_status == CHASSIS_TEST_FEEDBACK_LOST);
    assert(g_chassis.command.run == 0U && g_chassis_test.stopping);
    assert(last_frame(&hfdcan2, 1U)->data[7] == 0xFDU);
}

/** @brief All ten fresh-feedback checks survive an ISR crossing the next ms tick. */
static void test_feedback_arriving_after_cycle_time(void)
{
    uint32_t motor;
    for (motor = 1U; motor < (uint32_t)CHASSIS_TEST_MOTOR_COUNT; ++motor)
    {
        reset_fixture();
        if (motor < (uint32_t)CHASSIS_TEST_LEG_RIGHT_FRONT)
        {
            select_motor(motor);
            interrupt_motor = motor;
            next_ms_feedback_interrupt();
            g_chassis.command.speed_rpm = 100.0F;
            g_chassis.command.run = 1U;
            step(true);
        }
        else
        {
            start_leg(motor - (uint32_t)CHASSIS_TEST_LEG_RIGHT_FRONT);
        }
        assert(g_chassis.test_status == CHASSIS_TEST_RUNNING);
        interrupt_motor = motor;
        interrupt_leg_state = 1U;
        before_critical_interrupt = next_ms_feedback_interrupt;
        step(true);
        assert(before_critical_interrupt == NULL);
        assert(g_chassis.test_status == CHASSIS_TEST_RUNNING);
        assert(g_chassis.command.run == 1U);
    }

    /* The same timing window must not postpone a fresh disable acknowledgement. */
    reset_fixture();
    start_leg(0U);
    g_chassis.command.run = 0U;
    step(true);
    assert(g_chassis.test_status == CHASSIS_TEST_STOPPING);
    interrupt_motor = CHASSIS_TEST_LEG_RIGHT_FRONT;
    interrupt_leg_state = 0U;
    before_critical_interrupt = next_ms_feedback_interrupt;
    step(true);
    assert(g_chassis.active_test_motor == CHASSIS_TEST_MOTOR_NONE);
}

/** @brief Invalid debugger values and real motor fault codes cannot reach PD output. */
static void test_invalid_and_motor_fault(void)
{
    reset_fixture();
    select_motor(1U);
    dji_feedback(&hfdcan1, 0U, 0);
    g_chassis.command.run = 1U;
    g_chassis.command.speed_rpm = NAN;
    step(true);
    assert(g_chassis.test_status == CHASSIS_TEST_INVALID_COMMAND);
    check_dji_currents(NULL, 0U, 0);
    g_chassis.command.motor = 99U;
    step(true);
    g_chassis.command.run = 1U;
    step(true);
    assert(g_chassis.test_status == CHASSIS_TEST_INVALID_COMMAND);

    reset_fixture();
    start_leg(0U);
    g_chassis.command.position_deg = INFINITY;
    step(true);
    assert(g_chassis.command.run == 0U && g_chassis_test.stopping);
    assert(g_chassis.test_status == CHASSIS_TEST_INVALID_COMMAND);

    reset_fixture();
    start_leg(0U);
    leg_feedback(0U, 0xBU, 35000U, 2047U);
    step(true);
    assert(g_chassis.test_status == CHASSIS_TEST_MOTOR_FAULT);
    assert(g_chassis.command.run == 0U);

    reset_fixture();
    select_motor(CHASSIS_TEST_LEG_RIGHT_FRONT);
    leg_feedback(0U, 0U, 35000U, 2047U);
    g_chassis.command.leg_kp = -1.0F;
    g_chassis.command.run = 1U;
    step(true);
    assert(g_chassis.test_status == CHASSIS_TEST_INVALID_COMMAND);
}

/** @brief Failed zero-torque preload never enables; failed sends preserve old records. */
static void test_send_failures(void)
{
    uint32_t index;
    reset_fixture();
    select_motor(CHASSIS_TEST_LEG_RIGHT_FRONT);
    leg_feedback(0U, 0U, 35000U, 2047U);
    failed_bus = &hfdcan2;
    failed_id = 1U;
    failures_left = 1U;
    frame_count = 0U;
    g_chassis.command.run = 1U;
    step(true);
    assert(g_chassis.test_status == CHASSIS_TEST_SEND_FAILED);
    assert(g_chassis.command.run == 0U);
    for (index = 0U; index < frame_count; ++index)
    {
        assert(!((frames[index].bus == &hfdcan2) && (frames[index].data[7] == 0xFCU)));
    }

    reset_fixture();
    start_leg(0U);
    g_chassis.leg_submitted_torque_nm[0] = 2.0F;
    failed_bus = &hfdcan2;
    failed_id = 1U;
    failures_left = 1U;
    assert(!chassis_send_leg_torque(0U, 3.0F));
    assert(g_chassis.leg_submitted_torque_nm[0] == 2.0F);

    reset_fixture();
    select_motor(CHASSIS_TEST_WHEEL_RIGHT_FRONT);
    dji_feedback(&hfdcan1, 0U, 0);
    g_chassis.command.run = 1U;
    g_chassis.command.speed_rpm = 100.0F;
    failed_bus = &hfdcan1;
    failed_id = 0x200U;
    failures_left = 1U;
    step(true);
    assert(g_chassis.test_status == CHASSIS_TEST_SEND_FAILED);
    assert(g_chassis.command.run == 0U);
    check_dji_currents(NULL, 0U, 0);
}

/** @brief Remote stick motion alone never feeds wheel targets or multi-wheel output. */
static void test_remote_sticks_disconnected(void)
{
    uint32_t wheel;
    reset_fixture();
    remote_ctrl.rc.ch[0] = RC_CH_VALUE_MAX;
    remote_ctrl.rc.ch[1] = RC_CH_VALUE_MAX;
    remote_ctrl.rc.ch[2] = RC_CH_VALUE_MAX;
    for (wheel = 0U; wheel < 4U; ++wheel)
    {
        dji_feedback(&hfdcan1, wheel, 0);
    }
    step(true);
    check_dji_currents(NULL, 0U, 0);
    select_motor(CHASSIS_TEST_WHEEL_LEFT_BACK);
    g_chassis.command.run = 1U;
    step(true);
    check_dji_currents(NULL, 0U, 0); /* Selected motor still has a direct zero target. */
    g_chassis.command.speed_rpm = 100.0F;
    step(true);
    check_dji_currents(&hfdcan1, 2U, 1000);
    remote_ctrl.rc.ch[0] = -RC_CH_VALUE_MAX;
    remote_ctrl.rc.ch[1] = -RC_CH_VALUE_MAX;
    remote_ctrl.rc.ch[2] = -RC_CH_VALUE_MAX;
    step(true);
    check_dji_currents(&hfdcan1, 2U, 1000);
    g_chassis.command.run = 0U;
    step(true);
    check_dji_currents(NULL, 0U, 0);
}

int main(void)
{
    initial_chassis = g_chassis;
    test_boot_and_management_rate();
    test_can_dispatch();
    test_all_speed_motors();
    test_all_leg_pd();
    test_stop_and_selection();
    test_feedback_and_wrap();
    test_feedback_arriving_after_cycle_time();
    test_invalid_and_motor_fault();
    test_send_failures();
    test_remote_sticks_disconnected();
    puts("PASS: all ten motors, CAN routing, PD/units, handshakes, next-ms ISR timing, stop/switch, faults and disconnected remote kinematics");
    return 0;
}
