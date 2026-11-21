#include "motor_test.h"
#include "chassis_motor_map.h"
#include "chassis.h"
#include "chassis_heading.h"
#include "board_bringup.h"
#include "main.h"
#if MOTOR_TEST_MODE == MOTOR_TEST_FIRST_ROUTE
#include "chassis_path.h"
#endif

typedef struct { uint8_t id; uint8_t forward; } Wheel;
static const Wheel wheels[4] = {
    {CHASSIS_MOTOR_LEFT_FRONT_ID, CHASSIS_MOTOR_LEFT_FRONT_FORWARD_DIR},
    {CHASSIS_MOTOR_LEFT_REAR_ID, CHASSIS_MOTOR_LEFT_REAR_FORWARD_DIR},
    {CHASSIS_MOTOR_RIGHT_REAR_ID, CHASSIS_MOTOR_RIGHT_REAR_FORWARD_DIR},
    {CHASSIS_MOTOR_RIGHT_FRONT_ID, CHASSIS_MOTOR_RIGHT_FRONT_FORWARD_DIR}
};
volatile MotorTestDiagnostics motor_test_diag;
static uint32_t stage_started, request_started, reply_changed;
static uint8_t last_length, pending, cursor;
#if MOTOR_TEST_MODE != MOTOR_TEST_CHECK_ONLY
static uint32_t run_started;
static uint8_t shutdown_abort;
static uint8_t active_mask;
#endif
#if MOTOR_TEST_MODE == MOTOR_TEST_FIRST_ROUTE
static uint8_t route_started;
static uint8_t route_slot;
static uint8_t route_reply_pending;
static uint32_t route_next_slot;
#endif

static void Enter(MotorTestStage stage)
{
    motor_test_diag.stage = stage;
    stage_started = HAL_GetTick();
    cursor = 0U;
    pending = 0U;
}

static void Fail(MotorTestFailure reason, const StepMotorReply *reply)
{
    if (motor_test_diag.first_failure != MOTOR_FAILURE_NONE) return;
    motor_test_diag.first_failure = reason;
    motor_test_diag.failure_stage = motor_test_diag.stage;
    motor_test_diag.failure_id = motor_test_diag.id;
    motor_test_diag.failure_command = motor_test_diag.command;
    if (reply != 0) motor_test_diag.failure_reply = *reply;
}

static void Request(uint8_t id, uint8_t command, uint8_t direction, uint16_t value)
{
    motor_test_diag.id = id;
    motor_test_diag.command = command;
    motor_test_diag.last_return_code = 0xFFU;
    if (command == 0xF3U)
        motor_test_diag.last_tx = StepMotor_Enable(id, (uint8_t)value);
    else if (command == 0x42U)
        motor_test_diag.last_tx = StepMotor_ReadConfig(id);
#if MOTOR_TEST_MODE != MOTOR_TEST_CHECK_ONLY
    else if (command == 0xF6U)
        motor_test_diag.last_tx = StepMotor_SetSpeedSync(id, direction, value, 0U, 1U);
    else
        motor_test_diag.last_tx = StepMotor_Stop(id);
#else
    else
        motor_test_diag.last_tx = STEP_MOTOR_TX_TXE_TIMEOUT;
    (void)direction;
#endif
    request_started = HAL_GetTick();
    reply_changed = request_started;
    last_length = 0U;
    pending = 1U;
}

/* -1 waiting, 0 failed, 1 accepted. No next TX while pending.
 * The RX quiet interval preserves existing duplicate-ACK support. */
static int8_t Poll(void)
{
    StepMotorReply reply = {0};
    uint8_t code = 0xFFU, complete;
    uint32_t now = HAL_GetTick();
    uint32_t timeout = MOTOR_TEST_REPLY_TIMEOUT_MS;
#if MOTOR_TEST_MODE != MOTOR_TEST_CHECK_ONLY
    if ((motor_test_diag.stage == MOTOR_STAGE_ZERO_SPEED) ||
        (motor_test_diag.stage == MOTOR_STAGE_STOP) ||
        (motor_test_diag.stage == MOTOR_STAGE_DISABLE))
        timeout = MOTOR_TEST_CLEANUP_TIMEOUT_MS;
#endif
    if (motor_test_diag.last_tx != STEP_MOTOR_TX_OK) {
        Fail(MOTOR_FAILURE_TX, 0);
        pending = 0U;
        return 0;
    }
    StepMotor_GetReply(&reply);
    motor_test_diag.last_reply = reply;
    if (reply.length != last_length) {
        last_length = reply.length;
        reply_changed = now;
    }
    if (motor_test_diag.command == 0x42U)
        complete = StepMotor_ParseConfigIdentity(&reply, (uint8_t)motor_test_diag.id);
    else
        complete = StepMotor_ReplyIsAck(&reply, (uint8_t)motor_test_diag.id,
                                      (uint8_t)motor_test_diag.command, &code);
    if ((complete != 0U) && ((now - reply_changed) >= MOTOR_TEST_REPLY_QUIET_MS)) {
        motor_test_diag.last_return_code = code;
        pending = 0U;
        if ((motor_test_diag.command != 0x42U) && (code != 0x02U)) {
            Fail(MOTOR_FAILURE_REJECTED, &reply);
            return 0;
        }
        return 1;
    }
    if ((now - request_started) < timeout) return -1;
    Fail((reply.length == 0U) ? MOTOR_FAILURE_NO_REPLY : MOTOR_FAILURE_INVALID_REPLY,
         &reply);
    pending = 0U;
    return 0;
}

static int8_t Command(uint8_t id, uint8_t command, uint8_t direction, uint16_t value)
{
    if (pending == 0U) Request(id, command, direction, value);
    return Poll();
}

static void BeginShutdown(uint8_t abort_test)
{
    motor_test_diag.disable_ok_mask = 0U;
#if MOTOR_TEST_MODE == MOTOR_TEST_FIRST_ROUTE
    ChassisHeading_EndMotion();
#endif
#if MOTOR_TEST_MODE == MOTOR_TEST_CHECK_ONLY
    (void)abort_test;
    /* CHECK_ONLY faults send only disable: no speed, sync or stop frames. */
    Enter(MOTOR_STAGE_DISABLE);
#else
    shutdown_abort = abort_test;
    /* Before any attempted enable, disable alone is sufficient. */
    Enter(active_mask ? MOTOR_STAGE_ZERO_SPEED : MOTOR_STAGE_DISABLE);
#endif
}

#if MOTOR_TEST_MODE != MOTOR_TEST_CHECK_ONLY
static uint8_t TargetIndex(void)
{
    return (MOTOR_TEST_MODE == MOTOR_TEST_WHEEL_SEQUENCE) ?
           (uint8_t)(motor_test_diag.wheel_index / 2U) : cursor;
}

static uint8_t TargetCount(void)
{
    return (MOTOR_TEST_MODE == MOTOR_TEST_WHEEL_SEQUENCE) ? 1U : 4U;
}

static uint8_t PrepareTargets(void)
{
    ChassisMotion motion;
    static const ChassisMotion demo[] = {
        CHASSIS_FORWARD, CHASSIS_BACKWARD, CHASSIS_LEFT,
        CHASSIS_RIGHT, CHASSIS_ROTATE_LEFT, CHASSIS_ROTATE_RIGHT
    };
    static const ChassisMotion no_imu_sequence[] = {
        CHASSIS_FORWARD, CHASSIS_ROTATE_LEFT, CHASSIS_ROTATE_RIGHT
    };
    StepMotor_ClearTargets();
#if MOTOR_TEST_MODE == MOTOR_TEST_FIRST_ROUTE
    route_started = 0U;
    route_slot = 0U;
    route_reply_pending = 0U;
    Chassis_PathResetRuntime();
    /* Before enable: validate the path and preload zero only. First control
     * calculation belongs to the first complete runtime motor group. */
    return Chassis_PathPointDurationMs(&CHASSIS_PATH_ACTIVE_ROUTE,
              (uint8_t)motor_test_diag.wheel_index) != 0U &&
           Chassis_InverseMotionControl(0.0f, 0.0f, 0.0f);
#endif
    if (MOTOR_TEST_MODE == MOTOR_TEST_WHEEL_SEQUENCE) {
        uint8_t index = TargetIndex();
        uint8_t dir = wheels[index].forward ^ (motor_test_diag.wheel_index & 1U);
        int32_t speed = MOTOR_TEST_SPEED_RPM;
        return StepMotor_SetTargetSpeed(wheels[index].id, dir ? -speed : speed);
    }
    switch (MOTOR_TEST_MODE) {
    case MOTOR_TEST_FORWARD: motion = CHASSIS_FORWARD; break;
    case MOTOR_TEST_BACKWARD: motion = CHASSIS_BACKWARD; break;
    case MOTOR_TEST_TURN_LEFT: motion = CHASSIS_ROTATE_LEFT; break;
    case MOTOR_TEST_TURN_RIGHT: motion = CHASSIS_ROTATE_RIGHT; break;
    case MOTOR_TEST_STRAFE_LEFT: motion = CHASSIS_LEFT; break;
    case MOTOR_TEST_STRAFE_RIGHT: motion = CHASSIS_RIGHT; break;
    case MOTOR_TEST_BASIC_DEMO:
        if (motor_test_diag.wheel_index >= sizeof(demo) / sizeof(demo[0])) return 0U;
        motion = demo[motor_test_diag.wheel_index]; break;
    case MOTOR_TEST_NO_IMU_SEQUENCE:
        if (motor_test_diag.wheel_index >=
            sizeof(no_imu_sequence) / sizeof(no_imu_sequence[0])) return 0U;
        motion = no_imu_sequence[motor_test_diag.wheel_index]; break;
    default: return 0U;
    }
    return Chassis_PrepareMotion(motion, MOTOR_TEST_SPEED_RPM);
}

static uint32_t RunDurationMs(void)
{
    if (MOTOR_TEST_MODE == MOTOR_TEST_WHEEL_SEQUENCE)
        return MOTOR_TEST_SINGLE_RUN_MS;
    if (MOTOR_TEST_MODE == MOTOR_TEST_NO_IMU_SEQUENCE &&
        motor_test_diag.wheel_index != 0U)
        return MOTOR_TEST_OPEN_LOOP_30_DEG_MS;
    return MOTOR_TEST_GROUP_RUN_MS;
}

static uint8_t Broadcast(void)
{
    motor_test_diag.id = 0U;
    motor_test_diag.command = 0xFFU;
    motor_test_diag.last_tx = StepMotor_SyncMove(0U);
    if (motor_test_diag.last_tx != STEP_MOTOR_TX_OK) {
        Fail(MOTOR_FAILURE_TX, 0);
        return 0U;
    }
    return 1U;
}
#endif

static uint32_t MarkerCount(void)
{
    return (MOTOR_TEST_MODE == MOTOR_TEST_WHEEL_SEQUENCE) ?
           motor_test_diag.wheel_index / 2U + 1U : motor_test_diag.wheel_index + 1U;
}

static void UpdateMarker(void)
{
    uint32_t elapsed = HAL_GetTick() - stage_started, phase;
    uint8_t on = 0U;
    if (motor_test_diag.stage == MOTOR_STAGE_COMPLETE) {
        /* Five short pulses: selected protocol checks passed, not physical motion. */
        phase = elapsed % 2400U;
        on = (uint8_t)((phase < 1500U) && ((phase % 300U) < 120U));
    } else if (motor_test_diag.stage == MOTOR_STAGE_FAILED) {
        phase = elapsed % 2600U; /* Two long pulses, then a full 1000 ms gap. */
        on = (uint8_t)((phase < 700U) || ((phase >= 900U) && (phase < 1600U)));
    } else if (motor_test_diag.stage == MOTOR_STAGE_WHEEL_MARKER) {
        on = (uint8_t)((elapsed < MarkerCount() * 300U) &&
                       ((elapsed % 300U) < 120U));
    } else if (motor_test_diag.stage == MOTOR_STAGE_RUNNING) {
        on = 1U;
#if MOTOR_TEST_MODE == MOTOR_TEST_FIRST_ROUTE
    } else if (route_started &&
              (motor_test_diag.stage == MOTOR_STAGE_SET_SPEED ||
               motor_test_diag.stage == MOTOR_STAGE_SYNC_START ||
               motor_test_diag.stage == MOTOR_STAGE_ROUTE_DRAIN ||
               motor_test_diag.stage == MOTOR_STAGE_ROUTE_RUNNING)) {
        on = 1U;
#endif
    } else if ((motor_test_diag.stage == MOTOR_STAGE_PREFLIGHT_DISABLE) ||
               (motor_test_diag.stage == MOTOR_STAGE_READ_CONFIG)) {
        on = (uint8_t)((elapsed % 1000U) < 120U);
    }
    BoardBringup_SetMarker(on);
}

void MotorTest_Init(void)
{
    const MotorTestDiagnostics initial = {0};
    BoardBringup_TestSignalLow();
    motor_test_diag = initial;
    motor_test_diag.mode = MOTOR_TEST_MODE;
    motor_test_diag.last_return_code = 0xFFU;
    last_length = 0U;
#if MOTOR_TEST_MODE != MOTOR_TEST_CHECK_ONLY
    shutdown_abort = 0U;
    active_mask = 0U;
#endif
#if MOTOR_TEST_MODE == MOTOR_TEST_FIRST_ROUTE
    route_started = 0U;
    route_slot = 0U;
    route_reply_pending = 0U;
    route_next_slot = 0U;
#endif
    Enter(MOTOR_STAGE_START_DELAY);
}

uint8_t MotorTest_NeedsFastService(void)
{
#if MOTOR_TEST_MODE == MOTOR_TEST_FIRST_ROUTE
    return motor_test_diag.stage == MOTOR_STAGE_ROUTE_RUNNING;
#else
    return 0U;
#endif
}

void MotorTest_Service(void)
{
    int8_t result;
#if MOTOR_TEST_MODE != MOTOR_TEST_CHECK_ONLY
    uint8_t target, direction;
    uint16_t speed;
    uint32_t now;
    StepMotorReply drain = {0};
#endif
#if MOTOR_TEST_MODE == MOTOR_TEST_FIRST_ROUTE
    /* Check before every enable/speed/sync slot, not just PID calculations.
     * A fault latches; cleanup still runs, no next route segment starts. */
    if ((motor_test_diag.stage == MOTOR_STAGE_ENABLE ||
         motor_test_diag.stage == MOTOR_STAGE_SET_SPEED ||
         motor_test_diag.stage == MOTOR_STAGE_SYNC_START ||
         motor_test_diag.stage == MOTOR_STAGE_RUNNING ||
         motor_test_diag.stage == MOTOR_STAGE_ROUTE_RUNNING) &&
        !ChassisHeading_IsHealthy()) {
        Fail(MOTOR_FAILURE_HEADING, 0);
        BeginShutdown(1U);
        return;
    }
#endif
    switch (motor_test_diag.stage) {
    case MOTOR_STAGE_START_DELAY:
        if ((HAL_GetTick() - stage_started) >= MOTOR_TEST_START_DELAY_MS) {
#if MOTOR_TEST_MODE == MOTOR_TEST_FIRST_ROUTE
            if (!ChassisHeading_ArmCurrent()) {
                Fail(MOTOR_FAILURE_HEADING, 0);
                BeginShutdown(1U);
                break;
            }
#endif
            Enter(MOTOR_STAGE_PREFLIGHT_DISABLE);
        }
        break;
    case MOTOR_STAGE_PREFLIGHT_DISABLE:
        result = Command(wheels[cursor].id, 0xF3U, 0U, 0U);
        if (result < 0) break;
        if (result > 0) motor_test_diag.disable_ok_mask |= 1UL << (wheels[cursor].id - 1U);
        cursor++;
        if (cursor == 4U) {
            if (motor_test_diag.first_failure != MOTOR_FAILURE_NONE)
                Enter(MOTOR_STAGE_FAILED);
            else Enter(MOTOR_STAGE_READ_CONFIG);
        }
        break;
    case MOTOR_STAGE_READ_CONFIG:
        result = Command(wheels[cursor].id, 0x42U, 0U, 0U);
        if (result < 0) break;
        if (result == 0) { BeginShutdown(1U); break; }
        motor_test_diag.config_ok_mask |= 1UL << (wheels[cursor].id - 1U);
        cursor++;
        if (cursor == 4U) {
#if MOTOR_TEST_MODE == MOTOR_TEST_CHECK_ONLY
            Enter(MOTOR_STAGE_COMPLETE);
#else
            Enter(MOTOR_STAGE_WHEEL_MARKER);
#endif
        }
        break;
#if MOTOR_TEST_MODE != MOTOR_TEST_CHECK_ONLY
    case MOTOR_STAGE_WHEEL_MARKER:
        if ((HAL_GetTick() - stage_started) >=
            MarkerCount() * 300U + 800U) {
            active_mask = 0U;
            if (!PrepareTargets()) {
                Fail(MOTOR_FAILURE_PARAMETER, 0);
                BeginShutdown(1U);
            } else Enter(MOTOR_STAGE_ENABLE);
        }
        break;
    case MOTOR_STAGE_ENABLE:
    case MOTOR_STAGE_SET_SPEED:
        target = TargetIndex();
        if (motor_test_diag.stage == MOTOR_STAGE_ENABLE) {
            /* A lost ACK does not prove enable was not executed. */
            active_mask |= 1U << target;
            result = Command(wheels[target].id, 0xF3U, 0U, 1U);
        } else {
            if (!StepMotor_GetTargetSpeed(wheels[target].id, &direction, &speed)) {
                Fail(MOTOR_FAILURE_PARAMETER, 0);
                BeginShutdown(1U);
                break;
            }
            result = Command(wheels[target].id, 0xF6U, direction, speed);
        }
        if (result < 0) break;
        if (result == 0) { BeginShutdown(1U); break; }
        cursor++;
        if (cursor == TargetCount()) {
#if MOTOR_TEST_MODE == MOTOR_TEST_FIRST_ROUTE
            if (motor_test_diag.stage == MOTOR_STAGE_ENABLE)
                Enter(MOTOR_STAGE_ROUTE_RUNNING);
            else Enter(MOTOR_STAGE_SYNC_START);
#else
            Enter((motor_test_diag.stage == MOTOR_STAGE_ENABLE) ?
                  MOTOR_STAGE_SET_SPEED : MOTOR_STAGE_SYNC_START);
#endif
        }
        break;
    case MOTOR_STAGE_SYNC_START:
        if (Broadcast() == 0U) { BeginShutdown(1U); break; }
        run_started = HAL_GetTick();
        Enter(MOTOR_STAGE_RUNNING);
        break;
    case MOTOR_STAGE_RUNNING:
        if ((HAL_GetTick() - run_started) >= RunDurationMs())
            BeginShutdown(0U);
        break;
#if MOTOR_TEST_MODE == MOTOR_TEST_FIRST_ROUTE
    case MOTOR_STAGE_ROUTE_DRAIN:
        Fail(MOTOR_FAILURE_PARAMETER, 0);
        BeginShutdown(1U);
        break;
    case MOTOR_STAGE_ROUTE_RUNNING:
        now = HAL_GetTick();
        if (route_started == 0U) {
            run_started = now;
            route_next_slot = now;
            route_started = 1U;
        }
        if ((now - run_started) >=
            Chassis_PathPointDurationMs(&CHASSIS_PATH_ACTIVE_ROUTE,
                                       (uint8_t)motor_test_diag.wheel_index)) {
            if (route_reply_pending != 0U && route_slot == 0U) {
                StepMotor_GetReply(&drain);
                motor_test_diag.last_reply = drain;
                if (drain.overflow || drain.uart_error) {
                    Fail(MOTOR_FAILURE_BROADCAST_NOISE, &drain);
                    BeginShutdown(1U);
                    break;
                }
            }
            if (Chassis_PathPointIsTurn((uint8_t)motor_test_diag.wheel_index) &&
                !Chassis_PathPointComplete((uint8_t)motor_test_diag.wheel_index)) {
                Fail(MOTOR_FAILURE_HEADING, 0);
                BeginShutdown(1U);
            } else BeginShutdown(0U);
        } else if ((int32_t)(now - route_next_slot) >= 0) {
            uint32_t elapsed = now - run_started;
            if (route_reply_pending != 0U) {
                uint8_t code = 0xFFU;
                StepMotor_GetReply(&drain);
                motor_test_diag.last_reply = drain;
                if (drain.overflow || drain.uart_error) {
                    Fail((route_slot == 0U) ? MOTOR_FAILURE_BROADCAST_NOISE :
                         MOTOR_FAILURE_INVALID_REPLY, &drain);
                    BeginShutdown(1U);
                    break;
                }
                if (route_slot != 0U) {
                    if (drain.length == 0U) {
                        Fail(MOTOR_FAILURE_NO_REPLY, &drain);
                        BeginShutdown(1U);
                        break;
                    }
                    if (!StepMotor_ReplyIsAck(&drain,
                                              wheels[route_slot - 1U].id,
                                              0xF6U, &code)) {
                        Fail(MOTOR_FAILURE_INVALID_REPLY, &drain);
                        BeginShutdown(1U);
                        break;
                    }
                    motor_test_diag.last_return_code = code;
                    if (code != 0x02U) {
                        Fail(MOTOR_FAILURE_REJECTED, &drain);
                        BeginShutdown(1U);
                        break;
                    }
                }
            }
            if (route_slot == 0U) {
                if (!Chassis_PathPreparePoint((uint8_t)motor_test_diag.wheel_index,
                                              elapsed)) {
                    Fail(ChassisHeading_IsHealthy() ? MOTOR_FAILURE_PARAMETER :
                         MOTOR_FAILURE_HEADING, 0);
                    BeginShutdown(1U);
                    break;
                }
                if (Chassis_PathPointComplete((uint8_t)motor_test_diag.wheel_index)) {
                    BeginShutdown(0U);
                    break;
                }
                motor_test_diag.id = wheels[0].id;
                motor_test_diag.command = 0xF6U;
                motor_test_diag.last_tx = StepMotor_SendTargetFast(wheels[0].id);
                route_slot = 1U;
            } else if (route_slot < 4U) {
                motor_test_diag.id = wheels[route_slot].id;
                motor_test_diag.command = 0xF6U;
                motor_test_diag.last_tx =
                    StepMotor_SendTargetFast(wheels[route_slot].id);
                route_slot++;
            } else {
                motor_test_diag.id = 0U;
                motor_test_diag.command = 0xFFU;
                motor_test_diag.last_tx = StepMotor_SyncMoveFast();
                route_slot = 0U;
            }
            if (motor_test_diag.last_tx != STEP_MOTOR_TX_OK) {
                Fail(MOTOR_FAILURE_TX, 0);
                BeginShutdown(1U);
                break;
            }
            route_reply_pending = 1U;
            route_next_slot += MOTOR_TEST_ROUTE_SLOT_MS;
            if ((int32_t)(HAL_GetTick() - route_next_slot) >=
                (int32_t)MOTOR_TEST_ROUTE_SLOT_MS) {
                Fail(MOTOR_FAILURE_SCHEDULE, 0);
                BeginShutdown(1U);
            }
        }
        break;
#endif
    case MOTOR_STAGE_ZERO_SPEED:
    case MOTOR_STAGE_STOP:
        /* Zero/STOP only attempted-enable targets; still disable all four. */
        while (cursor < 4U && !(active_mask & (1U << cursor))) cursor++;
        if (cursor == 4U) {
            Enter((motor_test_diag.stage == MOTOR_STAGE_ZERO_SPEED) ?
                  MOTOR_STAGE_ZERO_SYNC : MOTOR_STAGE_DISABLE);
            break;
        }
        direction = 0U;
        (void)StepMotor_GetTargetSpeed(wheels[cursor].id, &direction, &speed);
        result = Command(wheels[cursor].id,
                         (motor_test_diag.stage == MOTOR_STAGE_ZERO_SPEED) ? 0xF6U : 0xFEU,
                         direction, 0U);
        if (result < 0) break;
        if (result == 0) {
            shutdown_abort = 1U;
            motor_test_diag.cleanup_failure_mask |= 1UL << (wheels[cursor].id - 1U);
        }
        cursor++;
        if (cursor == 4U)
            Enter((motor_test_diag.stage == MOTOR_STAGE_ZERO_SPEED) ?
                  MOTOR_STAGE_ZERO_SYNC : MOTOR_STAGE_DISABLE);
        break;
    case MOTOR_STAGE_ZERO_SYNC:
        if (Broadcast() == 0U) shutdown_abort = 1U;
        Enter(MOTOR_STAGE_SYNC_DRAIN);
        reply_changed = HAL_GetTick();
        last_length = 0U;
        break;
    case MOTOR_STAGE_SYNC_DRAIN:
        now = HAL_GetTick();
        StepMotor_GetReply(&drain);
        if (drain.length != last_length) {
            last_length = drain.length;
            reply_changed = now;
        }
        if ((drain.overflow != 0U) || (drain.uart_error != 0U) ||
            (((now - stage_started) >= MOTOR_TEST_SYNC_DRAIN_MS) &&
             ((now - reply_changed) < MOTOR_TEST_SYNC_QUIET_MS))) {
            Fail(MOTOR_FAILURE_BROADCAST_NOISE, &drain);
            shutdown_abort = 1U;
            Enter(MOTOR_STAGE_STOP);
        } else if ((now - reply_changed) >= MOTOR_TEST_SYNC_QUIET_MS)
            Enter(MOTOR_STAGE_STOP);
        break;
#endif
    case MOTOR_STAGE_DISABLE:
        result = Command(wheels[cursor].id, 0xF3U, 0U, 0U);
        if (result < 0) break;
        if (result > 0) motor_test_diag.disable_ok_mask |= 1UL << (wheels[cursor].id - 1U);
        else {
            motor_test_diag.cleanup_failure_mask |= 1UL << (wheels[cursor].id - 1U);
#if MOTOR_TEST_MODE != MOTOR_TEST_CHECK_ONLY
            shutdown_abort = 1U;
#endif
        }
        cursor++;
        if (cursor == 4U) {
            StepMotor_ClearTargets();
#if MOTOR_TEST_MODE == MOTOR_TEST_CHECK_ONLY
            Enter(MOTOR_STAGE_FAILED);
#else
            if ((shutdown_abort != 0U) || (motor_test_diag.first_failure != MOTOR_FAILURE_NONE))
                Enter(MOTOR_STAGE_FAILED);
            else {
                motor_test_diag.completed_rounds++;
                if ((MOTOR_TEST_MODE == MOTOR_TEST_WHEEL_SEQUENCE) &&
                    (motor_test_diag.wheel_index < 7U)) Enter(MOTOR_STAGE_GAP);
                else if ((MOTOR_TEST_MODE == MOTOR_TEST_BASIC_DEMO) &&
                         (motor_test_diag.wheel_index < 5U)) Enter(MOTOR_STAGE_GAP);
                else if ((MOTOR_TEST_MODE == MOTOR_TEST_NO_IMU_SEQUENCE) &&
                         (motor_test_diag.wheel_index < 2U)) Enter(MOTOR_STAGE_GAP);
#if MOTOR_TEST_MODE == MOTOR_TEST_FIRST_ROUTE
                else if (motor_test_diag.wheel_index + 1U < CHASSIS_PATH_ACTIVE_ROUTE.Path_Size)
                    Enter(MOTOR_STAGE_GAP);
#endif
                else Enter(MOTOR_STAGE_COMPLETE);
            }
#endif
        }
        break;
#if MOTOR_TEST_MODE != MOTOR_TEST_CHECK_ONLY
    case MOTOR_STAGE_GAP:
        if ((HAL_GetTick() - stage_started) >= MOTOR_TEST_WHEEL_GAP_MS) {
            motor_test_diag.wheel_index++;
            Enter(MOTOR_STAGE_WHEEL_MARKER);
        }
        break;
#endif
    case MOTOR_STAGE_COMPLETE:
    case MOTOR_STAGE_FAILED:
        break; /* Terminal: no further UART until an intentional reset. */
    default:
        BeginShutdown(1U);
        break;
    }
    UpdateMarker();
}
