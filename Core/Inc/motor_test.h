#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

#include "step_motor.h"

#define MOTOR_TEST_CHECK_ONLY     0U
#define MOTOR_TEST_WHEEL_SEQUENCE 1U
#define MOTOR_TEST_FORWARD        2U
#define MOTOR_TEST_BACKWARD       3U
#define MOTOR_TEST_TURN_LEFT      4U
#define MOTOR_TEST_TURN_RIGHT     5U
#define MOTOR_TEST_STRAFE_LEFT    6U
#define MOTOR_TEST_STRAFE_RIGHT   7U
#define MOTOR_TEST_BASIC_DEMO     8U
#define MOTOR_TEST_FIRST_ROUTE    9U
#define MOTOR_TEST_NO_IMU_SEQUENCE 10U

/* One selected test per reset. Motion requires a deliberate new build. */
#ifndef MOTOR_TEST_MODE
#define MOTOR_TEST_MODE MOTOR_TEST_NO_IMU_SEQUENCE
#endif
#if MOTOR_TEST_MODE > MOTOR_TEST_NO_IMU_SEQUENCE
#error "Invalid MOTOR_TEST_MODE"
#endif

#define MOTOR_TEST_REPLY_TIMEOUT_MS 300U
#define MOTOR_TEST_CLEANUP_TIMEOUT_MS 60U
#define MOTOR_TEST_REPLY_QUIET_MS     5U
#define MOTOR_TEST_START_DELAY_MS  3000U
#define MOTOR_TEST_SPEED_RPM         50U
#define MOTOR_TEST_SINGLE_RUN_MS   1000U
#define MOTOR_TEST_GROUP_RUN_MS    2000U
/* Historical 50 RPM / 2000 ms turns were observed near 180 degrees.
 * First no-IMU 30-degree estimate: 2000 * 30 / 180 = 333 ms. */
#define MOTOR_TEST_OPEN_LOOP_30_DEG_MS 333U
#define MOTOR_TEST_WHEEL_GAP_MS    2000U
#define MOTOR_TEST_SYNC_QUIET_MS     20U
#define MOTOR_TEST_SYNC_DRAIN_MS    100U
#define MOTOR_TEST_ROUTE_SLOT_MS      2U
#define MOTOR_TEST_ROUTE_CONTROL_MS  10U
#include "chassis_heading.h"
#if MOTOR_TEST_ROUTE_CONTROL_MS != CHASSIS_HEADING_PERIOD_MS
#error "Heading and complete motor group periods must match"
#endif
#if MOTOR_TEST_ROUTE_CONTROL_MS != (5U * MOTOR_TEST_ROUTE_SLOT_MS)
#error "Route control period must equal four motor slots plus one sync slot"
#endif

typedef enum {
    MOTOR_STAGE_START_DELAY = 0,
    MOTOR_STAGE_PREFLIGHT_DISABLE,
    MOTOR_STAGE_READ_CONFIG,
    MOTOR_STAGE_WHEEL_MARKER,
    MOTOR_STAGE_ENABLE,
    MOTOR_STAGE_SET_SPEED,
    MOTOR_STAGE_SYNC_START,
    MOTOR_STAGE_RUNNING,
    MOTOR_STAGE_ZERO_SPEED,
    MOTOR_STAGE_ZERO_SYNC,
    MOTOR_STAGE_SYNC_DRAIN,
    MOTOR_STAGE_STOP,
    MOTOR_STAGE_DISABLE,
    MOTOR_STAGE_GAP,
    MOTOR_STAGE_COMPLETE,
    MOTOR_STAGE_FAILED,
    MOTOR_STAGE_ROUTE_DRAIN, /* Historical value retained for diagnostics. */
    MOTOR_STAGE_ROUTE_RUNNING
} MotorTestStage;

typedef enum {
    MOTOR_FAILURE_NONE = 0,
    MOTOR_FAILURE_TX,
    MOTOR_FAILURE_NO_REPLY,
    MOTOR_FAILURE_INVALID_REPLY,
    MOTOR_FAILURE_REJECTED,
    MOTOR_FAILURE_BROADCAST_NOISE,
    MOTOR_FAILURE_PARAMETER,
    MOTOR_FAILURE_HEADING,
    MOTOR_FAILURE_SCHEDULE /* Slot deadline missed; separate from UART TX. */
} MotorTestFailure;

/* Protocol evidence only. Inspect with the debugger after motors stop. */
typedef struct {
    uint32_t mode;
    MotorTestStage stage;
    uint32_t wheel_index; /* Wheel test 0..7, demo 0..5, first route 0..2. */
    uint32_t id;
    uint32_t command;
    StepMotorTxResult last_tx;
    uint32_t last_return_code;
    StepMotorReply last_reply;
    MotorTestFailure first_failure;
    MotorTestStage failure_stage;
    uint32_t failure_id;
    uint32_t failure_command;
    StepMotorReply failure_reply;
    uint32_t config_ok_mask;
    uint32_t disable_ok_mask;
    uint32_t cleanup_failure_mask;
    uint32_t completed_rounds;
} MotorTestDiagnostics;

extern volatile MotorTestDiagnostics motor_test_diag;
void MotorTest_Init(void);
void MotorTest_Service(void);
/* Main-loop idle delay must not consume the active motor slot budget. */
uint8_t MotorTest_NeedsFastService(void);

#endif /* MOTOR_TEST_H */
