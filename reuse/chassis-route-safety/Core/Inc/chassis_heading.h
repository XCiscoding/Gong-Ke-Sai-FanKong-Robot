#ifndef CHASSIS_HEADING_H
#define CHASSIS_HEADING_H

#include "main.h"
#include <stdint.h>
/* P-only HOLD candidate.  Kp and polarity are carried from the historical
 * USART3 chassis, but the direct-step physical correction sign is still the
 * first powered acceptance item.  Output units are mixer rotation RPM. */
#define CHASSIS_HEADING_KP             1.0f
#define CHASSIS_HEADING_KI             0.0f
#define CHASSIS_HEADING_KD             0.0f
#define CHASSIS_HEADING_DEADBAND_DEG   0.55f
/* Current MS901M installation reports clockwise yaw as negative, while the
 * inherited mixer sends positive w as wire direction 0 on all four motors,
 * which is clockwise on this chassis. Invert only at the mixer boundary. */
#define CHASSIS_HEADING_CORRECTION_POLARITY (-1.0f)
/* Staged commissioning build: a short HOLD-controlled forward leg, a full
 * stop/gap, then a relative +30 deg TURN. HOLD parameters above are frozen
 * from the passed dynamic trial. */
#ifndef CHASSIS_HEADING_TURN_TEST
#define CHASSIS_HEADING_TURN_TEST APP_PROFILE_LEGACY_MODULES
#endif
#if CHASSIS_HEADING_TURN_TEST != 0 && CHASSIS_HEADING_TURN_TEST != 1
#error "Invalid CHASSIS_HEADING_TURN_TEST"
#endif
#define CHASSIS_TURN_KP                    2.0f
#define CHASSIS_TURN_KI                    0.0f
#define CHASSIS_TURN_KD                    0.5f
#define CHASSIS_TURN_DEADBAND_DEG          0.55f
#define CHASSIS_TURN_LIMIT_RPM            30.0f
#define CHASSIS_TURN_SLEW_RPM_PER_UPDATE   2.0f
#define CHASSIS_TURN_SETTLE_TOLERANCE_DEG  2.0f
#define CHASSIS_TURN_SETTLE_MS            500U
#define CHASSIS_TURN_TIMEOUT_MS          4000U
#define CHASSIS_TURN_MAX_DELTA_DEG         30.0f
/* First powered trial: flat-ground short forward leg only. */
#ifndef CHASSIS_HEADING_LOW_SPEED_TEST
#define CHASSIS_HEADING_LOW_SPEED_TEST 1
#endif
#if CHASSIS_HEADING_LOW_SPEED_TEST != 0 && CHASSIS_HEADING_LOW_SPEED_TEST != 1
#error "Invalid CHASSIS_HEADING_LOW_SPEED_TEST"
#endif
#if CHASSIS_HEADING_LOW_SPEED_TEST
#define CHASSIS_HEADING_LIMIT_RPM     15.0f
#define CHASSIS_HEADING_HOLD_ERROR_LIMIT_TENTHS 350
#else
#define CHASSIS_HEADING_LIMIT_RPM     30.0f
#endif
#define CHASSIS_HEADING_PERIOD_MS     10U
#define CHASSIS_HEADING_MAX_GAP_MS    20U
#define CHASSIS_HEADING_ERROR_LIMIT    3U
#define CHASSIS_HEADING_FRESH_MS     100U
enum { HEADING_WAITING = 0U, HEADING_ACTIVE = 1U, HEADING_FAULT = 2U };
enum { HEADING_MODE_HOLD = 0U, HEADING_MODE_TURN = 1U,
       HEADING_MODE_SETTLE = 2U };
enum { HEADING_OK = 0U, HEADING_NOT_READY = 1U, HEADING_STALE = 2U,
       HEADING_RECEIVE_ERRORS = 3U, HEADING_PARAMETER = 4U,
       HEADING_SCHEDULE = 5U, HEADING_EXCESS_ERROR = 6U,
       HEADING_TURN_TIMEOUT = 7U };
/* All fields four bytes; signed angles in tenths of degrees.
 * correction_rpm and P/I/D fields are the values applied to the mixer. */
typedef struct {
    uint32_t diagnostic_only, state, fault, target_locked;
    int32_t target_yaw_tenths, yaw_tenths, error_tenths;
    float correction_rpm;
    uint32_t control_updates, last_control_ms, age_ms, receive_errors;
    float p_out, i_out, d_out;
    uint32_t mode, turn_complete, turn_started_ms, settle_started_ms;
    int32_t commanded_delta_tenths;
} ChassisHeadingDiagnostics;
extern volatile ChassisHeadingDiagnostics chassis_heading_diag;
/* First short trial retains per-group calculations for reading after stop.
 * A calculation record does not itself prove ACK/sync or physical motion. */
#define CHASSIS_HEADING_TRACE_PERIOD_MS 20U
#define CHASSIS_HEADING_TRACE_CAPACITY 256U
typedef struct {
    uint32_t at_ms;
    int32_t yaw_tenths, error_tenths;
    float correction_rpm, p_out, d_out;
    uint32_t age_ms, mode;
} ChassisHeadingTraceEntry;
extern volatile ChassisHeadingTraceEntry chassis_heading_trace[CHASSIS_HEADING_TRACE_CAPACITY];
extern volatile uint32_t chassis_heading_trace_count;
/** @brief 初始化航向控制状态；不会锁定目标，也不会产生电机命令。 */
void ChassisHeading_Init(void);

/**
 * @brief 更新非运动阶段的航向健康状态。
 * @note 恒速HOLD运行阶段由上层显式调用ChassisHeading_Compute()，本函数不驱动电机。
 */
void ChassisHeading_Service(void);

/**
 * @brief 将当前新鲜Yaw锁定为本次直行目标。
 * @return 1表示目标已锁定；0表示IMU未建零、不新鲜、参数异常或控制器已故障。
 * @note 只允许在电机仍停止的STABILIZE阶段调用。
 */
uint8_t ChassisHeading_ArmCurrent(void);

/**
 * @brief 检查IMU新鲜度、接收错误、35°保护线和10ms调度间隔。
 * @return 1表示可以继续控制；0表示已保持等待或进入锁存故障。
 */
uint8_t ChassisHeading_IsHealthy(void);

/**
 * @brief 按固定10ms周期计算一次纯P航向修正。
 * @param correction_rpm 输出旋转修正量，单位RPM；不得为空。
 * @return 1表示控制器健康；0表示参数或安全检查失败。
 *
 * P=1.0、I/D=0、死区0.55°、输出限幅±15 RPM。两次计算之间调用时返回
 * 上一次修正量但不增加control_updates，上层据此不得重复提交四轮PWM事务。
 */
uint8_t ChassisHeading_Compute(float *correction_rpm);

/** @brief 将绝对Yaw目标设置为[-180°,180°]内的指定值。 */
uint8_t ChassisHeading_SetTarget(float target_degrees);
/** @brief 在当前理论目标上增加一个相对角度；恒速HOLD测试不调用。 */
uint8_t ChassisHeading_AddTarget(float delta_degrees);
/** @brief 返回“目标Yaw-当前Yaw”的最短有符号误差，单位度。 */
float ChassisHeading_GetErrorDegrees(void);
/** @brief 保留给旧复合画像的主动转向入口；恒速HOLD画像禁止调用。 */
uint8_t ChassisHeading_StartTurn(float delta_degrees);
/** @brief 查询旧主动转向流程是否稳定完成；恒速HOLD画像禁止调用。 */
uint8_t ChassisHeading_IsTurnComplete(void);
/** @brief 返回旧路线坐标变换使用的当前方向差，单位度。 */
float ChassisHeading_DirectionDeltaDegrees(void);

/**
 * @brief 清零PID历史和旋转输出，但保留已经锁定的理论航向目标。
 * @note 正常停车与全部故障路径调用；不会直接操作电机硬件。
 */
void ChassisHeading_EndMotion(void);
#endif
