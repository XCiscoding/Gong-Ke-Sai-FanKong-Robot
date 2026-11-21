#include "chassis_route_runner.h"

#include "chassis.h"
#include "chassis_config.h"
#include "chassis_distance.h"
#include "chassis_heading.h"
#include "imu_ms901m.h"
#include "motor.h"
#include "motor_units.h"

#include <math.h>

volatile ChassisRouteRunnerDiagnostics chassis_route_runner_diag;

static ChassisRouteSegment route_segment;
static ChassisRouteResolvedSegment resolved_segment;
static uint8_t route_segment_present;
static uint32_t last_applied_control_update;

/**
 * @brief 统一停止底盘、结束航向控制并锁存首个停止原因。
 * @param reason 正常完成或首个失败原因。
 */
static void ChassisRouteRunner_Stop(ChassisRouteStopReason reason)
{
    Chassis_Stop();
    ChassisHeading_EndMotion();
    chassis_route_runner_diag.last_heading_w_rpm = 0.0f;
    chassis_route_runner_diag.stop_reason = (uint32_t)reason;
    chassis_route_runner_diag.completed =
        (reason == CHASSIS_ROUTE_STOP_COMPLETE) ? 1U : 0U;
    chassis_route_runner_diag.state = CHASSIS_ROUTE_RUNNER_STOP_LATCHED;
    chassis_route_runner_diag.state_started_ms = HAL_GetTick();
}

/**
 * @brief 根据目标STEP数和基础RPM估算无滑移理论运行时间。
 * @return 理论运行时间，单位毫秒；参数无效时返回0。
 */
static uint32_t ChassisRouteRunner_ExpectedRunMs(void)
{
    float duration_ms;

    if (route_segment.speed_rpm <= 0.0f) {
        return 0U;
    }
    duration_ms = ((float)resolved_segment.target_steps * 60000.0f) /
                  ((float)MOTOR_STEPS_PER_REV * route_segment.speed_rpm);
    if (!isfinite(duration_ms) || duration_ms < 1.0f ||
        duration_ms > 4294967000.0f) {
        return 0U;
    }
    duration_ms += (float)CHASSIS_ROUTE_START_RAMP_MS;
    if (duration_ms > 4294967000.0f) {
        return 0U;
    }
    return (uint32_t)(duration_ms + 0.5f);
}

/** @brief 返回当前段的统一平移/航向比例，不更改目标STEP或坐标语义。 */
static float ChassisRouteRunner_StartRampScale(uint32_t now)
{
    float elapsed_ratio;

    if (chassis_route_runner_diag.state != CHASSIS_ROUTE_RUNNER_START_RAMP ||
        CHASSIS_ROUTE_START_RAMP_MS == 0U) {
        return 1.0f;
    }
    if ((uint32_t)(now - chassis_route_runner_diag.ramp_started_ms) >=
        CHASSIS_ROUTE_START_RAMP_MS) {
        return 1.0f;
    }
    elapsed_ratio = (float)(now - chassis_route_runner_diag.ramp_started_ms) /
                    (float)CHASSIS_ROUTE_START_RAMP_MS;
    return CHASSIS_ROUTE_START_RAMP_INITIAL_RATIO +
           (1.0f - CHASSIS_ROUTE_START_RAMP_INITIAL_RATIO) * elapsed_ratio;
}

/** @brief 对路线基础XY和航向修正使用同一比例提交，避免斜坡内改变运动向量。 */
static uint8_t ChassisRouteRunner_Submit(float heading_w, float speed_scale)
{
    return Chassis_MoveConstant(resolved_segment.internal_vx_rpm * speed_scale,
                                resolved_segment.internal_vy_rpm * speed_scale,
                                heading_w * speed_scale);
}

/**
 * @brief 读取四轮带符号STEP并更新物理路线进度诊断。
 * @return 按目标方向归一化后的平移STEP数。
 */
static float ChassisRouteRunner_UpdateProgress(void)
{
    const ChassisWheelSteps wheel_steps = {
        (float)Motor_GetSignedStepCount(MOTOR_ID_1),
        (float)Motor_GetSignedStepCount(MOTOR_ID_2),
        (float)Motor_GetSignedStepCount(MOTOR_ID_3),
        (float)Motor_GetSignedStepCount(MOTOR_ID_4)
    };
    float projected_steps = ChassisDistance_ProjectSteps(
        resolved_segment.step_projection, &wheel_steps);
    float directed_steps = (float)resolved_segment.distance_sign * projected_steps;
    float steps_per_cm = ChassisDistance_StepsPerCm(resolved_segment.distance_axis);

    chassis_route_runner_diag.progress_steps =
        (int32_t)lroundf(directed_steps);
    chassis_route_runner_diag.progress_cm =
        (steps_per_cm > 0.0f) ? directed_steps / steps_per_cm : 0.0f;
    return directed_steps;
}

/**
 * @brief 对已解析路线锁定起步航向、计数清零并提交唯一一次启动命令。
 *
 * 任一前置条件失败都调用统一停车出口，成功后进入RUN_ONCE。
 */
static void ChassisRouteRunner_Start(void)
{
    uint32_t index;
    float heading_w = 0.0f;
    if (ChassisHeading_ArmCurrent() == 0U) {
        ChassisRouteRunner_Stop(CHASSIS_ROUTE_STOP_HEADING_ARM);
        return;
    }
    /* 当前厘米直行段保持起步瞬间Yaw；不得用段内的历史0度字段覆盖它。 */
    if (ChassisHeading_Compute(&heading_w) == 0U) {
        ChassisRouteRunner_Stop(CHASSIS_ROUTE_STOP_HEADING_FAULT);
        return;
    }

    for (index = 0U; index < (uint32_t)MOTOR_ID_COUNT; ++index) {
        Motor_ResetStepCount((MotorId)index);
        Motor_SetStepLimit((MotorId)index, 0U);
    }
    chassis_route_runner_diag.target_steps = resolved_segment.target_steps;
    chassis_route_runner_diag.expected_run_ms =
        ChassisRouteRunner_ExpectedRunMs();
    chassis_route_runner_diag.run_started_ms = HAL_GetTick();
    chassis_route_runner_diag.ramp_started_ms = chassis_route_runner_diag.run_started_ms;
    chassis_route_runner_diag.state_started_ms = chassis_route_runner_diag.run_started_ms;
    chassis_route_runner_diag.state = CHASSIS_ROUTE_RUNNER_START_RAMP;
    chassis_route_runner_diag.last_base_speed_scale =
        ChassisRouteRunner_StartRampScale(chassis_route_runner_diag.run_started_ms);
    if (ChassisRouteRunner_Submit(heading_w,
                                  chassis_route_runner_diag.last_base_speed_scale) == 0U) {
        ChassisRouteRunner_Stop(CHASSIS_ROUTE_STOP_MOTOR_SUBMIT);
        return;
    }
    chassis_route_runner_diag.last_heading_w_rpm =
        heading_w * chassis_route_runner_diag.last_base_speed_scale;
    chassis_route_runner_diag.command_submissions = 1U;
    last_applied_control_update = chassis_heading_diag.control_updates;
}

/**
 * @brief 校验并复制单段路线配置，在四轮全停时进入BOOT_SAFE。
 * @param segment 待运行的单段路线。
 */
uint8_t ChassisRouteRunner_Init(const ChassisRouteSegment *segment)
{
    const ChassisRouteRunnerDiagnostics initial = {0};
    ChassisRouteResolveStatus status;

    if (segment == 0 ||
        (route_segment_present != 0U &&
         chassis_route_runner_diag.state != CHASSIS_ROUTE_RUNNER_STOP_LATCHED)) {
        return 0U;
    }
    status = ChassisRoute_ResolveSegment(segment, &resolved_segment);
    if (status != CHASSIS_ROUTE_RESOLVE_OK) {
        return 0U;
    }

    Chassis_Stop();
    ChassisHeading_EndMotion();
    chassis_route_runner_diag = initial;
    chassis_route_runner_diag.state = CHASSIS_ROUTE_RUNNER_BOOT_SAFE;
    chassis_route_runner_diag.state_started_ms = HAL_GetTick();
    route_segment_present = 1U;
    route_segment = *segment;
    chassis_route_runner_diag.resolve_status = (uint32_t)status;
    chassis_route_runner_diag.target_distance_cm = segment->distance_cm;
    last_applied_control_update = 0U;
    return 1U;
}

/**
 * @brief 非阻塞推进一次厘米路线安全状态机。
 *
 * 调用方每轮应先更新IMU；本函数负责稳定、单次运行、距离完成、超时、
 * 航向故障和终态停车，不会从STOP_LATCHED自动重新启动。
 */
void ChassisRouteRunner_Service(void)
{
    uint32_t now = HAL_GetTick();
    float heading_w = 0.0f;
    float directed_steps;

    switch ((ChassisRouteRunnerState)chassis_route_runner_diag.state) {
    case CHASSIS_ROUTE_RUNNER_BOOT_SAFE:
        /* 上电第一拍只确认全停，再进入传感器稳定窗口。 */
        Chassis_Stop();
        if (route_segment_present == 0U) {
            ChassisRouteRunner_Stop(CHASSIS_ROUTE_STOP_INVALID_ARGUMENT);
            break;
        }
        chassis_route_runner_diag.state_started_ms = now;
        chassis_route_runner_diag.state = CHASSIS_ROUTE_RUNNER_STABILIZE;
        break;

    case CHASSIS_ROUTE_RUNNER_STABILIZE:
        /* 电机持续失能；只等待IMU建零和最短稳定时间。 */
        Chassis_Stop();
        if ((uint32_t)(now - chassis_route_runner_diag.state_started_ms) >=
            CHASSIS_ROUTE_START_TIMEOUT_MS) {
            ChassisRouteRunner_Stop(CHASSIS_ROUTE_STOP_IMU_NOT_READY);
            break;
        }
        if ((uint32_t)(now - chassis_route_runner_diag.state_started_ms) <
            CHASSIS_ROUTE_STABILIZE_MS || imu_ms901m_diag.zero_ready == 0U) {
            break;
        }
        ChassisRouteRunner_Start();
        break;

    case CHASSIS_ROUTE_RUNNER_START_RAMP:
    case CHASSIS_ROUTE_RUNNER_RUN_ONCE:
        /* 先判断距离/超时，再消费新航向更新；任一失败都统一停车。 */
        directed_steps = ChassisRouteRunner_UpdateProgress();
        if (directed_steps >= (float)resolved_segment.target_steps) {
            ChassisRouteRunner_Stop(CHASSIS_ROUTE_STOP_COMPLETE);
            break;
        }
        if (chassis_route_runner_diag.expected_run_ms == 0U ||
            (uint32_t)(now - chassis_route_runner_diag.run_started_ms) >=
            chassis_route_runner_diag.expected_run_ms +
            CHASSIS_ROUTE_RUN_TIMEOUT_MARGIN_MS) {
            ChassisRouteRunner_Stop(CHASSIS_ROUTE_STOP_TIMEOUT);
            break;
        }
        if (ChassisHeading_Compute(&heading_w) == 0U) {
            ChassisRouteRunner_Stop(CHASSIS_ROUTE_STOP_HEADING_FAULT);
            break;
        }
        if (chassis_heading_diag.control_updates !=
            last_applied_control_update) {
            const float speed_scale = ChassisRouteRunner_StartRampScale(now);
            if (ChassisRouteRunner_Submit(heading_w, speed_scale) == 0U) {
                ChassisRouteRunner_Stop(CHASSIS_ROUTE_STOP_MOTOR_SUBMIT);
                break;
            }
            chassis_route_runner_diag.last_heading_w_rpm = heading_w * speed_scale;
            chassis_route_runner_diag.last_base_speed_scale = speed_scale;
            chassis_route_runner_diag.command_submissions++;
            last_applied_control_update = chassis_heading_diag.control_updates;
        }
        if (chassis_route_runner_diag.state == CHASSIS_ROUTE_RUNNER_START_RAMP &&
            ChassisRouteRunner_StartRampScale(now) >= 1.0f) {
            chassis_route_runner_diag.state = CHASSIS_ROUTE_RUNNER_RUN_ONCE;
            chassis_route_runner_diag.state_started_ms = now;
        }
        break;

    case CHASSIS_ROUTE_RUNNER_STOP_LATCHED:
    default:
        /* 终态不自动跳回启动态；循环只重复保证四轮全停。 */
        Chassis_Stop();
        break;
    }
}
