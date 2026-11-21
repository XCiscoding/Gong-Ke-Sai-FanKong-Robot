#include "chassis.h"
#include "main.h"
#include "chassis_motor_map.h"
#if CHASSIS_USE_STEP_BACKEND
#include "motor.h"
#else
#include "step_motor.h"
#endif
#include "chassis_heading.h"
#include <math.h>

/**
 * @brief 返回浮点数绝对值，供四轮RPM限幅计算使用。
 * @param value 输入值。
 * @return value的非负绝对值。
 */
static float Chassis_Abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

/**
 * @brief 把路线基础XY、视觉X轴附加量和航向W混合为直脉冲M1—M4轮速。
 *
 * 路线层始终可以给出基础X/Y平移；视觉层只允许贡献delta_vx；IMU方向环只
 * 贡献w。四轮公式来自已采用供体的M1=左前、M2=右前、M3=左后、M4=右后
 * 布局。超限时按同一比例缩放全部轮速，避免单轮裁剪改变运动方向。
 */
uint8_t Chassis_ComputeDirectWheelRpm(float route_vx, float route_vy,
                                      float vision_delta_vx, float heading_w,
                                      ChassisDirectWheelRpm *wheel_rpm)
{
    float x;
    float max_abs;
    float ratio;

    if ((wheel_rpm == 0) || !isfinite(route_vx) || !isfinite(route_vy) ||
        !isfinite(vision_delta_vx) || !isfinite(heading_w)) {
        return 0U;
    }

    x = CHASSIS_DIRECT_X_INPUT_POLARITY * (route_vx + vision_delta_vx);
    wheel_rpm->motor1_rpm = route_vy + x + heading_w;
    wheel_rpm->motor2_rpm = route_vy - x - heading_w;
    wheel_rpm->motor3_rpm = route_vy - x + heading_w;
    wheel_rpm->motor4_rpm = route_vy + x - heading_w;

    max_abs = Chassis_Abs(wheel_rpm->motor1_rpm);
    if (Chassis_Abs(wheel_rpm->motor2_rpm) > max_abs)
        max_abs = Chassis_Abs(wheel_rpm->motor2_rpm);
    if (Chassis_Abs(wheel_rpm->motor3_rpm) > max_abs)
        max_abs = Chassis_Abs(wheel_rpm->motor3_rpm);
    if (Chassis_Abs(wheel_rpm->motor4_rpm) > max_abs)
        max_abs = Chassis_Abs(wheel_rpm->motor4_rpm);

    if (max_abs > CHASSIS_DIRECT_MAX_WHEEL_RPM) {
        ratio = CHASSIS_DIRECT_MAX_WHEEL_RPM / max_abs;
        wheel_rpm->motor1_rpm *= ratio;
        wheel_rpm->motor2_rpm *= ratio;
        wheel_rpm->motor3_rpm *= ratio;
        wheel_rpm->motor4_rpm *= ratio;
    }
    return 1U;
}

/**
 * @brief Stop the selected backend and converge every caller on a safe output state.
 */
void Chassis_Stop(void)
{
#if CHASSIS_USE_STEP_BACKEND
    Motor_StopAll();
#else
    StepMotor_ClearTargets();
#endif
}

/**
 * @brief Initialize chassis-owned state without creating a motion command.
 */
void Chassis_Init(void)
{
    Chassis_Stop();
}

/**
 * @brief Mix and immediately submit one constant-speed chassis update.
 *
 * This function intentionally owns no ramp, delay or duration.  The profile
 * state machine must call it only after a fresh heading update and must call
 * Chassis_Stop() on completion or any fault.
 */
uint8_t Chassis_ApplyCommand(float route_vx, float route_vy,
                             float vision_delta_vx, float heading_w)
{
#if CHASSIS_USE_STEP_BACKEND
    ChassisDirectWheelRpm wheel_rpm;

    if (!Chassis_ComputeDirectWheelRpm(route_vx, route_vy,
                                       vision_delta_vx, heading_w,
                                       &wheel_rpm)) {
        Chassis_Stop();
        return 0U;
    }
    if (Motor_SetFourRpm(wheel_rpm.motor1_rpm,
                         wheel_rpm.motor2_rpm,
                         wheel_rpm.motor3_rpm,
                         wheel_rpm.motor4_rpm) != HAL_OK) {
        Chassis_Stop();
        return 0U;
    }
    return 1U;
#else
    return Chassis_InverseMotionControl(route_vx + vision_delta_vx,
                                        route_vy, heading_w);
#endif
}

/**
 * @brief Submit the donor-compatible constant-speed path without vision input.
 *
 * Keeping this small wrapper makes the selected motion semantics explicit at
 * the application call site: constant translation plus heading correction,
 * never the retained sinusoidal acceleration path.
 */
uint8_t Chassis_MoveConstant(float vx, float vy, float heading_w)
{
    return Chassis_ApplyCommand(vx, vy, 0.0f, heading_w);
}

/**
 * @brief Preserve the historical inverse-motion entry while selecting the configured backend.
 */
uint8_t Chassis_InverseMotionControl(float v_x, float v_y, float w)
{
#if CHASSIS_USE_STEP_BACKEND
    return Chassis_ApplyCommand(v_x, v_y, 0.0f, w);
#else
    const float SignX = CHASSIS_WHEEL_PATTERN_X ? -1.0f : 1.0f;
    const float reference_vx = CHASSIS_X_INPUT_POLARITY * v_x;
    const float values[4] = {
        roundf(SignX * reference_vx + v_y - SignX * w), /* left front */
        roundf(SignX * reference_vx - v_y - SignX * w), /* right front */
        roundf(-SignX * reference_vx + v_y - SignX * w), /* left rear */
        roundf(-SignX * reference_vx - v_y - SignX * w) /* right rear */
    };
    const uint8_t ids[4] = {
        CHASSIS_MOTOR_LEFT_FRONT_ID, CHASSIS_MOTOR_RIGHT_FRONT_ID,
        CHASSIS_MOTOR_LEFT_REAR_ID, CHASSIS_MOTOR_RIGHT_REAR_ID
    };
    uint8_t i;
    for (i = 0U; i < 4U; i++) {
        if (!isfinite(values[i]) || values[i] < -3000.0f || values[i] > 3000.0f) {
            StepMotor_ClearTargets();
            return 0U;
        }
    }
    for (i = 0U; i < 4U; i++)
        if (!StepMotor_SetTargetSpeed(ids[i], (int32_t)values[i])) return 0U;
    return 1U;
#endif
}
/**
 * @brief Rotate a route-frame command into the body frame and apply yaw HOLD.
 *
 * This legacy-compatible helper is not used by the constant-speed proof
 * profile, which computes heading once and calls Chassis_MoveConstant().
 */
uint8_t Chassis_SetSpeed(float vx, float vy, float theta)
{
    float w, vx_body, vy_body, sine, cosine;
    if (!isfinite(vx) || !isfinite(vy) || !isfinite(theta) ||
        !ChassisHeading_Compute(&w)) {
        Chassis_Stop(); return 0U;
    }
    theta *= 0.01745329252f;
    cosine = cosf(theta);
    sine = sinf(theta);
    vx_body = vx * cosine + vy * sine;
    vy_body = -vx * sine + vy * cosine;
    return Chassis_InverseMotionControl(vx_body, vy_body, w);
}
/**
 * @brief Convert one basic motion enum into a fixed body-frame command.
 */
uint8_t Chassis_PrepareMotion(ChassisMotion motion, uint16_t speed_rpm)
{
    const float speed = (float)speed_rpm;
    switch (motion) {
    case CHASSIS_FORWARD: return Chassis_InverseMotionControl(0.0f, speed, 0.0f);
    case CHASSIS_BACKWARD: return Chassis_InverseMotionControl(0.0f, -speed, 0.0f);
    case CHASSIS_LEFT: return Chassis_InverseMotionControl(-speed, 0.0f, 0.0f);
    case CHASSIS_RIGHT: return Chassis_InverseMotionControl(speed, 0.0f, 0.0f);
    case CHASSIS_ROTATE_LEFT: return Chassis_InverseMotionControl(0.0f, 0.0f, speed);
    case CHASSIS_ROTATE_RIGHT: return Chassis_InverseMotionControl(0.0f, 0.0f, -speed);
    default: Chassis_Stop(); return 0U;
    }
}
