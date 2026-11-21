#include "chassis_distance.h"

#include "chassis_config.h"
#include "motor_units.h"

#include <limits.h>
#include <math.h>

#define CHASSIS_PI_F 3.14159265358979323846f

/**
 * @brief 检查物理距离标定轴是否有效。
 * @param axis 待检查的物理X/Y轴。
 * @return 1表示有效，0表示无效。
 */
static uint8_t ChassisDistance_IsValidAxis(ChassisDistanceAxis axis)
{
    return (uint8_t)(axis == CHASSIS_DISTANCE_AXIS_X ||
                     axis == CHASSIS_DISTANCE_AXIS_Y);
}

/**
 * @brief 根据轮径、传动比和每圈STEP数计算理论STEP/cm。
 * @return 理论STEP/cm；配置无效时返回0。
 */
float ChassisDistance_BaseStepsPerCm(void)
{
    const float wheel_circumference_cm =
        CHASSIS_PI_F * CHASSIS_WHEEL_DIAMETER_MM / 10.0f;

    if (!isfinite(wheel_circumference_cm) ||
        !isfinite(CHASSIS_MOTOR_TO_WHEEL_RATIO) ||
        wheel_circumference_cm <= 0.0f ||
        CHASSIS_MOTOR_TO_WHEEL_RATIO <= 0.0f) {
        return 0.0f;
    }

    return ((float)MOTOR_STEPS_PER_REV *
            CHASSIS_MOTOR_TO_WHEEL_RATIO) / wheel_circumference_cm;
}

/**
 * @brief 读取叠加物理X/Y独立修正后的STEP/cm。
 * @param axis 物理标定轴。
 * @return 标定STEP/cm；参数无效时返回0。
 */
float ChassisDistance_StepsPerCm(ChassisDistanceAxis axis)
{
    const float theoretical = ChassisDistance_BaseStepsPerCm();
    float cal;

    if (!ChassisDistance_IsValidAxis(axis) || theoretical <= 0.0f) {
        return 0.0f;
    }

    cal = (axis == CHASSIS_DISTANCE_AXIS_X)
        ? CHASSIS_X_CAL
        : CHASSIS_Y_CAL;
    if (!isfinite(cal) || cal <= 0.0f) {
        return 0.0f;
    }

    return theoretical * cal;
}

/**
 * @brief 将正距离厘米数换算成目标平移STEP数。
 * @param axis 物理标定轴。
 * @param distance_cm 正距离，单位cm。
 * @param target_steps 输出目标STEP数。
 * @return 1表示成功，0表示参数或结果无效。
 */
uint8_t ChassisDistance_CmToSteps(ChassisDistanceAxis axis,
                                 float distance_cm,
                                 uint32_t *target_steps)
{
    const float steps_per_cm = ChassisDistance_StepsPerCm(axis);
    float steps;

    if (target_steps == 0 || !isfinite(distance_cm) ||
        distance_cm <= 0.0f || steps_per_cm <= 0.0f) {
        return 0U;
    }

    steps = distance_cm * steps_per_cm;
    if (!isfinite(steps) || steps < 1.0f ||
        steps > (float)UINT_MAX - 0.5f) {
        return 0U;
    }

    *target_steps = (uint32_t)(steps + 0.5f);
    return 1U;
}

/**
 * @brief 将四轮带符号STEP按内部麦轮平移分量投影。
 * @param projection 内部混控X/Y投影公式。
 * @param wheel_steps 四轮带符号STEP累计值。
 * @return 带方向的投影STEP数；参数无效时返回0。
 */
float ChassisDistance_ProjectSteps(
    ChassisDistanceProjection projection,
    const ChassisWheelSteps *wheel_steps)
{
    float axis_steps;

    if (wheel_steps == 0 ||
        !isfinite(wheel_steps->motor1_steps) ||
        !isfinite(wheel_steps->motor2_steps) ||
        !isfinite(wheel_steps->motor3_steps) ||
        !isfinite(wheel_steps->motor4_steps)) {
        return 0.0f;
    }

    if (projection == CHASSIS_DISTANCE_PROJECTION_MIXER_X) {
        axis_steps = (wheel_steps->motor1_steps -
                      wheel_steps->motor2_steps -
                      wheel_steps->motor3_steps +
                      wheel_steps->motor4_steps) * 0.25f;
    } else if (projection == CHASSIS_DISTANCE_PROJECTION_MIXER_Y) {
        axis_steps = (wheel_steps->motor1_steps +
                      wheel_steps->motor2_steps +
                      wheel_steps->motor3_steps +
                      wheel_steps->motor4_steps) * 0.25f;
    } else {
        return 0.0f;
    }

    return axis_steps;
}

/**
 * @brief 使用指定物理轴标定，把内部脉冲投影换算成厘米。
 * @param axis 选择物理X或Y标定系数。
 * @param projection 选择内部混控脉冲投影公式。
 * @param wheel_steps 四轮带符号STEP累计值。
 * @return 带方向的厘米距离；参数无效时返回0。
 */
float ChassisDistance_StepsToCm(
    ChassisDistanceAxis axis,
    ChassisDistanceProjection projection,
    const ChassisWheelSteps *wheel_steps)
{
    const float steps_per_cm = ChassisDistance_StepsPerCm(axis);

    if (steps_per_cm <= 0.0f) {
        return 0.0f;
    }
    return ChassisDistance_ProjectSteps(
               projection, wheel_steps) / steps_per_cm;
}
