#include "chassis_config.h"
#include "chassis_distance.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

/** @brief 在给定绝对误差内比较两个浮点数。 */
static int NearlyEqual(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

/** @brief 验证70mm理论值、分轴标定和四轮脉冲投影合同。 */
int main(void)
{
    const float theoretical = ChassisDistance_BaseStepsPerCm();
    const float x_steps_per_cm = theoretical * CHASSIS_X_CAL;
    const float y_steps_per_cm = theoretical * CHASSIS_Y_CAL;
    ChassisWheelSteps wheel_steps;
    uint32_t target_steps = 0U;

    assert(NearlyEqual(theoretical, 145.513f, 0.01f));
    assert(NearlyEqual(ChassisDistance_StepsPerCm(CHASSIS_DISTANCE_AXIS_X),
                       x_steps_per_cm, 0.001f));
    assert(NearlyEqual(ChassisDistance_StepsPerCm(CHASSIS_DISTANCE_AXIS_Y),
                       y_steps_per_cm, 0.001f));

    assert(ChassisDistance_CmToSteps(CHASSIS_DISTANCE_AXIS_X,
                                     10.0f, &target_steps) == 1U);
    assert(target_steps == (uint32_t)(x_steps_per_cm * 10.0f + 0.5f));
    assert(ChassisDistance_CmToSteps(CHASSIS_DISTANCE_AXIS_Y,
                                     65.0f, &target_steps) == 1U);
    assert(target_steps == (uint32_t)(y_steps_per_cm * 65.0f + 0.5f));
    assert(ChassisDistance_CmToSteps(CHASSIS_DISTANCE_AXIS_X,
                                     0.0f, &target_steps) == 0U);
    assert(ChassisDistance_CmToSteps((ChassisDistanceAxis)99,
                                     10.0f, &target_steps) == 0U);
    assert(ChassisDistance_CmToSteps(CHASSIS_DISTANCE_AXIS_X,
                                     10.0f, 0) == 0U);

    /* 纯X：M1/M4为正，M2/M3为负。 */
    wheel_steps = (ChassisWheelSteps){
        x_steps_per_cm * 10.0f, -x_steps_per_cm * 10.0f,
        -x_steps_per_cm * 10.0f, x_steps_per_cm * 10.0f
    };
    assert(NearlyEqual(ChassisDistance_StepsToCm(
                           CHASSIS_DISTANCE_AXIS_X,
                           CHASSIS_DISTANCE_PROJECTION_MIXER_X, &wheel_steps),
                       10.0f, 0.01f));
    assert(NearlyEqual(ChassisDistance_StepsToCm(
                           CHASSIS_DISTANCE_AXIS_Y,
                           CHASSIS_DISTANCE_PROJECTION_MIXER_Y, &wheel_steps),
                       0.0f, 0.01f));

    /* 纯Y：四轮同号。 */
    wheel_steps = (ChassisWheelSteps){
        y_steps_per_cm * 20.0f, y_steps_per_cm * 20.0f,
        y_steps_per_cm * 20.0f, y_steps_per_cm * 20.0f
    };
    assert(NearlyEqual(ChassisDistance_StepsToCm(
                           CHASSIS_DISTANCE_AXIS_Y,
                           CHASSIS_DISTANCE_PROJECTION_MIXER_Y, &wheel_steps),
                       20.0f, 0.01f));
    /* 当前实车物理Y映射到历史混控Y，并使用物理Y标定系数。 */
    assert(NearlyEqual(ChassisDistance_StepsToCm(
                           CHASSIS_DISTANCE_AXIS_Y,
                           CHASSIS_DISTANCE_PROJECTION_MIXER_Y, &wheel_steps),
                       20.0f, 0.01f));

    /* 纯W：旋转脉冲在X/Y投影中都必须相消。 */
    wheel_steps = (ChassisWheelSteps){1000.0f, -1000.0f,
                                      1000.0f, -1000.0f};
    assert(NearlyEqual(ChassisDistance_StepsToCm(
                           CHASSIS_DISTANCE_AXIS_X,
                           CHASSIS_DISTANCE_PROJECTION_MIXER_X, &wheel_steps),
                       0.0f, 0.001f));
    assert(NearlyEqual(ChassisDistance_StepsToCm(
                           CHASSIS_DISTANCE_AXIS_Y,
                           CHASSIS_DISTANCE_PROJECTION_MIXER_Y, &wheel_steps),
                       0.0f, 0.001f));

    return 0;
}
