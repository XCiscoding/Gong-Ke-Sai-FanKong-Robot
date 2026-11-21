#ifndef CHASSIS_DISTANCE_H
#define CHASSIS_DISTANCE_H

#include <stdint.h>

/** 车体平移轴；方向由上层命令的正负号表达。 */
typedef enum {
    CHASSIS_DISTANCE_AXIS_X = 0,
    CHASSIS_DISTANCE_AXIS_Y
} ChassisDistanceAxis;

/** 四轮脉冲按底盘混控的哪一个内部平移分量投影。 */
typedef enum {
    CHASSIS_DISTANCE_PROJECTION_MIXER_X = 0,
    CHASSIS_DISTANCE_PROJECTION_MIXER_Y
} ChassisDistanceProjection;

/**
 * 四轮带方向的STEP累计值。
 * 正负号来自当时的轮速方向，不能把电机层的无符号计数直接填入。
 */
typedef struct {
    float motor1_steps;
    float motor2_steps;
    float motor3_steps;
    float motor4_steps;
} ChassisWheelSteps;

/** @return 只使用轮径、传动比和细分参数得到的理论STEP/cm。 */
float ChassisDistance_BaseStepsPerCm(void);

/** @return 叠加X或Y独立实车修正系数后的STEP/cm；参数无效返回0。 */
float ChassisDistance_StepsPerCm(ChassisDistanceAxis axis);

/**
 * @brief 把正距离厘米数换算为目标平移STEP数。
 * @return 1表示换算成功；0表示参数无效或结果超出uint32_t。
 */
uint8_t ChassisDistance_CmToSteps(ChassisDistanceAxis axis,
                                 float distance_cm,
                                 uint32_t *target_steps);

/**
 * @brief 从四轮带方向STEP累计值中投影出一个内部平移分量。
 * @return 带方向的投影STEP数；参数无效返回0。
 */
float ChassisDistance_ProjectSteps(
    ChassisDistanceProjection projection,
    const ChassisWheelSteps *wheel_steps);

/**
 * @brief 用物理轴标定系数把指定混控投影换算为平移距离。
 * @return 带方向的厘米值；参数无效返回0。
 *
 * 物理轴决定使用X或Y标定系数，projection决定四轮怎样相加减。两者分开，
 * 因为当前实车“物理X+”映射到历史混控的vy+，不能把坐标命名和混控公式混为一层。
 */
float ChassisDistance_StepsToCm(
    ChassisDistanceAxis axis,
    ChassisDistanceProjection projection,
    const ChassisWheelSteps *wheel_steps);

#endif /* CHASSIS_DISTANCE_H */
