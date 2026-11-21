#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f4xx_hal.h"
#include "motor_units.h"
#include <stdint.h>

/** 四路物理电机的稳定逻辑标识；数组下标不得直接泄漏给调用方。 */
typedef enum
{
  MOTOR_ID_1 = 0,
  MOTOR_ID_2,
  MOTOR_ID_3,
  MOTOR_ID_4,
  MOTOR_ID_COUNT
} MotorId;

/**
 * @brief 初始化四路STEP定时器与DIR/EN GPIO，并保持全部电机失能。
 * @return HAL_OK表示四路均初始化完成；任一路失败均返回HAL_ERROR。
 */
HAL_StatusTypeDef Motor_Init(void);

/**
 * @brief 设置单路电机转速；正负号表示两个相反方向，0 RPM立即失能该路。
 * @param motor 电机标识。
 * @param rpm 目标转速。
 * @return HAL_OK表示命令已提交；参数无效或定时器启动失败返回HAL_ERROR。
 */
HAL_StatusTypeDef Motor_SetRpm(MotorId motor, float rpm);

/**
 * @brief 以同一事务准备并启动四路恒频STEP脉冲，保留供体四轮命令语义。
 * @param motor1_rpm 至 motor4_rpm 四路有符号目标RPM；0表示该路保持失能。
 * @return HAL_OK表示四路均已提交；任一准备或启动失败时全部四路失能并返回HAL_ERROR。
 *
 * @note 本函数是执行器层的恒速入口，只把RPM换算为固定脉冲周期；不生成
 *       加速、减速、路线、航向或持续时间。上层需要改变速度时必须显式再次
 *       调用；上层结束或故障时必须调用Motor_StopAll()。
 * @note 提交顺序固定为“全部预置且保持失能→全部启动PWM→批量使能”；不得
 *       改成逐路边准备边使能，否则四轮起动时刻不再属于同一事务。
 */
HAL_StatusTypeDef Motor_SetFourRpm(float motor1_rpm,
                                   float motor2_rpm,
                                   float motor3_rpm,
                                   float motor4_rpm);

/**
 * @brief 停止并失能指定电机。
 * @param motor 电机标识；无效标识不产生硬件动作。
 */
void Motor_Stop(MotorId motor);

/**
 * @brief 立即停止PWM并失能全部四路电机。
 *
 * 这是初始化失败、控制故障、运动超时和正常结束共享的唯一执行器安全出口。
 * 函数不会安排减速曲线，也不会自动恢复或重新启动。
 */
void Motor_StopAll(void);

/**
 * @brief 清零指定电机由定时器更新中断累计的STEP计数。
 * @param motor 电机标识；无效标识不改变计数。
 */
void Motor_ResetStepCount(MotorId motor);

/**
 * @brief 设置指定电机的自动停机脉冲上限，0表示不按计数自动停机。
 * @param motor 电机标识。
 * @param step_limit STEP上升沿计数上限。
 */
void Motor_SetStepLimit(MotorId motor, uint32_t step_limit);

/**
 * @brief 原子读取指定电机的STEP计数。
 * @param motor 电机标识。
 * @return 当前计数；无效标识返回0。
 */
uint32_t Motor_GetStepCount(MotorId motor);

/**
 * @brief 原子读取带当前命令方向符号的累计STEP计数。
 * @return 正RPM累计为正、负RPM累计为负；无效标识返回0。
 *
 * 该值只表达电机执行层实际提交的DIR符号，不解释车体X/Y方向。
 */
int32_t Motor_GetSignedStepCount(MotorId motor);

#endif /* MOTOR_H */
