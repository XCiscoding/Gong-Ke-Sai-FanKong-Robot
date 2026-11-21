#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/*
 * 底盘常用标定参数集中入口。
 * CAL是calibration（校准）的缩写；1.0表示不修正。每轮调试只修改一种
 * 参数：轮径/传动结构、X或Y距离校准、路线速度上限。
 */
#define CHASSIS_WHEEL_DIAMETER_MM          70.0f
#define CHASSIS_MOTOR_TO_WHEEL_RATIO        1.0f
#define CHASSIS_X_CAL                        1.0f
#define CHASSIS_Y_CAL                        1.0f
#define CHASSIS_ROUTE_MAX_SPEED_RPM        120.0f

/* 厘米路线安全时序；具体X/Y速度和距离由任务层函数调用给出。 */
#define CHASSIS_ROUTE_STABILIZE_MS               3000U
#define CHASSIS_ROUTE_START_TIMEOUT_MS            5000U
#define CHASSIS_ROUTE_RUN_TIMEOUT_MARGIN_MS       2000U

/*
 * 当前新直脉冲后端只实物确认了一个方向：内部vy为正时，车辆按用户坐标
 * 沿物理X+移动。其余方向必须逐项实测后才能改为1，禁止由对称性直接放行。
 */
#define CHASSIS_PHYSICAL_X_POSITIVE_VERIFIED 1U
#define CHASSIS_PHYSICAL_X_NEGATIVE_VERIFIED 0U
#define CHASSIS_PHYSICAL_Y_POSITIVE_VERIFIED 0U
#define CHASSIS_PHYSICAL_Y_NEGATIVE_VERIFIED 0U

#endif /* CHASSIS_CONFIG_H */
