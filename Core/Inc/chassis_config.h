#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/*
 * 底盘常用标定参数集中入口。
 * CAL是calibration（校准）的缩写；1.0表示不修正。每轮调试只修改一种
 * 参数：轮径/传动结构、X或Y距离校准、路线速度上限。
 */
#define CHASSIS_WHEEL_DIAMETER_MM          70.0f
#define CHASSIS_MOTOR_TO_WHEEL_RATIO        1.0f
#define CHASSIS_X_CAL                        0.922f
#define CHASSIS_Y_CAL                        0.90f
#define CHASSIS_ROUTE_MAX_SPEED_RPM        120.0f

/* 厘米路线安全时序；具体X/Y速度和距离由任务层函数调用给出。 */
#define CHASSIS_ROUTE_STABILIZE_MS               3000U
#define CHASSIS_ROUTE_START_TIMEOUT_MS            5000U
#define CHASSIS_ROUTE_RUN_TIMEOUT_MARGIN_MS       2000U
/* Every X/Y route segment starts from this fraction and reaches its requested
 * speed smoothly.  This is route orchestration, not a motor-layer ramp. */
#define CHASSIS_ROUTE_START_RAMP_MS                 300U
#define CHASSIS_ROUTE_START_RAMP_INITIAL_RATIO     0.35f

#endif /* CHASSIS_CONFIG_H */
