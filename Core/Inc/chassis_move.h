#ifndef CHASSIS_MOVE_H
#define CHASSIS_MOVE_H

#include <stdint.h>

/**
 * @brief 启动一段单轴厘米移动。
 * @param x_rpm X轴速度，正数向X+，负数向X-，0表示不沿X移动。
 * @param y_rpm Y轴速度，正数向Y+，负数向Y-，0表示不沿Y移动。
 * @param distance_cm 正整数距离，单位cm。
 * @return 1表示命令已交给路线运行器；0表示参数、方向验证或速度范围无效。
 *
 * 当前只接受X、Y中恰好一个非零；调用者不接触STEP换算、四轮混控或HAL。
 */
uint8_t Chassis_MoveCm(int16_t x_rpm, int16_t y_rpm,
                       uint16_t distance_cm);

/**
 * @brief 推进当前厘米移动，不创建新任务。
 *
 * 主循环每轮先更新IMU，再调用本函数。距离完成、故障与超时均由内部路线
 * 运行器统一停车；本函数不会自动重复上一条命令。
 */
void Chassis_Service(void);

#endif /* CHASSIS_MOVE_H */
