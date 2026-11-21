#ifndef CHASSIS_ROUTE_H
#define CHASSIS_ROUTE_H

#include "chassis_distance.h"

#include <stdint.h>

/** 用户坐标中的四个平移方向；不使用前后左右。 */
typedef enum {
    CHASSIS_ROUTE_X_POSITIVE = 0,
    CHASSIS_ROUTE_X_NEGATIVE,
    CHASSIS_ROUTE_Y_POSITIVE,
    CHASSIS_ROUTE_Y_NEGATIVE
} ChassisRouteDirection;

/** 用户可读的一段路线定义。 */
typedef struct {
    ChassisRouteDirection direction;
    float distance_cm;
    float speed_rpm;
} ChassisRouteSegment;

/** 路线层解析后交给底盘控制层的内部命令，不包含HAL或电机引脚。 */
typedef struct {
    float internal_vx_rpm;
    float internal_vy_rpm;
    ChassisDistanceAxis distance_axis;
    ChassisDistanceProjection step_projection;
    int8_t distance_sign;
    uint32_t target_steps;
} ChassisRouteResolvedSegment;

typedef enum {
    CHASSIS_ROUTE_RESOLVE_OK = 0,
    CHASSIS_ROUTE_RESOLVE_INVALID_ARGUMENT,
    CHASSIS_ROUTE_RESOLVE_INVALID_DISTANCE,
    CHASSIS_ROUTE_RESOLVE_INVALID_SPEED,
    CHASSIS_ROUTE_RESOLVE_INVALID_DIRECTION
} ChassisRouteResolveStatus;

/**
 * @brief 把用户坐标中的厘米路线段解析为内部vx/vy与目标平移脉冲。
 *
 * 方向只决定内部XY映射；实体方向验收属于项目证据，不能成为方向专用
 * 固件权限门。本函数不启动电机、不读取IMU，也不决定状态机何时切换下一段。
 */
ChassisRouteResolveStatus ChassisRoute_ResolveSegment(
    const ChassisRouteSegment *segment,
    ChassisRouteResolvedSegment *resolved);

#endif /* CHASSIS_ROUTE_H */
