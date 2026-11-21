#include "chassis_route.h"

#include "chassis_config.h"

#include <math.h>

/**
 * @brief 将用户可读路线段解析为内部平移命令、投影规则和目标STEP数。
 * @param segment 输入的物理方向、厘米、RPM和目标航向。
 * @param resolved 输出给运行编排层的内部命令。
 * @return 精确的解析状态；只有OK才允许提交电机命令。
 */
ChassisRouteResolveStatus ChassisRoute_ResolveSegment(
    const ChassisRouteSegment *segment,
    ChassisRouteResolvedSegment *resolved)
{
    ChassisDistanceAxis axis;
    ChassisDistanceProjection projection;
    int8_t sign;

    if (segment == 0 || resolved == 0) {
        return CHASSIS_ROUTE_RESOLVE_INVALID_ARGUMENT;
    }
    if (!isfinite(segment->distance_cm) || segment->distance_cm <= 0.0f) {
        return CHASSIS_ROUTE_RESOLVE_INVALID_DISTANCE;
    }
    if (!isfinite(segment->speed_rpm) || segment->speed_rpm <= 0.0f ||
        segment->speed_rpm > CHASSIS_ROUTE_MAX_SPEED_RPM) {
        return CHASSIS_ROUTE_RESOLVE_INVALID_SPEED;
    }
    resolved->internal_vx_rpm = 0.0f;
    resolved->internal_vy_rpm = 0.0f;

    switch (segment->direction) {
    case CHASSIS_ROUTE_X_POSITIVE:
        axis = CHASSIS_DISTANCE_AXIS_X;
        projection = CHASSIS_DISTANCE_PROJECTION_MIXER_X;
        sign = 1;
        resolved->internal_vx_rpm = segment->speed_rpm;
        break;
    case CHASSIS_ROUTE_X_NEGATIVE:
        axis = CHASSIS_DISTANCE_AXIS_X;
        projection = CHASSIS_DISTANCE_PROJECTION_MIXER_X;
        sign = -1;
        resolved->internal_vx_rpm = -segment->speed_rpm;
        break;
    case CHASSIS_ROUTE_Y_POSITIVE:
        axis = CHASSIS_DISTANCE_AXIS_Y;
        projection = CHASSIS_DISTANCE_PROJECTION_MIXER_Y;
        sign = 1;
        resolved->internal_vy_rpm = segment->speed_rpm;
        break;
    case CHASSIS_ROUTE_Y_NEGATIVE:
        axis = CHASSIS_DISTANCE_AXIS_Y;
        projection = CHASSIS_DISTANCE_PROJECTION_MIXER_Y;
        sign = -1;
        resolved->internal_vy_rpm = -segment->speed_rpm;
        break;
    default:
        return CHASSIS_ROUTE_RESOLVE_INVALID_DIRECTION;
    }

    if (!ChassisDistance_CmToSteps(axis, segment->distance_cm,
                                   &resolved->target_steps)) {
        return CHASSIS_ROUTE_RESOLVE_INVALID_DISTANCE;
    }

    resolved->distance_axis = axis;
    resolved->step_projection = projection;
    resolved->distance_sign = sign;
    return CHASSIS_ROUTE_RESOLVE_OK;
}
