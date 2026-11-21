#include "chassis_route.h"

#include "chassis_config.h"

#include <math.h>

/**
 * @brief 查询某个用户物理方向是否已有实车证据并被显式放行。
 * @param direction 用户坐标中的X/Y正负方向。
 * @return 1表示已验证，0表示未验证或参数无效。
 */
static uint8_t ChassisRoute_IsDirectionVerified(ChassisRouteDirection direction)
{
    switch (direction) {
    case CHASSIS_ROUTE_X_POSITIVE:
        return CHASSIS_PHYSICAL_X_POSITIVE_VERIFIED;
    case CHASSIS_ROUTE_X_NEGATIVE:
        return CHASSIS_PHYSICAL_X_NEGATIVE_VERIFIED;
    case CHASSIS_ROUTE_Y_POSITIVE:
        return CHASSIS_PHYSICAL_Y_POSITIVE_VERIFIED;
    case CHASSIS_ROUTE_Y_NEGATIVE:
        return CHASSIS_PHYSICAL_Y_NEGATIVE_VERIFIED;
    default:
        return 0U;
    }
}

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
    if (!isfinite(segment->target_heading_degrees)) {
        return CHASSIS_ROUTE_RESOLVE_INVALID_HEADING;
    }
    if (!ChassisRoute_IsDirectionVerified(segment->direction)) {
        return CHASSIS_ROUTE_RESOLVE_DIRECTION_UNVERIFIED;
    }

    resolved->internal_vx_rpm = 0.0f;
    resolved->internal_vy_rpm = 0.0f;
    resolved->target_heading_degrees = segment->target_heading_degrees;

    switch (segment->direction) {
    case CHASSIS_ROUTE_X_POSITIVE:
        /* 最新实物证据：内部vy+对应用户坐标物理X+。 */
        axis = CHASSIS_DISTANCE_AXIS_X;
        projection = CHASSIS_DISTANCE_PROJECTION_MIXER_Y;
        sign = 1;
        resolved->internal_vy_rpm = segment->speed_rpm;
        break;
    case CHASSIS_ROUTE_X_NEGATIVE:
        axis = CHASSIS_DISTANCE_AXIS_X;
        projection = CHASSIS_DISTANCE_PROJECTION_MIXER_Y;
        sign = -1;
        resolved->internal_vy_rpm = -segment->speed_rpm;
        break;
    case CHASSIS_ROUTE_Y_POSITIVE:
        axis = CHASSIS_DISTANCE_AXIS_Y;
        projection = CHASSIS_DISTANCE_PROJECTION_MIXER_X;
        sign = 1;
        resolved->internal_vx_rpm = segment->speed_rpm;
        break;
    case CHASSIS_ROUTE_Y_NEGATIVE:
        axis = CHASSIS_DISTANCE_AXIS_Y;
        projection = CHASSIS_DISTANCE_PROJECTION_MIXER_X;
        sign = -1;
        resolved->internal_vx_rpm = -segment->speed_rpm;
        break;
    default:
        return CHASSIS_ROUTE_RESOLVE_DIRECTION_UNVERIFIED;
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
