#include "chassis_config.h"
#include "chassis_route.h"

#include <assert.h>
#include <math.h>

/** @brief 在给定绝对误差内比较两个浮点数。 */
static int NearlyEqual(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

/** @brief 验证物理方向门、内部映射和厘米目标解析合同。 */
int main(void)
{
    ChassisRouteSegment segment = {
        CHASSIS_ROUTE_X_POSITIVE,
        10.0f,
        50.0f,
        0.0f
    };
    ChassisRouteResolvedSegment resolved;

    assert(ChassisRoute_ResolveSegment(&segment, &resolved) ==
           CHASSIS_ROUTE_RESOLVE_OK);
    assert(NearlyEqual(resolved.internal_vx_rpm, 0.0f, 0.001f));
    assert(NearlyEqual(resolved.internal_vy_rpm, 50.0f, 0.001f));
    assert(resolved.distance_axis == CHASSIS_DISTANCE_AXIS_X);
    assert(resolved.step_projection == CHASSIS_DISTANCE_PROJECTION_MIXER_Y);
    assert(resolved.distance_sign == 1);
    assert(resolved.target_steps == 1455U);

    segment.direction = CHASSIS_ROUTE_X_NEGATIVE;
    assert(ChassisRoute_ResolveSegment(&segment, &resolved) ==
           CHASSIS_ROUTE_RESOLVE_DIRECTION_UNVERIFIED);
    segment.direction = CHASSIS_ROUTE_Y_POSITIVE;
    assert(ChassisRoute_ResolveSegment(&segment, &resolved) ==
           CHASSIS_ROUTE_RESOLVE_DIRECTION_UNVERIFIED);
    segment.direction = CHASSIS_ROUTE_Y_NEGATIVE;
    assert(ChassisRoute_ResolveSegment(&segment, &resolved) ==
           CHASSIS_ROUTE_RESOLVE_DIRECTION_UNVERIFIED);

    segment.direction = CHASSIS_ROUTE_X_POSITIVE;
    segment.distance_cm = 0.0f;
    assert(ChassisRoute_ResolveSegment(&segment, &resolved) ==
           CHASSIS_ROUTE_RESOLVE_INVALID_DISTANCE);
    segment.distance_cm = 10.0f;
    segment.speed_rpm = 0.0f;
    assert(ChassisRoute_ResolveSegment(&segment, &resolved) ==
           CHASSIS_ROUTE_RESOLVE_INVALID_SPEED);
    segment.speed_rpm = CHASSIS_ROUTE_MAX_SPEED_RPM + 1.0f;
    assert(ChassisRoute_ResolveSegment(&segment, &resolved) ==
           CHASSIS_ROUTE_RESOLVE_INVALID_SPEED);

    return 0;
}
