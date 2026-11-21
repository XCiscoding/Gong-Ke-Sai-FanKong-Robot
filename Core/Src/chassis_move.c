#include "chassis_move.h"

#include "chassis_route.h"
#include "chassis_route_runner.h"

/**
 * @brief 把任务层整数X/Y速度和厘米距离转换为一段轴向路线。
 * @param x_rpm X轴有符号速度，单位RPM。
 * @param y_rpm Y轴有符号速度，单位RPM。
 * @param distance_cm 正整数距离，单位cm。
 * @param segment 输出的内部路线段。
 * @return 1表示转换成功；0表示空指针、零距离或不是单轴命令。
 */
static uint8_t BuildSegment(int16_t x_rpm, int16_t y_rpm,
                            uint16_t distance_cm,
                            ChassisRouteSegment *segment)
{
    int32_t signed_speed;

    if (segment == 0 || distance_cm == 0U ||
        ((x_rpm == 0) == (y_rpm == 0))) {
        return 0U;
    }

    if (x_rpm != 0) {
        segment->direction = (x_rpm > 0) ? CHASSIS_ROUTE_X_POSITIVE
                                         : CHASSIS_ROUTE_X_NEGATIVE;
        signed_speed = x_rpm;
    } else {
        segment->direction = (y_rpm > 0) ? CHASSIS_ROUTE_Y_POSITIVE
                                         : CHASSIS_ROUTE_Y_NEGATIVE;
        signed_speed = y_rpm;
    }

    segment->speed_rpm = (float)((signed_speed < 0) ? -signed_speed
                                                    : signed_speed);
    segment->distance_cm = (float)distance_cm;
    segment->target_heading_degrees = 0.0f;
    return 1U;
}

/**
 * @brief 校验并启动一段任务层单轴厘米移动。
 * @param x_rpm X轴有符号速度，单位RPM。
 * @param y_rpm Y轴有符号速度，单位RPM。
 * @param distance_cm 正整数距离，单位cm。
 * @return 1表示路线已接受；0表示输入或当前方向合同不允许执行。
 */
uint8_t Chassis_MoveCm(int16_t x_rpm, int16_t y_rpm,
                       uint16_t distance_cm)
{
    ChassisRouteSegment segment;
    ChassisRouteResolvedSegment resolved;

    if (BuildSegment(x_rpm, y_rpm, distance_cm, &segment) == 0U) {
        return 0U;
    }
    if (ChassisRoute_ResolveSegment(&segment, &resolved) !=
        CHASSIS_ROUTE_RESOLVE_OK) {
        return 0U;
    }

    ChassisRouteRunner_Init(&segment);
    return 1U;
}

/**
 * @brief 推进当前路线运行器一次。
 * @return 无。
 */
void Chassis_Service(void)
{
    ChassisRouteRunner_Service();
}
