#include "chassis_move.h"
#include "chassis_route.h"
#include "chassis_route_runner.h"

#include <assert.h>
#include <math.h>

static ChassisRouteSegment captured_segment;
static ChassisRouteResolveStatus resolve_status = CHASSIS_ROUTE_RESOLVE_OK;
static uint32_t init_calls;
static uint32_t service_calls;

/**
 * @brief 以可控结果替代路线解析器，并保留门面传入的路线段。
 * @param segment 待解析的路线段。
 * @param resolved 未参与本合同的解析结果缓冲区。
 * @return 测试指定的解析状态。
 */
ChassisRouteResolveStatus ChassisRoute_ResolveSegment(
    const ChassisRouteSegment *segment,
    ChassisRouteResolvedSegment *resolved)
{
    (void)resolved;
    captured_segment = *segment;
    return resolve_status;
}

/**
 * @brief 记录门面是否把已验证路线交给运行器。
 * @param segment 待运行的路线段。
 */
void ChassisRouteRunner_Init(const ChassisRouteSegment *segment)
{
    captured_segment = *segment;
    init_calls++;
}

/**
 * @brief 记录任务层Service是否只委托一次路线运行器。
 */
void ChassisRouteRunner_Service(void)
{
    service_calls++;
}

/**
 * @brief 比较两个浮点合同值是否在测试容差内相等。
 * @param left 左值。
 * @param right 右值。
 * @return 相等返回1，否则返回0。
 */
static int NearlyEqual(float left, float right)
{
    return fabsf(left - right) < 0.001f;
}

/**
 * @brief 验证整数任务接口的四向映射、无效输入和委托边界。
 * @return 全部断言通过返回0。
 */
int main(void)
{
    assert(Chassis_MoveCm(50, 0, 10) == 1U);
    assert(captured_segment.direction == CHASSIS_ROUTE_X_POSITIVE);
    assert(NearlyEqual(captured_segment.speed_rpm, 50.0f));
    assert(NearlyEqual(captured_segment.distance_cm, 10.0f));
    assert(NearlyEqual(captured_segment.target_heading_degrees, 0.0f));

    assert(Chassis_MoveCm(-40, 0, 15) == 1U);
    assert(captured_segment.direction == CHASSIS_ROUTE_X_NEGATIVE);
    assert(NearlyEqual(captured_segment.speed_rpm, 40.0f));

    assert(Chassis_MoveCm(0, 30, 20) == 1U);
    assert(captured_segment.direction == CHASSIS_ROUTE_Y_POSITIVE);
    assert(Chassis_MoveCm(0, -30, 25) == 1U);
    assert(captured_segment.direction == CHASSIS_ROUTE_Y_NEGATIVE);

    assert(Chassis_MoveCm(0, 0, 10) == 0U);
    assert(Chassis_MoveCm(10, 10, 10) == 0U);
    assert(Chassis_MoveCm(10, 0, 0) == 0U);

    resolve_status = CHASSIS_ROUTE_RESOLVE_DIRECTION_UNVERIFIED;
    assert(Chassis_MoveCm(-50, 0, 10) == 0U);
    assert(init_calls == 4U);

    Chassis_Service();
    assert(service_calls == 1U);
    return 0;
}
