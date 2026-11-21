#include "mission.h"

#include "chassis_move.h"

#include <assert.h>

typedef struct {
    int16_t x_rpm;
    int16_t y_rpm;
    uint16_t distance_cm;
} CapturedMove;

static CapturedMove captured_moves[8];
static uint32_t move_calls;
static uint32_t stop_calls;
static ChassisMoveStatus service_status = CHASSIS_MOVE_IN_PROGRESS;

/** @brief 记录任务层提交的厘米命令，不接触硬件。 */
uint8_t Chassis_MoveCm(int16_t x_rpm, int16_t y_rpm, uint16_t distance_cm)
{
    if (move_calls >= 8U) return 0U;
    captured_moves[move_calls].x_rpm = x_rpm;
    captured_moves[move_calls].y_rpm = y_rpm;
    captured_moves[move_calls].distance_cm = distance_cm;
    move_calls++;
    return 1U;
}

/** @brief 返回可控的厘米段状态，验证任务层不会抢跑。 */
ChassisMoveStatus Chassis_Service(void)
{
    return service_status;
}

/** @brief 记录统一停车出口。 */
void Chassis_Stop(void)
{
    stop_calls++;
}

/** @brief 验证Y+完成后直接进入X-及完整地图路线的停机锁存。 */
int main(void)
{
    assert(Mission_Init() == 1U);
    assert(move_calls == 1U);
    assert(captured_moves[0].x_rpm == 0);
    assert(captured_moves[0].y_rpm == 75);
    assert(captured_moves[0].distance_cm == 70U);
    assert(Mission_Service() == MISSION_IN_PROGRESS);
    assert(move_calls == 1U);
    service_status = CHASSIS_MOVE_COMPLETE;
    assert(Mission_Service() == MISSION_IN_PROGRESS);
    assert(move_calls == 2U);
    assert(captured_moves[1].x_rpm == -60);
    assert(captured_moves[1].y_rpm == 0);
    assert(captured_moves[1].distance_cm == 73U);

    assert(Mission_Service() == MISSION_IN_PROGRESS);
    assert(move_calls == 3U);
    assert(captured_moves[2].x_rpm == 0);
    assert(captured_moves[2].y_rpm == 90);
    assert(captured_moves[2].distance_cm == 104U);

    assert(Mission_Service() == MISSION_IN_PROGRESS);
    assert(move_calls == 4U);
    assert(captured_moves[3].x_rpm == -75);
    assert(captured_moves[3].y_rpm == 0);
    assert(captured_moves[3].distance_cm == 90U);

    assert(Mission_Service() == MISSION_IN_PROGRESS);
    assert(move_calls == 5U);
    assert(captured_moves[4].x_rpm == 0);
    assert(captured_moves[4].y_rpm == 75);
    assert(captured_moves[4].distance_cm == 105U);

    assert(Mission_Service() == MISSION_IN_PROGRESS);
    assert(move_calls == 6U);
    assert(captured_moves[5].x_rpm == 75);
    assert(captured_moves[5].y_rpm == 0);
    assert(captured_moves[5].distance_cm == 300U);

    assert(Mission_Service() == MISSION_IN_PROGRESS);
    assert(move_calls == 7U);
    assert(captured_moves[6].x_rpm == 0);
    assert(captured_moves[6].y_rpm == -75);
    assert(captured_moves[6].distance_cm == 260U);

    assert(Mission_Service() == MISSION_COMPLETE);
    assert(stop_calls >= 1U);
    assert(Mission_Service() == MISSION_COMPLETE);
    assert(move_calls == 7U);

    return 0;
}
