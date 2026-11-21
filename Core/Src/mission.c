#include "mission.h"

#include "chassis.h"
#include "chassis_move.h"

typedef enum {
    MISSION_ACTION_MOVE = 0U,
    MISSION_ACTION_ALIGN_HOLD_START_YAW,
    MISSION_ACTION_CHECKPOINT,
    MISSION_ACTION_TASK_STOP
} MissionAction;

typedef enum {
    MISSION_NODE_NONE = 0U,
    MISSION_NODE_PRE_BUMP_CHECK,
    MISSION_NODE_POST_BUMP_CHECK,
    MISSION_NODE_END_CHECK
} MissionNode;

typedef struct {
    uint8_t action;
    uint8_t node_id;
    int16_t x_rpm;
    int16_t y_rpm;
    uint16_t distance_cm;
} MissionStep;

/* 第一阶段预验已由用户接受；本表是唯一活动的完整地图路线。
 * Y+首段完成后直接进入X-；不停车等待二维码，不接视觉或纠偏。 */
static const MissionStep mission_plan[] = {
    { MISSION_ACTION_MOVE, MISSION_NODE_NONE, 0, 75, 70 },
    { MISSION_ACTION_MOVE, MISSION_NODE_NONE, -60, 0, 73 },
    { MISSION_ACTION_CHECKPOINT, MISSION_NODE_PRE_BUMP_CHECK, 0, 0, 0 },
    { MISSION_ACTION_MOVE, MISSION_NODE_NONE, 0, 90, 104 },
    { MISSION_ACTION_CHECKPOINT, MISSION_NODE_POST_BUMP_CHECK, 0, 0, 0 },
    { MISSION_ACTION_MOVE, MISSION_NODE_NONE, -75, 0, 90 },
    { MISSION_ACTION_MOVE, MISSION_NODE_NONE, 0, 75, 105 },
    { MISSION_ACTION_CHECKPOINT, MISSION_NODE_END_CHECK, 0, 0, 0 },
    { MISSION_ACTION_MOVE, MISSION_NODE_NONE, 75, 0, 300 },
    { MISSION_ACTION_MOVE, MISSION_NODE_NONE, 0, -75, 260 },
    { MISSION_ACTION_TASK_STOP, MISSION_NODE_NONE, 0, 0, 0 }
};

#define MISSION_PLAN_COUNT ((uint32_t)(sizeof(mission_plan) / sizeof(mission_plan[0])))

static uint32_t mission_index;
static uint8_t mission_finished;
static uint8_t mission_failed;
static uint8_t mission_current_node;

/**
 * @brief 启动数据表中的下一个实际MOVE，或消费不产生执行器动作的任务节点。
 * @return 1表示当前步骤被接受；0表示数据无效或厘米运动入口拒绝。
 */
static uint8_t Mission_StartCurrent(void)
{
    const MissionStep *step;

    while (mission_index < MISSION_PLAN_COUNT) {
        step = &mission_plan[mission_index];
        if (step->action == MISSION_ACTION_MOVE) {
            return Chassis_MoveCm(step->x_rpm, step->y_rpm, step->distance_cm);
        }
        if (step->action == MISSION_ACTION_ALIGN_HOLD_START_YAW) {
            /* 下一MOVE由运行器在全停后锁定当前Yaw，不申请主动转向。 */
            mission_index++;
            continue;
        }
        if (step->action == MISSION_ACTION_CHECKPOINT) {
            /* 仅保留任务数据边界；视觉接入前绝不申请纠偏或其它执行器。 */
            if (step->node_id != MISSION_NODE_PRE_BUMP_CHECK &&
                step->node_id != MISSION_NODE_POST_BUMP_CHECK &&
                step->node_id != MISSION_NODE_END_CHECK) {
                return 0U;
            }
            mission_current_node = step->node_id;
            mission_index++;
            continue;
        }
        if (step->action == MISSION_ACTION_TASK_STOP) {
            Chassis_Stop();
            mission_finished = 1U;
            return 1U;
        }
        return 0U;
    }

    mission_finished = 1U;
    return 1U;
}

/**
 * @brief 复位任务推进状态并启动首段；不直接提交底盘底层命令。
 * @return 1表示首个任务步骤被接受；0表示初始化失败。
 */
uint8_t Mission_Init(void)
{
    mission_index = 0U;
    mission_finished = 0U;
    mission_failed = 0U;
    mission_current_node = MISSION_NODE_NONE;
    if (Mission_StartCurrent() == 0U) {
        mission_failed = 1U;
        Chassis_Stop();
        return 0U;
    }
    return 1U;
}

/**
 * @brief 只在当前厘米段正常完成后推进路线；任何失败都统一停车并锁存。
 * @return 任务运行状态；终态不会自动回到首段。
 */
MissionStatus Mission_Service(void)
{
    ChassisMoveStatus status;

    if (mission_failed != 0U) {
        Chassis_Stop();
        return MISSION_FAILED;
    }
    if (mission_finished != 0U) {
        Chassis_Stop();
        return MISSION_COMPLETE;
    }

    status = Chassis_Service();
    if (status == CHASSIS_MOVE_FAILED) {
        mission_failed = 1U;
        Chassis_Stop();
        return MISSION_FAILED;
    }
    if (status != CHASSIS_MOVE_COMPLETE) {
        return MISSION_IN_PROGRESS;
    }

    mission_index++;
    if (mission_index >= MISSION_PLAN_COUNT || Mission_StartCurrent() == 0U) {
        mission_failed = (mission_finished == 0U) ? 1U : 0U;
        Chassis_Stop();
    }
    return (mission_failed != 0U) ? MISSION_FAILED :
           (mission_finished != 0U) ? MISSION_COMPLETE : MISSION_IN_PROGRESS;
}
