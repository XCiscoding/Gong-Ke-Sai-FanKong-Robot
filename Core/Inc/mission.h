#ifndef MISSION_H
#define MISSION_H

#include <stdint.h>

/** 当前任务路线的可观测终态；实体结果必须另行记录。 */
typedef enum {
    MISSION_IN_PROGRESS = 0U,
    MISSION_COMPLETE,
    MISSION_FAILED
} MissionStatus;

/**
 * @brief 初始化当前编入映像的任务路线，不直接绕过厘米运动门面。
 * @return 1表示首步已被接受；0表示路线数据或底盘任务入口拒绝。
 */
uint8_t Mission_Init(void);

/**
 * @brief 推进当前任务路线一次；每个MOVE完成后才允许启动下一步。
 * @return 运行中、完成或失败；完成和失败均不会自动重新启动。
 */
MissionStatus Mission_Service(void);

#endif /* MISSION_H */
