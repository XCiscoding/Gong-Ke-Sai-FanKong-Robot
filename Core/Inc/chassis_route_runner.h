#ifndef CHASSIS_ROUTE_RUNNER_H
#define CHASSIS_ROUTE_RUNNER_H

#include "chassis_route.h"

#include <stdint.h>

/** 一段厘米路线的非阻塞运行阶段；终态只能由硬件复位重新开始。 */
typedef enum {
    CHASSIS_ROUTE_RUNNER_BOOT_SAFE = 0U,
    CHASSIS_ROUTE_RUNNER_STABILIZE,
    CHASSIS_ROUTE_RUNNER_START_RAMP,
    CHASSIS_ROUTE_RUNNER_RUN_ONCE,
    CHASSIS_ROUTE_RUNNER_STOP_LATCHED
} ChassisRouteRunnerState;

/** 正常完成和首个失败层使用同一个停车出口，但保留可读原因。 */
typedef enum {
    CHASSIS_ROUTE_STOP_NONE = 0U,
    CHASSIS_ROUTE_STOP_COMPLETE,
    CHASSIS_ROUTE_STOP_INVALID_ARGUMENT,
    CHASSIS_ROUTE_STOP_IMU_NOT_READY,
    CHASSIS_ROUTE_STOP_HEADING_ARM,
    CHASSIS_ROUTE_STOP_HEADING_FAULT,
    CHASSIS_ROUTE_STOP_MOTOR_SUBMIT,
    CHASSIS_ROUTE_STOP_TIMEOUT
} ChassisRouteStopReason;

/** RAM只读诊断；数字结果不能替代烧录身份、厘米精度或实车路线验收。 */
typedef struct {
    uint32_t state;
    uint32_t stop_reason;
    uint32_t resolve_status;
    uint32_t state_started_ms;
    uint32_t run_started_ms;
    uint32_t ramp_started_ms;
    uint32_t expected_run_ms;
    uint32_t target_steps;
    int32_t progress_steps;
    uint32_t command_submissions;
    uint32_t completed;
    float target_distance_cm;
    float progress_cm;
    float last_heading_w_rpm;
    float last_base_speed_scale;
} ChassisRouteRunnerDiagnostics;

extern volatile ChassisRouteRunnerDiagnostics chassis_route_runner_diag;

/**
 * @brief 校验并复制一段路线定义，随后进入全停BOOT_SAFE；不会启动电机。
 * @return 1表示段已接受；0表示参数无效、路线解析失败或上一段尚未结束。
 */
uint8_t ChassisRouteRunner_Init(const ChassisRouteSegment *segment);

/** @brief 非阻塞推进一次安全状态机；调用方负责先更新IMU。 */
void ChassisRouteRunner_Service(void);

#endif /* CHASSIS_ROUTE_RUNNER_H */
