#ifndef ARM_GRASP_FROZEN_BASELINE_H
#define ARM_GRASP_FROZEN_BASELINE_H

/*
 * 抓球已冻结基线（2026-11-06）：
 * 这些是用户完成抓球链时保留的当前源码角度，不属于放置阶段的日常调参区。
 * 若未来有新的、独立的实体证据要修订抓球姿态，必须明确恢复“抓球基线修订”工作，
 * 不得通过修改PLACE_*或入口姿态反向覆盖本组数值。
 *
 * 本文件冻结的是源码基线，不把尚未独立复核的角度宣称为全部实体验收通过。
 */
#define ARM_CAPTURE_FROZEN_CLEAR_CH2_FOR_CH1_DEG   85
#define ARM_CAPTURE_FROZEN_RAISE_CH1_DEG           110
#define ARM_CAPTURE_FROZEN_GRASP_CH0_TURN_DEG      87
#define ARM_CAPTURE_FROZEN_GRASP_CH2_FULL_DIVE_DEG 70
#define ARM_CAPTURE_FROZEN_GRASP_WRIST_DEG         0
#define ARM_CAPTURE_FROZEN_GRASP_CLOSE_DEG         22

#endif
