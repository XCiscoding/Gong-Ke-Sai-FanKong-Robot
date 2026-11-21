#ifndef ARM_GRASP_STAGE_PANEL_H
#define ARM_GRASP_STAGE_PANEL_H

#include "arm_tune.h"
#include "arm_grasp_frozen_baseline.h"

/*
 * 抓球采集唯一用户编辑区：
 * 1. 抓球链的已冻结值显示在本面板的冻结区，并由arm_grasp_frozen_baseline.h校验；禁止改写。
 * 2. 每次只改 ARM_GRASP_CAPTURE_CARD 和当前PLACE_*卡对应的目标。
 * 3. 每张卡只改变一个关节，其余关节从入口/前卡继承并持续保持；0仅为未调起点，不是最终姿态。
 * 4. 不得改 arm_tune.c 的 zero_us、direction、比例、限位、PWM/I2C 或任何掩码。
 *
 * 姿态卡的真实机械顺序（不是通道编号顺序）：
 *   CLEAR_CH2_FOR_CH1：CH2先让位，给CH1抬起留空间；
 *   RAISE_CH1：只抬CH1；底盘随后自行按视觉做最终位置微调，CH3不动；
 *   GRASP_CH0_TURN(+87) → GRASP_CH2_FULL_DIVE → GRASP_WRIST → GRASP_CLOSE；
 *   放置阶段：PLACE_CH1_PREPARE → PLACE_CH2_HALF_DIVE → PLACE_CH3_PREPARE →
 *   PLACE_CH0_TURN；CH1、CH3目标未填写时保持UNSET，程序拒绝输出，最后PLACE_RELEASE。
 * 每张卡只改自己对应的一个*_DEG；其它已经出现的关节由控制层继承并保持。
 */
/* ===== 用户编辑区开始：入口支撑姿态 =====
 * 必须填入五个已确认的逻辑角度；保留UNSET时程序拒绝输出，避免猜填0°。
 * 入口姿态会先持续输出，当前卡只让指定关节从其入口/前卡目标渐变。 */
#define ARM_CAPTURE_DEG_UNSET                 ((int16_t)-32768)
#define ARM_CAPTURE_ENTRY_CH0_DEG             0
#define ARM_CAPTURE_ENTRY_CH1_DEG             0
#define ARM_CAPTURE_ENTRY_CH2_DEG             0
#define ARM_CAPTURE_ENTRY_CH3_DEG             0
#define ARM_CAPTURE_ENTRY_CH4_DEG             0
/* 首次五路入口PWM写入后，保持入口支撑姿态的时长；只在本卡渐变前生效。 */
#define ARM_CAPTURE_ENTRY_HOLD_MS             5000U

/* ===== 用户编辑区：当前调试卡 =====
 * 此处只能填ARM_CAPTURE_*卡名，不能填ARM_CAPTURE_TARGET_*目标参数名。
 * 可选顺序：ARM_CAPTURE_CLEAR_CH2_FOR_CH1 → ARM_CAPTURE_RAISE_CH1 →
 * ARM_CAPTURE_GRASP_CH0_TURN → ARM_CAPTURE_GRASP_CH2_FULL_DIVE →
 * ARM_CAPTURE_GRASP_WRIST → ARM_CAPTURE_GRASP_CLOSE → ARM_CAPTURE_PLACE_CH1_PREPARE →
 * ARM_CAPTURE_PLACE_CH2_HALF_DIVE → ARM_CAPTURE_PLACE_CH3_PREPARE → ARM_CAPTURE_PLACE_CH0_TURN。 */
#define ARM_GRASP_CAPTURE_CARD                 ARM_CAPTURE_GRASP_CH2_FULL_DIVE

/* ===== 抓球阶段冻结数据（只读，已覆盖回当前面板） =====
 * 这六项是此前抓球实验保留的完整数值；不属于PLACE调参。
 * 若误改，编译会以Frozen grasp baseline mismatch拒绝，不能静默覆盖。 */
#define ARM_CAPTURE_TARGET_CLEAR_CH2_FOR_CH1_DEG   85
#define ARM_CAPTURE_TARGET_RAISE_CH1_DEG           110
#define ARM_CAPTURE_TARGET_GRASP_CH0_TURN_DEG      87
#define ARM_CAPTURE_TARGET_GRASP_CH2_FULL_DIVE_DEG 70
#define ARM_CAPTURE_TARGET_GRASP_WRIST_DEG         0
#define ARM_CAPTURE_TARGET_GRASP_CLOSE_DEG         22

#if ARM_CAPTURE_TARGET_CLEAR_CH2_FOR_CH1_DEG != ARM_CAPTURE_FROZEN_CLEAR_CH2_FOR_CH1_DEG
#error "Frozen grasp baseline mismatch: CLEAR_CH2"
#endif
#if ARM_CAPTURE_TARGET_RAISE_CH1_DEG != ARM_CAPTURE_FROZEN_RAISE_CH1_DEG
#error "Frozen grasp baseline mismatch: RAISE_CH1"
#endif
#if ARM_CAPTURE_TARGET_GRASP_CH0_TURN_DEG != ARM_CAPTURE_FROZEN_GRASP_CH0_TURN_DEG
#error "Frozen grasp baseline mismatch: GRASP_CH0"
#endif
#if ARM_CAPTURE_TARGET_GRASP_CH2_FULL_DIVE_DEG != ARM_CAPTURE_FROZEN_GRASP_CH2_FULL_DIVE_DEG
#error "Frozen grasp baseline mismatch: GRASP_CH2_FULL_DIVE"
#endif
#if ARM_CAPTURE_TARGET_GRASP_WRIST_DEG != ARM_CAPTURE_FROZEN_GRASP_WRIST_DEG
#error "Frozen grasp baseline mismatch: GRASP_WRIST"
#endif
#if ARM_CAPTURE_TARGET_GRASP_CLOSE_DEG != ARM_CAPTURE_FROZEN_GRASP_CLOSE_DEG
#error "Frozen grasp baseline mismatch: GRASP_CLOSE"
#endif

/* ===== 用户编辑区：放置阶段候选目标 =====
 * 抓球链的六个目标已冻结在arm_grasp_frozen_baseline.h；这里仅允许调PLACE_*。 */
#define ARM_CAPTURE_TARGET_PLACE_CH1_PREPARE_DEG   ARM_CAPTURE_DEG_UNSET
#define ARM_CAPTURE_TARGET_PLACE_CH2_HALF_DIVE_DEG 85
#define ARM_CAPTURE_TARGET_PLACE_CH3_PREPARE_DEG   ARM_CAPTURE_DEG_UNSET
#define ARM_CAPTURE_TARGET_PLACE_CH0_TURN_DEG      -105
#define ARM_CAPTURE_TARGET_PLACE_CH2_FULL_DIVE_DEG 0
#define ARM_CAPTURE_TARGET_PLACE_WRIST_DEG         0
#define ARM_CAPTURE_TARGET_PLACE_RELEASE_DEG       0
/* ===== 用户编辑区结束 ===== */

/* 读取面板并提交给控制层；失败返回0，控制层会保持FAULT_LOCKED。 */
uint8_t ArmGraspStagePanel_Request(void);

#endif
