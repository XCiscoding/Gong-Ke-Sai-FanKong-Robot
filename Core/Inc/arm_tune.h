#ifndef ARM_TUNE_H
#define ARM_TUNE_H

#include <stdint.h>

typedef enum {
  ARM_JOINT_BASE = 0U, ARM_JOINT_BIG_ARM, ARM_JOINT_SMALL_ARM,
  ARM_JOINT_WRIST, ARM_JOINT_GRIPPER, ARM_JOINT_COUNT
} ArmJointId;

typedef enum { ARM_TUNE_IDLE = 0U, ARM_TUNE_RAMP, ARM_TUNE_HOLD, ARM_TUNE_FAULT_LOCKED } ArmTuneState;

typedef struct {
  uint8_t enabled;
  uint8_t channel;
  int16_t home_deg;
  int16_t min_deg;
  int16_t max_deg;
  uint16_t zero_us;
  int8_t direction;
  uint16_t us_per_deg_x10;
  uint16_t speed_deg_s;
  uint16_t settle_ms;
  uint8_t output_ready;
} ArmJointParam;

/* 一个姿态只描述各关节的逻辑目标角度；脉宽换算、限位和渐变仍归机械臂控制层。 */
typedef struct {
  int16_t target_deg[ARM_JOINT_COUNT];
} ArmPose;

extern ArmJointParam arm_user_table[ARM_JOINT_COUNT];
extern volatile ArmTuneState arm_tune_state;
extern volatile uint32_t arm_tune_fault;

uint8_t ArmTune_Init(void);
void ArmTune_Service(uint32_t now_ms);
/* 同一次运行内请求下一个姿态：从当前RAM角度渐变，不重新写启动零位。 */
uint8_t ArmTune_RequestPose(const ArmPose *pose);
/* 掩码请求：先完整输出入口姿态并保持entry_hold_ms；随后ramp_mask只允许一个当前关节渐变。 */
uint8_t ArmTune_RequestMaskedPose(const ArmPose *pose, uint32_t hold_mask, uint32_t ramp_mask,
                                  int16_t ramp_start_deg, uint32_t entry_hold_ms);
uint8_t ArmTune_RequestHome(void);
uint8_t ArmTune_RequestManualPose(void);
uint8_t ArmTune_IsAtTarget(void);

#endif
