#ifndef ARM_GRASP_CAPTURE_H
#define ARM_GRASP_CAPTURE_H

#include "arm_tune.h"

#define ARM_CAPTURE_MASK(joint) (1UL << (uint32_t)(joint))

typedef enum {
  ARM_CAPTURE_CLEAR_CH2_FOR_CH1 = 0U,
  ARM_CAPTURE_RAISE_CH1,
  ARM_CAPTURE_GRASP_CH0_TURN,
  ARM_CAPTURE_GRASP_CH2_FULL_DIVE,
  ARM_CAPTURE_GRASP_WRIST,
  ARM_CAPTURE_GRASP_CLOSE,
  ARM_CAPTURE_PLACE_CH1_PREPARE,
  ARM_CAPTURE_PLACE_CH2_HALF_DIVE,
  ARM_CAPTURE_PLACE_CH3_PREPARE,
  ARM_CAPTURE_PLACE_CH0_TURN,
  ARM_CAPTURE_PLACE_CH2_FULL_DIVE,
  ARM_CAPTURE_PLACE_WRIST,
  ARM_CAPTURE_PLACE_RELEASE,
  ARM_CAPTURE_CARD_COUNT
} ArmGraspCaptureCard;

/* 仅记录抓球姿态卡的累计目标；不含PWM、脉宽、I2C、状态机或任务编排。 */
typedef struct {
  ArmPose target;
  int16_t ramp_start_deg;
  uint32_t active_mask;
  uint32_t hold_mask;
  ArmJointId tuning_joint;
} ArmGraspCapture;

extern ArmGraspCapture arm_grasp_capture;

void ArmGraspCapture_Reset(void);
/* 写入复位后的全关节入口保持姿态；随后姿态卡只改变指定的当前关节。 */
uint8_t ArmGraspCapture_SeedSupportPose(const ArmPose *support_pose);
/* 每张卡只改变一个关节；该关节此前的累计目标会成为本卡渐变起点。 */
uint8_t ArmGraspCapture_ApplyCard(ArmJointId joint, int16_t target_deg);
/* 输出请求仍是纯数据：前卡关节保持，本卡关节从ramp_start_deg渐变。 */
uint8_t ArmGraspCapture_GetOutputRequest(ArmPose *pose, uint32_t *hold_mask,
                                          uint32_t *ramp_mask, int16_t *ramp_start_deg);

#endif
