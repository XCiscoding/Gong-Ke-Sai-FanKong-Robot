#include "arm_grasp_capture.h"

ArmGraspCapture arm_grasp_capture;

void ArmGraspCapture_Reset(void)
{
  uint32_t i;
  for (i = 0U; i < ARM_JOINT_COUNT; ++i) arm_grasp_capture.target.target_deg[i] = 0;
  arm_grasp_capture.ramp_start_deg = 0;
  arm_grasp_capture.active_mask = 0U;
  arm_grasp_capture.hold_mask = 0U;
  arm_grasp_capture.tuning_joint = ARM_JOINT_COUNT;
}

uint8_t ArmGraspCapture_SeedSupportPose(const ArmPose *support_pose)
{
  uint32_t i;
  if (support_pose == 0) return 0U;
  for (i = 0U; i < ARM_JOINT_COUNT; ++i) {
    arm_grasp_capture.target.target_deg[i] = support_pose->target_deg[i];
  }
  arm_grasp_capture.active_mask = (1UL << ARM_JOINT_COUNT) - 1UL;
  arm_grasp_capture.hold_mask = arm_grasp_capture.active_mask;
  arm_grasp_capture.tuning_joint = ARM_JOINT_COUNT;
  return 1U;
}

uint8_t ArmGraspCapture_ApplyCard(ArmJointId joint, int16_t target_deg)
{
  if (joint >= ARM_JOINT_COUNT) return 0U;
  /* 不按通道编号约束：真实安全顺序可为CH2→CH1→CH2→CH0。 */
  arm_grasp_capture.tuning_joint = joint;
  arm_grasp_capture.ramp_start_deg = arm_grasp_capture.target.target_deg[joint];
  arm_grasp_capture.target.target_deg[arm_grasp_capture.tuning_joint] = target_deg;
  arm_grasp_capture.active_mask |= ARM_CAPTURE_MASK(joint);
  arm_grasp_capture.hold_mask = arm_grasp_capture.active_mask & ~ARM_CAPTURE_MASK(joint);
  return 1U;
}

uint8_t ArmGraspCapture_GetOutputRequest(ArmPose *pose, uint32_t *hold_mask,
                                         uint32_t *ramp_mask, int16_t *ramp_start_deg)
{
  if (pose == 0 || hold_mask == 0 || ramp_mask == 0 ||
      arm_grasp_capture.tuning_joint >= ARM_JOINT_COUNT) return 0U;
  *pose = arm_grasp_capture.target;
  *hold_mask = arm_grasp_capture.hold_mask;
  *ramp_mask = ARM_CAPTURE_MASK(arm_grasp_capture.tuning_joint);
  *ramp_start_deg = arm_grasp_capture.ramp_start_deg;
  return 1U;
}
