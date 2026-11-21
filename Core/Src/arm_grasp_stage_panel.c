#include "arm_grasp_stage_panel.h"
#include "arm_grasp_capture.h"

typedef struct { ArmJointId joint; int16_t target_deg; } ArmGraspStageCard;

static const ArmGraspStageCard arm_grasp_cards[ARM_CAPTURE_CARD_COUNT] = {
  /* 起始：CH2先让位，再CH1抬起；视觉最终对位属于底盘，不在此驱动CH3。 */
  {ARM_JOINT_SMALL_ARM, ARM_CAPTURE_FROZEN_CLEAR_CH2_FOR_CH1_DEG},
  {ARM_JOINT_BIG_ARM, ARM_CAPTURE_FROZEN_RAISE_CH1_DEG},
  /* 抓取：CH2保持让位至CH0转向完成，随后才继续下探。 */
  {ARM_JOINT_BASE, ARM_CAPTURE_FROZEN_GRASP_CH0_TURN_DEG},
  {ARM_JOINT_SMALL_ARM, ARM_CAPTURE_FROZEN_GRASP_CH2_FULL_DIVE_DEG},
  {ARM_JOINT_WRIST, ARM_CAPTURE_FROZEN_GRASP_WRIST_DEG},
  {ARM_JOINT_GRIPPER, ARM_CAPTURE_FROZEN_GRASP_CLOSE_DEG},
  /* 放置：CH1准备→CH2半下探→CH3准备→CH0转向；两个未调目标保持UNSET并拒绝输出。 */
  {ARM_JOINT_BIG_ARM, ARM_CAPTURE_TARGET_PLACE_CH1_PREPARE_DEG},
  {ARM_JOINT_SMALL_ARM, ARM_CAPTURE_TARGET_PLACE_CH2_HALF_DIVE_DEG},
  {ARM_JOINT_WRIST, ARM_CAPTURE_TARGET_PLACE_CH3_PREPARE_DEG},
  {ARM_JOINT_BASE, ARM_CAPTURE_TARGET_PLACE_CH0_TURN_DEG},
  {ARM_JOINT_SMALL_ARM, ARM_CAPTURE_TARGET_PLACE_CH2_FULL_DIVE_DEG},
  {ARM_JOINT_WRIST, ARM_CAPTURE_TARGET_PLACE_WRIST_DEG},
  {ARM_JOINT_GRIPPER, ARM_CAPTURE_TARGET_PLACE_RELEASE_DEG}
};

uint8_t ArmGraspStagePanel_Request(void)
{
  ArmPose pose, entry_support_pose;
  uint32_t hold_mask, ramp_mask, card;
  int16_t ramp_start_deg;

  if (ARM_GRASP_CAPTURE_CARD >= ARM_CAPTURE_CARD_COUNT) return 0U;
  entry_support_pose.target_deg[ARM_JOINT_BASE] = ARM_CAPTURE_ENTRY_CH0_DEG;
  entry_support_pose.target_deg[ARM_JOINT_BIG_ARM] = ARM_CAPTURE_ENTRY_CH1_DEG;
  entry_support_pose.target_deg[ARM_JOINT_SMALL_ARM] = ARM_CAPTURE_ENTRY_CH2_DEG;
  entry_support_pose.target_deg[ARM_JOINT_WRIST] = ARM_CAPTURE_ENTRY_CH3_DEG;
  entry_support_pose.target_deg[ARM_JOINT_GRIPPER] = ARM_CAPTURE_ENTRY_CH4_DEG;
  for (card = 0U; card < ARM_JOINT_COUNT; ++card) {
    if (entry_support_pose.target_deg[card] == ARM_CAPTURE_DEG_UNSET) return 0U;
  }
  ArmGraspCapture_Reset();
  if (!ArmGraspCapture_SeedSupportPose(&entry_support_pose)) return 0U;
  for (card = 0U; card <= ARM_GRASP_CAPTURE_CARD; ++card) {
    if (arm_grasp_cards[card].target_deg == ARM_CAPTURE_DEG_UNSET) return 0U;
    if (!ArmGraspCapture_ApplyCard(arm_grasp_cards[card].joint,
                                   arm_grasp_cards[card].target_deg)) return 0U;
  }
  if (!ArmGraspCapture_GetOutputRequest(&pose, &hold_mask, &ramp_mask, &ramp_start_deg)) return 0U;
  return ArmTune_RequestMaskedPose(&pose, hold_mask, ramp_mask, ramp_start_deg,
                                   ARM_CAPTURE_ENTRY_HOLD_MS);
}
