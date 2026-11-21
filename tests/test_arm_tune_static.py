from pathlib import Path

root = Path(__file__).resolve().parents[1]
source = (root / "Core" / "Src" / "arm_tune.c").read_text(encoding="utf-8")
header = (root / "Core" / "Inc" / "arm_tune.h").read_text(encoding="utf-8")
main_h = (root / "Core" / "Inc" / "main.h").read_text(encoding="utf-8")
main_c = (root / "Core" / "Src" / "main.c").read_text(encoding="utf-8")

assert "#define APP_PROFILE_ARM_TUNE" in main_h
assert "#define APP_PROFILE_ARM_GRASP_CAPTURE" in main_h
assert "#define APP_PROFILE_PASSIVE_VISION_REPORT 0U" in main_h
assert "#define APP_PROFILE_VISION_DISTANCE_ALIGN 0U" in main_h
assert "ARM_JOINT_BASE" in header and "ARM_JOINT_GRIPPER" in header
# 调参表由现场用户维护：本合同只校验结构，绝不冻结enabled、零点、限位或速度字面值。
assert "I2C2" in source and "GPIO_PIN_10 | GPIO_PIN_11" in source
assert "ARM_TUNE_FAULT_LOCKED" in source
assert "arm_user_table[i].enabled != 0U" in source
assert "speed_deg_s" in source and "Arm_StepPeriodMs" in source
assert "initial_output_pending" in source and "Pca_WritePulse" in source
assert "ArmTune_RequestPose" in source and "ArmTune_RequestPose" in header
assert "ArmTune_IsAtTarget" in source and "ArmTune_IsAtTarget" in header
assert "ArmTune_RequestHome" in source and "ArmTune_RequestManualPose" in source
assert "home_deg" in source and "us_per_deg_x10" in source and "output_ready" in source
assert "-135, 135" in source and "74U" in source
assert "ArmJointParam arm_user_table[ARM_JOINT_COUNT]" in source
assert "ARM_TUNE_TARGET_CH0_DEG" in source and "ARM_TUNE_TARGET_CH4_DEG" in source
assert "#define ARM_TUNE_PROBE_ENABLED  APP_PROFILE_ARM_TUNE" in source
assert "#define ARM_TUNE_PROBE_JOINT    ARM_JOINT_BASE" in source
assert "#define ARM_TUNE_PROBE_DEG      87" in source
assert "逻辑0°=1300us" not in source
assert "CH0：正=左" in source and "CH1：正=上" in source
assert "CH2：正=后" in source and "CH3：正=上" in source and "CH4：正=闭合" in source
assert "static uint8_t I2c2_Write4" in source
pulse_body = source.split("static uint8_t Pca_WritePulse", 1)[1].split("static uint8_t Arm_PulseForDeg", 1)[0]
assert "I2c2_Write4" in pulse_body and "I2c2_Write((uint8_t)(PCA_LED0_ON_L" not in pulse_body
assert "ARM_TUNE_SEQUENCE_OUTPUT_ENABLE" not in source and "ArmTune_SequentialProbeService" not in source
assert "#if ARM_TUNE_PROBE_ENABLED\n  ArmTune_SingleProbeService(now_ms);" in source
assert "#if ARM_TUNE_PROBE_ENABLED\nstatic void ArmTune_ProbeFault" in source
assert "ARM_PROBE_ZERO" in source and "ARM_PROBE_HOLD_HOME" in source
assert "ARM_TUNE_PROBE_JOINT >= ARM_JOINT_COUNT" in source
assert "ARM_TUNE_PROBE_DEG > p->max_deg" in source
assert "current_deg保留当前运行态，不触发启动零位输出" in source
request_body = source.split("uint8_t ArmTune_RequestPose", 1)[1].split("uint8_t ArmTune_IsAtTarget", 1)[0]
assert "requested_deg[i] = pose->target_deg[i]" in request_body
assert "uint8_t ArmTune_RequestMaskedPose" in request_body
assert "current_deg[i] = ((hold_mask" in request_body
assert "initial_output_pending = 1U" in request_body
assert "active_mask" in source and "hold_mask" in source and "ramp_mask" in source
assert "current_deg[i], &pulse" in source
assert "ramp_mask & (1UL << i)" in source
assert "ArmTune_RequestManualPose()" in main_c
print("PASS arm_tune_static")
