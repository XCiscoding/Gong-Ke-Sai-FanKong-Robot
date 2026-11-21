from pathlib import Path

root = Path(__file__).resolve().parents[1]
header = (root / "Core" / "Inc" / "arm_grasp_capture.h").read_text(encoding="utf-8")
source = (root / "Core" / "Src" / "arm_grasp_capture.c").read_text(encoding="utf-8")
main = (root / "Core" / "Src" / "main.c").read_text(encoding="utf-8")

assert "ArmGraspCapture" in header and "hold_mask" in header
assert "ARM_CAPTURE_MASK" in header
assert "ArmGraspCaptureCard" in header
assert "ARM_CAPTURE_CLEAR_CH2_FOR_CH1" in header
assert "ARM_CAPTURE_PLACE_RELEASE" in header
assert "ArmGraspCapture_ApplyCard" in source
assert "ArmGraspCapture_SeedSupportPose" in header
assert "ArmGraspCapture_SeedSupportPose" in source
assert "ramp_start_deg" in source
assert "active_mask" in source
assert "CH2→CH1→CH2→CH0" in source
assert "Pca_WritePulse" not in source and "I2C" not in source
assert "mission" not in source.lower()
assert "tuning_joint = ARM_JOINT_COUNT" in source
assert "active_mask = (1UL << ARM_JOINT_COUNT) - 1UL" in source
assert "arm_grasp_capture" not in main
print("PASS arm_grasp_capture_static")
