from pathlib import Path
from xml.etree import ElementTree

ROOT = Path(__file__).resolve().parents[1]
PANEL = (ROOT / "Core" / "Inc" / "arm_grasp_stage_panel.h").read_text(encoding="utf-8")
FROZEN = (ROOT / "Core" / "Inc" / "arm_grasp_frozen_baseline.h").read_text(encoding="utf-8")
PANEL_C = (ROOT / "Core" / "Src" / "arm_grasp_stage_panel.c").read_text(encoding="utf-8")
TUNE_H = (ROOT / "Core" / "Inc" / "arm_tune.h").read_text(encoding="utf-8")
TUNE_C = (ROOT / "Core" / "Src" / "arm_tune.c").read_text(encoding="utf-8")
MAIN_H = (ROOT / "Core" / "Inc" / "main.h").read_text(encoding="utf-8")
MAIN_C = (ROOT / "Core" / "Src" / "main.c").read_text(encoding="utf-8")
PROJECT_CONFIG = ElementTree.parse(ROOT / ".cproject")

assert "#define ARM_GRASP_CAPTURE_CARD" in PANEL
assert "ARM_CAPTURE_DEG_UNSET" in PANEL
assert "ARM_CAPTURE_ENTRY_CH0_DEG" in PANEL
assert "ARM_CAPTURE_ENTRY_CH4_DEG" in PANEL
assert "ARM_CAPTURE_CLEAR_CH2_FOR_CH1" in PANEL
assert '#include "arm_grasp_frozen_baseline.h"' in PANEL
assert "抓球阶段冻结数据（只读" in PANEL
assert "ARM_CAPTURE_TARGET_CLEAR_CH2_FOR_CH1_DEG   85" in PANEL
assert "ARM_CAPTURE_TARGET_RAISE_CH1_DEG           110" in PANEL
assert "ARM_CAPTURE_TARGET_GRASP_CH0_TURN_DEG      87" in PANEL
assert "ARM_CAPTURE_TARGET_GRASP_CH2_FULL_DIVE_DEG 70" in PANEL
assert "ARM_CAPTURE_TARGET_GRASP_WRIST_DEG         0" in PANEL
assert "ARM_CAPTURE_TARGET_GRASP_CLOSE_DEG         22" in PANEL
assert "Frozen grasp baseline mismatch: GRASP_CH2_FULL_DIVE" in PANEL
assert "ARM_CAPTURE_FROZEN_GRASP_CH0_TURN_DEG      87" in FROZEN
assert "ARM_CAPTURE_FROZEN_GRASP_CH2_FULL_DIVE_DEG 70" in FROZEN
assert "ARM_CAPTURE_FROZEN_GRASP_CLOSE_DEG         22" in FROZEN
assert "ARM_CAPTURE_TARGET_PLACE_CH0_TURN_DEG" in PANEL
assert "ArmTune_RequestMaskedPose" in TUNE_H and "ArmTune_RequestMaskedPose" in TUNE_C
assert "current_deg[i], &pulse" in TUNE_C
assert "ramp_mask & (1UL << i)" in TUNE_C
assert "APP_PROFILE_ARM_GRASP_CAPTURE" in MAIN_H
assert "#define APP_PROFILE_ARM_TUNE              0U" in MAIN_H
assert "#define APP_PROFILE_ARM_GRASP_CAPTURE     0U" in MAIN_H
assert "ArmGraspStagePanel_Request()" in MAIN_C
assert "APP_PROFILE_ARM_TUNE || APP_PROFILE_ARM_GRASP_CAPTURE" in MAIN_C
assert "ArmGraspCapture_GetOutputRequest" in PANEL_C
assert "ArmGraspCapture_SeedSupportPose" in PANEL_C
assert "ARM_CAPTURE_DEG_UNSET" in PANEL_C
assert "ARM_JOINT_SMALL_ARM" in PANEL_C
assert "ARM_JOINT_BIG_ARM" in PANEL_C
assert "ARM_JOINT_WRIST" in PANEL_C
assert "ARM_JOINT_BASE" in PANEL_C
assert "ARM_CAPTURE_FROZEN_GRASP_CH2_FULL_DIVE_DEG" in PANEL_C
assert "ARM_CAPTURE_PLACE_CH1_PREPARE" in PANEL
assert "ARM_CAPTURE_PLACE_CH3_PREPARE" in PANEL
assert "ARM_CAPTURE_TARGET_PLACE_CH1_PREPARE_DEG   ARM_CAPTURE_DEG_UNSET" in PANEL
assert "ARM_CAPTURE_TARGET_PLACE_CH3_PREPARE_DEG   ARM_CAPTURE_DEG_UNSET" in PANEL
stage_source = "Src/arm_grasp_stage_panel.c"
assert (ROOT / "Core" / stage_source).is_file()
core_entries = PROJECT_CONFIG.findall(".//sourceEntries/entry[@name='Core']")
assert core_entries
assert all(stage_source not in entry.get("excluding", "").split("|") for entry in core_entries)
print("PASS arm_grasp_stage_static")
