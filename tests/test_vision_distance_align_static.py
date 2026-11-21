from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
MAIN_C = (ROOT / "Core" / "Src" / "main.c").read_text(encoding="utf-8")
MAIN_H = (ROOT / "Core" / "Inc" / "main.h").read_text(encoding="utf-8")


class VisionDistanceAlignContract(unittest.TestCase):
    def test_profile_is_explicit_and_uses_step_backend(self):
        self.assertRegex(MAIN_H, r"#define\s+APP_PROFILE_VISION_DISTANCE_ALIGN\s+0U")
        self.assertIn("APP_PROFILE_VISION_DISTANCE_ALIGN)", MAIN_H)

    def test_fixed9_dx_mapping_becomes_bounded_distance(self):
        self.assertIn("VISION_ALIGN_DX_TO_X_MM             0.29411765f", MAIN_C)
        self.assertIn("VISION_ALIGN_MAX_DISTANCE_CM               3U", MAIN_C)
        self.assertIn("Chassis_MoveCm(x_rpm, 0, distance_cm)", MAIN_C)
        self.assertIn("VisionAlign_BuildMove", MAIN_C)

    def test_lost_and_complete_paths_stop_without_mission_autorun(self):
        moving = MAIN_C.split("case VISION_ALIGN_MOVING:", 1)[1].split(
            "case VISION_ALIGN_CENTER_CONFIRM:", 1
        )[0]
        self.assertRegex(
            moving,
            r"if \(\(have_observation == 0U\) \|\| \(report.control_valid == 0U\)\)"
            r"\s*\{\s*Chassis_Stop\(\);",
        )
        self.assertIn("VisionController_Reset(&vision_align_controller);", moving)
        main = MAIN_C.split("int main(void)", 1)[1]
        align_branches = main.split("#elif APP_PROFILE_VISION_DISTANCE_ALIGN")[1:]
        self.assertEqual(len(align_branches), 2)
        for branch in align_branches:
            self.assertNotIn("Mission_", branch.split("#elif APP_PROFILE_PASSIVE_VISION_REPORT", 1)[0])
        loop = align_branches[1].split("#elif APP_PROFILE_PASSIVE_VISION_REPORT", 1)[0]
        self.assertRegex(
            loop,
            r"if \(vision_align_state == VISION_ALIGN_COMPLETE\)\s*\{\s*Chassis_Stop\(\);",
        )


if __name__ == "__main__":
    unittest.main()
