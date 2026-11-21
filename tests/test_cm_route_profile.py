import pathlib
import re
import unittest


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]


class CmRouteProfileContractTest(unittest.TestCase):
    """验证厘米路线画像的编译选择、调用链和安全生命周期。"""

    def setUp(self) -> None:
        """读取每项合同所依赖的当前源码。"""
        self.main_h = (PROJECT_ROOT / "Core/Inc/main.h").read_text(encoding="utf-8")
        self.main_c = (PROJECT_ROOT / "Core/Src/main.c").read_text(encoding="utf-8")
        self.runner_c = (
            PROJECT_ROOT / "Core/Src/chassis_route_runner.c"
        ).read_text(encoding="utf-8")
        self.move_c = (
            PROJECT_ROOT / "Core/Src/chassis_move.c"
        ).read_text(encoding="utf-8")
        self.motor_c = (PROJECT_ROOT / "Core/Src/motor.c").read_text(encoding="utf-8")
        self.naming_source = "\n".join(
            (PROJECT_ROOT / path).read_text(encoding="utf-8")
            for path in (
                "Core/Inc/chassis_config.h",
                "Core/Inc/chassis_distance.h",
                "Core/Inc/chassis_move.h",
                "Core/Inc/chassis_route.h",
                "Core/Inc/chassis_route_runner.h",
                "Core/Inc/main.h",
                "Core/Inc/motor.h",
                "Core/Inc/motor_units.h",
                "Core/Src/chassis_distance.c",
                "Core/Src/chassis_move.c",
                "Core/Src/chassis_route.c",
                "Core/Src/chassis_route_runner.c",
                "Core/Src/main.c",
                "Core/Src/motor.c",
            )
        )

    def test_exactly_one_profile_and_cm_route_is_selected(self) -> None:
        """画像必须且只能选择厘米X正路线，并使用直脉冲后端。"""
        values = {
            name: int(value)
            for name, value in re.findall(
                r"#define\s+(APP_PROFILE_[A-Z0-9_]+)\s+([01])U", self.main_h
            )
        }
        self.assertEqual(sum(values.values()), 1, values)
        self.assertEqual(values["APP_PROFILE_CM_ROUTE_X_POSITIVE"], 1)
        self.assertIn("APP_PROFILE_CM_ROUTE_X_POSITIVE", self.main_h)
        self.assertIn("CHASSIS_USE_STEP_BACKEND", self.main_h)

    def test_main_connects_only_the_cm_route_service_for_this_profile(self) -> None:
        """main必须设置CMX1身份并按IMU→路线顺序调度。"""
        self.assertIn("APP_CM_ROUTE_X_POSITIVE_MARKER 0x434D5831U", self.main_c)
        self.assertIn("Chassis_MoveCm(50, 0, 10)", self.main_c)
        self.assertIn("ImuMs901m_Service();\n    Chassis_Service();", self.main_c)

    def test_task_api_is_integer_only_and_keeps_runner_internal(self) -> None:
        """任务层只传整数X/Y/cm，门面再委托既有路线运行器。"""
        move_h = (PROJECT_ROOT / "Core/Inc/chassis_move.h").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            "Chassis_MoveCm(int16_t x_rpm, int16_t y_rpm,",
            move_h,
        )
        self.assertIn("uint16_t distance_cm", move_h)
        self.assertIn("ChassisRoute_ResolveSegment(&segment, &resolved)", self.move_c)
        self.assertIn("ChassisRouteRunner_Init(&segment)", self.move_c)
        self.assertNotIn("Motor_", self.move_c)
        self.assertNotIn("HAL_", self.move_c)

    def test_runner_uses_safe_one_shot_lifecycle(self) -> None:
        """运行器必须具有单次运行、超时和统一停车锁存状态。"""
        for state in (
            "CHASSIS_ROUTE_RUNNER_BOOT_SAFE",
            "CHASSIS_ROUTE_RUNNER_STABILIZE",
            "CHASSIS_ROUTE_RUNNER_RUN_ONCE",
            "CHASSIS_ROUTE_RUNNER_STOP_LATCHED",
        ):
            self.assertIn(state, self.runner_c)
        self.assertIn("ChassisRouteRunner_Stop(CHASSIS_ROUTE_STOP_COMPLETE)", self.runner_c)
        self.assertIn("ChassisRouteRunner_Stop(CHASSIS_ROUTE_STOP_TIMEOUT)", self.runner_c)
        self.assertIn("Chassis_Stop();", self.runner_c)

    def test_motor_layer_records_signed_steps_without_owning_xy_semantics(self) -> None:
        """电机层只记录方向脉冲，不得反向依赖路线坐标。"""
        self.assertIn("motor_signed_step_count[index] += motor_command_sign[index]", self.motor_c)
        self.assertIn("int32_t Motor_GetSignedStepCount", self.motor_c)
        self.assertNotIn("CHASSIS_ROUTE_", self.motor_c)

    def test_removed_names_do_not_return(self) -> None:
        """活动接口不得重新引入旧pulse或scale命名。"""
        for removed_name in (
            "APP_PROFILE_ORIGINAL_PULSE_TEST",
            "CHASSIS_USE_DIRECT_PULSE_BACKEND",
            "Motor_GetPulseCount",
            "Motor_GetSignedPulseCount",
            "Motor_ResetPulseCount",
            "Motor_SetPulseLimit",
            "target_translation_pulses",
            "CHASSIS_X_PULSES_PER_CM_SCALE",
            "CHASSIS_Y_PULSES_PER_CM_SCALE",
        ):
            self.assertNotIn(removed_name, self.naming_source)


if __name__ == "__main__":
    unittest.main()
