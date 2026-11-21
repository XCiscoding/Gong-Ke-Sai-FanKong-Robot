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
        self.mission_h = (
            PROJECT_ROOT / "Core/Inc/mission.h"
        ).read_text(encoding="utf-8")
        self.mission_c = (
            PROJECT_ROOT / "Core/Src/mission.c"
        ).read_text(encoding="utf-8")
        self.vision_link_c = (
            PROJECT_ROOT / "Core/Src/vision_link.c"
        ).read_text(encoding="utf-8")
        self.runner_c = (
            PROJECT_ROOT / "Core/Src/chassis_route_runner.c"
        ).read_text(encoding="utf-8")
        self.move_c = (
            PROJECT_ROOT / "Core/Src/chassis_move.c"
        ).read_text(encoding="utf-8")
        self.move_h = (
            PROJECT_ROOT / "Core/Inc/chassis_move.h"
        ).read_text(encoding="utf-8")
        self.route_h = (
            PROJECT_ROOT / "Core/Inc/chassis_route.h"
        ).read_text(encoding="utf-8")
        self.route_c = (
            PROJECT_ROOT / "Core/Src/chassis_route.c"
        ).read_text(encoding="utf-8")
        self.motor_c = (PROJECT_ROOT / "Core/Src/motor.c").read_text(encoding="utf-8")
        self.heading_h = (
            PROJECT_ROOT / "Core/Inc/chassis_heading.h"
        ).read_text(encoding="utf-8")
        self.chassis_config_h = (
            PROJECT_ROOT / "Core/Inc/chassis_config.h"
        ).read_text(encoding="utf-8")
        self.naming_source = "\n".join(
            (PROJECT_ROOT / path).read_text(encoding="utf-8")
            for path in (
                "Core/Inc/chassis_config.h",
                "Core/Inc/chassis_distance.h",
                "Core/Inc/chassis_move.h",
                "Core/Inc/chassis_route.h",
                "Core/Inc/chassis_route_runner.h",
                "Core/Inc/main.h",
                "Core/Inc/mission.h",
                "Core/Inc/motor.h",
                "Core/Inc/motor_units.h",
                "Core/Src/chassis_distance.c",
                "Core/Src/chassis_move.c",
                "Core/Src/chassis_route.c",
                "Core/Src/chassis_route_runner.c",
                "Core/Src/main.c",
                "Core/Src/mission.c",
                "Core/Src/motor.c",
            )
        )

    def test_exactly_one_profile_and_cm_route_is_selected(self) -> None:
        """画像必须且只能选择一个；当前运行完整路线而不依赖视觉前置。"""
        values = {
            name: int(value)
            for name, value in re.findall(
                r"#define\s+(APP_PROFILE_[A-Z0-9_]+)\s+([01])U", self.main_h
            )
        }
        self.assertEqual(sum(values.values()), 1, values)
        self.assertEqual(values["APP_PROFILE_CM_ROUTE_TEST"], 1)
        self.assertEqual(values["APP_PROFILE_VISION_DISTANCE_ALIGN"], 0)
        self.assertIn("APP_PROFILE_CM_ROUTE_TEST", self.main_h)
        self.assertIn("CHASSIS_USE_STEP_BACKEND", self.main_h)

    def test_cm_route_starts_mission_without_vision_gate(self) -> None:
        """纯路线画像直接初始化并服务任务，K230对齐仅属于独立画像。"""
        self.assertIn("APP_CM_ROUTE_TEST_MARKER 0x434D5231U", self.main_c)
        self.assertIn('#include "mission.h"', self.main_c)
        self.assertIn("Mission_Init()", self.main_c)
        self.assertIn("Mission_Service() == MISSION_FAILED", self.main_c)
        self.assertIn('#include "vision_controller.h"', self.main_c)
        self.assertIn("VisionAlign_Init();", self.main_c)
        self.assertIn("VisionAlign_Service();", self.main_c)
        runtime = self.main_c.split("int main(void)", 1)[1]
        route_startup = runtime.split("#elif APP_PROFILE_CM_ROUTE_TEST", 1)[1].split(
            "#elif APP_PROFILE_VISION_DISTANCE_ALIGN", 1
        )[0]
        self.assertIn("Mission_Init()", route_startup)
        self.assertIn("Vision_Init();", route_startup)
        self.assertNotIn("VisionAlign_", route_startup)
        route_loop = self.main_c.split("#elif APP_PROFILE_CM_ROUTE_TEST\n    /* 纯路线", 1)[1].split(
            "#elif APP_PROFILE_VISION_DISTANCE_ALIGN", 1
        )[0]
        self.assertIn("Mission_Service()", route_loop)
        self.assertIn("Vision_Service();", route_loop)
        self.assertIn("Vision_EnableReceiveOnly();", route_startup)
        self.assertIn("void VisionLink_EnableReceiveOnly(void)", self.vision_link_c)
        self.assertIn("if (receive_only != 0U) return 0U;", self.vision_link_c)
        self.assertIn("if (receive_only == 0U && !link_diag.link_active", self.vision_link_c)
        self.assertNotIn("UART4", self.vision_link_c)
        self.assertNotIn("VisionLinkTelemetry_", self.vision_link_c)
        self.assertNotIn("VisionAlign_", route_loop)
        self.assertNotIn("cm_route_test_plan", self.main_c)
        self.assertNotIn("CmRouteTest_", self.main_c)
        self.assertIn("MISSION_ACTION_MOVE", self.mission_c)
        self.assertIn("0, 75, 70", self.mission_c)
        self.assertIn("-60, 0, 73", self.mission_c)
        self.assertIn("0, 90, 104", self.mission_c)
        self.assertIn("-75, 0, 90", self.mission_c)
        self.assertIn("0, 75, 105", self.mission_c)
        self.assertIn("MISSION_NODE_PRE_BUMP_CHECK", self.mission_c)
        self.assertIn("MISSION_NODE_POST_BUMP_CHECK", self.mission_c)
        self.assertIn("MISSION_NODE_END_CHECK", self.mission_c)
        self.assertIn("MISSION_ACTION_TASK_STOP", self.mission_c)
        plan = self.mission_c.split("mission_plan[] = {", 1)[1].split("};", 1)[0]
        self.assertEqual(plan.count("MISSION_ACTION_MOVE"), 7)
        self.assertEqual(plan.count("MISSION_ACTION_CHECKPOINT"), 3)
        self.assertNotIn("MISSION_ACTION_QR_WAIT", plan)
        self.assertNotIn("MISSION_ACTION_BALL_OBSERVE", plan)
        self.assertNotIn("MISSION_QR_WAIT_TIMEOUT_MS", self.mission_c)
        self.assertNotIn("Vision_RequestQr", self.mission_c)
        self.assertNotIn("Vision_TakeQrReady", self.mission_c)
        self.assertNotIn("Vision_RequestTarget", self.mission_c)
        self.assertNotIn("Vision_RequestTrack", self.mission_c)
        self.assertNotIn("mission_waiting_qr", self.mission_c)
        self.assertIn("status = Chassis_Service()", self.mission_c)
        self.assertIn("status == CHASSIS_MOVE_FAILED", self.mission_c)
        self.assertIn("MISSION_COMPLETE", self.mission_h)

    def test_vision_alignment_is_bounded_in_its_own_profile(self) -> None:
        """视觉对齐保留独立画像、失帧、超时和全停边界，不阻塞纯路线。"""
        self.assertIn("VISION_ALIGN_TOTAL_TIMEOUT_MS", self.main_c)
        self.assertIn("VISION_ALIGN_CENTER_CONFIRM_SAMPLES", self.main_c)
        self.assertIn("VISION_ALIGN_DX_TO_X_MM", self.main_c)
        self.assertIn("VISION_ALIGN_MAX_DISTANCE_CM", self.main_c)
        self.assertIn("VISION_CONTROLLER_REASON_CENTERED", self.main_c)
        self.assertIn("VISION_CONTROLLER_REASON_TRACKING", self.main_c)
        self.assertIn("Chassis_MoveCm(x_rpm, 0, distance_cm)", self.main_c)
        self.assertIn("Chassis_Service()", self.main_c)
        self.assertIn("Chassis_Stop();", self.main_c)
        runtime = self.main_c.split("int main(void)", 1)[1]
        self.assertLess(runtime.index("VisionAlign_Init();"), runtime.index("while (1)"))
        self.assertIn("#elif APP_PROFILE_VISION_DISTANCE_ALIGN", runtime)
        self.assertNotIn("mission_started", runtime)

    def test_cm_route_profile_does_not_own_arm_output(self) -> None:
        """完整路线画像不初始化机械臂或提交PCA输出，保持既有初始姿态。"""
        runtime = self.main_c.split("int main(void)", 1)[1]
        route_startup = runtime.split("#elif APP_PROFILE_CM_ROUTE_TEST", 1)[1].split(
            "#elif APP_PROFILE_VISION_DISTANCE_ALIGN", 1
        )[0]
        route_loop = self.main_c.split("#elif APP_PROFILE_CM_ROUTE_TEST\n    /* 纯路线", 1)[1].split(
            "#elif APP_PROFILE_VISION_DISTANCE_ALIGN", 1
        )[0]
        for branch in (route_startup, route_loop):
            self.assertNotIn("ArmTune_", branch)
            self.assertNotIn("Pca_", branch)

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
        self.assertIn("ChassisRouteRunner_Init(&segment)", self.move_c)
        self.assertNotIn("Motor_", self.move_c)
        self.assertNotIn("HAL_", self.move_c)

    def test_runner_uses_safe_one_shot_lifecycle(self) -> None:
        """运行器必须具有单次运行、超时和统一停车锁存状态。"""
        for state in (
            "CHASSIS_ROUTE_RUNNER_BOOT_SAFE",
            "CHASSIS_ROUTE_RUNNER_STABILIZE",
            "CHASSIS_ROUTE_RUNNER_START_RAMP",
            "CHASSIS_ROUTE_RUNNER_RUN_ONCE",
            "CHASSIS_ROUTE_RUNNER_STOP_LATCHED",
        ):
            self.assertIn(state, self.runner_c)
        self.assertIn("ChassisRouteRunner_Stop(CHASSIS_ROUTE_STOP_COMPLETE)", self.runner_c)
        self.assertIn("ChassisRouteRunner_Stop(CHASSIS_ROUTE_STOP_TIMEOUT)", self.runner_c)
        self.assertIn("Chassis_Stop();", self.runner_c)

    def test_runner_keeps_the_yaw_armed_at_start(self) -> None:
        """厘米直行段必须保持起步Yaw，不能改追绝对0度。"""
        start = self.runner_c.split(
            "static void ChassisRouteRunner_Start(void)", 1
        )[1].split("uint8_t ChassisRouteRunner_Init", 1)[0]
        self.assertIn("ChassisHeading_ArmCurrent()", start)
        self.assertIn("ChassisHeading_Compute(&heading_w)", start)
        self.assertNotIn("ChassisHeading_SetTarget(", start)

    def test_active_heading_gain_changes_only_p(self) -> None:
        """路线回归已实测P基线，保持I/D与安全边界不变。"""
        self.assertIn("#define CHASSIS_HEADING_KP             1.0f", self.heading_h)
        self.assertIn("#define CHASSIS_HEADING_KI             0.0f", self.heading_h)
        self.assertIn("#define CHASSIS_HEADING_KD             0.0f", self.heading_h)
        self.assertIn("#define CHASSIS_HEADING_DEADBAND_DEG   0.55f", self.heading_h)
        self.assertIn("#define CHASSIS_HEADING_LIMIT_RPM     15.0f", self.heading_h)
        self.assertIn("#define CHASSIS_HEADING_HOLD_ERROR_LIMIT_TENTHS 400", self.heading_h)

    def test_user_axis_mapping_uses_matching_internal_channels(self) -> None:
        """用户X/Y坐标分别进入同名内部通道与脉冲投影。"""
        x_positive = self.route_c.split("case CHASSIS_ROUTE_X_POSITIVE:", 1)[1].split(
            "case CHASSIS_ROUTE_X_NEGATIVE:", 1
        )[0]
        y_positive = self.route_c.split("case CHASSIS_ROUTE_Y_POSITIVE:", 1)[1].split(
            "case CHASSIS_ROUTE_Y_NEGATIVE:", 1
        )[0]
        self.assertIn("projection = CHASSIS_DISTANCE_PROJECTION_MIXER_X", x_positive)
        self.assertIn("resolved->internal_vx_rpm = segment->speed_rpm", x_positive)
        self.assertIn("projection = CHASSIS_DISTANCE_PROJECTION_MIXER_Y", y_positive)
        self.assertIn("resolved->internal_vy_rpm = segment->speed_rpm", y_positive)

    def test_runner_uses_a_uniform_start_ramp_without_recalibrating_distance(self) -> None:
        """起步斜坡只缩放路线提交，不得改厘米标定、路线几何或电机事务语义。"""
        self.assertIn("#define CHASSIS_ROUTE_START_RAMP_MS                 300U", self.chassis_config_h)
        self.assertIn("#define CHASSIS_ROUTE_START_RAMP_INITIAL_RATIO     0.35f", self.chassis_config_h)
        self.assertIn("ChassisRouteRunner_StartRampScale", self.runner_c)
        self.assertIn("ChassisRouteRunner_Submit", self.runner_c)
        self.assertIn("heading_w * speed_scale", self.runner_c)
        self.assertIn("duration_ms += (float)CHASSIS_ROUTE_START_RAMP_MS", self.runner_c)
        self.assertNotIn("CHASSIS_X_CAL", self.runner_c)
        self.assertNotIn("CHASSIS_Y_CAL", self.runner_c)

    def test_active_axis_calibration_matches_the_frozen_y_and_x_candidate(self) -> None:
        """Y+冻结标定保持不变，X标定只改为本轮实测推导值。"""
        self.assertIn("#define CHASSIS_X_CAL                        0.922f", self.chassis_config_h)
        self.assertIn("#define CHASSIS_Y_CAL                        0.90f", self.chassis_config_h)

    def test_direction_evidence_is_not_a_firmware_permission_system(self) -> None:
        """方向证据留在项目记录；固件只解析合法的XY符号命令。"""
        for removed_name in (
            "Chassis_ProbeXPositiveCm",
            "CHASSIS_ROUTE_PERMISSION_",
            "CHASSIS_PHYSICAL_X_POSITIVE_VERIFIED",
            "CHASSIS_X_POSITIVE_PROBE_AUTHORIZED",
            "APP_PROFILE_CM_ROUTE_X_POSITIVE_PROBE",
            "APP_PROFILE_CM_ROUTE_Y_POSITIVE",
        ):
            self.assertNotIn(removed_name, self.naming_source)

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
