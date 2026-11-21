import pathlib
import re
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
MAIN_H = (ROOT / "Core" / "Inc" / "main.h").read_text(encoding="utf-8")
MAIN_C = (ROOT / "Core" / "Src" / "main.c").read_text(encoding="utf-8")
VISION_C = (ROOT / "Core" / "Src" / "vision.c").read_text(encoding="utf-8")
OBSERVER_C = (ROOT / "Core" / "Src" / "vision_controller_observer.c").read_text(encoding="utf-8")
VISION_UART_H = (ROOT / "Core" / "Inc" / "vision_uart.h").read_text(encoding="utf-8")
VISION_UART_C = (ROOT / "Core" / "Src" / "vision_uart.c").read_text(encoding="utf-8")


class PassiveVisionReportStaticContract(unittest.TestCase):
    def test_passive_profile_is_available_and_opt_in(self):
        self.assertRegex(MAIN_H, r"#define\s+APP_PROFILE_PASSIVE_VISION_REPORT\s+0U")
        guard = MAIN_H.split("#if (APP_PROFILE_STEP_TEST", 1)[1].split("#error", 1)[0]
        self.assertIn("APP_PROFILE_PASSIVE_VISION_REPORT +", guard)

    def test_main_only_observes_and_reports_in_passive_profile(self):
        init_branch = re.search(
            r"#elif APP_PROFILE_PASSIVE_VISION_REPORT\n"
            r"  /\* Passive report image.*?(?P<body>.*?)#else",
            MAIN_C,
            re.DOTALL,
        )
        self.assertIsNotNone(init_branch)
        self.assertIn("Vision_Init();", init_branch.group("body"))
        self.assertIn("VisionControllerObserver_Init();", init_branch.group("body"))
        loop_branch = re.search(
            r"#elif APP_PROFILE_PASSIVE_VISION_REPORT\n"
            r"    /\* Only observe.*?(?P<body>.*?)#else",
            MAIN_C,
            re.DOTALL,
        )
        self.assertIsNotNone(loop_branch)
        self.assertIn("Vision_Service();", loop_branch.group("body"))
        self.assertIn("VisionControllerObserver_Service(HAL_GetTick());", loop_branch.group("body"))
        self.assertNotRegex(loop_branch.group("body"), r"\b(?:Motor|Chassis|StepMotor)_")

    def test_only_accepted_frames_advance_sample_identity(self):
        accepted = re.search(
            r"if \(VisionLink_TakeMessage\(&message\)\) \{(?P<body>.*?)\n    \}",
            VISION_C,
            re.DOTALL,
        )
        self.assertIsNotNone(accepted)
        self.assertIn("++vision_latest.sample_id;", accepted.group("body"))

    def test_observer_cannot_adopt_a_report_as_motion(self):
        self.assertIn("VisionController_Update", OBSERVER_C)
        self.assertIn("GPIO_PIN_10", OBSERVER_C)
        self.assertNotRegex(OBSERVER_C, r"\b(?:Chassis|Motor|Arm|StepMotor)_")

    def test_rough_mapping_is_explicit_and_visible_in_report(self):
        self.assertIn("rough_profile.raw_dx_bias = 342;", OBSERVER_C)
        self.assertIn("rough_profile.raw_dx_to_vehicle_mm = 0.353f;", OBSERVER_C)
        self.assertIn("rough_profile.error_to_delta_vx_polarity = -1;", OBSERVER_C)
        self.assertIn("e10=%ld", OBSERVER_C)

    def test_k230_transport_owns_usart6_on_pc6_pc7(self):
        self.assertIn("USART6, PC6 TX / PC7 RX", VISION_UART_H)
        self.assertIn("__HAL_RCC_GPIOC_CLK_ENABLE();", VISION_UART_C)
        self.assertIn("__HAL_RCC_USART6_CLK_ENABLE();", VISION_UART_C)
        self.assertIn("gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;", VISION_UART_C)
        self.assertIn("gpio.Alternate = GPIO_AF8_USART6;", VISION_UART_C)
        self.assertIn("HAL_GPIO_Init(GPIOC, &gpio);", VISION_UART_C)
        self.assertIn("HAL_RCC_GetPCLK2Freq()", VISION_UART_C)
        self.assertIn("void USART6_IRQHandler(void)", VISION_UART_C)
        self.assertNotIn("USART2", VISION_UART_C)
        self.assertNotIn("PD5/PD6", OBSERVER_C)


if __name__ == "__main__":
    unittest.main()
