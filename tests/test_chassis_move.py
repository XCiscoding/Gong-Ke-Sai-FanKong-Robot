import pathlib
import shutil
import subprocess
import tempfile
import unittest


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]


class ChassisMoveContractTest(unittest.TestCase):
    """验证整数任务门面只负责路线转换与委托。"""

    def test_native_c_contract(self) -> None:
        """编译并运行门面映射、拒绝和Service委托合同。"""
        gcc = shutil.which("gcc")
        self.assertIsNotNone(gcc, "gcc is required for the host-side C contract")

        with tempfile.TemporaryDirectory() as temp_dir:
            executable = pathlib.Path(temp_dir) / "chassis_move_contract.exe"
            compile_result = subprocess.run(
                [
                    gcc,
                    "-std=c11",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-I",
                    str(PROJECT_ROOT / "Core" / "Inc"),
                    str(PROJECT_ROOT / "Core" / "Src" / "chassis_move.c"),
                    str(PROJECT_ROOT / "tests" / "chassis_move_contract.c"),
                    "-lm",
                    "-o",
                    str(executable),
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(
                compile_result.returncode,
                0,
                compile_result.stdout + compile_result.stderr,
            )
            run_result = subprocess.run(
                [str(executable)], capture_output=True, text=True, check=False
            )
            self.assertEqual(
                run_result.returncode,
                0,
                run_result.stdout + run_result.stderr,
            )


if __name__ == "__main__":
    unittest.main()
