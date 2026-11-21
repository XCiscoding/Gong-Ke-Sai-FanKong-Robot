import pathlib
import shutil
import subprocess
import tempfile
import unittest


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]


class MissionContractTest(unittest.TestCase):
    """验证任务路线顺序，整个合同不打开任何硬件。"""

    def test_native_c_contract(self) -> None:
        gcc = shutil.which("gcc")
        self.assertIsNotNone(gcc, "a runnable host GCC is required for the C contract")
        with tempfile.TemporaryDirectory(prefix="pbrobot-mission-") as temp_dir:
            executable = pathlib.Path(temp_dir) / "mission_contract.exe"
            compile_result = subprocess.run(
                [
                    gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-I", str(PROJECT_ROOT / "Core" / "Inc"),
                    str(PROJECT_ROOT / "Core" / "Src" / "mission.c"),
                    str(PROJECT_ROOT / "tests" / "mission_contract.c"),
                    "-o", str(executable),
                ],
                cwd=PROJECT_ROOT, capture_output=True, text=True, check=False,
            )
            self.assertEqual(
                compile_result.returncode, 0,
                compile_result.stdout + compile_result.stderr,
            )
            run_result = subprocess.run(
                [str(executable)], capture_output=True, text=True, check=False
            )
            self.assertEqual(
                run_result.returncode, 0,
                run_result.stdout + run_result.stderr,
            )


if __name__ == "__main__":
    unittest.main()
