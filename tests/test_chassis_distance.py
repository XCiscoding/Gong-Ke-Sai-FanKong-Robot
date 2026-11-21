import pathlib
import shutil
import subprocess
import tempfile
import unittest


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]


class ChassisDistanceContractTest(unittest.TestCase):
    def test_native_c_contract(self) -> None:
        gcc = shutil.which("gcc")
        self.assertIsNotNone(gcc, "gcc is required for the host-side C contract")

        with tempfile.TemporaryDirectory(prefix="pbrobot-distance-") as temp_dir:
            executable = pathlib.Path(temp_dir) / "chassis_distance_contract.exe"
            compile_result = subprocess.run(
                [
                    gcc,
                    "-std=c11",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-I",
                    str(PROJECT_ROOT / "Core" / "Inc"),
                    str(PROJECT_ROOT / "Core" / "Src" / "chassis_distance.c"),
                    str(PROJECT_ROOT / "tests" / "chassis_distance_contract.c"),
                    "-o",
                    str(executable),
                ],
                cwd=PROJECT_ROOT,
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
                [str(executable)],
                cwd=PROJECT_ROOT,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(
                run_result.returncode,
                0,
                run_result.stdout + run_result.stderr,
            )


if __name__ == "__main__":
    unittest.main()
