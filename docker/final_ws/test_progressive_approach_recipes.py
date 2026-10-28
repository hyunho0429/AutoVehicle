"""Saved approach settings must reach profile 2 without launching Docker/ROS."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


DIRECTORY = Path(__file__).resolve().parent
BASH = shutil.which("bash")
if os.name == "nt" and Path("C:/Program Files/Git/bin/bash.exe").is_file():
    BASH = "C:/Program Files/Git/bin/bash.exe"


def write_utf8(path, text):
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(text)


@unittest.skipUnless(BASH, "bash is required for the host recipe")
class ProgressiveApproachRecipesTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="morai-approach-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        for name in ("run_test.sh", "curvature_runtime_files.sh", "highway-test.env.example"):
            write_utf8(self.root / name, (DIRECTORY / name).read_text(encoding="utf-8"))
        write_utf8(self.root / "highway.env",
            "UBUNTU_IP=192.168.10.21\nMORAI_IP=192.168.10.14\nCONTAINER_NAME=fixture\n",
        )
        self.settings = self.root / "highway-test.env"
        write_utf8(self.settings,
            "TEST_PROFILE=curvature_signal\nMAX_SPEED_KPH=35.0\n"
            "STOPLINE_PLANNING_DECEL_MPS2=0.6\nSTOPLINE_HOLD_DISTANCE_M=6.0\n"
            "STOPLINE_BRAKE_RAMP_DISTANCE_M=30.0\nSTOPLINE_SETTLE_DISTANCE_M=0.8\n")

    def run_recipe(self, *arguments, success=True):
        result = subprocess.run([BASH, str(self.root / "run_test.sh"), *arguments],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode == 0, success, result.stdout + result.stderr)
        return result.stdout

    def test_saved_old_tuning_is_replaced_and_reaches_launch_arguments(self):
        previous = self.settings.read_text()
        self.run_recipe("approach", "smooth")
        self.assertEqual((self.root / "highway-test.env.bak").read_text(), previous)
        output = self.run_recipe("show", "2")
        for argument in ("max_speed_kph:=35.0", "stopline_hold_distance_m:=0.5",
                         "stopline_planning_decel_mps2:=1.2",
                         "stopline_brake_ramp_distance_m:=0.0",
                         "stopline_settle_distance_m:=0.15"):
            self.assertIn(argument, output)

    def test_approach_tune_respects_a_lower_configured_braking_limit(self):
        with self.settings.open("a") as stream:
            stream.write("MAX_DECEL_MPS2=0.8\n")
        self.run_recipe("approach", "smooth")
        self.assertIn("stopline_planning_decel_mps2:=0.8", self.run_recipe("show", "2"))

    def test_invalid_preset_does_not_modify_saved_settings(self):
        previous = self.settings.read_text()
        self.run_recipe("approach", "unknown", success=False)
        self.assertEqual(self.settings.read_text(), previous)

    def test_turn_preset_keeps_half_metre_stop_clearance(self):
        self.run_recipe("turn", "mission")
        output = self.run_recipe("show", "2")
        self.assertIn("max_speed_kph:=55.0", output)
        self.assertIn("stopline_hold_distance_m:=0.5", output)
        self.assertIn("stopline_planning_decel_mps2:=1.2", output)


if __name__ == "__main__":
    unittest.main()
