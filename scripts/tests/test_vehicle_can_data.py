import os
import shlex
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BOARD = ROOT / "main" / "boards" / "bread-compact-wifi-lcd"


class VehicleCanDataTest(unittest.TestCase):
    def test_twai_dependency_is_declared_for_early_expansion(self):
        cmake = (ROOT / "main" / "CMakeLists.txt").read_text()
        registration = cmake.split("idf_component_register(", 1)[1].split(")", 1)[0]
        private_requirements = registration.split("PRIV_REQUIRES", 1)[1].split()
        self.assertIn("esp_driver_twai", private_requirements)

    def test_vehicle_frames(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            executable = Path(temp_dir) / "vehicle_can_data_test"
            subprocess.run(
                shlex.split(os.environ.get("CXX", "c++"))
                + [
                    "-std=c++23",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-I",
                    str(BOARD),
                    str(Path(__file__).with_name("vehicle_can_data_test.cc")),
                    "-o",
                    str(executable),
                ],
                check=True,
            )
            subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    unittest.main()
