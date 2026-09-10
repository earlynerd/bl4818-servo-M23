"""Compile production firmware logic against an in-memory peripheral model.

Run with a native GCC installed: python tests/test_firmware_native.py
This does not use the ARM cross-compiler or connect to hardware.
"""
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which("gcc"), "native GCC is required for firmware execution tests")
class FirmwareNativeTests(unittest.TestCase):
    def test_firmware_regressions(self):
        with tempfile.TemporaryDirectory(prefix="m2003-tests-") as directory:
            executable = Path(directory) / "firmware-tests.exe"
            sources = ["tests/native/firmware_regressions.c", "src/motor.c",
                       "src/strike.c", "src/pid.c", "src/crc16.c"]
            command = ["gcc", "-std=c11", "-O1", "-g", "-Wall", "-Wextra",
                       "-Wno-int-to-pointer-cast", "-ffunction-sections", "-fdata-sections",
                       "-Itests/native", "-Iinclude", *sources, "-Wl,--gc-sections",
                       "-o", str(executable)]
            build = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
            self.assertEqual(build.returncode, 0, build.stdout + build.stderr)
            run = subprocess.run([str(executable)], capture_output=True, text=True, timeout=10)
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            print(run.stdout.strip())


if __name__ == "__main__":
    unittest.main()
