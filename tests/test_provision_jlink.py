import sys
import struct
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from firmware_image import build_manifest, prepare_image  # noqa: E402
from make_provision_jlink import (  # noqa: E402
    AHBCLK_RESET_WITH_ISP,
    CLK_AHBCLK,
    CMD_PAGE_ERASE,
    CMD_PROGRAM,
    CONFIG_BASE,
    generate_config_read_script,
    generate_provision_script,
    generate_verify_script,
)
from m2003_configure_ldrom import (  # noqa: E402
    CONFIG2_UNLOCKED,
    CONFIG_BASE as CONFIG_UPDATE_BASE,
    ConfigPlan,
    generate_config_erase_script,
    generate_config_program_script,
    make_config_plan,
    parse_config_capture,
    read_config_capture,
)


def valid_image(payload: bytes) -> bytes:
    return prepare_image(struct.pack("<II", 0x20000FF0, 0x00000009) + payload)


class ProvisionJLinkTests(unittest.TestCase):
    def test_provision_script_never_targets_configuration(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            image = valid_image(b"\x00\x01\x02\x03" * 64)
            app = root / "app.bin"
            manifest = root / "manifest.bin"
            ldrom = root / "ldrom.bin"
            output = root / "provision.jlink"
            app.write_bytes(image)
            manifest.write_bytes(build_manifest(image))
            ldrom.write_bytes(b"\x00\x00\x00\x00")

            generate_provision_script(app, manifest, ldrom, output)
            text = output.read_text(encoding="ascii")

            self.assertNotIn(f"0x{CONFIG_BASE:08X}", text)
            self.assertIn(f"0x{CMD_PROGRAM:08X}", text)
            self.assertIn(f"0x{CMD_PAGE_ERASE:08X}", text)
            self.assertIn("NO CONFIG WRITES", text)
            manifest_target = f"w4 0x4000C004 0x00007A00"
            manifest_word_one = f"w4 0x4000C004 0x00007A04"
            app_target = f"w4 0x4000C004 0x00000000"
            self.assertLess(text.index(manifest_target), text.index(app_target))
            self.assertGreater(text.rindex(manifest_target), text.rindex(manifest_word_one))

    def test_config_script_contains_reads_but_no_program_or_erase(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "read-config.jlink"
            generate_config_read_script(output)
            text = output.read_text(encoding="ascii")

            self.assertIn(f"0x{CONFIG_BASE:08X}", text)
            self.assertNotIn(f"0x{CMD_PROGRAM:08X}", text)
            self.assertNotIn(f"0x{CMD_PAGE_ERASE:08X}", text)
            self.assertIn("READ ONLY", text)
            self.assertIn(
                f"w4 0x{CLK_AHBCLK:08X} 0x{AHBCLK_RESET_WITH_ISP:08X}", text
            )

    def test_verify_script_reads_aprom_vector_page_through_fmc(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            image = valid_image(b"physical verification" * 40)
            app = root / "app.bin"
            manifest = root / "manifest.bin"
            ldrom = root / "ldrom.bin"
            output = root / "verify.jlink"
            app.write_bytes(image)
            manifest.write_bytes(build_manifest(image))
            ldrom.write_bytes(b"\x00\x00\x00\x00")

            generate_verify_script(app, manifest, ldrom, output)
            text = output.read_text(encoding="ascii")

            self.assertIn("physical firmware verification (READ ONLY)", text)
            self.assertEqual(text.count("mem32 0x4000C008 1"), 0x200 // 4)
            self.assertIn("app-tail-readback.bin", text)
            self.assertIn("0x00000200", text)
            self.assertNotIn("savebin " + str(root / "app-readback.bin"), text)
            self.assertNotIn(f"0x{CMD_PROGRAM:08X}", text)
            self.assertNotIn(f"0x{CMD_PAGE_ERASE:08X}", text)

    def test_provision_rejects_manifest_for_a_different_application(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            app = root / "app.bin"
            manifest = root / "manifest.bin"
            ldrom = root / "ldrom.bin"
            app.write_bytes(valid_image(b"application A"))
            manifest.write_bytes(build_manifest(valid_image(b"application B")))
            ldrom.write_bytes(b"\x00\x00\x00\x00")

            with self.assertRaisesRegex(ValueError, "CRC does not match"):
                generate_provision_script(
                    app, manifest, ldrom, root / "provision.jlink"
                )

    def test_standard_jlink_path_always_includes_ldrom(self):
        makefile = (REPO_ROOT / "Makefile").read_text(encoding="utf-8")
        flash_script = (REPO_ROOT / "scripts" / "flash-jlink.ps1").read_text(
            encoding="utf-8"
        )

        self.assertIn("build-jlink: images", makefile)
        self.assertNotIn("provision-jlink:", makefile)
        self.assertIn("m2003-motor.jlink", flash_script)
        self.assertIn("m2003-firmware-verify.jlink", flash_script)
        self.assertIn("Convert-FmcReadLogToBinary", flash_script)
        self.assertIn('Assert-BinaryEqual "LDROM"', flash_script)
        self.assertIn("m2003_configure_ldrom.py", flash_script)
        self.assertIn("--phase erased", flash_script)
        self.assertIn("--phase final", flash_script)
        self.assertNotIn("0xFFFFFF7F", flash_script)


class ConfigureLdromBootTests(unittest.TestCase):
    @staticmethod
    def capture(*values: int) -> str:
        return "\n".join(f"4000C008 = {value:08X} " for value in values)

    def test_capture_parser_requires_exactly_three_config_reads(self):
        expected = (0xFFFFFFFF, 0x12345678, CONFIG2_UNLOCKED)
        self.assertEqual(parse_config_capture(self.capture(*expected)), expected)
        with self.assertRaisesRegex(ValueError, "exactly three"):
            parse_config_capture(self.capture(*expected[:2]))

    def test_capture_reader_accepts_windows_utf16_logs(self):
        with tempfile.TemporaryDirectory() as tmp:
            capture = Path(tmp) / "capture.log"
            expected = (0xFFFFFFFF, 0xFFFFFFFF, CONFIG2_UNLOCKED)
            capture.write_text(self.capture(*expected), encoding="utf-16")
            self.assertEqual(read_config_capture(capture), expected)

    def test_plan_clears_only_cbs_and_preserves_other_words(self):
        original = (0xFFFFFDFF, 0x12345678, CONFIG2_UNLOCKED)
        plan = make_config_plan(original)
        self.assertEqual(plan.desired, (0xFFFFFD7F, original[1], original[2]))
        self.assertTrue(plan.needs_update)

    def test_plan_rejects_locked_or_unexpected_security_config(self):
        with self.assertRaisesRegex(ValueError, "LOCK"):
            make_config_plan((0xFFFFFFFD, 0xFFFFFFFF, CONFIG2_UNLOCKED))
        with self.assertRaisesRegex(ValueError, "not unlocked"):
            make_config_plan((0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF))

    def test_config_erase_and_program_scripts_are_two_verified_phases(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            erase = root / "erase.jlink"
            program = root / "program.jlink"
            plan = ConfigPlan(
                original=(0xFFFFFFDF, 0xFFFFFFFE, CONFIG2_UNLOCKED),
                desired=(0xFFFFFF5F, 0xFFFFFFFE, CONFIG2_UNLOCKED),
            )
            generate_config_erase_script(erase)
            generate_config_program_script(plan, program)
            erase_text = erase.read_text(encoding="ascii")
            program_text = program.read_text(encoding="ascii")

            self.assertIn(f"0x{CMD_PAGE_ERASE:08X}", erase_text)
            self.assertNotIn(f"0x{CMD_PROGRAM:08X}", erase_text)
            self.assertIn(f"0x{CMD_PROGRAM:08X}", program_text)
            config0_target = f"w4 0x4000C004 0x{CONFIG_UPDATE_BASE:08X}"
            config1_target = f"w4 0x4000C004 0x{CONFIG_UPDATE_BASE + 4:08X}"
            self.assertLess(program_text.index(config1_target), program_text.index(config0_target))
            self.assertNotIn("w4 0x4000C008 0xFFFFFF5A", program_text)


if __name__ == "__main__":
    unittest.main()
