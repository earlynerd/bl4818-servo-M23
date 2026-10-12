"""Startup performance guard and scoped registry writes; no hardware access."""
import contextlib
import io
import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import serial_latency as latency
import ring_midi_server as server


def device(port="COM29", serial="A10JO345A", vid=0x0403, pid=0x6001):
    return SimpleNamespace(device=port, serial_number=serial, vid=vid, pid=pid,
                           manufacturer="FTDI" if vid==0x0403 else "wch.cn", hwid="")


def registry_path(d):
    return (f"SYSTEM\\CurrentControlSet\\Enum\\FTDIBUS\\VID_{d.vid:04X}+PID_{d.pid:04X}"
            f"+{d.serial_number}\\0000\\Device Parameters")


class FakeRegistry:
    HKEY_LOCAL_MACHINE = "HKLM"
    REG_DWORD = 4
    KEY_READ = 1
    KEY_SET_VALUE = 2

    def __init__(self, devices):
        self.values = {registry_path(d): {"PortName": (d.device, 1), "LatencyTimer": (2, 4)}
                       for d in devices if d.vid==0x0403}
        self.opens = []
        self.writes = []
        self.deny_write = None
        self.fail_set = None

    @contextlib.contextmanager
    def OpenKey(self, root, path, reserved=0, access=1):
        self.opens.append((path,access))
        if path not in self.values:
            raise FileNotFoundError(path)
        if access & self.KEY_SET_VALUE and path==self.deny_write:
            raise PermissionError("requires Administrator")
        yield path

    def QueryValueEx(self, key, name):
        if name not in self.values[key]:
            raise FileNotFoundError(name)
        return self.values[key][name]

    def SetValueEx(self, key, name, reserved, kind, value):
        self.writes.append((key,name,value))
        if key==self.fail_set and value==2:
            raise OSError("simulated write failure")
        self.values[key][name]=(value,kind)


class SerialLatencyTests(unittest.TestCase):
    def setUp(self):
        self.first=device()
        self.second=device("COM48","BG03STIBA")
        self.other=device("COM14","OTHER",0x1A86,0x55D5)
        self.devices=[self.first,self.second,self.other]
        self.registry=FakeRegistry(self.devices)
        for patcher in [patch.object(latency.sys,"platform","win32"),
                        patch.object(latency.list_ports,"comports",return_value=self.devices),
                        patch.dict(sys.modules,{"winreg":self.registry})]:
            patcher.start()
            self.addCleanup(patcher.stop)

    def test_saved_2ms_checks_are_read_only_and_exactly_scoped(self):
        result=latency.check_serial_latency(["com29",r"\\.\COM29"])
        self.assertEqual(len(result),1)
        self.assertEqual(result[0].latency_ms,2)
        self.assertEqual(self.registry.opens,[(registry_path(self.first),1)])
        self.assertEqual(self.registry.writes,[])

    def test_16ms_blocks_with_specific_repair_command(self):
        self.registry.values[registry_path(self.second)]["LatencyTimer"]=(16,4)
        with self.assertRaisesRegex(latency.SerialLatencyError,"--port COM48 --set-2ms"):
            latency.check_serial_latency(["COM29","COM48"])
        self.assertEqual(self.registry.writes,[])

    def test_non_ftdi_port_does_not_read_or_write_ftdi_registry(self):
        result=latency.check_serial_latency(["COM14"])
        self.assertIsNone(result[0].registry_key)
        self.assertEqual(self.registry.opens,[])

    def test_non_windows_preserves_existing_port_workflow(self):
        with patch.object(latency.sys,"platform","linux"):
            self.assertEqual(latency.check_serial_latency(["/dev/ttyUSB0"])[0].port,"/dev/ttyUSB0")
        self.assertEqual(self.registry.opens,[])

    def test_missing_duplicate_or_incomplete_identity_is_not_assumed_fast(self):
        with self.assertRaises(latency.SerialLatencyError):
            latency.check_serial_latency(["COM99"])
        with patch.object(latency.list_ports,"comports",return_value=[self.first,self.first]):
            with self.assertRaises(latency.SerialLatencyError):
                latency.check_serial_latency(["COM29"])
        self.first.serial_number=None
        with self.assertRaises(latency.SerialLatencyError):
            latency.check_serial_latency(["COM29"])

    def test_registry_port_identity_mismatch_blocks(self):
        self.registry.values[registry_path(self.first)]["PortName"]=("COM99",1)
        with self.assertRaisesRegex(latency.SerialLatencyError,"identity belongs to COM99"):
            latency.check_serial_latency(["COM29"])

    def test_missing_or_invalid_latency_cannot_pass(self):
        for value in [(0,4),(256,4),("2",1)]:
            with self.subTest(value=value):
                self.registry.values[registry_path(self.first)]["LatencyTimer"]=value
                with self.assertRaises(latency.SerialLatencyError):
                    latency.check_serial_latency(["COM29"])
        del self.registry.values[registry_path(self.first)]["LatencyTimer"]
        with self.assertRaisesRegex(latency.SerialLatencyError,"cannot read"):
            latency.check_serial_latency(["COM29"])

    def test_explicit_fix_only_writes_requested_adapter(self):
        self.registry.values[registry_path(self.first)]["LatencyTimer"]=(16,4)
        self.registry.values[registry_path(self.second)]["LatencyTimer"]=(16,4)
        latency.configure_serial_latency(["COM29"])
        self.assertEqual(self.registry.writes,[(registry_path(self.first),"LatencyTimer",2)])
        self.assertEqual(self.registry.values[registry_path(self.second)]["LatencyTimer"],(16,4))

    def test_correct_setting_needs_no_write(self):
        latency.configure_serial_latency(["COM29"])
        self.assertEqual(self.registry.writes,[])

    def test_all_write_permissions_checked_before_first_change(self):
        self.registry.values[registry_path(self.first)]["LatencyTimer"]=(16,4)
        self.registry.deny_write=registry_path(self.second)
        with self.assertRaisesRegex(latency.SerialLatencyError,"No settings changed"):
            latency.configure_serial_latency(["COM29","COM48"])
        self.assertEqual(self.registry.writes,[])

    def test_partial_failure_restores_original_values(self):
        for d in [self.first,self.second]:
            self.registry.values[registry_path(d)]["LatencyTimer"]=(16,4)
        self.registry.fail_set=registry_path(self.second)
        with self.assertRaisesRegex(latency.SerialLatencyError,"Original settings restored"):
            latency.configure_serial_latency(["COM29","COM48"])
        for d in [self.first,self.second]:
            self.assertEqual(self.registry.values[registry_path(d)]["LatencyTimer"],(16,4))

    def test_mixed_adapter_fix_is_rejected_before_any_write(self):
        with self.assertRaisesRegex(latency.SerialLatencyError,"only identified FTDI"):
            latency.configure_serial_latency(["COM29","COM14"])
        self.assertEqual(self.registry.writes,[])

    def test_server_checks_all_rings_before_any_port_is_opened(self):
        self.registry.values[registry_path(self.second)]["LatencyTimer"]=(16,4)
        with patch.object(server,"Bridge") as bridge, patch.object(server,"RingFleet") as fleet:
            with contextlib.redirect_stderr(io.StringIO()) as stderr:
                result=server.main(["--ring","first=COM29,10","--ring","second=COM48,10"])
            self.assertEqual(result,1)
            self.assertIn("COM48",stderr.getvalue())
            bridge.assert_not_called()
            fleet.assert_not_called()

    def test_check_only_mode_exits_before_serial_or_http_server(self):
        with patch.object(server,"Bridge") as bridge, patch.object(server,"ThreadingHTTPServer") as http:
            with contextlib.redirect_stdout(io.StringIO()) as output:
                self.assertEqual(server.main(["-p","COM29","--check-serial-latency"]),0)
            self.assertIn("saved latency timer 2 ms",output.getvalue())
            bridge.assert_not_called()
            http.assert_not_called()

    def test_autodetected_port_receives_the_same_check(self):
        self.registry.values[registry_path(self.first)]["LatencyTimer"]=(16,4)
        with patch.object(server,"auto_detect_port",return_value="COM29"), patch.object(server,"Bridge") as bridge:
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(server.main([]),1)
            bridge.assert_not_called()


if __name__ == "__main__":
    unittest.main()
