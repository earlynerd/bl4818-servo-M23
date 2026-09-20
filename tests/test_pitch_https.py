"""HTTPS setup and player assets; never opens a serial port."""
import contextlib
import io
import shutil
import ssl
import subprocess
import sys
import tempfile
import threading
import unittest
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import ring_midi_server as server


class PitchHttpsTests(unittest.TestCase):
    def test_http_remains_default_and_incomplete_tls_fails_before_serial(self):
        self.assertIsNone(server.make_tls_context(None, None))
        for args in (["--tls-cert", "missing.pem"], ["--tls-key", "missing.pem"],
                     ["--tls-cert", "missing.pem", "--tls-key", "missing.key"]):
            with patch.object(server, "Bridge") as bridge, patch.object(server, "auto_detect_port") as detect:
                with contextlib.redirect_stderr(io.StringIO()):
                    self.assertEqual(server.main(args), 1)
                bridge.assert_not_called()
                detect.assert_not_called()

    @unittest.skipUnless(shutil.which("openssl"), "openssl is needed to generate a disposable test certificate")
    def test_trusted_https_serves_both_detector_assets_and_player(self):
        with tempfile.TemporaryDirectory() as temporary:
            cert, key = Path(temporary) / "cert.pem", Path(temporary) / "key.pem"
            config = Path(temporary) / "openssl.cnf"
            config.write_text("[req]\ndistinguished_name=dn\n[dn]\n", encoding="ascii")
            issued = subprocess.run([
                "openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes",
                "-config", str(config),
                "-keyout", str(key), "-out", str(cert), "-days", "1",
                "-subj", "/CN=localhost", "-addext", "subjectAltName=IP:127.0.0.1,DNS:localhost",
            ], capture_output=True, text=True)
            self.assertEqual(issued.returncode, 0, issued.stderr)
            tls = server.make_tls_context(str(cert), str(key))
            self.assertEqual(tls.minimum_version, ssl.TLSVersion.TLSv1_2)
            http = ThreadingHTTPServer(("127.0.0.1", 0), server.Handler)
            http.socket = tls.wrap_socket(http.socket, server_side=True, do_handshake_on_connect=False)
            thread = threading.Thread(target=http.serve_forever, daemon=True)
            thread.start()
            client = ssl.create_default_context(cafile=str(cert))
            try:
                for path, expected in [("/player.html", b"Detect pitches with microphone"),
                                       ("/pitch_detector.js", b"PitchDetector"),
                                       ("/pitch_assignment.js", b"startPitchDetection")]:
                    with urllib.request.urlopen(f"https://127.0.0.1:{http.server_port}{path}", context=client, timeout=3) as response:
                        self.assertEqual(response.status, 200)
                        self.assertIn(expected, response.read())
                        if path.endswith(".js"):
                            self.assertIn("javascript", response.headers["Content-Type"])
            finally:
                http.shutdown()
                http.server_close()
                thread.join(timeout=3)


if __name__ == "__main__":
    unittest.main()
