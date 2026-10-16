"""Exercise the execution wire protocol without Isaac or flight traffic."""

import json
from pathlib import Path
import socket
import tempfile
import threading
import unittest

from tools.isaac_client import execute


class IsaacClientTests(unittest.TestCase):
    def test_authenticated_request_half_close_and_fragmented_response(self):
        received = []
        errors = []
        with socket.socket() as listener, tempfile.TemporaryDirectory() as directory:
            listener.bind(("127.0.0.1", 0))
            listener.listen(1)
            listener.settimeout(3)
            session = Path(directory) / "session.json"
            session.write_text(
                json.dumps(
                    dict(host="127.0.0.1", port=listener.getsockname()[1], auth_token="test-token")
                ),
                encoding="utf-8",
            )

            def serve():
                try:
                    with listener.accept()[0] as connection:
                        connection.settimeout(3)
                        data = bytearray()
                        while chunk := connection.recv(16):
                            data.extend(chunk)
                        received.append(json.loads(data))
                        response = json.dumps(dict(status="ok", output="read-only pose\n")).encode()
                        for offset in range(0, len(response), 3):
                            connection.sendall(response[offset : offset + 3])
                except Exception as error:
                    errors.append(error)

            worker = threading.Thread(target=serve)
            worker.start()
            result = execute("print('pose')", session=session, timeout=3)
            worker.join(timeout=4)
            self.assertFalse(worker.is_alive())
            self.assertFalse(errors)
            self.assertEqual(result, dict(status="ok", output="read-only pose\n"))
            self.assertEqual(received[0]["auth_token"], "test-token")
            self.assertEqual(received[0]["code"], "print('pose')")
            self.assertEqual(received[0]["context"], "drone_lab")
            self.assertNotIn("fire_and_forget", received[0])

    def test_refuses_nonlocal_endpoint_or_missing_auth(self):
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory) / "session.json"
            for config in (
                dict(host="192.0.2.1", port=8226, auth_token="test-token"),
                dict(host="127.0.0.1", port=8226, auth_token=""),
            ):
                session.write_text(json.dumps(config), encoding="utf-8")
                with self.assertRaises(ValueError):
                    execute("print('pose')", session=session)


if __name__ == "__main__":
    unittest.main()
