"""Hidden-window smoke/integration test; uses fake serial, never opens COM ports."""
import json
import tkinter as tk
import unittest
from unittest.mock import patch
from hils_gui import HilsGUI
from protocol import frame


class Port:
    def __init__(self):
        self.sent = []

    def write(self, data):
        self.sent.append(data)
        return len(data)

    def read(self, count):
        return b""

    def close(self):
        pass


class GuiTests(unittest.TestCase):
    def test_stream_status_commands_and_recovery(self):
        root = tk.Tk()
        root.withdraw()
        with patch("hils_gui.list_ports.comports", return_value=[]):
            app = HilsGUI(root)
        port = Port()
        app.console = port
        status = {"type": "status", "ms": 200, "seq": 42, "input": 1,
                  "height": 0, "ground": 100000, "fresh": 1, "state": 0, "saved": 1, "cal": 2}
        app.receive(json.dumps(status).encode())
        self.assertTrue(app.ready)
        self.assertEqual(app.seq, 43)
        self.assertEqual(app.ground.get(), "100000")
        app.stream()
        app.poll()
        self.assertTrue(port.sent[-1].startswith(b"H1,P,44,100000.000,25.0*"))
        app.command(1)
        self.assertEqual(port.sent[-1], frame("H1,C,1"))
        app.command(3)
        self.assertEqual(port.sent[-1], frame("H1,C,3"))
        app.radio = Port()
        app.command(2)
        self.assertEqual(app.radio.sent[-1], b"\xa5\x02\x5a")
        app.run_profile()
        self.assertIsNotNone(app.profile_start)
        app.pause()
        self.assertTrue(app.profile_paused)
        app.stream()
        self.assertIsNotNone(app.profile_start)
        with patch("hils_gui.messagebox.askokcancel", return_value=True):
            app.reset()
        self.assertEqual(port.sent[-1], frame("H1,RESET"))
        self.assertFalse(app.ready)
        self.assertIsNone(app.profile_start)
        app.receive(b'{"type":"boot","protocol":1}')
        status.update(ms=1, input=0, state=3, recovered=1, hdrm=1)
        app.receive(json.dumps(status).encode())
        self.assertTrue(app.ready)
        self.assertEqual(app.status["state"], 3)
        app.poll()
        self.assertIn("DESCENT_2", app.live.get())
        app.close()


if __name__ == "__main__":
    unittest.main()
