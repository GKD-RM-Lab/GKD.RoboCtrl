import importlib.util
from pathlib import Path
import struct
import unittest


def load_tool(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).resolve().parents[1] / "tools" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


log = load_tool("tuning_log")
plot = load_tool("plot_tuning")


def frame(kind, payload):
    return struct.pack("<HB", len(payload) + 3, kind) + payload


class TuningToolsTests(unittest.TestCase):
    def test_remote_frame_vector_and_repeated_registration(self):
        self.assertEqual(log.name_id("hello"), 0x4F9F2CAB)
        # Known legacy little-endian bytes: register x, then x = -1.25.
        data = bytes.fromhex("09000087500cfd01780f000187500cfd000000000000f4bf")
        self.assertEqual(log.decode_datagram(data, {})[0], [("x", -1.25)])
        name = "yaw_rad_s"
        ident = log.name_id(name)
        register = frame(0, struct.pack("<IB", ident, len(name)) + name.encode())
        value = frame(1, struct.pack("<Id", ident, -1.25))
        names = {}
        samples, messages = log.decode_datagram(register + value, names)
        self.assertEqual(samples, [(name, -1.25)])
        self.assertEqual(messages, [])
        self.assertEqual(log.decode_datagram(register + value, names)[0], samples)
        self.assertEqual(log.decode_datagram(frame(2, b"\x02\x00ok"), names)[1], [(2, "ok")])

    def test_invalid_datagram_is_atomic(self):
        ident = log.name_id("x")
        register = frame(0, struct.pack("<IB", ident, 1) + b"x")
        names = {}
        for invalid in (b"", b"\x01", b"\x02\x00\x00", frame(99, b""),
                        frame(1, struct.pack("<Id", ident, 1.0)),
                        register + frame(1, struct.pack("<Id", ident, float("nan"))),
                        register + b"\x01", frame(2, b"\x03\x00ok")):
            with self.assertRaises(ValueError):
                log.decode_datagram(invalid, names)
            self.assertEqual(names, {})

    def test_csv_units_and_legacy_friction_signs(self):
        self.assertEqual(plot.read_samples("time_s,name,value\n0.5,x,-2\n"), [(0.5, "x", -2.0)])
        self.assertEqual(plot.read_samples("1000,2,-3\n", columns=["time_ms", "set", "feedback"]),
                         [(1.0, "set", 2.0), (1.0, "feedback", -3.0)])
        samples = plot.read_samples("set: 0, left: -1e-2, right: .5", sample_period=0.002)
        self.assertEqual(samples, [(0.0, "set", 0.0), (0.0, "left", -0.01), (0.0, "right", 0.5)])
        for text in ("0,1,2", "time_s,name,value\n0,x,nan", "time_s,x\n0", "set: 0, left: 0, right: 0"):
            with self.assertRaises(ValueError):
                plot.read_samples(text)

    def test_timestamped_data_and_html_escaping(self):
        samples = plot.read_samples("[2026-01-01 12:00:00] yaw: 1\n[2026-01-01 12:00:01] yaw: 2")
        self.assertEqual(samples, [(0.0, "yaw", 1.0), (1.0, "yaw", 2.0)])
        page = plot.render_html([(0, "</script><script>alert(1)</script>", 1)], "<unsafe>")
        self.assertIn("&lt;unsafe&gt;", page)
        self.assertNotIn("</script><script>alert", page)
        self.assertIn("\\u003c/script>", page)


if __name__ == "__main__":
    unittest.main()
