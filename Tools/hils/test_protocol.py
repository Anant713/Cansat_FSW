import tempfile
import unittest
from pathlib import Path
from protocol import frame, pressure_for_height, height_for_pressure, load_profile, profile_height, DEFAULT_PROFILE


class ProtocolTests(unittest.TestCase):
    def test_checksum(self):
        self.assertEqual(frame("123456789"), b"123456789*29B1\n")

    def test_pressure_roundtrip(self):
        for reference in (90000, 101325):
            for height in (-100, 0, 20, 30, 600, 700, 850, 2000):
                p = pressure_for_height(height, reference)
                self.assertAlmostEqual(height_for_pressure(p, reference), height, places=7)
        for height in (float("nan"), float("inf"), 44330, 20000, -1e308):
            with self.assertRaises(ValueError):
                pressure_for_height(height)

    def test_profile(self):
        self.assertEqual(profile_height(DEFAULT_PROFILE, -1), 0)
        self.assertEqual(profile_height(DEFAULT_PROFILE, 15), 425)
        self.assertEqual(profile_height(DEFAULT_PROFILE, 101), 0)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "profile.csv"
            path.write_text("time_s,height_m\n0,0\n10,800\n")
            self.assertEqual(load_profile(path), [(0, 0), (10, 800)])
            path.write_text("time_s,height_m\n0,0\n0,800\n")
            with self.assertRaises(ValueError):
                load_profile(path)


if __name__ == "__main__":
    unittest.main()
