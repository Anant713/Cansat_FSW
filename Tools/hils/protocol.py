"""TestHILS protocol and pressure/profile helpers; no GUI/serial dependency."""
import binascii
import csv
import math

STATES = {0: "LAUNCH_PAD", 1: "ASCENT", 2: "DESCENT_1", 3: "DESCENT_2", 4: "TOUCHDOWN", 255: "UNINITIALIZED"}
CALIBRATION = {0: "IDLE", 1: "RUNNING", 2: "SUCCESS", 3: "FAILED"}
CAMERA_PHASES = {0: "OFF", 1: "INIT_HIGH", 2: "INIT_WAIT", 3: "CAPTURE_HIGH", 4: "RECORDING"}
DEFAULT_PROFILE = [(0, 0), (5, 0), (25, 850), (30, 850), (40, 750),
                   (55, 650), (65, 550), (85, 100), (95, 0), (100, 0)]


def frame(body):
    payload = body.encode("ascii")
    return payload + f"*{binascii.crc_hqx(payload, 0xffff):04X}\n".encode("ascii")


def pressure_for_height(height, ground=101325.0):
    if not math.isfinite(height) or not math.isfinite(ground) or not 30000 <= ground <= 110000 or height >= 44330:
        raise ValueError("Invalid height or reference pressure")
    try:
        pressure = ground * (1.0 - height / 44330.0) ** (1.0 / 0.1903)
    except OverflowError as exc:
        raise ValueError("Height is outside the simulated pressure range") from exc
    if not 30000 <= pressure <= 110000:
        raise ValueError("Simulated pressure must be between 30000 and 110000 Pa")
    return pressure


def height_for_pressure(pressure, ground):
    return 44330.0 * (1.0 - (pressure / ground) ** 0.1903)


def load_profile(path):
    with open(path, newline="", encoding="utf-8-sig") as stream:
        rows = [(float(row["time_s"]), float(row["height_m"])) for row in csv.DictReader(stream)]
    if len(rows) < 2 or rows[0][0] != 0:
        raise ValueError("Profile needs at least two points and must start at time_s=0")
    for index, (seconds, height) in enumerate(rows):
        if not math.isfinite(seconds) or not math.isfinite(height) or (index and seconds <= rows[index - 1][0]):
            raise ValueError("Profile times must be finite and strictly increasing")
    return rows


def profile_height(rows, elapsed):
    if elapsed <= rows[0][0]:
        return rows[0][1]
    for (t0, h0), (t1, h1) in zip(rows, rows[1:]):
        if elapsed <= t1:
            return h0 + (h1 - h0) * (elapsed - t0) / (t1 - t0)
    return rows[-1][1]
