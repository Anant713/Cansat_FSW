"""Run: python Tools/hils/hils_gui.py (Python 3.10+, tkinter, pyserial)."""
import json
import math
from pathlib import Path
import random
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import serial
from serial.tools import list_ports
from protocol import (CALIBRATION, CAMERA_PHASES, DEFAULT_PROFILE, STATES, frame,
                      load_profile, pressure_for_height, profile_height)


class HilsGUI:
    def __init__(self, root):
        self.root = root
        root.title("CanSat TestHILS — real actuator outputs")
        root.geometry("1100x850")
        self.console = self.radio = None
        self.buffer = bytearray()
        self.capture = None
        self.ready = False
        self.streaming = False
        self.last_send = 0.0
        self.last_status = 0.0
        self.status = {}
        self.seq = int(time.monotonic() * 1000) & 0xffffffff
        self.profile = DEFAULT_PROFILE
        self.profile_start = None
        self.profile_elapsed = 0.0
        self.profile_paused = False
        self.last_height = 0.0
        self.trace = []
        self.port = tk.StringVar()
        self.radio_port = tk.StringVar(value="Console injection")
        self.height = tk.StringVar(value="0")
        self.ground = tk.StringVar(value="101325")
        self.noise = tk.StringVar(value="0")
        self.live = tk.StringVar(value="Disconnected")
        self.details = tk.StringVar(value="Waiting for MCU status")
        self.profile_label = tk.StringVar(value="Nominal profile (100 s)")
        connection = ttk.Frame(root, padding=8)
        connection.pack(fill="x")
        ttk.Label(connection, text="Console USART3 @115200:").pack(side="left")
        self.ports = ttk.Combobox(connection, textvariable=self.port, width=12, state="readonly")
        self.ports.pack(side="left", padx=5)
        ttk.Label(connection, text="Command route:").pack(side="left")
        self.radios = ttk.Combobox(connection, textvariable=self.radio_port, width=20, state="readonly")
        self.radios.pack(side="left", padx=5)
        ttk.Button(connection, text="Refresh", command=self.refresh).pack(side="left")
        ttk.Button(connection, text="Connect", command=self.connect).pack(side="left", padx=5)
        ttk.Button(connection, text="Disconnect", command=self.disconnect).pack(side="left")
        ttk.Label(root, text="REAL camera / HDRM / buzzer outputs. Use an indicator load for the first run.", foreground="#a33").pack(anchor="w", padx=10)
        inputs = ttk.LabelFrame(root, text="Pressure generator — 10 samples/s; MCU reports actual state", padding=8)
        inputs.pack(fill="x", padx=8, pady=5)
        for label, variable in [("Height (m)", self.height), ("Ground pressure (Pa)", self.ground), ("Noise ±m", self.noise)]:
            ttk.Label(inputs, text=label).pack(side="left", padx=4)
            ttk.Entry(inputs, textvariable=variable, width=10).pack(side="left")
        ttk.Button(inputs, text="Stream / resume", command=self.stream).pack(side="left", padx=8)
        ttk.Button(inputs, text="Pause input", command=self.pause).pack(side="left")
        profiles = ttk.Frame(root, padding=8)
        profiles.pack(fill="x")
        ttk.Label(profiles, textvariable=self.profile_label).pack(side="left", padx=5)
        ttk.Button(profiles, text="Load CSV", command=self.load).pack(side="left")
        ttk.Button(profiles, text="Run profile", command=self.run_profile).pack(side="left", padx=5)
        ttk.Button(profiles, text="Hold current height", command=self.hold).pack(side="left")
        commands = ttk.LabelFrame(root, text="Ground commands (selected route above)", padding=8)
        commands.pack(fill="x", padx=8)
        for label, command in [("1: Telemetry ON", 1), ("2: Telemetry OFF", 2), ("3: Calibration (no-op)", 3)]:
            ttk.Button(commands, text=label, command=lambda c=command: self.command(c)).pack(side="left", padx=5)
        ttk.Button(commands, text="New mission (test only)", command=self.new_mission).pack(side="left", padx=5)
        ttk.Button(commands, text="MCU reset", command=self.reset).pack(side="left", padx=5)
        ttk.Label(root, textvariable=self.live, font=("Consolas", 12), padding=8).pack(anchor="w")
        ttk.Label(root, textvariable=self.details, font=("Consolas", 10), padding=8, justify="left").pack(anchor="w")
        self.canvas = tk.Canvas(root, height=250, bg="#101b26", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=8)
        ttk.Label(root, text="Cyan: requested height   •   Orange: fresh MCU barometric height   •   Last 60 seconds").pack()
        self.events = tk.Text(root, height=7, state="disabled", font=("Consolas", 9))
        self.events.pack(fill="x", padx=8, pady=6)
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.refresh()
        self.root.after(20, self.poll)

    def event(self, text):
        self.events.configure(state="normal")
        self.events.insert("end", f"{time.strftime('%H:%M:%S')}  {text}\n")
        if int(self.events.index("end-1c").split(".")[0]) > 300:
            self.events.delete("1.0", "100.0")
        self.events.see("end")
        self.events.configure(state="disabled")

    def refresh(self):
        ports = [p.device for p in list_ports.comports()]
        self.ports["values"] = ports
        self.radios["values"] = ["Console injection"] + ports
        if ports and self.port.get() not in ports:
            self.port.set(ports[0])

    def connect(self):
        self.disconnect()
        try:
            if self.port.get() == self.radio_port.get():
                raise ValueError("Console and ground XBee must use different ports")
            self.console = serial.Serial(self.port.get(), 115200, timeout=0, write_timeout=0.2)
            if self.radio_port.get() != "Console injection":
                self.radio = serial.Serial(self.radio_port.get(), 115200, timeout=0, write_timeout=0.2)
            self.buffer.clear()
            self.status = {}
            self.ready = False
            self.last_status = time.monotonic()
            self.capture = filedialog.asksaveasfile(mode="w", defaultextension=".jsonl", filetypes=[("HILS session", "*.jsonl")], title="Optional session capture (Cancel to skip)")
            self.send("H1,STATUS")
            self.event("Connected; waiting for TestHILS status. No pressure is sent until Stream / resume.")
        except (serial.SerialException, OSError, ValueError) as exc:
            self.disconnect()
            messagebox.showerror("Connection", str(exc))

    def disconnect(self):
        self.pause()
        for port in (self.console, self.radio):
            if port:
                port.close()
        if self.capture:
            self.capture.close()
        self.console = self.radio = self.capture = None
        self.ready = False
        self.live.set("Disconnected")

    def send(self, body):
        if self.console:
            packet = frame(body)
            if self.console.write(packet) != len(packet):
                raise OSError("Incomplete console write")

    def capture_row(self, row):
        if self.capture:
            self.capture.write(json.dumps({"pc_time": time.time(), **row}) + "\n")
            self.capture.flush()

    def validate_inputs(self):
        height, ground, noise = float(self.height.get()), float(self.ground.get()), float(self.noise.get())
        if not math.isfinite(noise) or noise < 0:
            raise ValueError("Noise must be finite and nonnegative")
        pressure_for_height(height - noise, ground)
        pressure_for_height(height + noise, ground)
        return height, ground, noise

    def stream(self):
        try:
            if not self.ready:
                raise ValueError("Connect and wait for MCU status first")
            self.validate_inputs()
            if self.profile_paused:
                self.profile_start = time.monotonic() - self.profile_elapsed
                self.profile_paused = False
            self.streaming = True
            self.last_send = 0
        except ValueError as exc:
            messagebox.showerror("Generator", str(exc))

    def pause(self):
        if self.profile_start is not None:
            self.profile_elapsed = time.monotonic() - self.profile_start
            self.profile_paused = True
            self.profile_start = None
        self.streaming = False

    def hold(self):
        self.profile_start = None
        self.profile_paused = False
        self.height.set(f"{self.last_height:.2f}")

    def load(self):
        path = filedialog.askopenfilename(filetypes=[("Height profile", "*.csv")])
        if path:
            try:
                self.profile = load_profile(path)
                self.profile_label.set(Path(path).name)
            except (ValueError, KeyError, OSError) as exc:
                messagebox.showerror("Profile", str(exc))

    def run_profile(self):
        if not self.ready or not self.status.get("saved") or self.status.get("state") != 0 or self.status.get("cal") != 2:
            messagebox.showerror("Profile", "Stream stable ground pressure until startup calibration reports SUCCESS in LAUNCH_PAD.")
            return
        try:
            ground = float(self.ground.get())
            for _, height in self.profile:
                pressure_for_height(height, ground)
            self.height.set(str(self.profile[0][1]))
            self.stream()
            if self.streaming:
                self.profile_start = time.monotonic()
                self.profile_paused = False
        except ValueError as exc:
            messagebox.showerror("Profile", str(exc))

    def command(self, command):
        if not self.ready:
            return
        try:
            if self.radio:
                packet = bytes([0xa5, command, 0x5a])
                if self.radio.write(packet) != len(packet):
                    raise OSError("Incomplete radio write")
            else:
                self.send(f"H1,C,{command}")
            self.event(f"Sent command {command} via {'XBee' if self.radio else 'console parser injection'}; wait for MCU result.")
        except (serial.SerialException, OSError) as exc:
            self.event(str(exc))

    def new_mission(self):
        if self.ready and messagebox.askokcancel("New mission", "Append LAUNCH_PAD to flash, reset outputs and start a new SD log? Calibration is retained. This does not erase flash."):
            self.hold()
            self.height.set("0")
            self.last_height = 0
            try:
                self.send("H1,NEW")
            except (serial.SerialException, OSError) as exc:
                self.event(str(exc))

    def reset(self):
        if self.ready and messagebox.askokcancel("Reset MCU", "Reset without erasing flash? Recovery can replay camera startup and an incomplete HDRM pulse."):
            self.hold()
            try:
                self.send("H1,RESET")
            except (serial.SerialException, OSError) as exc:
                self.event(str(exc))
                return
            self.ready = False
            self.last_status = time.monotonic()
            self.event("Reset requested; holding requested height. Stream resumes after MCU status returns.")

    def receive(self, line):
        try:
            row = json.loads(line)
            if not isinstance(row, dict):
                return
        except (ValueError, UnicodeError):
            self.event(f"Non-JSON console output: {line[:120]!r}")
            return
        self.capture_row(row)
        if row.get("type") != "status":
            self.event(json.dumps(row))
            if row.get("type") == "boot":
                self.ready = False
            return
        previous = self.status
        self.status = row
        self.last_status = time.monotonic()
        if not self.ready or row.get("ground") != previous.get("ground"):
            if row.get("ground", 0) > 0:
                self.ground.set(str(row["ground"]))
                self.event(f"Generator reference = recovered/calibrated {row['ground']} Pa")
        # Follow firmware sequence on reconnect; normal PC sequence remains monotonic.
        if not self.ready and row.get("input"):
            self.seq = (row["seq"] + 1) & 0xffffffff
        self.ready = True
        if row.get("state") != previous.get("state"):
            self.event(f"MCU state: {STATES.get(row.get('state'), row.get('state'))}")
        self.trace.append((self.last_status, self.last_height, row.get("height", 0) if row.get("fresh") else None))
        self.trace = [point for point in self.trace if point[0] >= self.last_status - 60]
        self.draw()

    def draw(self):
        canvas = self.canvas
        canvas.delete("all")
        width, height = max(canvas.winfo_width(), 200), max(canvas.winfo_height(), 100)
        values = [v for point in self.trace for v in point[1:] if v is not None]
        lo, hi = min([0] + values), max([100] + values)
        hi += 20
        def xy(t, v):
            return (55 + (t - (self.last_status - 60)) / 60 * (width - 70),
                    height - 25 - (v - lo) / (hi - lo) * (height - 45))
        for value in (lo, (lo + hi) / 2, hi):
            _, y = xy(self.last_status, value)
            canvas.create_text(25, y, text=f"{value:.0f}", fill="white")
            canvas.create_line(55, y, width, y, fill="#324253")
        for index, color in ((1, "#53d3eb"), (2, "#ffa44d")):
            points = []
            for point in self.trace + [(self.last_status, None, None)]:
                if point[index] is None:
                    if len(points) >= 4:
                        canvas.create_line(*points, fill=color, width=2)
                    points = []
                else:
                    points.extend(xy(point[0], point[index]))

    def poll(self):
        try:
            now = time.monotonic()
            if self.console:
                self.buffer.extend(self.console.read(4096))
                if len(self.buffer) > 16384:
                    self.buffer.clear()
                    self.event("Discarded oversized console data")
                for _ in range(20):
                    if b"\n" not in self.buffer:
                        break
                    line, _, rest = self.buffer.partition(b"\n")
                    self.buffer = bytearray(rest)
                    self.receive(line)
                if self.radio:
                    self.radio.read(4096)  # Drain raw telemetry; MCU status is the GUI's source.
                if self.streaming and self.ready and now - self.last_status <= 3 and now - self.last_send >= 0.1:
                    height, ground, noise = self.validate_inputs()
                    if self.profile_start is not None:
                        elapsed = now - self.profile_start
                        height = profile_height(self.profile, elapsed)
                        self.height.set(f"{height:.2f}")
                        if elapsed >= self.profile[-1][0]:
                            self.profile_start = None
                            self.event("Profile complete; continuing final height")
                    height += random.uniform(-noise, noise)
                    pressure = pressure_for_height(height, ground)
                    self.seq = (self.seq + 1) & 0xffffffff
                    self.send(f"H1,P,{self.seq},{pressure:.3f},25.0")
                    self.capture_row({"type": "input", "seq": self.seq, "height": height, "pressure": pressure})
                    self.last_height, self.last_send = height, now
                if now - self.last_status > 3:
                    self.live.set("MCU status stale — waiting for reboot / checking serial link; no pressure sent")
                else:
                    s = self.status
                    self.live.set(f"{STATES.get(s.get('state'), 'Waiting')} | Sent {self.last_height:.2f} m | MCU {s.get('height', 0):.2f} m | "
                                  f"{'FRESH' if s.get('fresh') else 'INVALID / STALE'} | {'STREAMING' if self.streaming else 'INPUT PAUSED'}")
                    self.details.set(
                        f"Pressure {s.get('pressure', 0):.2f} Pa | Ground {s.get('ground', 0):.2f} Pa | Peak {s.get('peak', 0):.2f} m | Confirmation {s.get('confirm', 0)} | Input age {s.get('age', 0)} ms\n"
                        f"Camera {CAMERA_PHASES.get(s.get('phase'), '?')} / recording={s.get('recording')} | HDRM={s.get('hdrm')} | Deployed={s.get('deployed')} | Buzzer={s.get('buzzer')}\n"
                        f"Telemetry={s.get('telemetry')} TX={s.get('tx')} errors={s.get('tx_errors')} | Logging={s.get('logging')} rows={s.get('rows')} errors={s.get('log_errors')} {s.get('file', '')}\n"
                        f"Command last={s.get('command')} success={s.get('executed')} last successful={s.get('last_ok')} RX={s.get('rx')} | Calibration={CALIBRATION.get(s.get('cal'), '?')} saved={s.get('saved')}\n"
                        f"Recovered={s.get('recovered')} flash OK={s.get('flash')} | Init sensors/radio/state={s.get('sensor_init')}/{s.get('radio_init')}/{s.get('state_init')} | Bad frames={s.get('bad_frames')} UART drop/error={s.get('rx_dropped')}/{s.get('rx_errors')}")
        except (serial.SerialException, OSError) as exc:
            self.event(f"Disconnected: {exc}")
            self.disconnect()
        except ValueError as exc:
            self.pause()
            self.event(f"Generator paused: {exc}")
        self.root.after(20, self.poll)

    def close(self):
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    HilsGUI(root)
    root.mainloop()
