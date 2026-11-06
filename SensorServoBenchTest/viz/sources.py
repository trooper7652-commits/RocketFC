"""
Where samples come from: the live board, a recorded CSV, or a synthetic demo.

All three expose the same tiny interface, so the renderer never knows or cares
which one it is talking to:

    src.latest()   -> Sample or None
    src.rate_hz    -> measured samples/second
    src.label      -> short string for the HUD
    src.close()

Only SerialSource touches hardware, and it is the single place in this program
that ever writes to the serial port (see SerialSource.send).
"""

import collections
import csv
import math
import os
import threading
import time
from dataclasses import dataclass, field

import numpy as np

import quatmath as qm

# Column order of the firmware's "V," rows. The board also announces this in a
# "#VIZ1 ..." header line; this list is the fallback for a host that attached
# mid-stream and missed it, and the two are cross-checked when the header does
# arrive.
VIZ1_FIELDS = [
    "t_ms", "qw", "qx", "qy", "qz", "ax", "ay", "az", "gx", "gy", "gz",
    "baro_alt", "tilt_deg", "gimx_deg", "gimy_deg", "servoA_us", "servoB_us",
    "tvc_mode", "imu_ok", "baro_ok",
]

TVC_MODE_NAMES = {0: "off", 1: "calc", 2: "DRIVE"}


@dataclass
class Sample:
    t_ms: int = 0
    q: np.ndarray = field(default_factory=lambda: qm.IDENTITY.copy())
    accel: np.ndarray = field(default_factory=lambda: np.zeros(3))  # m/s^2, body
    gyro: np.ndarray = field(default_factory=lambda: np.zeros(3))   # deg/s, body
    baro_alt: float = 0.0
    tilt_deg: float = 0.0
    gimx_deg: float = 0.0
    gimy_deg: float = 0.0
    servo_a_us: float = 1500.0
    servo_b_us: float = 1500.0
    tvc_mode: int = 0
    imu_ok: bool = True
    baro_ok: bool = True
    state: str = ""  # only populated when replaying a RocketFC flight log

    @property
    def tvc_mode_name(self):
        return TVC_MODE_NAMES.get(self.tvc_mode, "?")


def _sample_from_viz_row(parts):
    """Build a Sample from an already-split 'V,' row (without the 'V' tag)."""
    if len(parts) < len(VIZ1_FIELDS):
        return None
    v = [float(p) for p in parts[:len(VIZ1_FIELDS)]]
    return Sample(
        t_ms=int(v[0]),
        q=qm.quat_normalize(np.array(v[1:5])),
        accel=np.array(v[5:8]),
        gyro=np.array(v[8:11]),
        baro_alt=v[11],
        tilt_deg=v[12],
        gimx_deg=v[13],
        gimy_deg=v[14],
        servo_a_us=v[15],
        servo_b_us=v[16],
        tvc_mode=int(v[17]),
        imu_ok=bool(int(v[18])),
        baro_ok=bool(int(v[19])),
    )


class _RateMeter:
    """Samples/second over a short sliding window."""

    def __init__(self, window=2.0):
        self._window = window
        self._times = collections.deque()

    def tick(self):
        now = time.monotonic()
        self._times.append(now)
        cutoff = now - self._window
        while self._times and self._times[0] < cutoff:
            self._times.popleft()

    @property
    def hz(self):
        if len(self._times) < 2:
            return 0.0
        span = self._times[-1] - self._times[0]
        return (len(self._times) - 1) / span if span > 1e-6 else 0.0


class SerialSource:
    """
    Live telemetry from the Teensy over USB serial.

    Runs a daemon reader thread so a slow or stalled board can never block the
    render loop -- the window keeps responding and the HUD's packet rate falls
    to zero, which is itself the diagnostic.

    Lines that do not start with "V," are firmware chatter (CLI replies, boot
    banner, event messages). Those are handed to `on_text` so the caller can
    show them, because this program holds the COM port exclusively and they
    would otherwise be invisible.
    """

    def __init__(self, port, baud=115200, on_text=None):
        import serial  # imported here so --demo/--csv work without pyserial

        self.label = port
        self._on_text = on_text
        self._rate = _RateMeter()
        self._latest = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._bad_rows = 0
        self._header_fields = None

        # The Teensy's USB CDC ignores the baud rate, but pyserial still wants
        # one. The timeout keeps the reader thread responsive to shutdown.
        self._ser = serial.Serial(port, baud, timeout=0.2)
        self._thread = threading.Thread(target=self._run, name="serial-reader",
                                        daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            try:
                raw = self._ser.readline()
            except Exception as exc:  # port yanked, USB reset, etc.
                self._emit("[serial] read failed: {}".format(exc))
                return
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            if line.startswith("V,"):
                parts = line[2:].split(",")
                try:
                    s = _sample_from_viz_row(parts)
                except ValueError:
                    s = None
                if s is None:
                    self._bad_rows += 1
                    continue
                with self._lock:
                    self._latest = s
                self._rate.tick()
            elif line.startswith("#VIZ1"):
                self._header_fields = line.split(None, 1)[1].split(",")
                if self._header_fields != VIZ1_FIELDS:
                    self._emit("[serial] WARNING: board's column layout differs "
                               "from this visualizer's. Re-upload the firmware "
                               "or update viz/sources.py.")
            else:
                self._emit(line)

    def _emit(self, text):
        if self._on_text:
            self._on_text(text)
        else:
            print(text)

    def send(self, command):
        """
        Write one CLI command to the board.

        This is the ONLY place the visualizer writes to the serial port. It
        exists because the bench procedure needs `cal` and `tvc ...` while you
        are watching the 3D view, and only one program can hold a COM port at a
        time -- without this you would have to quit, open a serial monitor,
        type, quit that, and restart.
        """
        try:
            self._ser.write((command.strip() + "\n").encode("ascii", "replace"))
            return True
        except Exception as exc:
            self._emit("[serial] write failed: {}".format(exc))
            return False

    def latest(self):
        with self._lock:
            return self._latest

    @property
    def rate_hz(self):
        return self._rate.hz

    @property
    def bad_rows(self):
        return self._bad_rows

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        try:
            self._ser.close()
        except Exception:
            pass


class CsvSource:
    """
    Replay a recorded CSV in real time, with pause / speed / seek.

    Two formats are accepted and told apart by their header:

      * a bench recording written by recorder.py (the VIZ1 columns), and
      * a RocketFC SD flight log, FLIGHTS/flight_NNN.csv, whose 34-column
        layout already carries qw..qz at 100 Hz.

    Flight logs interleave event rows (every data column empty, last column
    "EVT:name:value"); those are collected separately rather than drawn.
    """

    def __init__(self, path, speed=1.0):
        self.label = os.path.basename(path)
        self.speed = speed
        self.paused = False
        self.events = []
        self._rate = _RateMeter()
        self._samples = self._load(path)
        if not self._samples:
            raise ValueError("no usable data rows in {}".format(path))
        self._i = 0
        self._t0_wall = time.monotonic()
        self._t0_data = self._samples[0].t_ms / 1000.0
        self._last_index = -1

    # -- loading ----------------------------------------------------------
    def _load(self, path):
        with open(path, "r", newline="", encoding="utf-8", errors="replace") as f:
            rows = list(csv.reader(f))

        header = None
        data = []
        for row in rows:
            if not row or not row[0].strip():
                continue
            first = row[0].strip()
            if first.startswith("#"):
                continue  # "#META {...}" from synth_flight.py, or a #VIZ1 banner
            if header is None and not _looks_numeric(first):
                header = [c.strip() for c in row]
                continue
            data.append(row)

        if header is None:
            raise ValueError("{}: no header row found".format(path))

        if "qw" not in header:
            raise ValueError(
                "{}: no quaternion columns (qw,qx,qy,qz) in this file.\n"
                "Synthetic replay cases from RocketFC's tools/synth_flight.py "
                "carry raw sensor data only, not attitude -- record a bench "
                "session or use a real FLIGHTS/flight_NNN.csv instead."
                .format(path))

        idx = {name: i for i, name in enumerate(header)}
        is_flight_log = "state" in idx and "kf_alt" in idx
        return (self._load_flight_log(data, idx) if is_flight_log
                else self._load_bench_recording(data, idx))

    def _load_flight_log(self, data, idx):
        """RocketFC SD log: gyro in rad/s, has a state column and EVT rows."""
        out = []
        rad2deg = 180.0 / math.pi
        for row in data:
            if len(row) < len(idx):
                row = row + [""] * (len(idx) - len(row))
            if row[-1].startswith("EVT:"):
                try:
                    self.events.append((int(float(row[idx["t_ms"]])), row[-1][4:]))
                except ValueError:
                    pass
                continue
            try:
                s = Sample(
                    t_ms=int(float(row[idx["t_ms"]])),
                    q=qm.quat_normalize(np.array([
                        float(row[idx["qw"]]), float(row[idx["qx"]]),
                        float(row[idx["qy"]]), float(row[idx["qz"]])])),
                    accel=np.array([float(row[idx["ax"]]), float(row[idx["ay"]]),
                                    float(row[idx["az"]])]),
                    # The SD log stores body rates in rad/s; the wire format and
                    # the whole HUD use deg/s.
                    gyro=np.array([float(row[idx["gx"]]), float(row[idx["gy"]]),
                                   float(row[idx["gz"]])]) * rad2deg,
                    baro_alt=float(row[idx["baro_alt"]]),
                    tilt_deg=float(row[idx["tilt_deg"]]),
                    gimx_deg=float(row[idx["gimx_deg"]]),
                    gimy_deg=float(row[idx["gimy_deg"]]),
                    servo_a_us=float(row[idx["servoA_us"]]),
                    servo_b_us=float(row[idx["servoB_us"]]),
                    tvc_mode=0,
                    state=row[idx["state"]],
                )
            except (ValueError, IndexError):
                continue
            out.append(s)
        return out

    def _load_bench_recording(self, data, idx):
        out = []
        for row in data:
            try:
                out.append(_sample_from_viz_row([row[idx[n]] for n in VIZ1_FIELDS]))
            except (ValueError, IndexError, KeyError):
                continue
        return [s for s in out if s is not None]

    # -- playback ---------------------------------------------------------
    @property
    def duration_s(self):
        return (self._samples[-1].t_ms - self._samples[0].t_ms) / 1000.0

    @property
    def position_s(self):
        return self._samples[self._i].t_ms / 1000.0 - self._t0_data

    def toggle_pause(self):
        if self.paused:
            # Resume where we left off rather than snapping forward in time.
            self._t0_wall = time.monotonic() - self.position_s / max(self.speed, 1e-6)
        self.paused = not self.paused

    def seek(self, delta_s):
        target = max(0.0, min(self.duration_s, self.position_s + delta_s))
        self._t0_wall = time.monotonic() - target / max(self.speed, 1e-6)
        self._advance_to(target)

    def set_speed(self, speed):
        here = self.position_s
        self.speed = max(0.05, min(20.0, speed))
        self._t0_wall = time.monotonic() - here / self.speed

    def _advance_to(self, t_rel):
        target_ms = self._t0_data * 1000.0 + t_rel * 1000.0
        i = self._i
        if self._samples[i].t_ms > target_ms:
            i = 0
        while i + 1 < len(self._samples) and self._samples[i + 1].t_ms <= target_ms:
            i += 1
        self._i = i

    def latest(self):
        if not self.paused:
            elapsed = (time.monotonic() - self._t0_wall) * self.speed
            if elapsed > self.duration_s:  # loop
                self._t0_wall = time.monotonic()
                elapsed = 0.0
            self._advance_to(elapsed)
        if self._i != self._last_index:
            self._rate.tick()
            self._last_index = self._i
        return self._samples[self._i]

    @property
    def rate_hz(self):
        return self._rate.hz

    @property
    def bad_rows(self):
        return 0

    def send(self, command):
        return False  # nothing to command in a replay

    def close(self):
        pass


class DemoSource:
    """
    Synthetic motion, no hardware and no serial port.

    The point of --demo is to separate "my graphics stack is broken" from "my
    wiring is broken", so it must depend on nothing but numpy: run it first,
    and if a rocket tumbles, every later failure is on the hardware side.
    """

    label = "demo (no hardware)"

    def __init__(self):
        self._rate = _RateMeter()
        self._t0 = time.monotonic()

    def latest(self):
        t = time.monotonic() - self._t0
        # A slow lean that grows and shrinks, combined with a steady roll, so
        # tilt and heading are both visibly exercised.
        lean = math.radians(35.0 * math.sin(t * 0.35))
        roll = t * 0.8
        axis = np.array([math.cos(roll), math.sin(roll), 0.0])
        q_lean = np.concatenate(([math.cos(lean / 2)], axis * math.sin(lean / 2)))
        q = qm.quat_normalize(q_lean)

        # A believable accelerometer for that attitude: gravity, expressed in
        # body axes. This keeps the ghost rocket meaningful in demo mode.
        accel = qm.rotate_inv(q, qm.WORLD_UP * 9.81)
        self._rate.tick()
        return Sample(
            t_ms=int(t * 1000),
            q=q,
            accel=accel,
            gyro=np.array([math.degrees(0.8 * math.sin(t)),
                           math.degrees(0.8 * math.cos(t)),
                           math.degrees(0.8)]),
            baro_alt=0.05 * math.sin(t * 0.7),
            tilt_deg=qm.tilt_deg(q),
            gimx_deg=-8.0 * math.sin(t * 0.35),
            gimy_deg=3.0 * math.cos(t * 0.5),
            servo_a_us=1500 - 60 * math.sin(t * 0.35),
            servo_b_us=1500 + 25 * math.cos(t * 0.5),
            tvc_mode=1,
        )

    @property
    def rate_hz(self):
        return self._rate.hz

    @property
    def bad_rows(self):
        return 0

    def send(self, command):
        return False

    def close(self):
        pass


def _looks_numeric(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


def list_ports():
    """Human-readable list of serial ports, for --list-ports."""
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        return ["pyserial is not installed -- run: pip install pyserial"]
    ports = list(lp.comports())
    if not ports:
        return ["no serial ports found (is the Teensy plugged in?)"]
    return ["{}  {}".format(p.device, p.description) for p in ports]
