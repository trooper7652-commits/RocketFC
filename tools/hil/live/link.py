"""Link to the HIL build in LIVE mode (closed-loop bench sim).

Two ports, as for the static-fire flow (tools/hil/hil_ctl.py):
  console  the normal RocketFC CLI (`arm`, `mode`, `chute lock`, EVENT lines)
  hil      the HIL control channel; in LIVE it carries framed packets
             0xA5 0x5A <type> <len> <payload> <crc8 over type,len,payload>
           PC -> Teensy 'F' sensor frame, 'C' text command
           Teensy -> PC 'T' telemetry (every 500 Hz tick), 'M' text reply

A port is a COM device ("COM7") or "socket://127.0.0.1:PORT" for the PC
build of the same firmware (tools/hil/test/host_live.cpp). Must match
tools/hil/firmware/sensors_hil.h (HilLiveFrame / HilTelemetry).
"""
import re
import select
import socket
import struct
import time

SYNC = b"\xA5\x5A"
PJRC_VID = 0x16C0

FRAME = struct.Struct("<HBB7f")                  # HilLiveFrame, 32 bytes
TELEM = struct.Struct("<IHBBffHHHBBfff4f6fHH")   # HilTelemetry, 80 bytes
assert FRAME.size == 32 and TELEM.size == 80

FLAG_BARO = 1
FLAG_IMU_FAIL = 2

BIT_LAND_FIRE = 1
BIT_LEGS_CMD = 2
BIT_LEGS_PIN = 4
BIT_CHUTE_RELEASE = 8
BIT_CHUTE_DETECTED = 16
BIT_IN_FLIGHT = 32
BIT_TVC_ACTIVE = 64
BIT_IMU_BAD = 128

CHUTE_POS = {0: "LOCK", 1: "OPEN", 2: "JOG", 3: "SET"}

EVENT_RE = re.compile(r"^\[(\d+) ms\] EVENT (\w+) (-?[\d.]+|nan|inf)")


def _crc_table():
    tab = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        tab.append(c)
    return tab


_CRC = _crc_table()


def crc8(data, crc=0):
    for b in data:
        crc = _CRC[crc ^ b]
    return crc


def packet(typ, payload):
    body = bytes((ord(typ), len(payload))) + payload
    return SYNC + body + bytes((crc8(body),))


def frame_packet(seq, flags, accel, gyro, baro_alt):
    return packet("F", FRAME.pack(seq & 0xFFFF, flags, 0, accel[0], accel[1], accel[2],
                                  gyro[0], gyro[1], gyro[2], baro_alt))


def cmd_packet(text):
    return packet("C", text.encode()[:150])


def parse_telem(payload):
    (ms, seq, state, abort, gx, gy, us_a, us_b, us_chute, chute_pos, bits,
     kf_alt, kf_vel, tilt, qw, qx, qy, qz, p_x, i_x, d_x, p_y, i_y, d_y,
     stale, period_us) = TELEM.unpack(payload)
    return {"ms": ms, "seq": seq, "state": state, "abort": abort,
            "gimbal_x": gx, "gimbal_y": gy, "us_a": us_a, "us_b": us_b,
            "us_chute": us_chute, "chute_pos": chute_pos, "bits": bits,
            "kf_alt": kf_alt, "kf_vel": kf_vel, "tilt_deg": tilt,
            "quat": (qw, qx, qy, qz),
            "p_x": p_x, "i_x": i_x, "d_x": d_x, "p_y": p_y, "i_y": i_y, "d_y": d_y,
            "stale": stale, "period_us": period_us}


class Decoder:
    """Splits the HIL channel's byte stream into packets and (outside them)
    text lines, in arrival order -- the one text line in LIVE is the
    `OK live` that enters it (telemetry follows it immediately)."""

    def __init__(self):
        self.buf = bytearray()
        self.text = bytearray()
        self.lines = []
        self.crc_errors = 0

    def _add_text(self, b, out):
        self.text += b
        while b"\n" in self.text:
            ln, _, rest = bytes(self.text).partition(b"\n")
            s = ln.decode("utf-8", "replace").rstrip("\r")
            self.lines.append(s)
            out.append(("L", s))
            self.text = bytearray(rest)

    def feed(self, data):
        """Returns [(kind, payload)]: ('T'|'M'|..., bytes) per packet and
        ('L', str) per text line."""
        self.buf += data
        out = []
        buf = self.buf
        while True:
            i = buf.find(SYNC)
            if i < 0:
                keep = 1 if buf.endswith(b"\xA5") else 0
                if len(buf) > keep:
                    self._add_text(bytes(buf[:len(buf) - keep]), out)
                    del buf[:len(buf) - keep]
                break
            if i:
                self._add_text(bytes(buf[:i]), out)
                del buf[:i]
            if len(buf) < 4:
                break
            n = buf[3]
            if len(buf) < 5 + n:
                break
            if crc8(buf[2:4 + n]) != buf[4 + n]:
                self.crc_errors += 1
                del buf[:1]
                continue
            out.append((chr(buf[2]), bytes(buf[4:4 + n])))
            del buf[:5 + n]
        return out


class SerialPort:
    def __init__(self, dev):
        import serial
        self.name = dev
        self.s = serial.Serial(dev, 115200, timeout=0.001, write_timeout=1.0)

    def read(self, wait=False):
        n = self.s.in_waiting
        if n:
            return self.s.read(n)
        if not wait:
            return b""
        b = self.s.read(1)  # up to the 1 ms timeout
        if b:
            n = self.s.in_waiting
            if n:
                b += self.s.read(n)
        return b

    def write(self, b):
        self.s.write(b)

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass


class SocketPort:
    def __init__(self, url, timeout=10.0):
        m = re.match(r"socket://([^:/]+):(\d+)", url)
        if not m:
            raise ValueError("bad socket URL: " + url)
        self.name = url
        end = time.time() + timeout
        while True:
            try:
                self.k = socket.create_connection((m.group(1), int(m.group(2))), timeout=2.0)
                break
            except OSError:
                if time.time() > end:
                    raise
                time.sleep(0.1)
        self.k.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.k.settimeout(2.0)

    def read(self, wait=False):
        r, _, _ = select.select([self.k], [], [], 0.001 if wait else 0)
        if not r:
            return b""
        b = self.k.recv(65536)
        if not b:
            raise ConnectionError(self.name + " closed")
        return b

    def write(self, b):
        self.k.sendall(b)

    def close(self):
        try:
            self.k.close()
        except Exception:
            pass


def open_port(spec):
    return SocketPort(spec) if spec.startswith("socket://") else SerialPort(spec)


def find_ports():
    """(hil, console) COM names among PJRC ports. The HIL port answers
    `hello`, or is already streaming LIVE telemetry."""
    import serial
    import serial.tools.list_ports
    cands = [p.device for p in serial.tools.list_ports.comports() if p.vid == PJRC_VID]
    hil = None
    for dev in cands:
        try:
            with serial.Serial(dev, 115200, timeout=0.05) as s:
                s.reset_input_buffer()
                got = b""
                end = time.time() + 0.3
                while time.time() < end:
                    got += s.read(512)
                if SYNC + b"T" in got:
                    hil = dev
                    break
                s.write(b"\nhello\n")
                end = time.time() + 1.0
                while time.time() < end and b"OK hello" not in got:
                    got += s.read(256)
                if b"OK hello" in got:
                    hil = dev
                    break
        except Exception:
            continue
    others = [d for d in cands if d != hil]
    return hil, (others[0] if len(others) == 1 else None), cands


class Console:
    """The RocketFC CLI port: send lines, collect lines, parse EVENTs."""

    def __init__(self, port):
        self.port = port
        self.part = b""
        self.lines = []      # every line received, in order
        self.events = []     # (ms, name, value)
        self._seen = 0

    def poll(self):
        b = self.port.read()
        if not b:
            return []
        self.part += b
        *done, self.part = self.part.split(b"\n")
        new = [d.decode("utf-8", "replace").rstrip("\r") for d in done]
        for ln in new:
            m = EVENT_RE.match(ln)
            if m:
                self.events.append((int(m.group(1)), m.group(2), float(m.group(3))))
        self.lines.extend(new)
        return new

    def send(self, line):
        self.port.write((line + "\n").encode())

    def new_lines(self):
        out = self.lines[self._seen:]
        self._seen = len(self.lines)
        return out
