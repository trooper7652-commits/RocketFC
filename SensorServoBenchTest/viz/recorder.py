"""
Write received samples to CSV so a bench session can be replayed later.

The columns are exactly the firmware's VIZ1 wire format, so a recording made
here can be fed straight back in with --csv. Rows are flushed as they arrive:
a bench session that ends by unplugging the board should still leave a usable
file.
"""

import csv
import math

from sources import VIZ1_FIELDS


class Recorder:
    def __init__(self, path):
        self.path = path
        self.rows = 0
        self._last_t = None
        self._f = open(path, "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)
        self._w.writerow(VIZ1_FIELDS)

    def add(self, s):
        """Append one sample, skipping repeats of a timestamp already written."""
        if s is None or s.t_ms == self._last_t:
            return
        self._last_t = s.t_ms
        self._w.writerow([
            s.t_ms,
            _f4(s.q[0]), _f4(s.q[1]), _f4(s.q[2]), _f4(s.q[3]),
            _f2(s.accel[0]), _f2(s.accel[1]), _f2(s.accel[2]),
            _f1(s.gyro[0]), _f1(s.gyro[1]), _f1(s.gyro[2]),
            _f2(s.baro_alt), _f2(s.tilt_deg),
            _f2(s.gimx_deg), _f2(s.gimy_deg),
            _f0(s.servo_a_us), _f0(s.servo_b_us),
            s.tvc_mode, int(s.imu_ok), int(s.baro_ok),
        ])
        self.rows += 1
        self._f.flush()

    def close(self):
        try:
            self._f.close()
        except Exception:
            pass


def _fmt(v, nd):
    v = float(v)
    if math.isnan(v) or math.isinf(v):
        return "0"
    return "{:.{}f}".format(v, nd)


def _f0(v): return _fmt(v, 0)
def _f1(v): return _fmt(v, 1)
def _f2(v): return _fmt(v, 2)
def _f4(v): return _fmt(v, 4)
