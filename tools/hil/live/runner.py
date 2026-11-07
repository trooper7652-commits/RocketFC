"""Closed-loop bench flight: the real Teensy flies the simulated rocket.

    r = Runner(hil="COM7", console="COM6", scen=Scenario(mode="chute"))
    r.start()                    # enter LIVE, stream a still pad
    r.prepare(); r.arm(); r.launch(); r.abort()    # requests, non-blocking
    while not r.flight_over:
        r.poll()                 # THE 500 Hz loop -- call it continuously
    result, session_dir = r.finish()

Every telemetry packet from the Teensy steps the plant to that packet's
millis() (flight.py), takes the commands it carries and is answered with the
sensor frame for that moment. If several packets arrive at once (USB batching,
a slow PC moment) each one still steps the plant and draws its sensor noise,
but only the newest frame is sent, carrying the baro flag of any it replaced
-- the Teensy just uses the newest frame it has, and the telemetry says which
(so twin.py can replay exactly that).

runner_process() runs a Runner in its own process for the dashboard
(tools/sim/hil_panel.py): Tk can't then stall the loop.
"""
import datetime
import json
import os
import queue
import time

import flight
import link
from flight import core
from scenario import Scenario

SESSIONS_DIR = os.path.join(flight.HIL_DIR, "sessions")
PAD0 = (0.0, 0.0, flight.G0)


class RunnerError(RuntimeError):
    pass


def _state_name(v):
    try:
        return core.FlightState(v).name
    except ValueError:
        return str(v)


class Runner:
    def __init__(self, hil=None, console=None, scen=None, sketch_dir=flight.SKETCH_DIR,
                 log=None, sessions_dir=SESSIONS_DIR):
        self.sc = scen or Scenario()
        self.log = log or (lambda m: print(m, flush=True))
        self.sketch_dir = sketch_dir
        self.sessions_dir = sessions_dir
        self.profile = flight.load_profile(sketch_dir)
        self.dll = flight.flight_dll(sketch_dir)
        self.snap = core.FlightCore.config_snapshot(self.dll)
        self.vcfg = flight.vehicle_for(self.dll, self.profile)
        self.fl = flight.LiveFlight(self.sc, self.vcfg, self.snap)
        self.hil_spec, self.con_spec = hil, console
        self.hil = self.con = None
        self.dec = link.Decoder()
        self.live = False          # past this session's `OK live`
        self.ticks = []            # [ms, period_us, seq, bits, state] per telemetry packet
        self.rows = []             # run.simulate()-style rows, from Prepare on
        self.events = []           # {t, ms, code, value} from the console
        self.replies = []          # 'M' replies
        self._nreply = 0
        self.ms0 = None
        self.t = 0.0
        self.timeline = {}
        self.pending = set()
        self.task = None
        self.phase = "idle"
        self.flight_over = False
        self.end_reason = None
        self.error = None
        self.last_tel = None
        self.state = "?"
        self.landed_t = None
        self.td_state_t = None     # plant time the Teensy first reported TOUCHDOWN
        self.record = False
        self._abort_sent = False
        self._con_seen = 0
        self.stats = {"telemetry": 0, "frames_sent": 0, "max_stale": 0, "max_gap_ms": 0.0,
                      "max_batch": 0, "crc_errors": 0}
        self._last_rx = None
        self._rate_n = 0
        self._rate_t0 = time.perf_counter()
        self.rate_hz = 0.0

    # ---------------------------------------------------------- connect
    def start(self, timeout=6.0):
        if not self.hil_spec:
            hil, con, cands = link.find_ports()
            if not hil:
                raise RunnerError(
                    "no HIL port among the PJRC ports ({}). Is the HIL build uploaded? "
                    "python tools/hil/make_hil.py --profile e18w_1400g --upload COMx".format(
                        ", ".join(cands) or "none"))
            self.hil_spec = hil
            self.con_spec = self.con_spec or con
        if not self.con_spec:
            raise RunnerError("couldn't tell which port is the RocketFC console; pick it")
        self.hil = link.open_port(self.hil_spec)
        self.con = link.Console(link.open_port(self.con_spec))
        self.log("HIL channel {}, console {}".format(self.hil_spec, self.con_spec))
        line = self._live_line(PAD0, 0.25)
        self.hil.write(("\n" + line + "\n").encode())
        end = time.time() + timeout
        stale_session = False
        sent_off = False
        while not self.live:
            if time.time() > end:
                raise RunnerError("no `OK live` from the HIL channel (got: {})".format(
                    (self.dec.lines + self.replies)[-3:] or "nothing"))
            self._pump(wait=True)
            for txt in self._new_replies():
                if txt.startswith("ERR"):
                    raise RunnerError("Teensy: {}{}".format(
                        txt, " -- reboot it first (core not IDLE)" if "IDLE" in txt else ""))
                if txt.startswith("OK live off"):
                    self.hil.write(("\n" + line + "\n").encode())
            for ln in self.dec.lines:
                if ln.startswith("ERR"):
                    raise RunnerError("Teensy: " + ln)
            if self._saw_old_telemetry and not sent_off:
                # Still LIVE from an earlier session: leave it, then enter fresh.
                stale_session = True
                sent_off = True
                self.hil.write(link.cmd_packet("live off"))
        self.phase = "pad"
        self.log("LIVE{}: sim pyros{}, streaming a still pad".format(
            " (restarted an old session)" if stale_session else "",
            "" if not self.sc.live_legs else " + LIVE LEGS (nichrome really powered)"))

    _saw_old_telemetry = False

    def _live_line(self, pad, var):
        return "live {:.6f} {:.6f} {:.6f} {:.6g} {}".format(
            pad[0], pad[1], pad[2], var, 1 if self.sc.live_legs else 0)

    # ------------------------------------------------------------- loop
    def poll(self):
        """One pass of the loop (blocks at most ~1 ms waiting for data)."""
        self._pump(wait=True)
        self._console()
        if self.task is not None:
            try:
                next(self.task)
            except StopIteration:
                self.task = None
            except RunnerError as e:
                self.task = None
                self.error = str(e)
                self.log("ERROR: " + str(e))
        self._check_end()
        now = time.perf_counter()
        if now - self._rate_t0 >= 1.0:
            self.rate_hz = self._rate_n / (now - self._rate_t0)
            self._rate_n = 0
            self._rate_t0 = now

    def _pump(self, wait):
        data = self.hil.read(wait=wait)
        if not data:
            return
        send = None
        flags = 0
        n = 0
        for kind, payload in self.dec.feed(data):
            if kind == "L":
                if payload.startswith("OK live") and not payload.startswith("OK live off"):
                    self.live = True
            elif kind == "M":
                txt = payload.decode("utf-8", "replace")
                self.replies.append(txt)
                if self.task is None and not txt.startswith("OK live"):
                    self.log("teensy: " + txt)
                if txt.startswith("OK live") and not txt.startswith("OK live off") and not self.live:
                    self.live = True
            elif kind == "T" and len(payload) == link.TELEM.size:
                if not self.live:
                    self._saw_old_telemetry = True
                    continue
                f = self._on_telem(link.parse_telem(payload))
                n += 1
                if f is not None:
                    flags |= f[1] & 1
                    send = f
        self.stats["crc_errors"] = self.dec.crc_errors
        if n:
            now = time.perf_counter()
            if self._last_rx is not None:
                self.stats["max_gap_ms"] = max(self.stats["max_gap_ms"], (now - self._last_rx) * 1e3)
            self._last_rx = now
            self._rate_n += n
            self.stats["max_batch"] = max(self.stats["max_batch"], n)
        if send is not None:
            seq, fl, a, g, baro = send
            self.hil.write(link.frame_packet(seq, (fl & ~1) | flags, a, g, baro))
            self.stats["frames_sent"] += 1

    def _on_telem(self, tel):
        if self.ms0 is None:
            self.ms0 = tel["ms"]
        t = (tel["ms"] - self.ms0) / 1000.0
        k = len(self.ticks)
        fl = self.fl
        if "cal" in self.pending:
            fl.finish_cal()
            self.timeline["cal_tick"] = k
            self.record = True
        if "launch" in self.pending:
            fl.launch()
            self.timeline["launch_tick"] = k
            self.phase = "flight"
            self.log("LAUNCH: simulated ascent motor lit at t = {:.3f} s".format(fl.t))
        self.pending.clear()
        fl.advance(t)
        b = tel["bits"]
        fl.command(tel["gimbal_x"], tel["gimbal_y"], b & link.BIT_LAND_FIRE,
                   b & link.BIT_CHUTE_RELEASE, b & link.BIT_LEGS_CMD, t)
        self.state = _state_name(tel["state"])
        if self.state == "ARMED" and "arm_tick" not in self.timeline:
            self.timeline["arm_tick"] = k
        if self._abort_sent and "abort_ms" not in self.timeline and b & link.BIT_IMU_BAD:
            self.timeline["abort_ms"] = tel["ms"]
        if self.record:
            out = {"state": self.state, "kf_alt": tel["kf_alt"], "kf_vel": tel["kf_vel"],
                   "tilt_deg": tel["tilt_deg"], "quat": tel["quat"],
                   "gimbal_x": tel["gimbal_x"], "gimbal_y": tel["gimbal_y"],
                   "chute_release": bool(b & link.BIT_CHUTE_RELEASE),
                   "chute_detected": bool(b & link.BIT_CHUTE_DETECTED),
                   "fire_landing": bool(b & link.BIT_LAND_FIRE),
                   "legs_burn": bool(b & link.BIT_LEGS_CMD),
                   "us_a": tel["us_a"], "us_b": tel["us_b"]}
            for key in ("p_x", "i_x", "d_x", "p_y", "i_y", "d_y"):
                out[key] = tel[key]
            r = fl.row(out)
            r.update({"us_chute": tel["us_chute"],
                      "chute_pos": link.CHUTE_POS.get(tel["chute_pos"], "?"),
                      "legs_pin": bool(b & link.BIT_LEGS_PIN),
                      "imu_bad": bool(b & link.BIT_IMU_BAD),
                      "tvc_active": bool(b & link.BIT_TVC_ACTIVE),
                      "stale": tel["stale"], "seq": tel["seq"]})
            self.rows.append(r)
        self.ticks.append([tel["ms"], tel["period_us"], tel["seq"], b, tel["state"]])
        self.stats["telemetry"] += 1
        if fl.cal is not None:  # the pad's first ticks have no frame yet
            self.stats["max_stale"] = max(self.stats["max_stale"], tel["stale"])
        self.last_tel = tel
        self.t = t
        return fl.frame()

    def _console(self):
        if self.con is None:
            return
        n0 = len(self.con.events)
        self.con.poll()
        for ms, name, value in self.con.events[n0:]:
            t = (ms - self.ms0) / 1000.0 if self.ms0 is not None else 0.0
            self.events.append({"t": t, "ms": ms, "code": flight.event_code(name), "value": value})
            if name == "ARM":
                self.timeline["arm_ms"] = ms
            self.log("EVENT {} {:g}  (t = {:+.3f} s from launch)".format(
                name, value, t - self.fl.launch_t) if self.fl.launch_t is not None
                else "EVENT {} {:g}".format(name, value))

    def _new_replies(self):
        out = self.replies[self._nreply:]
        self._nreply = len(self.replies)
        return out

    def _new_console(self):
        out = self.con.lines[self._con_seen:]
        self._con_seen = len(self.con.lines)
        return out

    # ----------------------------------------------------- sequences
    def _wait_reply(self, prefix, timeout):
        end = time.time() + timeout
        while time.time() < end:
            for txt in self._new_replies():
                if txt.startswith("ERR"):
                    raise RunnerError("Teensy: " + txt)
                if txt.startswith(prefix):
                    return txt
            yield
        raise RunnerError("no `{}` from the HIL channel".format(prefix))

    def _wait_console(self, needles, timeout, fails=()):
        end = time.time() + timeout
        while time.time() < end:
            for ln in self._new_console():
                if any(f in ln for f in fails):
                    raise RunnerError("console: " + ln)
                if any(n in ln for n in needles):
                    return ln
            yield
        raise RunnerError("console: no {} within {:.0f} s".format(" / ".join(needles), timeout))

    def _prepare(self):
        self.log("pad calibration (gyro bias, level, baro) ...")
        self.pending.add("cal")
        while self.fl.cal is None:
            yield
        c = self.fl.cal
        self._new_replies()
        self.hil.write(link.cmd_packet(self._live_line(c["pad_accel"], c["baro_var"])))
        yield from self._wait_reply("OK live", 2.0)
        self._new_console()
        self.con.send("mode " + self.sc.mode)
        yield from self._wait_console(("mode = ",), 3.0, fails=("only allowed", "usage"))
        self.con.send("chute lock")
        yield from self._wait_console(("chute latch LOCKED",), 3.0, fails=("only allowed",))
        self.phase = "prepared"
        self.log("prepared: mode {}, chute LOCK, pad tilt {:.2f} deg, baro std {:.3f} m".format(
            self.sc.mode, _tilt(c["pad_accel"]), c["baro_var"] ** 0.5))

    def _arm(self):
        self._new_console()
        self.con.send("arm")
        self.log("arming (the Teensy calibrates ~4 s; keep the bench still) ...")
        yield from self._wait_console(("*** ARMED",), 30.0,
                                      fails=("ARM REFUSED", "TIMED OUT", "only allowed"))
        self.phase = "armed"
        self.log("ARMED")

    def _busy(self):
        if self.task is not None:
            raise RunnerError("busy (wait for the current step)")

    def prepare(self):
        self._busy()
        if self.phase not in ("pad", "prepared"):
            raise RunnerError("prepare: only on the pad")
        self.task = self._prepare()

    def arm(self):
        self._busy()
        if self.phase != "prepared":
            raise RunnerError("arm: prepare first")
        self.task = self._arm()

    def launch(self):
        if self.state != "ARMED":
            raise RunnerError("launch: the Teensy isn't ARMED (it is {})".format(self.state))
        if self.fl.launch_t is None:
            self.pending.add("launch")

    def abort(self):
        """Operator abort: the HIL shim fails the IMU, so a core in flight
        aborts to the chute (on the pad: disarm on the console)."""
        self.hil.write(link.cmd_packet("abort"))
        self._abort_sent = True
        self.log("abort sent")

    def reboot(self):
        self.hil.write(link.cmd_packet("reboot"))
        self.log("reboot sent")

    def _check_end(self):
        fl = self.fl
        if fl.launch_t is None or self.flight_over:
            return
        if fl.pl.state.landed and self.landed_t is None:
            self.landed_t = fl.t
            self.log("on the ground ({:.2f} m/s at contact)".format(fl.touchdown_speed or 0.0))
        if self.state == "TOUCHDOWN" and self.td_state_t is None:
            self.td_state_t = fl.t
        if self.landed_t is not None:
            # 2 s after contact and 1 s after the Teensy's TOUCHDOWN state (so its
            # console EVENT line arrives too: under a chute after an abort the
            # detection can take > 4 s); 10 s after contact if it never comes.
            waited = fl.t - self.landed_t
            if (self.td_state_t is not None and waited >= 2.0
                    and fl.t - self.td_state_t >= 1.0) or waited >= 10.0:
                self.flight_over = True
                self.end_reason = "landed"
        elif fl.t - fl.launch_t > self.sc.max_flight_s:
            self.flight_over = True
            self.end_reason = "max flight time"
        if self.flight_over:
            self.phase = "done"
            self.log("flight over ({})".format(self.end_reason))

    # ---------------------------------------------------------- results
    def session_data(self):
        return {"ticks": self.ticks, "timeline": self.timeline, "pad0": PAD0,
                "scenario": self.sc.to_dict(), "profile": self.profile,
                "ports": [self.hil_spec, self.con_spec], "link": dict(self.stats),
                "console": self.con.lines if self.con else [],
                "hil_text": self.dec.lines + self.replies, "end_reason": self.end_reason}

    def result(self):
        """rows/events/summary like run.simulate(), rebased (see origin())."""
        res = {"rows": list(self.rows), "events": list(self.events),
               "summary": flight.summarize(self.fl, self.rows, self.events, self.sc.mode)}
        res["summary"]["link"] = dict(self.stats)
        return flight.rebase(res, self.origin(), self.sc.mode)

    def origin(self):
        if self.fl.launch_t is not None:
            return max(self.fl.launch_t - 1.0, 0.0)
        return self.rows[0]["t"] if self.rows else 0.0

    def finish(self, save=True, close=True):
        res = self.result()
        path = None
        if save:
            path = save_session(self.sessions_dir, self.session_data(), res, self.origin())
            self.log("saved " + path)
        if close:
            self.close()
        return res, path

    def close(self):
        for p in (self.hil, self.con.port if self.con else None):
            if p is not None:
                p.close()


def _tilt(a):
    import math
    n = math.sqrt(sum(x * x for x in a)) or 1.0
    return math.degrees(math.acos(max(-1.0, min(1.0, a[2] / n))))


# ------------------------------------------------------------ sessions
def _jsonable_events(evs):
    return [{"t": e["t"], "ms": e.get("ms"), "name": getattr(e["code"], "name", str(e["code"])),
             "value": e["value"]} for e in evs]


def save_session(root, session, result, origin, twin=None, report=None):
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(root, "live_" + stamp)
    os.makedirs(path, exist_ok=True)
    s = dict(session, origin=origin)
    with open(os.path.join(path, "session.json"), "w", encoding="utf-8") as f:
        json.dump(s, f)
    write_result(os.path.join(path, "result.json"), result)
    with open(os.path.join(path, "scenario.json"), "w", encoding="utf-8") as f:
        json.dump(session["scenario"], f, indent=1)
    return path


def write_result(path, result):
    out = dict(result, events=_jsonable_events(result["events"]))
    with open(path, "w", encoding="utf-8") as f:
        json.dump(out, f)


def read_result(path):
    with open(path, encoding="utf-8") as f:
        r = json.load(f)
    r["events"] = [dict(e, code=flight.event_code(e["name"])) for e in r["events"]]
    return r


def run_twin(session_dir):
    """Fly the twin for a saved session; writes twin.json + compare.txt.
    Returns (twin_result, table_rows, text)."""
    import twin
    with open(os.path.join(session_dir, "session.json"), encoding="utf-8") as f:
        session = json.load(f)
    live = read_result(os.path.join(session_dir, "result.json"))
    scen = Scenario.from_dict(session["scenario"])
    tw = twin.fly(session, scen, profile=session.get("profile"))
    flight.rebase(tw, session["origin"], scen.mode)
    write_result(os.path.join(session_dir, "twin.json"), tw)
    rows = twin.compare(live, tw)
    text = twin.format_table(rows)
    with open(os.path.join(session_dir, "compare.txt"), "w", encoding="utf-8") as f:
        f.write(text + "\n")
    return tw, rows, text


# ------------------------------------------------- dashboard process
LIVE_KEYS = ("t", "state", "h_true", "v_true", "x_true", "y_true", "quat_true", "quat_est",
             "kf_alt", "kf_vel", "tilt_deg", "gimbal_x_cmd", "gimbal_y_cmd", "gimbal_x_act",
             "gimbal_y_act", "chute_frac", "legs_frac", "thrust_n", "chute_release",
             "chute_detected", "legs_burn", "fire_landing", "chute_pos", "imu_bad")


def runner_process(cmd_q, out_q, hil, console, scen_dict, sketch_dir=flight.SKETCH_DIR):
    """Dashboard side: ("prepare"|"arm"|"launch"|"abort"|"reboot"|"stop") in,
    ("log", text) / ("status", dict) / ("rows", [row...]) / ("finished",
    session_dir) / ("twin", (session_dir, text, ok)) / ("error", text) /
    ("closed", None) out."""
    def log(m):
        out_q.put(("log", m))

    r = None
    try:
        r = Runner(hil, console, Scenario.from_dict(scen_dict), sketch_dir, log=log)
        r.start()
    except Exception as e:
        out_q.put(("error", str(e)))
        if r is not None:
            r.close()
        out_q.put(("closed", None))
        return
    sent = 0
    last = 0.0
    finished = False
    try:
        while True:
            r.poll()
            try:
                cmd = cmd_q.get_nowait()
            except queue.Empty:
                cmd = None
            if cmd is not None:
                if cmd == "stop":
                    break
                try:
                    if cmd == "reboot":
                        r.reboot()
                        break
                    getattr(r, cmd)()
                except RunnerError as e:
                    log("ERROR: " + str(e))
            now = time.perf_counter()
            if now - last >= 0.05:
                last = now
                new = r.rows[sent::5]          # 100 Hz is plenty for the live view
                if new:
                    sent += 5 * len(new)
                    out_q.put(("rows", [{k: x.get(k) for k in LIVE_KEYS} for x in new]))
                tel = r.last_tel or {}
                out_q.put(("status", {
                    "phase": r.phase, "state": r.state, "t": r.t, "hz": r.rate_hz,
                    "launch_t": r.fl.launch_t, "origin": r.origin(),
                    "kf_alt": tel.get("kf_alt"), "h_true": r.fl.pl.state.pos.z,
                    "tilt": tel.get("tilt_deg"), "chute": link.CHUTE_POS.get(tel.get("chute_pos")),
                    "stats": dict(r.stats), "legs": r.sc.live_legs, "error": r.error}))
            if r.flight_over and not finished:
                finished = True
                res, path = r.finish(save=True, close=False)
                out_q.put(("finished", path))
    except Exception as e:
        import traceback
        out_q.put(("error", "runner crashed: {}\n{}".format(e, traceback.format_exc())))
    finally:
        r.close()
        out_q.put(("closed", None))
