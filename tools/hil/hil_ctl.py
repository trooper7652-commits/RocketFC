#!/usr/bin/env python3
"""Laptop side of the HIL ground test: talks to the HIL build's two USB ports.

The HIL build enumerates as two serial ports: the normal RocketFC CLI
("console") and the HIL control channel. Both are found automatically (the
HIL port answers `hello`); override with --port / --console.

    python tools/hil/hil_ctl.py ports                  # which COM port is which
    python tools/hil/hil_ctl.py status
    python tools/hil/hil_ctl.py load tools/hil/traces/nominal
    python tools/hil/hil_ctl.py cli "chute lock"       # any RocketFC CLI command
    python tools/hil/hil_ctl.py cli arm
    python tools/hil/hil_ctl.py go tools/hil/traces/nominal
    python tools/hil/hil_ctl.py abort

`go` asks you to type GO, counts down 10 s, starts playback, and then shows
the cues: "LIGHT THE ASCENT MOTOR" at the trace's ignition time and when the
landing motor should fire. Ctrl-C during playback sends `abort`. Each run is
saved to tools/hil/sessions/ for compare.py.
"""
import argparse
import datetime
import json
import os
import sys
import time

import serial
import serial.tools.list_ports

import hil_trace

PJRC_VID = 0x16C0
SESSIONS_DIR = os.path.join(hil_trace.HIL_DIR, "sessions")


def beep(freq=880, ms=150):
    try:
        import winsound
        winsound.Beep(freq, ms)
    except (ImportError, RuntimeError):
        sys.stdout.write("\a")
        sys.stdout.flush()


class Link:
    def __init__(self, port, console=None):
        self.hil = serial.Serial(port, 115200, timeout=0.05)
        self.con = serial.Serial(console, 115200, timeout=0.05) if console else None
        self._buf = {"hil": b"", "con": b""}

    def lines(self, which):
        """Complete lines received so far on 'hil' or 'con' (non-blocking)."""
        s = self.hil if which == "hil" else self.con
        if s is None:
            return []
        self._buf[which] += s.read(s.in_waiting or 1)
        *done, self._buf[which] = self._buf[which].split(b"\n")
        return [d.decode("utf-8", "replace").rstrip("\r") for d in done]

    def send(self, line, which="hil"):
        s = self.hil if which == "hil" else self.con
        s.write((line + "\n").encode())

    def ask(self, line, timeout=2.0):
        """Send a HIL command, return its OK/ERR reply line (other lines are echoed)."""
        self.send(line)
        end = time.time() + timeout
        while time.time() < end:
            for ln in self.lines("hil"):
                if ln.startswith(("OK", "ERR")):
                    return ln
                print("  hil:", ln)
            self.echo_console()
        raise SystemExit("no reply to `{}` from the HIL port".format(line))

    def echo_console(self, prefix="  fc: "):
        out = self.lines("con")
        for ln in out:
            print(prefix + ln)
        return out


def find_ports():
    """(hil_port, console_port) among PJRC serial ports, by asking `hello`."""
    cands = [p.device for p in serial.tools.list_ports.comports() if p.vid == PJRC_VID]
    hil = None
    for dev in cands:
        try:
            with serial.Serial(dev, 115200, timeout=0.3) as s:
                s.reset_input_buffer()
                s.write(b"hello\n")
                end = time.time() + 1.0
                got = b""
                while time.time() < end and b"OK hello" not in got:
                    got += s.read(256)
                if b"OK hello" in got:
                    hil = dev
                    break
        except serial.SerialException:
            continue
    others = [d for d in cands if d != hil]
    return hil, (others[0] if len(others) == 1 else None), cands


def open_link(args, need_console=False):
    hil, con, cands = (args.port, args.console, None) if args.port else find_ports()
    if args.console:
        con = args.console
    if not hil:
        raise SystemExit("no HIL port found (PJRC ports: {}). Is the HIL build "
                         "uploaded (tools/hil/make_hil.py --upload)?".format(cands))
    if need_console and not con:
        raise SystemExit("couldn't tell which port is the RocketFC console; pass --console COMx")
    return Link(hil, con)


def cmd_ports(args):
    hil, con, cands = find_ports()
    print("PJRC serial ports:", ", ".join(cands) or "none")
    print("HIL control:      ", hil or "not found")
    print("RocketFC console: ", con or "unknown")


def cmd_status(args):
    link = open_link(args)
    print(link.ask("status"))


def cmd_load(args):
    blob, meta = hil_trace.load(args.trace)
    link = open_link(args)
    p = meta["pad_accel"]
    rep = link.ask("load {} {} {} {:.6f} {:.6f} {:.6f} {:.6g}".format(
        meta["frames"], meta["crc32"], meta["mode"], p[0], p[1], p[2], meta["baro_var"]))
    print(rep)
    if not rep.startswith("OK send"):
        return 1
    for i in range(0, len(blob), 4096):
        link.hil.write(blob[i:i + 4096])
    link.hil.flush()
    end = time.time() + 15.0
    while time.time() < end:
        for ln in link.lines("hil"):
            print(ln)
            if ln.startswith(("OK loaded", "ERR")):
                return 0 if ln.startswith("OK loaded") else 1
    print("no confirmation from the Teensy")
    return 1


def cmd_cli(args):
    link = open_link(args, need_console=True)
    link.lines("con")
    link.send(args.command, which="con")
    end = time.time() + args.wait
    while time.time() < end:
        for ln in link.lines("con"):
            print(ln)
            if "ARMED" in ln or "REFUSED" in ln or "TIMED OUT" in ln:
                end = min(end, time.time() + 0.3)
    return 0


def cmd_abort(args):
    link = open_link(args)
    print(link.ask("abort"))


def cmd_unload(args):
    link = open_link(args)
    print(link.ask("unload"))


def cmd_go(args):
    blob, meta = hil_trace.load(args.trace)
    link = open_link(args)
    st = link.ask("status")
    print(st)
    if "mode=HOLD" not in st or "core=ARMED" not in st:
        print("\nNot ready: need mode=HOLD (trace loaded) and core=ARMED.")
        return 1
    if "frames={}".format(meta["frames"]) not in st:
        print("\nThe loaded trace has a different frame count than", args.trace)
        return 1

    launch_s, fire_s = meta["launch_s"], meta.get("fire_s")
    print("\nTrace {}: launch cue at +{:.1f} s, landing-motor fire {}.".format(
        meta["name"], launch_s,
        "expected at +{:.1f} s".format(fire_s) if fire_s is not None else "NOT expected"))
    print("Everyone at the safety distance? Pyro battery plugged in? "
          "Ctrl-C at any time sends abort.")
    if input("Type GO to start the countdown: ").strip() != "GO":
        print("cancelled.")
        return 1

    session = {"trace": os.path.abspath(args.trace), "trace_name": meta["name"],
               "crc32": meta["crc32"], "started": datetime.datetime.now().isoformat(
                   timespec="seconds"), "go_ms": None, "hil": [], "console": []}
    playing = False
    try:
        for k in range(10, 0, -1):
            print("  T-{:>2}".format(int(round(k + launch_s))))
            beep(660, 120)
            link.echo_console()
            time.sleep(0.88)
        rep = link.ask("go")
        print(rep)
        if not rep.startswith("OK go"):
            return 1
        session["go_ms"] = int(rep.split()[2])
        playing = True
        t_go = time.time()
        launch_cued = fire_cued = False
        done_at = None
        while True:
            t = time.time() - t_go
            if not launch_cued and t >= launch_s - 3.0:
                for k in (3, 2, 1):
                    print("  T-{}".format(k))
                    beep(660, 120)
                    time.sleep(max(0.0, launch_s - k + 1 - (time.time() - t_go)))
                print("\n  >>> T-0  LIGHT THE ASCENT MOTOR NOW <<<\n")
                beep(1320, 600)
                launch_cued = True
            if fire_s is not None and not fire_cued and t >= fire_s:
                print("  (landing motor fire expected now, +{:.2f} s)".format(fire_s))
                fire_cued = True
            for ln in link.lines("hil"):
                session["hil"].append([round(t, 3), ln])
                if ln.startswith("P "):
                    _, i, n, state = ln.split()
                    print("  +{:5.1f} s  frame {:>5}/{}  {}".format(int(i) * hil_trace.FAST_DT,
                                                                   int(i), n, state))
                else:
                    print("  hil:", ln)
                    if ln.startswith("DONE"):
                        done_at = time.time()
            for ln in link.echo_console():
                session["console"].append([round(t, 3), ln])
            if done_at and time.time() - done_at > 12.0:
                break
    except KeyboardInterrupt:
        if not playing:
            print("\nCtrl-C: countdown cancelled before `go` -- nothing was started.")
            return 1
        print("\nCtrl-C: sending abort")
        link.send("abort")
        time.sleep(0.3)
        for ln in link.lines("hil"):
            print("  hil:", ln)
            session["hil"].append([None, ln])
        session["aborted_by_operator"] = True
    finally:
        os.makedirs(SESSIONS_DIR, exist_ok=True)
        path = os.path.join(SESSIONS_DIR, "session_{}.json".format(
            datetime.datetime.now().strftime("%Y%m%d_%H%M%S")))
        with open(path, "w") as f:
            json.dump(session, f, indent=1)
        print("session saved:", path)
    print("\nNext: copy FLIGHTS/flight_NNN.csv and HIL/IMU_NNN.BIN off the SD card, "
          "then run tools/hil/compare.py.")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", help="HIL control port (default: auto)")
    ap.add_argument("--console", help="RocketFC CLI port (default: auto)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ports").set_defaults(fn=cmd_ports)
    sub.add_parser("status").set_defaults(fn=cmd_status)
    sub.add_parser("abort").set_defaults(fn=cmd_abort)
    sub.add_parser("unload").set_defaults(fn=cmd_unload)
    p = sub.add_parser("load")
    p.add_argument("trace")
    p.set_defaults(fn=cmd_load)
    p = sub.add_parser("go")
    p.add_argument("trace")
    p.set_defaults(fn=cmd_go)
    p = sub.add_parser("cli", help="send one line to the RocketFC console and show the reply")
    p.add_argument("command")
    p.add_argument("--wait", type=float, default=8.0, help="seconds to show replies")
    p.set_defaults(fn=cmd_cli)
    args = ap.parse_args()
    try:
        return args.fn(args) or 0
    except KeyboardInterrupt:
        print("\ncancelled.")
        return 1


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    sys.exit(main())
