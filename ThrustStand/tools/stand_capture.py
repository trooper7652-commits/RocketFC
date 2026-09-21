#!/usr/bin/env python3
"""Live capture and plot for a ThrustStand run.

    python tools/stand_capture.py --port COM5 --motor C6-5

Connects to the stand over serial, mirrors every line to the terminal, writes
everything to runs/<timestamp>_<motor>.csv as it arrives, and shows a live
thrust-vs-time plot that updates while you arm and fire. When the stand
reaches SUMMARY the window keeps updating for a few more seconds (in case a
slow ejection/second event follows) and then the run is finalized: a static
PNG and stats are produced via plot_thrust.py, exactly as if you had run it
by hand afterward.

Ctrl-C at any time finalizes whatever was captured so far.

Requires:  pip install pyserial matplotlib
"""
import argparse
import datetime
import glob
import os
import re
import subprocess
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")

try:
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("matplotlib is required:  pip install matplotlib")

HEADER_RE = re.compile(r"^t_ms,state,raw,thrust_n,thrust_filt_n,evt$")
SETTLE_S = 3.0  # keep the live window open this long after SUMMARY


def find_port():
    ports = list(list_ports.comports())
    if not ports:
        return None
    # Prefer anything that looks like an Arduino/CH340/FTDI adapter.
    for p in ports:
        desc = (p.description or "").lower()
        if any(k in desc for k in ("arduino", "ch340", "usb serial", "uno", "nano")):
            return p.device
    return ports[0].device


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None, help="serial port (default: auto-detect)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--motor", default="run", help="label used in the output filename")
    ap.add_argument("--out-dir", default=os.path.join(os.path.dirname(__file__), "..", "runs"))
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        sys.exit("no serial port found -- pass --port COMx explicitly")
    print(f"connecting to {port} @ {args.baud}...")

    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    safe_motor = re.sub(r"[^A-Za-z0-9._-]+", "_", args.motor)
    csv_path = os.path.join(out_dir, f"{stamp}_{safe_motor}.csv")

    ser = serial.Serial(port, args.baud, timeout=0.2)
    time.sleep(2.0)  # AVR boards reset on port open; let the sketch boot
    ser.reset_input_buffer()

    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 5))
    line_raw, = ax.plot([], [], color="tab:gray", alpha=0.4, label="raw")
    line_filt, = ax.plot([], [], color="tab:blue", label="filtered")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("thrust (N)")
    ax.set_title(f"ThrustStand live -- {args.motor}")
    ax.legend(loc="upper right")
    status_text = ax.text(0.02, 0.95, "waiting for stand...", transform=ax.transAxes,
                          va="top", fontsize=10,
                          bbox=dict(boxstyle="round", fc="white", alpha=0.85))

    t0 = None
    ts, raws, filts = [], [], []
    summary_at = None
    header_seen = False
    n_written = 0

    print(f"writing {csv_path}")
    print("press Ctrl-C to stop and finalize the run\n")

    with open(csv_path, "w", newline="\n") as csv_f:
        try:
            while True:
                raw_line = ser.readline()
                if raw_line:
                    line = raw_line.decode("utf-8", errors="replace").rstrip("\r\n")
                    print(line)
                    csv_f.write(line + "\n")
                    n_written += 1
                    if n_written % 20 == 0:
                        csv_f.flush()

                    if HEADER_RE.match(line):
                        header_seen = True
                        continue
                    if line.startswith("#"):
                        if "state=" in line or line.startswith("#--"):
                            status_text.set_text(line.lstrip("# "))
                        continue
                    if not header_seen or "," not in line:
                        continue

                    parts = line.split(",")
                    if len(parts) < 6:
                        continue
                    t_ms, state, raw_c, n_v, filt_v, evt = parts[:6]
                    if evt.startswith("EVT:"):
                        name = evt[4:].split(":")[0]
                        status_text.set_text(f"EVT: {name}")
                        if name == "SAFE" and state == "":
                            pass
                        continue
                    if not n_v:
                        continue

                    tms = float(t_ms)
                    if t0 is None:
                        t0 = tms
                    ts.append((tms - t0) / 1000.0)
                    raws.append(float(n_v))
                    filts.append(float(filt_v))
                    status_text.set_text(f"state={state}  n={float(n_v):.2f} N")

                    if state == "SUMMARY" and summary_at is None:
                        summary_at = time.time()

                # Redraw at most ~15 Hz regardless of serial rate.
                if ts:
                    line_raw.set_data(ts, raws)
                    line_filt.set_data(ts, filts)
                    ax.relim()
                    ax.autoscale_view()
                    fig.canvas.draw_idle()
                fig.canvas.flush_events()

                if summary_at is not None and time.time() - summary_at > SETTLE_S:
                    print("\nrun complete.")
                    break
        except KeyboardInterrupt:
            print("\ninterrupted -- finalizing what was captured.")
        finally:
            ser.close()

    finalize(csv_path)
    plt.ioff()
    plt.show()


def finalize(csv_path):
    if os.path.getsize(csv_path) == 0:
        print("nothing was captured; not analyzing an empty file.")
        return
    plot_script = os.path.join(os.path.dirname(__file__), "plot_thrust.py")
    print(f"\nanalyzing {csv_path} ...")
    subprocess.run([sys.executable, plot_script, csv_path])


if __name__ == "__main__":
    main()
