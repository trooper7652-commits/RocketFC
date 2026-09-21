#!/usr/bin/env python3
"""RocketFC simulator + flight-log dashboard.

One window, one source selector: Simulate a closed-loop flight, or open a
real SD-card flight log -- both render through the same trajectory view,
trace plots, state timeline and scrub bar, so a simulated flight and a real
one are directly comparable (and can be overlaid).

Run:  python tools/sim/dashboard.py

Requires: numpy, matplotlib, tkinter (all already used elsewhere in this
repo / stdlib) -- no new dependencies.
"""
import json
import math
import os
import sys
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Circle

import core
import logfile
import reflight as reflight_mod
import run as run_mod
import vehicle

SIM_DIR = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------------------
# Parameter panel field list: (label, vehicle.py attr, kind) for the plant
# side (applies instantly, no rebuild) --
FIELD_SPECS = [
    ("Vehicle", [
        ("Mass, descent (kg)", "mass_descent_kg"),
        ("Inertia (kg*m^2)", "inertia_kgm2"),
        ("CG from nose (m)", "cg_from_nose_m"),
        ("Gimbal pivot from nose (m)", "gimbal_pivot_from_nose_m"),
        ("CP from nose (m)", "cp_from_nose_m"),
        ("Diameter (m)", "diameter_m"),
    ]),
    ("Aero / disturbance", [
        ("CdA descent (m^2)", "cda_m2"),
        ("CdA chute (m^2)", "cda_chute_m2"),
        ("Normal-force coeff (1/rad)", "normal_force_coeff"),
        ("Aero damping (Nms)", "aero_damping_nms"),
        ("Disturbance torque (Nm)", "disturbance_torque_nm"),
        ("Roll disturbance (Nm)", "roll_disturbance_torque_nm"),
    ]),
    ("Servo", [
        ("Slew rate (deg/s)", "servo_slew_dps"),
        ("Lag tau (s)", "servo_lag_tau_s"),
    ]),
    ("Environment", [
        ("Wind (m/s)", "wind_mps"),
        ("Gust (m/s)", "gust_mps"),
    ]),
]

# Firmware constants (require a rebuild via core.build_with_overrides).
# (label, config.h NAME, literal formatter)
FIRMWARE_SPECS = [
    ("Tilt abort (deg)", "TILT_ABORT_DEG", lambda v: "{:.1f}f".format(v)),
    ("Burn table margin (m)", "BURN_TABLE_MARGIN_M", lambda v: "{:.2f}f".format(v)),
    ("Boost Kp", ("GAINS_BOOST", "kp"), lambda v: "{:.3f}f".format(v)),
    ("Boost Ki", ("GAINS_BOOST", "ki"), lambda v: "{:.3f}f".format(v)),
    ("Boost Kd", ("GAINS_BOOST", "kd"), lambda v: "{:.3f}f".format(v)),
    ("Landing Kp", ("GAINS_LANDING", "kp"), lambda v: "{:.3f}f".format(v)),
    ("Landing Ki", ("GAINS_LANDING", "ki"), lambda v: "{:.3f}f".format(v)),
    ("Landing Kd", ("GAINS_LANDING", "kd"), lambda v: "{:.3f}f".format(v)),
]

STATE_COLORS = {
    "IDLE": "#cccccc", "ARMED": "#a3c9f7", "BOOST": "#ff8a3d",
    "COAST": "#ffd23d", "APOGEE": "#c77dff", "DESCENT": "#5fb3f5",
    "LANDING_BURN": "#ff5d5d", "DESCENT_CHUTE": "#57cc99",
    "TOUCHDOWN": "#2d6a4f", "ABORT": "#d00000",
}


def dead_reckon_xy(rows, dt_key="t"):
    """Double-integrate world-frame horizontal accel from a REAL log's
    quat_est + logged body-frame accel, since a real log has no ground-truth
    position. Drifts (unbounded double integration of noisy accel) -- this
    is clearly a display aid, not a measurement, and is labeled as such
    wherever it's drawn."""
    x = y = vx = vy = 0.0
    xs, ys = [], []
    prev_t = None
    for r in rows:
        t = r[dt_key]
        dt = (t - prev_t) if prev_t is not None else 0.0
        prev_t = t
        qw, qx, qy, qz = r.get("quat_est", (1.0, 0.0, 0.0, 0.0))
        # body -> world rotation (Hamilton, matches src/core/quat.h)
        ax, ay, az = r["ax"], r["ay"], r["az"]
        # rotate (ax,ay,az) by q
        tx = 2 * (qy * az - qz * ay)
        ty = 2 * (qz * ax - qx * az)
        tz = 2 * (qx * ay - qy * ax)
        wxr = ax + qw * tx + (qy * tz - qz * ty)
        wyr = ay + qw * ty + (qz * tx - qx * tz)
        vx += wxr * dt
        vy += wyr * dt
        x += vx * dt
        y += vy * dt
        xs.append(x); ys.append(y)
    return xs, ys


class Dashboard:
    def __init__(self, root):
        self.root = root
        root.title("RocketFC Simulator")
        self.vcfg = vehicle.default_vehicle_config(core.default_dll())
        self.field_vars = {}
        self.firmware_vars = {}
        self.result = None       # current run.simulate()/reflight() result
        self.rows = []
        self.events = []
        self.is_log = False
        self.playing = False
        self.play_speed = 1.0
        self.show_dead_reckon = tk.BooleanVar(value=False)
        self._build_ui()

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        top = ttk.Frame(self.root, padding=4)
        top.pack(side=tk.TOP, fill=tk.X)
        self.source = tk.StringVar(value="simulate")
        ttk.Radiobutton(top, text="Simulate", variable=self.source,
                       value="simulate").pack(side=tk.LEFT)
        ttk.Radiobutton(top, text="Flight log", variable=self.source,
                       value="log").pack(side=tk.LEFT)
        ttk.Label(top, text="  Mode:").pack(side=tk.LEFT)
        self.mode_var = tk.StringVar(value="land")
        ttk.Combobox(top, textvariable=self.mode_var, values=["land", "chute"],
                    width=6, state="readonly").pack(side=tk.LEFT)
        ttk.Button(top, text="Run", command=self.on_run).pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="Open log...", command=self.on_open_log).pack(side=tk.LEFT)
        ttk.Button(top, text="Reflight with panel settings",
                  command=self.on_reflight).pack(side=tk.LEFT, padx=6)
        ttk.Checkbutton(top, text="Downrange (dead-reckoned for logs)",
                       variable=self.show_dead_reckon,
                       command=self.redraw).pack(side=tk.LEFT, padx=10)
        self.status = ttk.Label(top, text="Ready")
        self.status.pack(side=tk.RIGHT)

        body = ttk.Frame(self.root)
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # -- left: parameter panel --
        left = ttk.Frame(body, padding=4)
        left.pack(side=tk.LEFT, fill=tk.Y)
        canvas = tk.Canvas(left, width=260, highlightthickness=0)
        scroll = ttk.Scrollbar(left, orient="vertical", command=canvas.yview)
        inner = ttk.Frame(canvas)
        inner.bind("<Configure>",
                  lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.configure(yscrollcommand=scroll.set)
        canvas.pack(side=tk.LEFT, fill=tk.Y)
        scroll.pack(side=tk.LEFT, fill=tk.Y)

        for group, fields in FIELD_SPECS:
            ttk.Label(inner, text=group, font=("", 9, "bold")).pack(
                anchor="w", pady=(8, 0))
            for label, attr in fields:
                self._add_field(inner, label, attr)

        ttk.Label(inner, text="Firmware (rebuilds)", font=("", 9, "bold")).pack(
            anchor="w", pady=(10, 0))
        for label, key, fmt in FIRMWARE_SPECS:
            row = ttk.Frame(inner)
            row.pack(fill=tk.X, pady=1)
            ttk.Label(row, text=label, width=18).pack(side=tk.LEFT)
            default = self._firmware_default(key)
            v = tk.StringVar(value="{:.4g}".format(default))
            ttk.Entry(row, textvariable=v, width=10).pack(side=tk.LEFT)
            self.firmware_vars[key] = (v, fmt)

        ttk.Button(inner, text="Save params...", command=self.on_save_params
                  ).pack(fill=tk.X, pady=(10, 2))
        ttk.Button(inner, text="Load params...", command=self.on_load_params
                  ).pack(fill=tk.X)

        # -- right: plots --
        right = ttk.Frame(body)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.fig = plt.Figure(figsize=(10, 7.5))
        gs = self.fig.add_gridspec(3, 2, height_ratios=[2, 1, 1],
                                   width_ratios=[1, 1.4])
        self.ax_traj = self.fig.add_subplot(gs[0, 0])
        self.ax_alt = self.fig.add_subplot(gs[0, 1])
        self.ax_tilt = self.fig.add_subplot(gs[1, :])
        self.ax_state = self.fig.add_subplot(gs[2, :])
        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # -- bottom: scrubber --
        bottom = ttk.Frame(self.root, padding=4)
        bottom.pack(side=tk.BOTTOM, fill=tk.X)
        self.play_btn = ttk.Button(bottom, text="Play", command=self.toggle_play,
                                   width=6)
        self.play_btn.pack(side=tk.LEFT)
        self.scrub_var = tk.DoubleVar(value=0.0)
        self.scrub = ttk.Scale(bottom, from_=0, to=1, variable=self.scrub_var,
                               command=lambda v: self.redraw())
        self.scrub.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        self.time_label = ttk.Label(bottom, text="t = 0.00 s", width=14)
        self.time_label.pack(side=tk.LEFT)
        self.summary_label = ttk.Label(self.root, text="", padding=4,
                                       anchor="w")
        self.summary_label.pack(side=tk.BOTTOM, fill=tk.X)

    def _add_field(self, parent, label, attr):
        row = ttk.Frame(parent)
        row.pack(fill=tk.X, pady=1)
        ttk.Label(row, text=label, width=18).pack(side=tk.LEFT)
        val = getattr(self.vcfg, attr)
        v = tk.StringVar(value="{:.4g}".format(val))
        ttk.Entry(row, textvariable=v, width=10).pack(side=tk.LEFT)
        self.field_vars[attr] = v

    def _firmware_default(self, key):
        snap = core.FlightCore.config_snapshot(core.default_dll())
        if isinstance(key, tuple):
            group, field = key
            return snap["gains" + ("Boost" if group == "GAINS_BOOST" else "Land") +
                       field[0].upper() + field[1:]]
        return snap.get({"TILT_ABORT_DEG": "tiltAbortDeg",
                         "BURN_TABLE_MARGIN_M": "burnTableMarginM"}.get(key, key), 0.0)

    # ------------------------------------------------------------ actions
    def _read_vcfg(self):
        for attr, var in self.field_vars.items():
            try:
                setattr(self.vcfg, attr, float(var.get()))
            except ValueError:
                pass
        return self.vcfg

    def _firmware_overrides(self):
        overrides = {}
        gains = {}
        for key, (var, fmt) in self.firmware_vars.items():
            try:
                val = float(var.get())
            except ValueError:
                continue
            lit = fmt(val)
            if isinstance(key, tuple):
                group, field = key
                gains.setdefault(group, {})[field] = lit
            else:
                default = self._firmware_default(key)
                if abs(val - default) > 1e-9:
                    overrides[key] = lit
        for group, fields in gains.items():
            # Only include the group if at least one field actually changed.
            snap = core.FlightCore.config_snapshot(core.default_dll())
            prefix = "gains" + ("Boost" if group == "GAINS_BOOST" else "Land")
            changed = False
            for f, lit in fields.items():
                cur = snap[prefix + f[0].upper() + f[1:]]
                try:
                    if abs(float(lit.rstrip("f")) - cur) > 1e-9:
                        changed = True
                except ValueError:
                    pass
            if changed:
                overrides[group] = fields
        return overrides

    def on_run(self):
        self.status.config(text="Building/running...")
        self.root.update_idletasks()
        try:
            vcfg = self._read_vcfg()
            overrides = self._firmware_overrides()
            dll = core.build_with_overrides(overrides) if overrides else core.default_dll()
            result = run_mod.simulate(vcfg, mode=self.mode_var.get(), dll_path=dll)
        except Exception as e:
            messagebox.showerror("Simulation failed", str(e))
            self.status.config(text="Error")
            return
        self.result = result
        self.rows = result["rows"]
        self.events = result["events"]
        self.is_log = False
        self._after_new_data()
        self.status.config(text="Done")

    def on_open_log(self):
        path = filedialog.askopenfilename(
            title="Open flight log", filetypes=[("CSV", "*.csv"), ("All", "*.*")])
        if not path:
            return
        try:
            log = logfile.read_log(path)
        except Exception as e:
            messagebox.showerror("Failed to read log", str(e))
            return
        self.rows = log["rows"]
        self.events = [{"t": e["t"], "code": None, "name": e["name"],
                        "value": e["value"]} for e in log["events"]]
        self.result = None
        self.is_log = True
        self._log_path = path
        self._after_new_data()
        self.status.config(text="Loaded {}".format(os.path.basename(path)))

    def on_reflight(self):
        if not self.is_log:
            messagebox.showinfo("Reflight", "Open a flight log first.")
            return
        try:
            overrides = self._firmware_overrides()
            dll = core.build_with_overrides(overrides) if overrides else core.default_dll()
            rf = reflight_mod.reflight(self._log_path, dll_path=dll)
        except Exception as e:
            messagebox.showerror("Reflight failed", str(e))
            return
        s = rf["summary"]
        msg = "Diverged: {}\norig events: {}\nnew events:  {}\nnew abort: {}".format(
            s["diverged"], s["orig_event_sequence"], s["new_event_sequence"],
            s.get("new_abort_reason"))
        messagebox.showinfo("Reflight result (open-loop -- see run.py's docstring "
                            "for what this can/can't test)", msg)

    def on_save_params(self):
        path = filedialog.asksaveasfilename(defaultextension=".json",
                                            filetypes=[("JSON", "*.json")])
        if path:
            self._read_vcfg().to_json(path)

    def on_load_params(self):
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if not path:
            return
        self.vcfg = vehicle.VehicleConfig.from_json(path)
        for attr, var in self.field_vars.items():
            var.set("{:.4g}".format(getattr(self.vcfg, attr)))

    # ------------------------------------------------------------- replay
    def _after_new_data(self):
        self.t_max = self.rows[-1]["t"] if self.rows else 1.0
        self.scrub.configure(to=max(self.t_max, 0.001))
        self.scrub_var.set(0.0)
        self._build_summary_text()
        self.redraw()

    def _build_summary_text(self):
        if self.result and not self.is_log:
            s = self.result["summary"]
            self.summary_label.config(
                text="apogee {:.1f} m @ {:.1f}s | touchdown {:.2f} m/s @ {} | "
                     "max tilt {:.1f} deg | abort {}".format(
                         s["apogee_h"] or 0, s["apogee_t"] or 0,
                         s["touchdown_speed_mps"] or float("nan"),
                         "{:.1f}s".format(s["touchdown_t"]) if s["touchdown_t"] else "n/a",
                         s["max_tilt_deg"], s["abort_reason"] or "none"))
        else:
            self.summary_label.config(text="Flight log: {} rows, {} events".format(
                len(self.rows), len(self.events)))

    def toggle_play(self):
        self.playing = not self.playing
        self.play_btn.config(text="Pause" if self.playing else "Play")
        if self.playing:
            self._tick()

    def _tick(self):
        if not self.playing:
            return
        t = self.scrub_var.get() + 0.05 * self.play_speed
        if t > self.t_max:
            t = 0.0
        self.scrub_var.set(t)
        self.redraw()
        self.root.after(50, self._tick)

    # -------------------------------------------------------------- draw
    def redraw(self):
        if not self.rows:
            return
        t_now = self.scrub_var.get()
        self.time_label.config(text="t = {:.2f} s".format(t_now))
        rows = self.rows
        times = [r["t"] for r in rows]

        for ax in (self.ax_traj, self.ax_alt, self.ax_tilt, self.ax_state):
            ax.clear()

        # -- altitude --
        kf = [r["kf_alt"] for r in rows]
        self.ax_alt.plot(times, kf, label="KF alt", color="#1f77b4")
        if not self.is_log:
            truth = [r["h_true"] for r in rows]
            self.ax_alt.plot(times, truth, label="truth", color="#999999",
                             linestyle="--", alpha=0.7)
        self.ax_alt.axvline(t_now, color="k", alpha=0.4)
        self.ax_alt.set_ylabel("altitude (m)")
        self.ax_alt.legend(loc="upper right", fontsize=8)

        # -- tilt + gimbal --
        tilt = [r["tilt_deg"] for r in rows]
        self.ax_tilt.plot(times, tilt, label="tilt (deg)", color="#d62728")
        if not self.is_log:
            gx = [math.degrees(r["gimbal_x_cmd"]) for r in rows]
            gy = [math.degrees(r["gimbal_y_cmd"]) for r in rows]
        else:
            gx = [r["gimbal_x_deg"] for r in rows]
            gy = [r["gimbal_y_deg"] for r in rows]
        self.ax_tilt.plot(times, gx, label="gimbal X (deg)", alpha=0.7)
        self.ax_tilt.plot(times, gy, label="gimbal Y (deg)", alpha=0.7)
        self.ax_tilt.axvline(t_now, color="k", alpha=0.4)
        self.ax_tilt.legend(loc="upper right", fontsize=8, ncol=3)
        self.ax_tilt.set_ylabel("deg")

        # -- state timeline --
        prev_state = rows[0]["state"]
        seg_start = times[0]
        for i in range(1, len(rows)):
            st = rows[i]["state"]
            if st != prev_state:
                self.ax_state.axvspan(seg_start, rows[i]["t"],
                                      color=STATE_COLORS.get(prev_state, "#ddd"))
                seg_start = rows[i]["t"]
                prev_state = st
        self.ax_state.axvspan(seg_start, times[-1],
                              color=STATE_COLORS.get(prev_state, "#ddd"))
        for ev in self.events:
            self.ax_state.axvline(ev["t"], color="k", alpha=0.5, linestyle="--")
        self.ax_state.axvline(t_now, color="k", alpha=0.8)
        self.ax_state.set_yticks([])
        self.ax_state.set_xlabel("t (s)")

        # -- trajectory / vehicle view --
        if not self.is_log:
            xs = [r["x_true"] for r in rows]
            zs = [r["h_true"] for r in rows]
            label = "true downrange"
        elif self.show_dead_reckon.get():
            xs, _ = dead_reckon_xy(rows)
            zs = [r["kf_alt"] for r in rows]
            label = "dead-reckoned (drifts, m within seconds)"
        else:
            xs = [0.0] * len(rows)
            zs = [r["kf_alt"] for r in rows]
            label = "altitude only (honest)"
        self.ax_traj.plot(xs, zs, color="#2ca02c", alpha=0.8)
        # current position marker + tilt-oriented rocket icon
        idx = min(range(len(times)), key=lambda i: abs(times[i] - t_now))
        cx, cz = xs[idx], zs[idx]
        ang = math.radians(tilt[idx])
        L = max(zs) * 0.06 + 0.5 if zs else 1.0
        dx, dz = L * math.sin(ang), L * math.cos(ang)
        self.ax_traj.plot([cx - dx, cx + dx], [cz - dz, cz + dz], color="#333",
                          linewidth=3)
        self.ax_traj.add_patch(Circle((cx + dx, cz + dz), L * 0.15, color="#d62728"))
        self.ax_traj.set_title(label, fontsize=8)
        self.ax_traj.set_xlabel("downrange (m)")
        self.ax_traj.set_ylabel("altitude (m)")

        self.fig.tight_layout()
        self.canvas.draw_idle()


def main():
    root = tk.Tk()
    Dashboard(root)
    root.geometry("1400x850")
    root.mainloop()


if __name__ == "__main__":
    sys.exit(main() or 0)
