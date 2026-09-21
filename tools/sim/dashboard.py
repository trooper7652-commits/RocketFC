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
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import core
# flight3d's module level is stdlib + logfile only -- pygame and PyOpenGL are
# imported inside FlightView, so a machine without them can still run the
# dashboard and only fails when the 3D window is actually opened.
import flight3d
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

STAGE_STRIP_H = 20

# Points per plotted series, for display only (see Dashboard._decimate).
PLOT_POINTS = 1500

# Frame-rate target for the 3D window. The renderer can go far faster, but
# past display refresh it is just heat -- and it shares a process with Tk.
TARGET_3D_FPS = 60.0
# How early a poll may be and still count as "on time" -- see _pump_3d.
FRAME_GATE_TOLERANCE = 0.005


# dead_reckon_xy now lives in logfile.py -- it is log-domain reconstruction,
# and flight3d.py needs the same answer for the same log.
dead_reckon_xy = logfile.dead_reckon_xy


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
        self.view3d = None          # flight3d.FlightView while the window is open
        self._pump_job = None       # the after() id driving it
        self._stage_bands = []      # canvas item ids, rebuilt per flight
        self._stage_playhead = None
        self._cursors = []          # (axes, animated vline) pairs
        self._backgrounds = None    # cached blit backgrounds, one per axes
        self._last_full_draw = 0.0  # throttle for the non-blit fallback
        self._last_3d_frame = None  # for real-time playback pacing
        self._cursor_due = 0.0      # next time the 2D cursors may update
        self._render_due = 0.0      # next time the 3D view may render
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
        ttk.Button(top, text="3D view", command=self.on_open_3d).pack(side=tk.LEFT)
        ttk.Checkbutton(top, text="3D: dead-reckon a log's downrange",
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
        # The trajectory used to live here; the 3D view shows it far better,
        # so the slot went to the PID terms -- recorded every tick and written
        # to the log, but until now plotted only by tools/plot_flight.py.
        self.ax_pid = self.fig.add_subplot(gs[0, 0])
        self.ax_alt = self.fig.add_subplot(gs[0, 1])
        self.ax_tilt = self.fig.add_subplot(gs[1, :])
        self.ax_state = self.fig.add_subplot(gs[2, :])
        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.mpl_connect("draw_event", self._on_mpl_draw)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # -- bottom: scrubber + stage strip --
        self.summary_label = ttk.Label(self.root, text="", padding=4,
                                       anchor="w")
        self.summary_label.pack(side=tk.BOTTOM, fill=tk.X)

        bottom = ttk.Frame(self.root, padding=4)
        bottom.pack(side=tk.BOTTOM, fill=tk.X)
        self.play_btn = ttk.Button(bottom, text="Play", command=self.toggle_play,
                                   width=6)
        self.play_btn.pack(side=tk.LEFT)

        track = ttk.Frame(bottom)
        track.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        self.scrub_var = tk.DoubleVar(value=0.0)
        self.scrub = ttk.Scale(track, from_=0, to=1, variable=self.scrub_var,
                               command=lambda v: self.redraw())
        self.scrub.pack(side=tk.TOP, fill=tk.X)
        # The stage strip sits directly under the slider on the same x-extent,
        # so a band lines up with the slider position that reaches it. It is
        # also click/drag-seekable, which makes the labelled strip itself a
        # transport control rather than just a legend.
        self.stage_canvas = tk.Canvas(track, height=STAGE_STRIP_H,
                                      highlightthickness=0, bd=0,
                                      background="#1e2126")
        self.stage_canvas.pack(side=tk.TOP, fill=tk.X)
        self.stage_canvas.bind("<Configure>", lambda e: self._draw_stage_strip())
        self.stage_canvas.bind("<Button-1>", self._on_stage_click)
        self.stage_canvas.bind("<B1-Motion>", self._on_stage_click)

        self.time_label = ttk.Label(bottom, text="t = 0.00 s", width=14)
        self.time_label.pack(side=tk.LEFT)

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
        self._draw_stage_strip()
        self._plot_flight()
        # An open 3D window is showing the PREVIOUS flight; re-point it at the
        # new one rather than leaving it silently stale.
        if self.view3d is not None and not self.view3d.closed:
            try:
                self.view3d.track = self._make_track()
                self.view3d.show_ghost = self.view3d.track.has_estimate
            except Exception:
                pass
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

    # ------------------------------------------------------- stage strip
    def _stages(self):
        """Contiguous runs of the same state, as (t0, t1, name)."""
        if not self.rows:
            return []
        out = []
        start = self.rows[0]["t"]
        cur = self.rows[0]["state"]
        for r in self.rows[1:]:
            if r["state"] != cur:
                out.append((start, r["t"], cur))
                start, cur = r["t"], r["state"]
        out.append((start, self.rows[-1]["t"], cur))
        return out

    def _draw_stage_strip(self):
        c = self.stage_canvas
        c.delete("all")
        self._stage_playhead = None
        if not self.rows:
            return
        w = max(c.winfo_width(), 1)
        h = STAGE_STRIP_H
        t_max = max(getattr(self, "t_max", 0.0), 1e-6)

        for t0, t1, name in self._stages():
            x0 = w * (t0 / t_max)
            x1 = max(x0 + 1, w * (t1 / t_max))
            c.create_rectangle(x0, 0, x1, h, fill=STATE_COLORS.get(name, "#888"),
                               width=0)
            # Abbreviate, then drop entirely rather than let a one-tick band
            # like APOGEE smear its name across its neighbours.
            label = flight3d.STAGE_ABBREV.get(name, name)
            if (x1 - x0) > len(label) * 7 + 6:
                c.create_text((x0 + x1) / 2, h / 2, text=label,
                              fill="#14161a", font=("TkDefaultFont", 7))

        for ev in self.events:
            ex = w * (ev["t"] / t_max)
            c.create_line(ex, 0, ex, 4, fill="#ffffff")

        px = w * (self.scrub_var.get() / t_max)
        self._stage_playhead = c.create_line(px, 0, px, h, fill="#ffffff",
                                             width=2)

    def _move_stage_playhead(self):
        if self._stage_playhead is None or not self.rows:
            return
        c = self.stage_canvas
        w = max(c.winfo_width(), 1)
        t_max = max(getattr(self, "t_max", 0.0), 1e-6)
        px = w * (self.scrub_var.get() / t_max)
        c.coords(self._stage_playhead, px, 0, px, STAGE_STRIP_H)

    def _on_stage_click(self, event):
        if not self.rows:
            return
        w = max(self.stage_canvas.winfo_width(), 1)
        t_max = max(getattr(self, "t_max", 0.0), 1e-6)
        self.scrub_var.set(max(0.0, min(1.0, event.x / w)) * t_max)
        self.redraw()

    # ------------------------------------------------------------ 3D view
    def _make_track(self):
        """Build the 3D view's Track from whatever is currently loaded.

        For a sim run the landing speed comes from run.simulate()'s own
        summary, which is derived from the plant's touchdown detection and is
        authoritative; Track's fallback derivation is only for bare logs.
        """
        landing = None
        if self.result and not self.is_log:
            landing = self.result["summary"].get("touchdown_speed_mps")
        return flight3d.Track(
            self.rows, self.events, is_log=self.is_log,
            dead_reckon=self.show_dead_reckon.get(),
            label=("flight log" if self.is_log else "simulated flight"),
            landing_speed=landing)

    def on_open_3d(self):
        if not self.rows:
            messagebox.showinfo("3D view", "Run a flight or open a log first.")
            return
        if self.view3d is not None and not self.view3d.closed:
            return  # already open
        try:
            snap = core.FlightCore.config_snapshot(core.default_dll())
            self.view3d = flight3d.FlightView(
                self._make_track(), vehicle_length_m=self.vcfg.length_m,
                gimbal_limit_deg=math.degrees(snap["gimbalMaxRad"]))
        except Exception as e:
            self.view3d = None
            messagebox.showerror(
                "3D view unavailable",
                "{}\n\nThe 3D window needs pygame and PyOpenGL:\n"
                "    pip install pygame PyOpenGL".format(e))
            return
        self._pump_3d()

    def _pump_3d(self):
        """Drive the 3D window from Tk's event loop.

        Runs whether or not playback is going, so the view stays orbitable
        while paused. Both windows read and write the same scrub_var, which
        is what keeps them in sync without any IPC.
        """
        self._pump_job = None
        view = self.view3d
        if view is None or view.closed:
            self.view3d = None
            return
        try:
            ev = view.pump()
            if ev["closed"]:
                self.view3d = None
                return
            if ev["toggle_play"]:
                self.toggle_play()

            now = time.perf_counter()
            dt = now - (self._last_3d_frame or now)
            self._last_3d_frame = now

            if ev["seek"] is not None:
                self.scrub_var.set(ev["seek"])
                self._cursor_due = 0.0      # a seek should show immediately
            elif self.playing:
                # The render loop paces playback: real elapsed time, so speed
                # is independent of however fast this machine draws.
                t = self.scrub_var.get() + dt * self.play_speed
                self.scrub_var.set(0.0 if t > self.t_max else t)

            self.time_label.config(
                text="t = {:.2f} s".format(self.scrub_var.get()))
            # The 2D cursors are cheap now (~4 ms blitted) but still pointless
            # above ~30 Hz.
            if now >= self._cursor_due:
                self._cursor_due = now + (1.0 / 30.0)
                self._move_stage_playhead()
                self._update_cursor()

            # Render gated on ELAPSED TIME, not on the after() interval: Tk's
            # timer granularity on Windows is ~10-15 ms, so asking for
            # after(16) actually delivered ~43 FPS. Poll often, draw at 60.
            #
            # The tolerance matters: with polls landing on a ~10-15 ms grid, a
            # strict "16.67 ms must have elapsed" test rejects the poll at
            # 15 ms and waits for the one at 25 ms, aliasing 60 FPS down to
            # ~49. Accepting a poll that is a few ms early costs nothing and
            # lands much closer to the target.
            if now + FRAME_GATE_TOLERANCE >= self._render_due:
                self._render_due = now + (1.0 / TARGET_3D_FPS)
                view.render(self.scrub_var.get())
        except Exception:
            # A dead GL context should close the window, not take the
            # dashboard down with it.
            try:
                view.close()
            except Exception:
                pass
            self.view3d = None
            return
        # Poll at ~200 Hz so mouse input stays responsive and the 60 FPS
        # render gate above can actually be hit; the callback is nearly
        # free on the polls where nothing is due.
        self._pump_job = self.root.after(5, self._pump_3d)

    def toggle_play(self):
        self.playing = not self.playing
        self.play_btn.config(text="Pause" if self.playing else "Play")
        if self.playing:
            self._tick()

    def _tick(self):
        """Playback clock, used only when the 3D window is NOT open.

        With the 3D window open, _pump_3d advances time instead, so playback
        is paced by the render loop and the two timers don't both drive the
        clock (which made playback speed depend on which one won).
        """
        if not self.playing:
            return
        if self.view3d is None or self.view3d.closed:
            self._advance(0.05 * self.play_speed)
        self.root.after(50, self._tick)

    def _advance(self, dt):
        t = self.scrub_var.get() + dt
        if t > self.t_max:
            t = 0.0
        self.scrub_var.set(t)
        self.redraw()

    # -------------------------------------------------------------- draw
    def redraw(self):
        """Cheap per-frame update: move the cursors, nothing else.

        This used to re-plot every series from scratch (~140 ms, measured),
        which saturated Tk's event loop and left the 3D window ~12 frames a
        second. The traces don't change while scrubbing -- only the playhead
        does -- so plotting now happens once per flight in _plot_flight() and
        this just moves four vertical lines.
        """
        if not self.rows:
            return
        self.time_label.config(text="t = {:.2f} s".format(self.scrub_var.get()))
        self._move_stage_playhead()
        self._update_cursor()

    def _decimate(self, seq, n=PLOT_POINTS):
        """Every k-th element, for DISPLAY only.

        ~6600 samples x 11 series is a lot for matplotlib to rasterise, and at
        this figure size it cannot resolve more than a couple of thousand
        points anyway. self.rows stays full resolution -- this only thins what
        gets handed to plot(). Same idea as flight3d.Track's trail decimation.
        """
        if len(seq) <= n:
            return seq
        step = len(seq) // n + 1
        out = seq[::step]
        if out[-1] is not seq[-1]:
            out = out + [seq[-1]]
        return out

    def _plot_flight(self):
        """Draw everything that doesn't move. Once per loaded flight."""
        if not self.rows:
            return
        rows = self.rows
        d = self._decimate
        times = d([r["t"] for r in rows])

        for ax in (self.ax_pid, self.ax_alt, self.ax_tilt, self.ax_state):
            ax.clear()

        # -- altitude --
        self.ax_alt.plot(times, d([r["kf_alt"] for r in rows]), label="KF alt",
                         color="#1f77b4")
        if not self.is_log:
            self.ax_alt.plot(times, d([r["h_true"] for r in rows]),
                             label="truth", color="#999999", linestyle="--",
                             alpha=0.7)
        self.ax_alt.set_ylabel("altitude (m)")
        self.ax_alt.legend(loc="upper right", fontsize=8)

        # -- tilt + gimbal --
        self.ax_tilt.plot(times, d([r["tilt_deg"] for r in rows]),
                          label="tilt (deg)", color="#d62728")
        if not self.is_log:
            gx = d([math.degrees(r["gimbal_x_cmd"]) for r in rows])
            gy = d([math.degrees(r["gimbal_y_cmd"]) for r in rows])
        else:
            gx = d([r["gimbal_x_deg"] for r in rows])
            gy = d([r["gimbal_y_deg"] for r in rows])
        self.ax_tilt.plot(times, gx, label="gimbal X (deg)", alpha=0.7)
        self.ax_tilt.plot(times, gy, label="gimbal Y (deg)", alpha=0.7)
        self.ax_tilt.legend(loc="upper right", fontsize=8, ncol=3)
        self.ax_tilt.set_ylabel("deg")

        # -- state timeline --
        for t0, t1, name in self._stages():
            self.ax_state.axvspan(t0, t1, color=STATE_COLORS.get(name, "#ddd"))
        for ev in self.events:
            self.ax_state.axvline(ev["t"], color="k", alpha=0.5, linestyle="--")
        self.ax_state.set_yticks([])
        self.ax_state.set_xlabel("t (s)")

        # -- PID terms --
        # Both axes' P/I/D contributions in gimbal radians. X solid, Y dashed,
        # matching colours per term, so the pair can be compared without six
        # separate legend lookups.
        for key, color, style, label in (
                ("p_x", "#1f77b4", "-", "P"), ("i_x", "#2ca02c", "-", "I"),
                ("d_x", "#d62728", "-", "D"),
                ("p_y", "#1f77b4", "--", None), ("i_y", "#2ca02c", "--", None),
                ("d_y", "#d62728", "--", None)):
            self.ax_pid.plot(times, d([r[key] for r in rows]), style,
                             color=color, alpha=0.8, linewidth=1.0, label=label)
        self.ax_pid.set_ylabel("PID terms (rad)")
        self.ax_pid.set_title("solid = X axis, dashed = Y", fontsize=8)
        self.ax_pid.legend(loc="upper left", fontsize=7, ncol=3)

        # The moving cursors, created once and thereafter only repositioned.
        # animated=True keeps them out of the cached background.
        t0 = self.rows[0]["t"]
        self._cursors = [
            (ax, ax.axvline(t0, color="k", alpha=alpha, animated=True))
            for ax, alpha in ((self.ax_pid, 0.4), (self.ax_alt, 0.4),
                              (self.ax_tilt, 0.4), (self.ax_state, 0.8))
        ]

        self.fig.tight_layout()   # once per flight, not once per frame (~31 ms)
        self._backgrounds = None
        self.canvas.draw_idle()

    def _on_mpl_draw(self, _event):
        """Re-cache the blit backgrounds after any full canvas draw.

        Covers the first paint, window resizes and anything else matplotlib
        redraws for its own reasons -- the classic way blitting breaks is
        caching a background once and never noticing the canvas changed
        underneath it.
        """
        try:
            self._backgrounds = [self.canvas.copy_from_bbox(ax.bbox)
                                 for ax, _line in self._cursors]
        except Exception:
            self._backgrounds = None

    def _update_cursor(self):
        """Move the four time cursors. ~1-3 ms via blitting, vs ~81 ms for a
        full canvas.draw()."""
        if not self._cursors:
            return
        t_now = self.scrub_var.get()
        for _ax, line in self._cursors:
            line.set_xdata([t_now, t_now])

        if self._backgrounds and len(self._backgrounds) == len(self._cursors):
            try:
                for (ax, line), bg in zip(self._cursors, self._backgrounds):
                    self.canvas.restore_region(bg)
                    ax.draw_artist(line)
                    self.canvas.blit(ax.bbox)
                return
            except Exception:
                # Backend declined to blit -- fall through to the slow path
                # rather than silently stop moving the cursor.
                self._backgrounds = None

        # Fallback: correctness without blitting, throttled so a backend that
        # can't blit degrades to a slow cursor rather than a frozen dashboard.
        now = time.perf_counter()
        if now - self._last_full_draw >= 0.1:
            self._last_full_draw = now
            self.canvas.draw_idle()


def main():
    root = tk.Tk()
    Dashboard(root)
    root.geometry("1400x850")
    root.mainloop()


if __name__ == "__main__":
    sys.exit(main() or 0)
