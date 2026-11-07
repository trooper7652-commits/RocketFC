"""The dashboard's "HIL bench..." window: fly the real Teensy (HIL build, LIVE
mode) through a simulated flight, watch it live, then compare it with its
pure-PC twin.

The 500 Hz loop (tools/hil/live/runner.py) runs in its OWN process, so Tk and
matplotlib can never stall it: commands go in through cmd_q, and log lines,
status, a 100 Hz copy of the rows, and the end of the flight come back
through out_q, polled here with after(). Spawn-safe (Windows): the child only
imports runner.py.

    dashboard.py top bar -> "HIL bench..." -> HilPanel(dashboard)
"""
import dataclasses
import math
import multiprocessing as mp
import os
import queue
import sys
import threading
import tkinter as tk
from tkinter import messagebox, simpledialog, ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
LIVE_DIR = os.path.join(os.path.dirname(SIM_DIR), "hil", "live")
if LIVE_DIR not in sys.path:
    sys.path.insert(0, LIVE_DIR)

import flight3d  # noqa: E402
import runner as runner_mod  # noqa: E402
from scenario import Scenario  # noqa: E402

PLOT_MS = 100        # live plot refresh (~10 Hz)
TRACK_MS = 250       # live 3D track rebuild (~4 Hz)
POLL_MS = 30         # out_q poll

STATE_COLORS = {
    "IDLE": "#cccccc", "ARMED": "#a3c9f7", "BOOST": "#ff8a3d",
    "COAST": "#ffd23d", "APOGEE": "#c77dff", "DESCENT": "#5fb3f5",
    "LANDING_BURN": "#ff5d5d", "DESCENT_CHUTE": "#57cc99",
    "TOUCHDOWN": "#2d6a4f", "ABORT": "#d00000",
}

# Scenario fields, grouped for the form: (group, [(field, label)])
FORM = [
    ("Flight", [("mode", "Mode"), ("seed", "Seed"), ("max_flight_s", "Max flight (s)")]),
    ("Wind", [("wind_mps", "Wind (m/s)"), ("gust_mps", "Gust (m/s)"),
              ("gust_at_s", "Gust at (s)"), ("gust_s", "Gust for (s)")]),
    ("Motors", [("dud_landing_igniter", "Dud landing igniter"),
                ("landing_thrust_pct", "Landing thrust (%)"),
                ("ignition_delay_s", "Ignition delay (s, <0 default)")]),
    ("Chute", [("chute_stuck_releases", "Stuck releases")]),
    ("Sensor faults (at s after launch, for s; 0 = off)", [
        ("imu_dropout", "IMU dropout"), ("imu_frozen", "IMU frozen"),
        ("baro_offset", "Baro offset"), ("baro_noise", "Baro noise"),
        ("baro_dropout", "Baro dropout")]),
]
# Faults with a size as well as a window
FAULT_SIZE = {"baro_offset": ("baro_offset_m", "m"), "baro_noise": ("baro_noise_m", "std m")}


def scenario_from_vars(vars_):
    """{field: tk var-like with .get()} -> Scenario (raises ValueError)."""
    sc = Scenario()
    for f in dataclasses.fields(Scenario):
        if f.name not in vars_:
            continue
        raw = vars_[f.name].get()
        cur = getattr(sc, f.name)
        if isinstance(cur, bool):
            val = bool(raw)
        elif isinstance(cur, int):
            val = int(float(raw))
        elif isinstance(cur, float):
            val = float(raw)
        else:
            val = str(raw)
        setattr(sc, f.name, val)
    if sc.mode not in ("chute", "land"):
        raise ValueError("mode must be chute or land")
    return sc


def port_specs(kind, com_hil, com_con, sock_port):
    """(hil, console) for runner_process: None/None = find them."""
    if kind == "auto":
        return None, None
    if kind == "com":
        return com_hil.strip() or None, com_con.strip() or None
    p = int(sock_port)
    return "socket://127.0.0.1:{}".format(p + 1), "socket://127.0.0.1:{}".format(p)


def status_text(st):
    s = st.get("stats", {})
    t_rel = ""
    if st.get("launch_t") is not None:
        t_rel = "  T{:+.2f} s".format(st["t"] - st["launch_t"])
    return ("link {:4.0f} Hz | {} | Teensy {}{} | gap max {:.1f} ms | stale max {} | "
            "batch max {} | crc {}{}".format(
                st.get("hz", 0.0), st.get("phase", "?"), st.get("state", "?"), t_rel,
                s.get("max_gap_ms", 0.0), s.get("max_stale", 0), s.get("max_batch", 0),
                s.get("crc_errors", 0), " | LIVE LEGS" if st.get("legs") else ""))


def state_spans(ts, states):
    """[(t0, t1, state)] runs of the same state."""
    out = []
    for t, s in zip(ts, states):
        if out and out[-1][2] == s:
            out[-1][1] = t
        else:
            if out:
                out[-1][1] = t
            out.append([t, t, s])
    return [tuple(x) for x in out]


class HilPanel:
    def __init__(self, dash):
        self.dash = dash
        self.win = tk.Toplevel(dash.root)
        self.win.title("RocketFC HIL bench (closed loop)")
        self.win.geometry("1250x820")
        self.win.protocol("WM_DELETE_WINDOW", self.on_close)
        self.ctx = mp.get_context("spawn")
        self.proc = None
        self.cmd_q = self.out_q = None
        self.rows = []
        self.launch_t = None
        self._dirty = False
        self._spans = []
        self._twin_q = queue.Queue()
        self._closing = False
        self.session_dir = None
        self._build()
        self._poll_job = self.win.after(POLL_MS, self._poll)
        self.win.after(PLOT_MS, self._plot_tick)
        self.win.after(TRACK_MS, self._track_tick)

    # ------------------------------------------------------------- UI
    def _build(self):
        w = self.win
        left = ttk.Frame(w, padding=4)
        left.pack(side=tk.LEFT, fill=tk.Y)
        right = ttk.Frame(w, padding=4)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # ports
        pf = ttk.LabelFrame(left, text="Ports", padding=4)
        pf.pack(fill=tk.X)
        self.port_kind = tk.StringVar(value="auto")
        for v, txt in (("auto", "Auto (find the Teensy)"), ("com", "COM ports"),
                       ("socket", "PC build (socket)")):
            ttk.Radiobutton(pf, text=txt, variable=self.port_kind, value=v).pack(anchor="w")
        row = ttk.Frame(pf)
        row.pack(fill=tk.X)
        ttk.Label(row, text="HIL").pack(side=tk.LEFT)
        self.com_hil = tk.StringVar()
        ttk.Entry(row, textvariable=self.com_hil, width=7).pack(side=tk.LEFT)
        ttk.Label(row, text=" console").pack(side=tk.LEFT)
        self.com_con = tk.StringVar()
        ttk.Entry(row, textvariable=self.com_con, width=7).pack(side=tk.LEFT)
        row = ttk.Frame(pf)
        row.pack(fill=tk.X)
        ttk.Label(row, text="socket port (console; HIL = +1)").pack(side=tk.LEFT)
        self.sock_port = tk.StringVar(value="7700")
        ttk.Entry(row, textvariable=self.sock_port, width=6).pack(side=tk.LEFT)

        # scenario
        sf = ttk.LabelFrame(left, text="Scenario (times: s after Launch)", padding=4)
        sf.pack(fill=tk.X, pady=4)
        d = Scenario()
        self.vars = {}
        r = 0
        for group, items in FORM:
            ttk.Label(sf, text=group, font=("TkDefaultFont", 9, "bold")).grid(
                row=r, column=0, columnspan=4, sticky="w", pady=(4, 0))
            r += 1
            for name, label in items:
                if hasattr(d, name + "_at_s"):          # a fault window
                    ttk.Label(sf, text=label).grid(row=r, column=0, sticky="w")
                    for col, suffix, width in ((1, "_at_s", 5), (2, "_s", 5)):
                        v = tk.StringVar(value="{:g}".format(getattr(d, name + suffix)))
                        self.vars[name + suffix] = v
                        ttk.Entry(sf, textvariable=v, width=width).grid(row=r, column=col)
                    if name in FAULT_SIZE:
                        fld, unit = FAULT_SIZE[name]
                        v = tk.StringVar(value="{:g}".format(getattr(d, fld)))
                        self.vars[fld] = v
                        ttk.Entry(sf, textvariable=v, width=5).grid(row=r, column=3)
                        ttk.Label(sf, text=unit).grid(row=r, column=4, sticky="w")
                elif isinstance(getattr(d, name), bool):
                    v = tk.BooleanVar(value=getattr(d, name))
                    self.vars[name] = v
                    ttk.Checkbutton(sf, text=label, variable=v).grid(
                        row=r, column=0, columnspan=3, sticky="w")
                elif name == "mode":
                    v = tk.StringVar(value=d.mode)
                    self.vars[name] = v
                    ttk.Label(sf, text=label).grid(row=r, column=0, sticky="w")
                    ttk.Combobox(sf, textvariable=v, values=["chute", "land"], width=6,
                                 state="readonly").grid(row=r, column=1, columnspan=2, sticky="w")
                else:
                    v = tk.StringVar(value="{:g}".format(getattr(d, name)))
                    self.vars[name] = v
                    ttk.Label(sf, text=label).grid(row=r, column=0, sticky="w")
                    ttk.Entry(sf, textvariable=v, width=7).grid(row=r, column=1, columnspan=2,
                                                               sticky="w")
                r += 1
        ttk.Label(sf, text="            at      for     size").grid(row=r, column=0,
                                                                  columnspan=4, sticky="w")
        self.live_legs = tk.BooleanVar(value=False)
        self.vars["live_legs"] = self.live_legs
        ttk.Checkbutton(left, text="Live legs (really power the leg nichrome)",
                        variable=self.live_legs, command=self._on_live_legs).pack(anchor="w")

        # buttons
        bf = ttk.Frame(left, padding=(0, 6))
        bf.pack(fill=tk.X)
        self.buttons = {}
        for i, (name, txt) in enumerate((("connect", "Connect"), ("prepare", "Prepare"),
                                         ("arm", "Arm"), ("launch", "Launch"),
                                         ("abort", "Abort"), ("reboot", "Reboot"),
                                         ("stop", "Stop"), ("view3d", "3D view"))):
            b = ttk.Button(bf, text=txt, width=9, command=lambda n=name: self.on_button(n))
            b.grid(row=i // 4, column=i % 4, padx=1, pady=1)
            self.buttons[name] = b

        # log
        lf = ttk.LabelFrame(left, text="Log", padding=2)
        lf.pack(fill=tk.BOTH, expand=True)
        self.logbox = tk.Text(lf, width=52, height=12, font=("Consolas", 9), wrap="word")
        sb = ttk.Scrollbar(lf, command=self.logbox.yview)
        self.logbox.configure(yscrollcommand=sb.set)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.logbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.logbox.tag_configure("ok", foreground="#1b7f3b")
        self.logbox.tag_configure("bad", foreground="#c00000")
        self.logbox.tag_configure("err", foreground="#c00000")

        # status + plots + twin table
        self.status = ttk.Label(right, text="not connected", font=("Consolas", 10))
        self.status.pack(fill=tk.X)
        self.fig = Figure(figsize=(8, 6), dpi=90)
        gs = self.fig.add_gridspec(5, 1, height_ratios=[3, 2, 2, 2, 0.4], hspace=0.35)
        self.ax_h = self.fig.add_subplot(gs[0])
        self.ax_v = self.fig.add_subplot(gs[1], sharex=self.ax_h)
        self.ax_t = self.fig.add_subplot(gs[2], sharex=self.ax_h)
        self.ax_g = self.fig.add_subplot(gs[3], sharex=self.ax_h)
        self.ax_s = self.fig.add_subplot(gs[4], sharex=self.ax_h)
        self.ln = {}
        self.ln["h_true"], = self.ax_h.plot([], [], label="truth (plant)", color="k", lw=1.2)
        self.ln["kf_alt"], = self.ax_h.plot([], [], label="Teensy estimate (KF)", color="C1", lw=1)
        self.ln["v_true"], = self.ax_v.plot([], [], label="truth", color="k", lw=1.2)
        self.ln["kf_vel"], = self.ax_v.plot([], [], label="KF", color="C1", lw=1)
        self.ln["tilt"], = self.ax_t.plot([], [], color="C2", lw=1)
        self.ln["gx"], = self.ax_g.plot([], [], label="X", color="C0", lw=1)
        self.ln["gy"], = self.ax_g.plot([], [], label="Y", color="C3", lw=1)
        self.ax_h.set_ylabel("altitude (m)")
        self.ax_v.set_ylabel("vert. speed\n(m/s)")
        self.ax_t.set_ylabel("tilt (deg)")
        self.ax_g.set_ylabel("gimbal\ncmd (deg)")
        self.ax_s.set_yticks([])
        self.ax_s.set_xlabel("s after Launch")
        for ax in (self.ax_h, self.ax_v, self.ax_g):
            ax.legend(loc="upper right", fontsize=7)
        for ax in (self.ax_h, self.ax_v, self.ax_t, self.ax_g):
            ax.grid(alpha=0.3)
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        tf = ttk.LabelFrame(right, text="Twin comparison (bench vs the same flight on the PC)",
                            padding=2)
        tf.pack(fill=tk.X)
        self.table = ttk.Treeview(tf, columns=("bench", "twin", "diff", "ok"), height=8)
        self.table.heading("#0", text="item")
        self.table.column("#0", width=200)
        for c, wdt in (("bench", 220), ("twin", 220), ("diff", 70), ("ok", 90)):
            self.table.heading(c, text=c)
            self.table.column(c, width=wdt, anchor="e")
        self.table.tag_configure("ok", foreground="#1b7f3b")
        self.table.tag_configure("bad", foreground="#c00000")
        self.table.pack(fill=tk.X)
        self._update_buttons("idle")

    def log(self, text, tag=None):
        self.logbox.insert(tk.END, text + "\n", (tag,) if tag else ())
        self.logbox.see(tk.END)

    def _on_live_legs(self):
        if not self.live_legs.get():
            return
        ans = simpledialog.askstring(
            "Live legs",
            "LIVE LEGS: the leg nichrome WILL be powered when the flight computer\n"
            "commands it (pyro LiPo + band on the bench, clear of anything).\n"
            "The landing e-match channel is never driven.\n\nType LEGS to continue:",
            parent=self.win)
        if (ans or "").strip() != "LEGS":
            self.live_legs.set(False)
            self.log("live legs not confirmed: legs stay simulated")

    def _update_buttons(self, phase):
        running = self.proc is not None and self.proc.is_alive()
        en = {"connect": not running, "stop": running, "abort": running, "reboot": running,
              "prepare": running and phase in ("pad", "prepared"),
              "arm": running and phase == "prepared",
              "launch": running and phase == "armed", "view3d": True}
        for k, b in self.buttons.items():
            b.state(["!disabled"] if en.get(k) else ["disabled"])

    # -------------------------------------------------------- actions
    def on_button(self, name):
        if name == "connect":
            return self.connect()
        if name == "view3d":
            return self.open_3d()
        if self.cmd_q is None:
            return
        if name == "abort":
            self.log("ABORT requested", "err")
        self.cmd_q.put(name)

    def connect(self):
        try:
            sc = scenario_from_vars(self.vars)
            hil, con = port_specs(self.port_kind.get(), self.com_hil.get(),
                                  self.com_con.get(), self.sock_port.get())
        except ValueError as e:
            messagebox.showerror("HIL bench", str(e), parent=self.win)
            return
        self.rows = []
        self.launch_t = None
        self._clear_spans()
        for item in self.table.get_children():
            self.table.delete(item)
        self.cmd_q, self.out_q = self.ctx.Queue(), self.ctx.Queue()
        self.proc = self.ctx.Process(target=runner_mod.runner_process,
                                     args=(self.cmd_q, self.out_q, hil, con, sc.to_dict()),
                                     daemon=True)
        self.proc.start()
        self.log("connecting ({}) | scenario: {} | mode {}".format(
            "auto" if hil is None else hil, ", ".join(sc.faults()), sc.mode))
        self._update_buttons("connecting")

    def open_3d(self):
        if self.dash.view3d is not None and not self.dash.view3d.closed:
            return
        trk = self._live_track()
        if trk is None and not self.dash.rows:
            messagebox.showinfo("3D view", "Connect first (or run a flight).", parent=self.win)
            return
        self.dash.on_open_3d(track=trk)

    def on_close(self):
        self._closing = True
        if self.proc is not None and self.proc.is_alive():
            self.cmd_q.put("stop")
            self.proc.join(timeout=3.0)
            if self.proc.is_alive():
                self.proc.terminate()
        self.win.destroy()

    # -------------------------------------------------------- polling
    def _poll(self):
        if self._closing:
            return
        self._poll_job = None
        try:
            for _ in range(200):
                kind, val = self.out_q.get_nowait() if self.out_q else (None, None)
                if kind is None:
                    break
                self._handle(kind, val)
        except queue.Empty:
            pass
        except (EOFError, OSError):
            pass
        try:
            while True:
                self._on_twin(*self._twin_q.get_nowait())
        except queue.Empty:
            pass
        self._poll_job = self.win.after(POLL_MS, self._poll)

    def _handle(self, kind, val):
        if kind == "log":
            self.log(val, "err" if val.startswith("ERROR") else None)
        elif kind == "error":
            self.log("ERROR: " + val, "err")
        elif kind == "status":
            self.status.config(text=status_text(val))
            if val.get("launch_t") is not None:
                self.launch_t = val["launch_t"]
            self._update_buttons(val.get("phase"))
        elif kind == "rows":
            self.rows.extend(val)
            self._dirty = True
        elif kind == "finished":
            self.session_dir = val
            self._load_result(val)
        elif kind == "closed":
            self.log("runner closed")
            self._update_buttons("idle")
            self.proc = None

    def _t0(self):
        return self.launch_t if self.launch_t is not None else (self.rows[0]["t"] if self.rows else 0.0)

    def _plot_tick(self):
        if self._closing:
            return
        if self._dirty and self.rows:
            self._dirty = False
            t0 = self._t0()
            ts = [r["t"] - t0 for r in self.rows]

            def col(k, f=None):
                return [(f(r[k]) if f else r[k]) if r.get(k) is not None else math.nan
                        for r in self.rows]
            self.ln["h_true"].set_data(ts, col("h_true"))
            self.ln["kf_alt"].set_data(ts, col("kf_alt"))
            self.ln["v_true"].set_data(ts, col("v_true"))
            self.ln["kf_vel"].set_data(ts, col("kf_vel"))
            self.ln["tilt"].set_data(ts, col("tilt_deg"))
            self.ln["gx"].set_data(ts, col("gimbal_x_cmd", math.degrees))
            self.ln["gy"].set_data(ts, col("gimbal_y_cmd", math.degrees))
            for ax in (self.ax_h, self.ax_v, self.ax_t, self.ax_g):
                ax.relim()
                ax.autoscale_view()
            self._draw_spans(ts)
            self.ax_s.set_xlim(ts[0], max(ts[-1], ts[0] + 1.0))
            self.canvas.draw_idle()
        self.win.after(PLOT_MS, self._plot_tick)

    def _clear_spans(self):
        for p in list(self.ax_s.patches):
            p.remove()
        self._spans = []

    def _draw_spans(self, ts):
        spans = state_spans(ts, [r["state"] for r in self.rows])
        # redraw only the last (growing) span and any new ones
        have = getattr(self, "_spans", [])
        keep = max(0, len(have) - 1)
        for p in have[keep:]:
            p.remove()
        have = have[:keep]
        for t0, t1, s in spans[keep:]:
            have.append(self.ax_s.axvspan(t0, max(t1, t0 + 0.002), color=STATE_COLORS.get(s, "#888")))
        self._spans = have

    def _live_track(self):
        if len(self.rows) < 2:
            return None
        vc = self.dash._read_vcfg()
        return flight3d.Track(self.rows, [], is_log=False, label="HIL bench (live)",
                              legs_cut_s=vc.legs_cut_s, legs_swing_s=vc.legs_swing_s,
                              chute_inflation_s=vc.chute_inflation_s)

    def _track_tick(self):
        if self._closing:
            return
        view = self.dash.view3d
        if (view is not None and not view.closed and self.proc is not None
                and len(self.rows) >= 2 and self.session_dir is None):
            try:
                trk = self._live_track()
                view.set_track(trk, follow_end=True)
                self.dash.show_live_time(trk.t_max)
            except Exception as e:  # never let the 3D view stop the bench
                self.log("3D: " + str(e))
        self.win.after(TRACK_MS, self._track_tick)

    # ------------------------------------------------------- results
    def _load_result(self, session_dir):
        try:
            res = runner_mod.read_result(os.path.join(session_dir, "result.json"))
        except Exception as e:
            self.log("couldn't read the result: {}".format(e), "err")
            return
        d = self.dash
        d.result, d.rows, d.events, d.is_log = res, res["rows"], res["events"], False
        d._after_new_data()
        d.status.config(text="HIL bench flight: " + os.path.basename(session_dir))
        self.log("flight saved: " + session_dir)
        self.log("flying the twin (the same flight on the PC) ...")

        def work():
            try:
                _, rows, text = runner_mod.run_twin(session_dir)
                self._twin_q.put((session_dir, rows, text, None))
            except Exception as e:  # pragma: no cover - shown in the log
                self._twin_q.put((session_dir, None, None, str(e)))
        threading.Thread(target=work, daemon=True).start()

    def _on_twin(self, session_dir, rows, text, err):
        if err:
            self.log("twin failed: " + err, "err")
            return
        fill_table(self.table, rows)
        n_bad = sum(1 for r in rows if not r[5])
        if n_bad:
            self.log("twin DIFFERS on {} of {} checks (see the table; {}/compare.txt)".format(
                n_bad, len(rows), session_dir), "bad")
        else:
            self.log("twin agrees: {} of {} checks".format(len(rows), len(rows)), "ok")


def _fmt(v):
    if v is None:
        return "-"
    if isinstance(v, float):
        return "nan" if math.isnan(v) else "{:.3f}".format(v)
    return str(v)


def table_rows(rows):
    """twin.compare() rows -> [(item, bench, twin, diff, verdict, tag)]."""
    out = []
    for item, a, b, d, _tol, ok in rows:
        out.append((item, _fmt(a), _fmt(b), _fmt(d) if d is not None else "",
                    "ok" if ok else "DIFFERS", "ok" if ok else "bad"))
    return out


def fill_table(tree, rows):
    for item in tree.get_children():
        tree.delete(item)
    for item, a, b, d, verdict, tag in table_rows(rows):
        tree.insert("", tk.END, text=item, values=(a, b, d, verdict), tags=(tag,))
