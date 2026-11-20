"""The dashboard's "HIL bench..." window: fly the real Teensy (HIL build, LIVE
mode) through a simulated flight, watch it live, then compare it with its
pure-PC twin.

The Teensy runs the real flight code; the PC only fakes the IMU and baro. The
gimbal and chute servos move as in flight (the simulated nozzle follows what
the gimbal servos are really sent), and each pyro channel is simulated unless
you opt it in as live.

The 500 Hz loop (tools/hil/live/runner.py) runs in its OWN process, so Tk and
matplotlib can never stall it: commands go in through cmd_q, and log lines,
console lines, flight events, status (with the servo and pyro state), a 100 Hz
copy of the rows, and the end of the flight come back through out_q, polled
here with after(). Spawn-safe (Windows): the child only imports runner.py.

    dashboard.py top bar -> "HIL bench..." -> HilPanel(dashboard)
"""
import dataclasses
import math
import multiprocessing as mp
import os
import queue
import sys
import threading
import time
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
RECONNECT_FIRST_MS = 4000   # after a reboot: USB re-enumerates in ~2-4 s
RECONNECT_RETRY_MS = 2500
RECONNECT_TRIES = 6
CONSOLE_MAX_LINES = 3000

STATE_COLORS = {
    "IDLE": "#cccccc", "ARMED": "#a3c9f7", "BOOST": "#ff8a3d",
    "COAST": "#ffd23d", "APOGEE": "#c77dff", "DESCENT": "#5fb3f5",
    "LANDING_BURN": "#ff5d5d", "DESCENT_CHUTE": "#57cc99",
    "TOUCHDOWN": "#2d6a4f", "ABORT": "#d00000",
}

# Scenario fields, grouped for the form: (group, [(field, label)])
FORM = [
    ("Flight", [("mode", "Mode"), ("seed", "Seed"),
                ("randomize", "Randomize (the sim's Monte Carlo spreads, from the seed)"),
                ("max_flight_s", "Max flight (s)")]),
    ("Legs", [("legs_cut_s", "Band cut time (s)")]),
    ("Wind (calm flights barely move the gimbal: add wind or a gust to see the TVC work)",
     [("wind_mps", "Wind (m/s)"), ("gust_mps", "Gust (m/s)"),
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

PYRO_OFF = "#d0d0d0"     # not firing
PYRO_SIM = "#ffb347"     # commanded by the flight code, channel simulated
PYRO_LIVE = "#e00000"    # really powered


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
    pyros = [n for n, on in (("LIVE LEGS", st.get("live_legs")),
                             ("LIVE LANDING", st.get("live_landing"))) if on]
    return ("link {:4.0f} Hz | {} | Teensy {}{} | gap max {:.1f} ms | stale max {} | "
            "batch max {} | crc {} | pyros {}".format(
                st.get("hz", 0.0), st.get("phase", "?"), st.get("state", "?"), t_rel,
                s.get("max_gap_ms", 0.0), s.get("max_stale", 0), s.get("max_batch", 0),
                s.get("crc_errors", 0), " + ".join(pyros) if pyros else "simulated"))


def pyro_color(cmd, pin):
    """Light colour for a pyro channel: really powered > commanded > off."""
    if pin:
        return PYRO_LIVE
    return PYRO_SIM if cmd else PYRO_OFF


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


def profile_legs_cut_s(default=0.75):
    """The band cut time the HIL build's profile flies with."""
    try:
        import flight
        return float((flight.load_profile().get("plant") or {}).get("legs_cut_s", default))
    except Exception:
        return default


def button_states(running, phase, state, busy):
    """Which buttons are usable now."""
    pad = phase in ("pad", "prepared")
    return {"connect": not running, "stop": running, "abort": running, "reboot": running,
            "run": running and not busy and phase in ("pad", "prepared", "armed"),
            "servo": running and not busy and pad and state == "IDLE",
            "prepare": running and not busy and pad,
            "arm": running and not busy and phase == "prepared",
            "launch": running and phase == "armed" and state == "ARMED", "view3d": True}


class ServoBars:
    """Live bars for the two gimbal servos (angle really sent, +-limit) and
    the chute servo (0-180 deg, LOCK/SET/OPEN marks)."""
    W, H = 400, 112
    X0, X1 = 78, 290

    def __init__(self, parent):
        c = self.c = tk.Canvas(parent, width=self.W, height=self.H, background="#fbfbfb",
                               highlightthickness=1, highlightbackground="#ccc")
        self.items = {}
        for i, (key, label) in enumerate((("a", "Gimbal A"), ("b", "Gimbal B"),
                                          ("chute", "Chute"))):
            y = 18 + i * 34
            c.create_text(6, y, text=label, anchor="w", font=("TkDefaultFont", 9, "bold"))
            c.create_rectangle(self.X0, y - 8, self.X1, y + 8, outline="#999", fill="#eee")
            if key == "chute":
                for deg, name in ((40, "L"), (120, "S"), (180, "O")):
                    x = self._chute_x(deg)
                    c.create_line(x, y - 11, x, y + 11, fill="#666")
                    c.create_text(x, y + 16, text=name, font=("TkDefaultFont", 7), fill="#555")
            else:
                xm = (self.X0 + self.X1) / 2
                c.create_line(xm, y - 11, xm, y + 11, fill="#666")
            bar = c.create_rectangle(0, y - 7, 0, y + 7, outline="", fill="#3a7bd5")
            txt = c.create_text(self.X1 + 6, y, text="-", anchor="w", font=("Consolas", 9))
            self.items[key] = (bar, txt, y)

    def _chute_x(self, deg):
        return self.X0 + (self.X1 - self.X0) * max(0.0, min(180.0, deg)) / 180.0

    def update(self, st):
        lim = max(st.get("gim_max_deg") or 1.0, 0.1)
        xm = (self.X0 + self.X1) / 2
        for key, deg_key, us_key in (("a", "wr_x_deg", "us_a"), ("b", "wr_y_deg", "us_b")):
            bar, txt, y = self.items[key]
            d = st.get(deg_key) or 0.0
            x = xm + (self.X1 - self.X0) / 2 * max(-1.0, min(1.0, d / lim))
            self.c.coords(bar, min(xm, x), y - 7, max(xm, x) + 1, y + 7)
            us = st.get(us_key)
            self.c.itemconfigure(txt, text="{:+5.2f}° {} µs".format(d, us if us else "-"))
        bar, txt, y = self.items["chute"]
        cd = st.get("chute_deg")
        if cd is not None:
            self.c.coords(bar, self.X0, y - 7, self._chute_x(cd), y + 7)
            self.c.itemconfigure(bar, fill="#2d9a4f" if st.get("chute") == "OPEN" else "#3a7bd5")
            self.c.itemconfigure(txt, text="{} {:.0f}° {} µs".format(
                st.get("chute") or "?", cd, st.get("us_chute") or "-"))


class PyroLights:
    """Landing e-match and leg nichrome: grey off, orange commanded with the
    channel simulated, red really powered; SIM / LIVE underneath."""
    W, H = 170, 112

    def __init__(self, parent):
        c = self.c = tk.Canvas(parent, width=self.W, height=self.H, background="#fbfbfb",
                               highlightthickness=1, highlightbackground="#ccc")
        self.items = {}
        for key, label, x in (("land", "Landing", 45), ("legs", "Legs", 125)):
            c.create_text(x, 12, text=label, font=("TkDefaultFont", 9, "bold"))
            lamp = c.create_oval(x - 18, 22, x + 18, 58, fill=PYRO_OFF, outline="#777", width=2)
            mode = c.create_text(x, 70, text="SIM", font=("TkDefaultFont", 9))
            self.items[key] = (lamp, mode)
        c.create_text(self.W / 2, 92, text="orange = commanded (sim)", font=("TkDefaultFont", 7))
        c.create_text(self.W / 2, 104, text="red = really powered", font=("TkDefaultFont", 7))

    def update(self, st):
        for key, cmd, pin, live in (("land", st.get("land_cmd"), st.get("land_pin"),
                                     st.get("live_landing")),
                                    ("legs", st.get("legs_cmd"), st.get("legs_pin"),
                                     st.get("live_legs"))):
            lamp, mode = self.items[key]
            self.c.itemconfigure(lamp, fill=pyro_color(cmd, pin),
                                 outline=PYRO_LIVE if live else "#777")
            self.c.itemconfigure(mode, text="LIVE" if live else "SIM",
                                 fill=PYRO_LIVE if live else "#333")


class HilPanel:
    def __init__(self, dash):
        self.dash = dash
        self.win = tk.Toplevel(dash.root)
        self.win.title("RocketFC HIL bench (real flight code, simulated sensors)")
        self.win.geometry("1420x900")
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
        self._phase = "idle"
        self._state = "?"
        self._busy = False
        self._new_session = False     # clear the plots/events at the next session's data
        self._reconnect = False       # a reboot was sent: reconnect when the runner closes
        self._reconnect_tries = 0
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

        setup = ttk.Notebook(left)
        setup.pack(fill=tk.X)
        self._build_scenario(setup)
        self._build_ports_pyros(setup)

        # buttons
        bf = ttk.Frame(left, padding=(0, 6))
        bf.pack(fill=tk.X)
        self.buttons = {}
        layout = (("connect", "Connect"), ("run", "RUN"), ("servo", "Servo check"),
                  ("abort", "ABORT"), ("prepare", "Prepare"), ("arm", "Arm"),
                  ("launch", "Launch"), ("reboot", "Reboot"), ("stop", "Stop"),
                  ("view3d", "3D view"))
        for i, (name, txt) in enumerate(layout):
            b = ttk.Button(bf, text=txt, width=11, command=lambda n=name: self.on_button(n))
            b.grid(row=i // 4, column=i % 4, padx=1, pady=1)
            self.buttons[name] = b

        # log / console
        nb = ttk.Notebook(left)
        nb.pack(fill=tk.BOTH, expand=True)
        lf = ttk.Frame(nb, padding=2)
        nb.add(lf, text="Log")
        self.logbox = tk.Text(lf, width=60, height=14, font=("Consolas", 9), wrap="word")
        sb = ttk.Scrollbar(lf, command=self.logbox.yview)
        self.logbox.configure(yscrollcommand=sb.set)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.logbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        for tag, col in (("ok", "#1b7f3b"), ("bad", "#c00000"), ("err", "#c00000")):
            self.logbox.tag_configure(tag, foreground=col)
        cf = ttk.Frame(nb, padding=2)
        nb.add(cf, text="Console (RocketFC CLI)")
        crow = ttk.Frame(cf)
        crow.pack(side=tk.BOTTOM, fill=tk.X)
        self.con_entry = tk.StringVar()
        ent = ttk.Entry(crow, textvariable=self.con_entry)
        ent.pack(side=tk.LEFT, fill=tk.X, expand=True)
        ent.bind("<Return>", lambda e: self.send_console())
        ttk.Button(crow, text="Send", command=self.send_console).pack(side=tk.LEFT)
        self.conbox = tk.Text(cf, width=60, height=14, font=("Consolas", 9), wrap="none",
                              background="#111", foreground="#ddd")
        csb = ttk.Scrollbar(cf, command=self.conbox.yview)
        self.conbox.configure(yscrollcommand=csb.set)
        csb.pack(side=tk.RIGHT, fill=tk.Y)
        self.conbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.conbox.tag_configure("me", foreground="#7fd4ff")

        # right: status, live panel, plots, twin table
        self.status = ttk.Label(right, text="not connected", font=("Consolas", 10))
        self.status.pack(fill=tk.X)
        live = ttk.Frame(right)
        live.pack(fill=tk.X, pady=2)
        sf = ttk.LabelFrame(live, text="Servos (what the Teensy sent)", padding=2)
        sf.pack(side=tk.LEFT)
        self.servos = ServoBars(sf)
        self.servos.c.pack()
        pf = ttk.LabelFrame(live, text="Pyro channels", padding=2)
        pf.pack(side=tk.LEFT, padx=4)
        self.pyros = PyroLights(pf)
        self.pyros.c.pack()
        ef = ttk.LabelFrame(live, text="Flight events (s after launch)", padding=2)
        ef.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.events = tk.Listbox(ef, height=6, font=("Consolas", 9))
        self.events.pack(fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(8, 5.4), dpi=90)
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
        self.ln["gx"], = self.ax_g.plot([], [], label="X sent to servo", color="C0", lw=1)
        self.ln["gy"], = self.ax_g.plot([], [], label="Y sent to servo", color="C3", lw=1)
        self.ax_h.set_ylabel("altitude (m)")
        self.ax_v.set_ylabel("vert. speed\n(m/s)")
        self.ax_t.set_ylabel("tilt (deg)")
        self.ax_g.set_ylabel("gimbal (deg)")
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
        self.table = ttk.Treeview(tf, columns=("bench", "twin", "diff", "ok"), height=7)
        self.table.heading("#0", text="item")
        self.table.column("#0", width=200)
        for c, wdt in (("bench", 220), ("twin", 220), ("diff", 70), ("ok", 90)):
            self.table.heading(c, text=c)
            self.table.column(c, width=wdt, anchor="e")
        self.table.tag_configure("ok", foreground="#1b7f3b")
        self.table.tag_configure("bad", foreground="#c00000")
        self.table.pack(fill=tk.X)
        self._update_buttons()

    def _build_scenario(self, nb):
        sf = ttk.Frame(nb, padding=4)
        nb.add(sf, text="Scenario (times: s after Launch)")
        d = Scenario()
        defaults = {"legs_cut_s": profile_legs_cut_s()}
        self.vars = {}
        r = 0
        for group, items in FORM:
            ttk.Label(sf, text=group, font=("TkDefaultFont", 9, "bold"), wraplength=380).grid(
                row=r, column=0, columnspan=5, sticky="w", pady=(4, 0))
            r += 1
            for name, label in items:
                if hasattr(d, name + "_at_s"):          # a fault window
                    ttk.Label(sf, text=label).grid(row=r, column=0, sticky="w")
                    for col, suffix in ((1, "_at_s"), (2, "_s")):
                        v = tk.StringVar(value="{:g}".format(getattr(d, name + suffix)))
                        self.vars[name + suffix] = v
                        ttk.Entry(sf, textvariable=v, width=5).grid(row=r, column=col)
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
                        row=r, column=0, columnspan=5, sticky="w")
                elif name == "mode":
                    v = tk.StringVar(value=d.mode)
                    self.vars[name] = v
                    ttk.Label(sf, text=label).grid(row=r, column=0, sticky="w")
                    ttk.Combobox(sf, textvariable=v, values=["chute", "land"], width=6,
                                 state="readonly").grid(row=r, column=1, columnspan=2, sticky="w")
                else:
                    v = tk.StringVar(value="{:g}".format(defaults.get(name, getattr(d, name))))
                    self.vars[name] = v
                    ttk.Label(sf, text=label).grid(row=r, column=0, sticky="w")
                    ttk.Entry(sf, textvariable=v, width=7).grid(row=r, column=1, columnspan=2,
                                                               sticky="w")
                r += 1
        ttk.Label(sf, text="            at      for     size").grid(row=r, column=0,
                                                                  columnspan=4, sticky="w")

    def _build_ports_pyros(self, nb):
        tab = ttk.Frame(nb, padding=4)
        nb.add(tab, text="Pyros, run, ports")
        pyf = ttk.LabelFrame(tab, text="Pyro channels (default: simulated, pins never driven)",
                             padding=4)
        pyf.pack(fill=tk.X)
        self.live_legs = tk.BooleanVar(value=False)
        self.vars["live_legs"] = self.live_legs
        ttk.Checkbutton(pyf, text="Live legs: really power the leg nichrome (type LEGS)",
                        variable=self.live_legs, command=self._on_live_legs).pack(anchor="w")
        self.live_landing = tk.BooleanVar(value=False)
        self.vars["live_landing"] = self.live_landing
        ttk.Checkbutton(pyf, text="Live landing: really fire the landing e-match channel (type FIRE)",
                        variable=self.live_landing, command=self._on_live_landing).pack(anchor="w")
        ttk.Label(pyf, wraplength=380, foreground="#555",
                  text="A live channel behaves exactly as in flight, including its real "
                       "continuity and pyro-battery checks at Arm. The simulated landing motor "
                       "still lights at the fire command + the ignition delay.").pack(anchor="w")

        rf = ttk.LabelFrame(tab, text="Run", padding=4)
        rf.pack(fill=tk.X, pady=4)
        row = ttk.Frame(rf)
        row.pack(fill=tk.X)
        ttk.Label(row, text="Countdown (s)").pack(side=tk.LEFT)
        self.countdown = tk.StringVar(value="10")
        ttk.Entry(row, textvariable=self.countdown, width=5).pack(side=tk.LEFT)
        self.auto_reboot = tk.BooleanVar(value=True)
        ttk.Checkbutton(rf, text="After each run: reboot the Teensy and reconnect "
                                 "(the chute servo stays where it is)",
                        variable=self.auto_reboot).pack(anchor="w")

        pf = ttk.LabelFrame(tab, text="Ports", padding=4)
        pf.pack(fill=tk.X)
        self.port_kind = tk.StringVar(value="auto")
        for v, txt in (("auto", "Auto (find the Teensy)"), ("com", "COM ports"),
                       ("socket", "PC build (socket, no Teensy)")):
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

    def log(self, text, tag=None):
        self.logbox.insert(tk.END, text + "\n", (tag,) if tag else ())
        self.logbox.see(tk.END)

    def console_line(self, text, tag=None):
        self.conbox.insert(tk.END, text + "\n", (tag,) if tag else ())
        n = int(self.conbox.index("end-1c").split(".")[0])
        if n > CONSOLE_MAX_LINES:
            self.conbox.delete("1.0", "{}.0".format(n - CONSOLE_MAX_LINES))
        self.conbox.see(tk.END)

    def _confirm(self, var, title, text, word):
        if not var.get():
            return
        ans = simpledialog.askstring(title, text + "\n\nType {} to continue:".format(word),
                                     parent=self.win)
        if (ans or "").strip() != word:
            var.set(False)
            self.log("{} not confirmed: the channel stays simulated".format(title.lower()))
        elif self.proc is not None:
            self.log("{}: takes effect at the next Connect".format(title), "bad")

    def _on_live_legs(self):
        self._confirm(self.live_legs, "Live legs",
                      "LIVE LEGS: the leg nichrome WILL be powered when the flight computer\n"
                      "commands it (pyro LiPo + band on the bench, clear of anything).", "LEGS")

    def _on_live_landing(self):
        self._confirm(self.live_landing, "Live landing",
                      "LIVE LANDING: the landing e-match channel WILL fire at the flight\n"
                      "computer's FIRE_LANDING -- whatever is connected: an e-match in a safe\n"
                      "holder, or a landing motor on a static-fire stand. Pyro LiPo plugged in,\n"
                      "everyone clear.", "FIRE")

    def _update_buttons(self):
        running = self.proc is not None and self.proc.is_alive()
        en = button_states(running, self._phase, self._state, self._busy)
        for k, b in self.buttons.items():
            b.state(["!disabled"] if en.get(k) else ["disabled"])

    # -------------------------------------------------------- actions
    def on_button(self, name):
        if name == "connect":
            self._reconnect = False
            return self.connect()
        if name == "view3d":
            return self.open_3d()
        if self.cmd_q is None:
            return
        if name == "abort":
            self.log("ABORT requested", "err")
        if name == "servo":
            name = "servo_check"
        if name == "stop":
            self._reconnect = False
        self.cmd_q.put(name)

    def send_console(self):
        text = self.con_entry.get().strip()
        if not text:
            return
        if self.cmd_q is None or self.proc is None:
            self.console_line("(not connected)", "me")
            return
        self.console_line("> " + text, "me")
        self.cmd_q.put(("console", text))
        self.con_entry.set("")

    def connect(self):
        try:
            sc = scenario_from_vars(self.vars)
            hil, con = port_specs(self.port_kind.get(), self.com_hil.get(),
                                  self.com_con.get(), self.sock_port.get())
            countdown = float(self.countdown.get())
        except ValueError as e:
            messagebox.showerror("HIL bench", str(e), parent=self.win)
            return
        self._new_session = True
        self.cmd_q, self.out_q = self.ctx.Queue(), self.ctx.Queue()
        opts = {"countdown_s": max(countdown, 0.0),
                "auto_reboot": bool(self.auto_reboot.get())}
        self.proc = self.ctx.Process(target=runner_mod.runner_process,
                                     args=(self.cmd_q, self.out_q, hil, con, sc.to_dict()),
                                     kwargs={"opts": opts}, daemon=True)
        self.proc.start()
        self._phase = "connecting"
        self.log("connecting ({}) | scenario: {} | mode {}".format(
            "auto" if hil is None else hil, ", ".join(sc.faults()), sc.mode))
        self._update_buttons()

    def _auto_reconnect(self):
        if self._closing or self.proc is not None or not self._reconnect:
            return
        self._reconnect_tries += 1
        self.log("reconnecting after the reboot (try {} of {}) ...".format(
            self._reconnect_tries, RECONNECT_TRIES))
        self.connect()

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
            for _ in range(400):
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

    def _start_new_session_view(self):
        """A new session's first data: clear the last flight's plots and events."""
        if not self._new_session:
            return
        self._new_session = False
        self.rows = []
        self.launch_t = None
        self.session_dir = None
        self._clear_spans()
        self.events.delete(0, tk.END)

    def _handle(self, kind, val):
        if kind == "log":
            self.log(val, "err" if val.startswith("ERROR") else None)
        elif kind == "console":
            self.console_line(val)
        elif kind == "event":
            self._start_new_session_view()
            t = val.get("t")
            self.events.insert(tk.END, "{:>8}  {} {:g}".format(
                "{:+.2f}".format(t) if t is not None else "pad", val["name"], val["value"]))
            self.events.see(tk.END)
        elif kind == "error":
            self.log("ERROR: " + val, "err")
        elif kind == "status":
            self.status.config(text=status_text(val))
            if val.get("launch_t") is not None:
                self.launch_t = val["launch_t"]
            self._phase, self._state = val.get("phase", "?"), val.get("state", "?")
            self._busy = bool(val.get("busy"))
            self.servos.update(val)
            self.pyros.update(val)
            self._reconnect_tries = 0
            self._update_buttons()
        elif kind == "rows":
            self._start_new_session_view()
            self.rows.extend(val)
            self._dirty = True
        elif kind == "finished":
            self.session_dir = val
            self._load_result(val)
        elif kind == "rebooting":
            self._reconnect = self.port_kind.get() != "socket"
            if not self._reconnect:
                self.log("PC build: host_live exits on reboot; start it again, then Connect")
        elif kind == "closed":
            self.log("runner closed")
            self.proc = None
            self._phase = "idle"
            self._update_buttons()
            if self._reconnect and self._reconnect_tries < RECONNECT_TRIES:
                self.win.after(RECONNECT_FIRST_MS if self._reconnect_tries == 0
                               else RECONNECT_RETRY_MS, self._auto_reconnect)
            elif self._reconnect:
                self._reconnect = False
                self.log("couldn't reconnect: check the USB cable, then Connect", "err")

    def _t0(self):
        return self.launch_t if self.launch_t is not None else (self.rows[0]["t"] if self.rows else 0.0)

    def _plot_tick(self):
        """Redraw the live plots. A full redraw of the five axes costs
        ~0.1-0.3 s, so the next one waits at least 3x as long as this one
        took: the window stays responsive (servo bars, lights, buttons) and
        the plots still refresh a few times a second."""
        if self._closing:
            return
        t_start = time.perf_counter()
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
            self.ln["gx"].set_data(ts, col("gimbal_x_wr", math.degrees))
            self.ln["gy"].set_data(ts, col("gimbal_y_wr", math.degrees))
            for ax in (self.ax_h, self.ax_v, self.ax_t, self.ax_g):
                ax.relim()
                ax.autoscale_view()
            self._draw_spans(ts)
            self.ax_s.set_xlim(ts[0], max(ts[-1], ts[0] + 1.0))
            self.canvas.draw()
        spent_ms = 1000.0 * (time.perf_counter() - t_start)
        self.win.after(int(max(PLOT_MS, 3.0 * spent_ms)), self._plot_tick)

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
