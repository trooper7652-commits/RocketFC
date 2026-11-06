#!/usr/bin/env python3
"""
tvc_pid_tuner.py
================
Single-axis attitude-control simulator + automatic PID gain finder for a
thrust-vectoring (gimbaled-motor) model rocket.

What it does
------------
1. Models the rocket's rotational dynamics about its CG for ONE axis (pitch or
   yaw -- symmetric for this airframe), including:
     * a time-varying motor thrust curve (the plant gain changes during the burn),
     * a realistic servo (rate limit + first-order lag) on the gimbal,
     * a noisy rate gyro feeding the controller at a fixed loop rate.
2. Implements the exact PID form used in the flight firmware.
3. Automatically searches for (Kp, Ki, Kd) with scipy.optimize.differential_evolution
   (or a coarse grid search), scored by a cost function that rewards a fast,
   well-damped, low-steady-state-error response that does not pin the gimbal.
4. For robustness, the cost is the WORST case across several thrust levels sampled
   from the burn, so the gains stay stable as the motor burns down.
5. Prints the recommended gains + achieved metrics and saves PNG plots (headless).

Run it:   python tvc_pid_tuner.py
Options:  python tvc_pid_tuner.py --method grid
          python tvc_pid_tuner.py --eng path/to/motor.eng
          python tvc_pid_tuner.py --out results_dir --seed 7

Only depends on numpy, scipy, matplotlib.

>>> READ THE CONFIG BLOCK BELOW <<<  Every physical number is a PLACEHOLDER.
Replace the values tagged "MEASURE/REPLACE" with your real measured numbers.
"""

import argparse
import functools
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

# Headless plotting: pick the non-interactive Agg backend BEFORE importing pyplot,
# so this runs on a machine with no display and never calls plt.show().
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from scipy.optimize import differential_evolution

import closed_loop_backend  # --closed-loop: tune against RocketFC's real C++ TvcControl


# ======================================================================================
# 1. CONFIG  --  ALL physical + tuning parameters live here.
#    Replace every value marked MEASURE/REPLACE with your real measured numbers.
#    Angles are entered in human-friendly DEGREES and converted to radians via the
#    properties at the bottom of the dataclass (the physics runs in radians).
# ======================================================================================
@dataclass
class Config:
    # ---- Rocket mass / geometry -------------------------------------------------------
    mass_kg: float = 0.6           # MEASURE/REPLACE: total liftoff mass with motor (kg).

    # Moment of inertia about the transverse axis through the CG -- THE most important
    # parameter; the response scales directly with it. Estimate it, then refine:
    #   * Uniform thin rod:      I ~= (1/12) * m * L_body^2     (L_body = overall length)
    #   * Refine: add the motor and nosecone as POINT MASSES at their distance d from the
    #     CG:  I += m_part * d^2   for each.
    #   * Best: measure with a BIFILAR (two-string) pendulum:
    #         I = (m * g * d^2 * T^2) / (16 * pi^2 * Ls)
    #     where d = half the string separation, Ls = string length, T = period of small
    #     torsional oscillation, m = rocket mass, g = 9.81.
    I_kgm2: float = 0.03           # MEASURE/REPLACE (ESTIMATE): transverse MOI about CG (kg*m^2).

    # Moment arm = distance from the CG to the gimbal pivot point (where thrust acts).
    # Measure the CG (balance the rocket on an edge) and the pivot location, subtract.
    L_arm_m: float = 0.25          # MEASURE/REPLACE: CG -> gimbal pivot distance (m).

    # ---- Gimbal / actuator ------------------------------------------------------------
    delta_max_deg: float = 7.0     # MEASURE/REPLACE: mechanical gimbal limit, +/- (deg).
    servo_slew_dps: float = 400.0  # MEASURE/REPLACE: max servo slew rate (deg/s). Servo
                                   #   speed is a PRIMARY cause of instability -- a "0.10 s
                                   #   per 60 deg" servo is ~600 deg/s; derate for the
                                   #   gimbal linkage. Measure it if you can.
    servo_lag_tau_s: float = 0.008 # MEASURE/REPLACE: first-order servo lag time constant (s).
                                   #   Set 0 to disable lag. Lag + loop delay drive oscillation
                                   #   and are a PRIMARY instability source: a realistic digital
                                   #   servo is ~5-15 ms; much above ~15 ms this high-gain plant
                                   #   becomes very hard to stabilize (try it -- the tuner will
                                   #   return sluggish, heavily-damped gains or struggle).

    # ---- Sensor / control loop --------------------------------------------------------
    loop_rate_hz: float = 200.0    # Controller update rate (Hz). Match your firmware.
    gyro_noise_dps: float = 0.5    # MEASURE/REPLACE: gyro rate noise std-dev (deg/s).
    angle_noise_deg: float = 0.0   # Optional measured-angle noise std-dev (deg). 0 = none.

    # ---- PID anti-windup --------------------------------------------------------------
    # Anti-windup clamp: the integral TERM (Ki * integral) is clamped so it can never
    # demand more than this fraction of the gimbal limit. Keeps the integrator from
    # winding up while the gimbal is saturated.
    integral_limit_frac: float = 1.0

    # ---- Simulation / disturbances ----------------------------------------------------
    dt_phys_s: float = 0.001       # Physics integration timestep (s). Small & fixed.
    sim_duration_s: float = 5.0    # Duration of the FINAL reported/plotted run (s).
    tune_duration_s: float = 3.5   # Shorter horizon used during gain optimization (s).
    # Optimize on the deterministic (noise-free) plant: gyro noise adds an irreducible
    # floor that obscures the cost and the optimal gains are essentially unchanged. The
    # FINAL reported run and the plots still include the configured noise.
    tune_with_noise: bool = False
    initial_tilt_deg: float = 12.0 # MEASURE/REPLACE intent: start tilted so the controller
                                   #   has to recover (10-15 deg is a good stress test).

    # Disturbance torque from thrust misalignment / CG offset. Proves the integral term
    # removes steady-state error (response should return to 0, not settle at a bias).
    # This is the torque AT THE REFERENCE THRUST and is scaled by F(t)/F_ref at runtime,
    # because a misalignment/CG-offset torque is produced BY the thrust -- so it is roughly
    # constant during the sustain and correctly drops to zero at burnout (no thrust = no
    # torque = no post-burnout tumble). MEASURE/REPLACE with a realistic worst case.
    disturbance_torque_Nm: float = 0.05
    # Reference thrust (N) the disturbance value is quoted at. 0 -> auto (average thrust),
    # set by main(). The constant-thrust cost runs scale it by level/F_ref accordingly.
    disturbance_ref_thrust_N: float = 0.0

    # Optional one-off "gust" impulse mid-flight (set gust_torque_Nm = 0 to disable).
    enable_gust: bool = False
    gust_time_s: float = 2.5
    gust_dur_s: float = 0.05
    gust_torque_Nm: float = 0.6

    # ---- Motor thrust source ----------------------------------------------------------
    eng_file: str = None           # Path to a RASP .eng thrust file. None -> built-in F15.

    # ---- Optimizer ranges (sane bounds for this airframe scale) -----------------------
    kp_bounds: tuple = (0.0, 8.0)
    ki_bounds: tuple = (0.0, 8.0)
    kd_bounds: tuple = (0.0, 4.0)
    de_maxiter: int = 30           # differential_evolution generations.
    de_popsize: int = 10           # population = popsize * n_params.
    grid_n: int = 8                # points per axis for the grid-search alternative.

    # ---- Cost-function weights (tunable; see cost() for what each penalizes) ----------
    w_itae: float = 20.0           # time-weighted absolute error (ITAE) of the tilt angle
    w_overshoot: float = 1.0       # overshoot beyond zero, as % of initial tilt
    w_sse: float = 400.0           # steady-state error (mean |tilt| over the tail)
    w_osc: float = 500.0           # residual oscillation (std of tilt over the tail)
    w_thrash: float = 40.0         # gimbal thrashing (integral of |d(delta)/dt|); kills
                                   #   bang-bang limit cycles the servo can't follow
    w_ctrl: float = 0.5            # control effort (integral of gimbal^2)
    w_sat: float = 5.0             # fraction of time the gimbal command is saturated
    w_unstable: float = 1.0e6      # large penalty if the response diverges / tumbles

    # ---- RNG seeds (fixed so the cost landscape is repeatable despite gyro noise) -----
    tune_seed: int = 12345         # seed used inside every cost-evaluation sim
    eval_seed: int = 999           # seed for the final reported run
    opt_seed: int = 1              # seed for differential_evolution itself

    # Filled in at runtime by main(): the constant thrust levels (N) used by the robust
    # worst-case cost (peak / average / tail of the active burn).
    thrust_levels: tuple = field(default_factory=tuple)

    # ---- Derived (radians) ------------------------------------------------------------
    @property
    def delta_max_rad(self) -> float:
        return math.radians(self.delta_max_deg)

    @property
    def initial_tilt_rad(self) -> float:
        return math.radians(self.initial_tilt_deg)

    @property
    def gyro_noise_rad(self) -> float:
        return math.radians(self.gyro_noise_dps)

    @property
    def angle_noise_rad(self) -> float:
        return math.radians(self.angle_noise_deg)

    @property
    def servo_slew_rad(self) -> float:
        return math.radians(self.servo_slew_dps)

    @property
    def integral_limit(self) -> float:
        # Max allowed |Ki * integral|, expressed in rad of gimbal demand.
        return self.integral_limit_frac * self.delta_max_rad

    @property
    def diverge_limit_rad(self) -> float:
        return math.radians(90.0)   # |tilt| past 90 deg = tumbling = diverged


# ======================================================================================
# 2. THRUST CURVE  --  built-in Estes F15 default + RASP .eng loader
# ======================================================================================
# Representative Estes F15 thrust-vs-time curve (PLACEHOLDER -- replace with your motor's
# real .eng via --eng). Total impulse ~= 49.6 N*s, burn ~= 3.4 s, peak ~= 25 N,
# average ~= 15.6 N. Stored as (time_s, thrust_N) points, linearly interpolated.
F15_CURVE = [
    (0.00, 0.0),
    (0.05, 3.0),
    (0.10, 7.0),
    (0.20, 12.5),
    (0.30, 18.0),
    (0.40, 22.0),
    (0.50, 24.5),
    (0.55, 25.0),   # peak ~25 N
    (0.65, 22.0),
    (0.75, 18.5),
    (0.85, 16.5),
    (1.00, 15.8),
    (1.50, 15.2),
    (2.00, 14.6),
    (2.50, 13.8),
    (3.00, 12.2),
    (3.20, 7.0),
    (3.35, 3.0),
    (3.45, 0.0),    # burnout
]


def _make_interp_thrust(times, thrusts):
    """Build a scalar thrust(t) function from sample points. Returns 0 before/after burn."""
    times = np.asarray(times, dtype=float)
    thrusts = np.asarray(thrusts, dtype=float)

    def thrust_at(t):
        return float(np.interp(t, times, thrusts, left=0.0, right=0.0))

    # Stash the raw samples on the function so callers can read burn time / peak.
    thrust_at.times = times
    thrust_at.thrusts = thrusts
    return thrust_at


def load_eng(path):
    """
    Load a standard RASP '.eng' thrust file.

    Format: ';' starts a comment line; the first non-comment line is the header
    (motor-name diameter length delays prop-mass total-mass manufacturer); every
    following line is a "time thrust" pair up to burnout.
    """
    times, thrusts = [], []
    header = None
    with open(path, "r") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith(";"):
                continue
            parts = line.split()
            if header is None:
                header = parts            # first data line is the motor header
                continue
            if len(parts) >= 2:
                try:
                    t, thr = float(parts[0]), float(parts[1])
                except ValueError:
                    continue
                times.append(t)
                thrusts.append(thr)
    if not times:
        raise ValueError(f"No thrust data found in .eng file: {path}")
    if times[0] > 0.0:                     # ensure the curve starts at t=0, F=0
        times.insert(0, 0.0)
        thrusts.insert(0, 0.0)
    fn = _make_interp_thrust(times, thrusts)
    fn.header = header
    return fn


def make_thrust_fn(cfg):
    """Return the active thrust(t): the .eng file if given, else the built-in F15 curve."""
    if cfg.eng_file:
        return load_eng(cfg.eng_file)
    times = [p[0] for p in F15_CURVE]
    thrusts = [p[1] for p in F15_CURVE]
    return _make_interp_thrust(times, thrusts)


# ======================================================================================
# 3. PLANT (rocket rotational dynamics)
# ======================================================================================
def _gust_torque(cfg, t, gust_active):
    """Optional one-off disturbance impulse, active for a short window mid-flight."""
    if gust_active and cfg.gust_time_s <= t < cfg.gust_time_s + cfg.gust_dur_s:
        return cfg.gust_torque_Nm
    return 0.0


def plant_step(theta, omega, delta_actual, t, thrust_fn, cfg, disturbance, gust_active):
    """
    Advance the rigid-body rotation by one physics timestep.

    Torque about the CG:
        tau = F(t) * L * sin(delta)      <- restoring torque from the gimbaled thrust
            + disturbance * F(t)/F_ref   <- misalignment/CG-offset bias (scales with thrust)
            + gust(t)                    <- optional impulse
    Angular acceleration: alpha = tau / I.

    The disturbance scales with thrust because a thrust-misalignment / CG-offset torque is
    produced by the thrust itself; this also makes it vanish at burnout (otherwise a bias
    torque with no thrust to oppose it would tumble the rocket no matter the gains).

    Sign convention: a positive tilt theta produces error = -theta < 0, so the PID
    commands delta < 0; with F,L > 0 that gives tau = F*L*sin(delta) < 0, which
    decelerates and reverses theta back toward vertical. (Consistent and stable.)
    """
    F = thrust_fn(t)
    ref = cfg.disturbance_ref_thrust_N
    dist = disturbance * (F / ref) if ref > 0 else disturbance
    tau = F * cfg.L_arm_m * math.sin(delta_actual) + dist + _gust_torque(cfg, t, gust_active)
    alpha = tau / cfg.I_kgm2
    # Semi-implicit (symplectic) Euler: update rate first, then integrate angle.
    omega = omega + alpha * cfg.dt_phys_s
    theta = theta + omega * cfg.dt_phys_s
    return theta, omega


# ======================================================================================
# 4. SERVO MODEL  --  rate limit + first-order lag, clamped to the mechanical stop
# ======================================================================================
class ServoModel:
    """
    Converts the commanded gimbal angle into the ACTUAL gimbal angle, accounting for
    finite servo speed (slew-rate limit) and lag (first-order time constant). Stepped
    at the physics rate for fidelity.
    """

    def __init__(self, slew_rate_rad, tau_s, delta_max_rad, dt):
        self.slew = slew_rate_rad          # max |d(delta)/dt|  (rad/s)
        self.tau = tau_s                   # first-order lag time constant (s)
        self.dmax = delta_max_rad
        self.dt = dt
        self.delta = 0.0                   # current actual gimbal angle (rad)

    def step(self, cmd):
        cmd = _clamp(cmd, -self.dmax, self.dmax)
        # 1) Rate limit: the servo can only move so far per timestep.
        max_move = self.slew * self.dt
        target = _clamp(cmd, self.delta - max_move, self.delta + max_move)
        # 2) First-order lag toward that rate-limited target.
        if self.tau > 1e-9:
            self.delta += (target - self.delta) * (self.dt / self.tau)
        else:
            self.delta = target
        # 3) Never exceed the mechanical stop.
        self.delta = _clamp(self.delta, -self.dmax, self.dmax)
        return self.delta


# ======================================================================================
# 5. PID  --  matches the flight firmware exactly
# ======================================================================================
class PID:
    """
    Attitude PID on the tilt angle.

        error    = setpoint - measured_angle          (setpoint = 0 = vertical)
        integral += error * dt   (clamped: anti-windup)
        output   = Kp*error + Ki*integral - Kd*measured_rate
                                                        ^^^^^^^^^^^^^^^^^^^^^^^
        Derivative is taken from the MEASURED RATE (gyro), NOT by differentiating the
        error. This avoids derivative kick on setpoint changes and matches the firmware.
        Output is clamped to the gimbal limit +/- delta_max.
    """

    def __init__(self, kp, ki, kd, dt_ctrl, delta_max, integral_limit, setpoint=0.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt_ctrl
        self.delta_max = delta_max
        self.integral_limit = integral_limit   # max |Ki * integral| (rad)
        self.setpoint = setpoint
        self.integral = 0.0

    def update(self, measured_angle, measured_rate):
        error = self.setpoint - measured_angle
        self.integral += error * self.dt
        # Anti-windup clamp: bound the integral so its contribution Ki*integral can
        # never demand more than the gimbal limit.
        if self.ki > 1e-9:
            i_cap = self.integral_limit / self.ki
            self.integral = _clamp(self.integral, -i_cap, i_cap)
        output = self.kp * error + self.ki * self.integral - self.kd * measured_rate
        return _clamp(output, -self.delta_max, self.delta_max)


def _clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


# ======================================================================================
# 6. SIMULATE  --  one closed-loop run
# ======================================================================================
def simulate(gains, cfg, thrust_fn, seed=0, duration=None, with_disturbance=True,
             with_gust=False, noise=True):
    """
    Run one closed-loop simulation and return arrays of the time histories.

    The physics integrates at dt_phys; the controller fires every `ctrl_every` physics
    steps (i.e. at loop_rate_hz). Sensors are read (with noise) only at controller ticks.
    """
    kp, ki, kd = gains
    dt = cfg.dt_phys_s
    dur = cfg.sim_duration_s if duration is None else duration
    n = int(round(dur / dt))
    ctrl_every = max(1, int(round((1.0 / cfg.loop_rate_hz) / dt)))

    rng = np.random.default_rng(seed)
    pid = PID(kp, ki, kd, 1.0 / cfg.loop_rate_hz, cfg.delta_max_rad, cfg.integral_limit)
    servo = ServoModel(cfg.servo_slew_rad, cfg.servo_lag_tau_s, cfg.delta_max_rad, dt)

    gyro_sd = cfg.gyro_noise_rad if noise else 0.0
    ang_sd = cfg.angle_noise_rad if noise else 0.0
    disturbance = cfg.disturbance_torque_Nm if with_disturbance else 0.0
    gust_active = bool(with_gust and cfg.enable_gust)

    # Preallocate time-history arrays (one sample per physics step).
    t_arr = np.empty(n + 1)
    th = np.empty(n + 1)
    om = np.empty(n + 1)
    dc = np.empty(n + 1)
    da = np.empty(n + 1)

    theta = cfg.initial_tilt_rad
    omega = 0.0
    delta_cmd = 0.0
    diverged = False

    for i in range(n + 1):
        t = i * dt
        # --- Controller tick (runs slower than the physics) ---
        if i % ctrl_every == 0:
            meas_rate = omega + (rng.normal(0.0, gyro_sd) if gyro_sd > 0 else 0.0)
            meas_ang = theta + (rng.normal(0.0, ang_sd) if ang_sd > 0 else 0.0)
            delta_cmd = pid.update(meas_ang, meas_rate)
        # --- Servo moves toward the command (rate limit + lag), every physics step ---
        delta_act = servo.step(delta_cmd)

        # Record the state at this instant.
        t_arr[i], th[i], om[i], dc[i], da[i] = t, theta, omega, delta_cmd, delta_act

        # --- Integrate the plant forward one step ---
        if i < n:
            theta, omega = plant_step(theta, omega, delta_act, t, thrust_fn, cfg,
                                      disturbance, gust_active)
            if not math.isfinite(theta) or abs(theta) > cfg.diverge_limit_rad:
                diverged = True
                # Fill the remainder with the diverged value so metrics are well-defined.
                t_arr[i + 1:] = np.arange(i + 1, n + 1) * dt
                th[i + 1:] = theta
                om[i + 1:] = omega
                dc[i + 1:] = delta_cmd
                da[i + 1:] = delta_act
                break

    return {"t": t_arr, "theta": th, "omega": om, "delta_cmd": dc,
            "delta_act": da, "diverged": diverged}


# ======================================================================================
# 7. COST  --  robust, worst-case-across-thrust score for the optimizer
# ======================================================================================
def _score_run(res, cfg, duration):
    """
    Turn one simulation into a scalar score (lower is better). Combines:
      ITAE              -- time-weighted |tilt|; punishes errors that linger.
      overshoot         -- worst opposite-sign excursion past zero, % of initial tilt.
      steady-state err  -- mean |tilt| over the final 20% (heavily weighted; this is
                           where Ki must cancel the disturbance torque).
      oscillation       -- std of tilt over the final 20%; catches limit cycles whose
                           mean is ~0 but that never actually settle.
      thrashing         -- total travel of the gimbal COMMAND (sum of |delta cmd changes|);
                           large for bang-bang limit cycles the servo cannot follow.
      control effort    -- integral of gimbal^2; mild, keeps gains from being twitchy.
      saturation        -- fraction of time the gimbal COMMAND is pinned at +/-delta_max;
                           discourages solutions that just slam the gimbal to its stop.
      instability       -- big constant if the run diverged / tumbled.
    """
    if res["diverged"]:
        return cfg.w_unstable

    t = res["t"]
    th = res["theta"]
    da = res["delta_act"]
    dc = res["delta_cmd"]
    dt = cfg.dt_phys_s
    n = len(t)
    abs_th = np.abs(th)
    theta0 = cfg.initial_tilt_rad

    # ITAE = integral of t*|theta| dt
    itae = float(np.sum(t * abs_th) * dt)

    # Overshoot: largest excursion of opposite sign to theta0 after the first zero crossing.
    overshoot = 0.0
    crossed = np.where(np.sign(th) != np.sign(theta0))[0]
    if theta0 != 0 and len(crossed):
        seg = th[crossed[0]:]
        peak_opp = -np.min(seg) if theta0 > 0 else np.max(seg)
        overshoot = max(0.0, peak_opp) / abs(theta0) * 100.0

    tail = max(1, int(0.2 * n))
    sse = float(np.mean(abs_th[-tail:]))            # steady-state error (rad)
    osc = float(np.std(th[-tail:]))                 # residual oscillation (rad)
    thrash = float(np.sum(np.abs(np.diff(dc))))     # total gimbal-command travel (rad)
    ctrl_eff = float(np.sum(da * da) * dt)          # control effort
    sat_frac = float(np.mean(np.abs(dc) >= cfg.delta_max_rad * 0.999))

    return (cfg.w_itae * itae
            + cfg.w_overshoot * overshoot
            + cfg.w_sse * sse
            + cfg.w_osc * osc
            + cfg.w_thrash * thrash
            + cfg.w_ctrl * ctrl_eff
            + cfg.w_sat * sat_frac)


def cost(gains, cfg):
    """
    Worst-case cost across the sampled thrust levels. The plant gain is proportional to
    thrust, so gains that are great at peak thrust can go unstable at low thrust (or vice
    versa). Taking the MAX over peak/mid/tail thrust forces gains that work all burn long.

    Each level is simulated at CONSTANT thrust (a fixed operating point) with the initial
    tilt + disturbance torque, over the shorter tuning horizon.
    """
    worst = 0.0
    for level in cfg.thrust_levels:
        const_thrust = (lambda L: (lambda t: L))(level)
        res = simulate(gains, cfg, const_thrust, seed=cfg.tune_seed,
                       duration=cfg.tune_duration_s, with_disturbance=True,
                       with_gust=False, noise=cfg.tune_with_noise)
        worst = max(worst, _score_run(res, cfg, cfg.tune_duration_s))
    return worst


# ======================================================================================
# 8. OPTIMIZERS
# ======================================================================================
def optimize_de(cfg, cost_fn=cost):
    """Find gains with differential evolution (robust for non-smooth, multi-modal cost).

    cost_fn defaults to this file's own rotation-only cost(); pass
    closed_loop_backend.cost_closed_loop (via --closed-loop) to tune against
    the real C++ TvcControl over a full 6-DOF flight instead.
    """
    bounds = [cfg.kp_bounds, cfg.ki_bounds, cfg.kd_bounds]
    result = differential_evolution(
        cost_fn, bounds, args=(cfg,),
        maxiter=cfg.de_maxiter, popsize=cfg.de_popsize,
        tol=0.01, mutation=(0.5, 1.0), recombination=0.7,
        seed=cfg.opt_seed, polish=True, init="latinhypercube", disp=True,
    )
    return tuple(result.x), float(result.fun)


def optimize_grid(cfg, cost_fn=cost):
    """
    Coarse grid-search alternative. Uses LOG-spaced points (plus 0 for Ki/Kd) because good
    gains for this plant span orders of magnitude and sit near the low end of the bounds --
    a linear grid would step right over the optimum. Coarser than differential evolution,
    but lands in the right neighbourhood.

    cost_fn: see optimize_de().
    """
    def log_axis(bounds, include_zero):
        lo = max(bounds[0], 1e-2)            # geomspace needs a positive floor
        pts = np.geomspace(lo, bounds[1], cfg.grid_n)
        return np.concatenate(([0.0], pts)) if include_zero else pts

    kps = log_axis(cfg.kp_bounds, include_zero=False)   # Kp must be > 0 to hold attitude
    kis = log_axis(cfg.ki_bounds, include_zero=True)
    kds = log_axis(cfg.kd_bounds, include_zero=True)
    best, best_c = None, math.inf
    total = len(kps) * len(kis) * len(kds)
    done = 0
    for kp in kps:
        for ki in kis:
            for kd in kds:
                c = cost_fn((kp, ki, kd), cfg)
                done += 1
                if c < best_c:
                    best_c, best = c, (kp, ki, kd)
        print(f"  grid search: {done}/{total} evaluated, best cost so far {best_c:.4g}")
    return best, best_c


# ======================================================================================
# 9. METRICS + PLOTS + MAIN
# ======================================================================================
def compute_metrics(res, cfg, burn_time):
    """
    Human-readable performance metrics from the final (time-varying-thrust) run.

    Steady-state error is measured over the last second of the POWERED phase (before
    burnout): after the motor stops there is no thrust and therefore no attitude control,
    so the post-burnout coast is not a fair measure of controller performance.
    """
    deg = 180.0 / math.pi
    t = res["t"]
    th = res["theta"]
    da = res["delta_act"]
    dc = res["delta_cmd"]
    theta0 = cfg.initial_tilt_rad

    # Overshoot (% of initial tilt).
    overshoot = 0.0
    crossed = np.where(np.sign(th) != np.sign(theta0))[0]
    if theta0 != 0 and len(crossed):
        seg = th[crossed[0]:]
        peak_opp = -np.min(seg) if theta0 > 0 else np.max(seg)
        overshoot = max(0.0, peak_opp) / abs(theta0) * 100.0

    # Settling time: last time |tilt| leaves a +/- band of max(2% of theta0, 0.5 deg),
    # searched only within the powered phase (control authority exists only while burning).
    band = max(0.02 * abs(theta0), math.radians(0.5))
    powered = t <= burn_time
    outside = np.where((np.abs(th) > band) & powered)[0]
    settling = float(t[outside[-1]]) if len(outside) else 0.0

    # Steady-state error: mean |tilt| over the last second of the burn.
    sse_mask = (t >= max(0.0, burn_time - 1.0)) & powered
    sse_deg = float(np.mean(np.abs(th[sse_mask])) * deg) if np.any(sse_mask) else float("nan")
    peak_delta_deg = float(np.max(np.abs(da)) * deg)
    saturated = bool(np.any(np.abs(dc) >= cfg.delta_max_rad * 0.999))

    return {
        "overshoot_pct": overshoot,
        "settling_s": settling,
        "sse_deg": sse_deg,
        "peak_delta_deg": peak_delta_deg,
        "saturated": saturated,
        "diverged": res["diverged"],
    }


def make_plots(res, cfg, gains, thrust_fn, outdir):
    """Save the four PNGs (tilt, gimbal, rate, robustness overlay). Headless (Agg)."""
    deg = 180.0 / math.pi
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    # (a) Tilt angle vs time
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(res["t"], res["theta"] * deg, color="C0")
    ax.axhline(0.0, color="k", lw=0.6)
    ax.set_xlabel("time (s)"); ax.set_ylabel("tilt angle (deg)")
    ax.set_title("Tilt angle vs time (final gains, real thrust curve)")
    ax.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(outdir / "tilt.png", dpi=130); plt.close(fig)

    # (b) Gimbal command + actual vs time, with +/- delta_max limit lines
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(res["t"], res["delta_cmd"] * deg, color="C1", label="gimbal command")
    ax.plot(res["t"], res["delta_act"] * deg, color="C2", alpha=0.8, label="gimbal actual")
    ax.axhline(cfg.delta_max_deg, color="r", ls="--", lw=1, label="+/- delta_max")
    ax.axhline(-cfg.delta_max_deg, color="r", ls="--", lw=1)
    ax.set_xlabel("time (s)"); ax.set_ylabel("gimbal angle (deg)")
    ax.set_title("Gimbal deflection vs time")
    ax.legend(loc="best"); ax.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(outdir / "gimbal.png", dpi=130); plt.close(fig)

    # (c) Angular rate vs time
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(res["t"], res["omega"] * deg, color="C3")
    ax.axhline(0.0, color="k", lw=0.6)
    ax.set_xlabel("time (s)"); ax.set_ylabel("angular rate (deg/s)")
    ax.set_title("Angular rate vs time")
    ax.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(outdir / "rate.png", dpi=130); plt.close(fig)

    # (d) Robustness overlay: same gains at low / mid / high CONSTANT thrust
    fig, ax = plt.subplots(figsize=(8, 4.5))
    levels = sorted(cfg.thrust_levels)
    labels = ["low thrust", "mid thrust", "high thrust"]
    for level, lab, color in zip(levels, labels, ["C4", "C5", "C6"]):
        const_thrust = (lambda L: (lambda t: L))(level)
        r = simulate(gains, cfg, const_thrust, seed=cfg.eval_seed,
                     duration=cfg.sim_duration_s, with_disturbance=True, with_gust=False)
        ax.plot(r["t"], r["theta"] * deg, color=color,
                label=f"{lab} ({level:.1f} N)")
    ax.axhline(0.0, color="k", lw=0.6)
    ax.set_xlabel("time (s)"); ax.set_ylabel("tilt angle (deg)")
    ax.set_title("Robustness: tilt response across the burn (constant-thrust operating points)")
    ax.legend(loc="best"); ax.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(outdir / "robustness.png", dpi=130); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(
        description="Auto-tune attitude PID gains for a thrust-vectoring model rocket.")
    parser.add_argument("--eng", default=None, help="Path to a RASP .eng thrust file.")
    parser.add_argument("--method", choices=["de", "grid"], default="de",
                        help="Optimizer: differential evolution (default) or grid search.")
    parser.add_argument("--out", default=".", help="Directory for the output PNGs.")
    parser.add_argument("--seed", type=int, default=None, help="Override the optimizer seed.")
    parser.add_argument("--closed-loop", action="store_true",
                        help="Tune against RocketFC's real C++ TvcControl over a full "
                             "6-DOF closed-loop flight (../RocketFC/tools/sim) instead of "
                             "this file's own rotation-only Python PID. Each candidate "
                             "gain triple needs a fresh DLL build (uncached) plus a full "
                             "flight sim, so this is CPU-bound -- expect minutes, not "
                             "seconds; --closed-loop halves de_maxiter/de_popsize by "
                             "default unless you pass your own via Config.")
    parser.add_argument("--rocketfc", default="../RocketFC",
                        help="Path to the RocketFC checkout (--closed-loop only). "
                             "Default matches ../SensorServoBenchTest/sync_core.py's "
                             "sibling-directory convention.")
    parser.add_argument("--phase", choices=["boost", "landing"], default="boost",
                        help="Which PidGains struct to tune (--closed-loop only): "
                             "GAINS_BOOST (ascent, mode=chute) or GAINS_LANDING "
                             "(F15 landing burn, mode=land).")
    args = parser.parse_args()

    cfg = Config()
    if args.eng:
        cfg.eng_file = args.eng
    if args.closed_loop:
        # Each evaluation now costs a DLL build + a full flight sim (~1-2s)
        # instead of a cheap rotation-only integration -- halve the search
        # budget so a default run finishes in single-digit minutes. Edit
        # Config's de_maxiter/de_popsize/grid_n directly for a more thorough
        # (and slower) search.
        cfg.de_maxiter = max(4, cfg.de_maxiter // 2)
        cfg.de_popsize = max(4, cfg.de_popsize // 2)
        cfg.grid_n = max(4, cfg.grid_n // 2)
    if args.seed is not None:
        cfg.opt_seed = args.seed
    if args.closed_loop:
        # Sync cfg.I_kgm2/L_arm_m from vehicle.py's own (more grounded)
        # geometry BEFORE the header below prints them -- see
        # make_vehicle_config()'s docstring for why vehicle.py wins.
        closed_loop_backend.make_vehicle_config(cfg, args.rocketfc)

    thrust_fn = make_thrust_fn(cfg)
    burn_time = float(thrust_fn.times[-1])
    peak_thrust = float(np.max(thrust_fn.thrusts))
    # np.trapz was renamed to np.trapezoid in NumPy 2.0; support both.
    _trapezoid = getattr(np, "trapezoid", getattr(np, "trapz", None))
    total_impulse = float(_trapezoid(thrust_fn.thrusts, thrust_fn.times))
    avg_thrust = total_impulse / burn_time if burn_time > 0 else peak_thrust

    # Robustness operating points: peak (highest plant gain), average (mid), and a
    # near-tail sample (low gain, ~90% through the burn). Floor at 1 N so control stays
    # meaningful.
    tail_thrust = max(1.0, thrust_fn(0.9 * burn_time))
    cfg.thrust_levels = (max(1.0, peak_thrust), max(1.0, avg_thrust), tail_thrust)
    # The disturbance torque is quoted at the average thrust and scales with F(t).
    if cfg.disturbance_ref_thrust_N <= 0:
        cfg.disturbance_ref_thrust_N = avg_thrust

    print("=" * 74)
    print(" TVC rocket attitude PID auto-tuner")
    print("=" * 74)
    src = f"RASP .eng file: {cfg.eng_file}" if cfg.eng_file else "built-in Estes F15 (PLACEHOLDER)"
    print(f" Motor source     : {src}")
    print(f" Burn time        : {burn_time:.2f} s")
    print(f" Total impulse    : {total_impulse:.1f} N*s   (avg {avg_thrust:.1f} N, peak {peak_thrust:.1f} N)")
    print(f" Robust thrust pts: {', '.join(f'{x:.1f} N' for x in cfg.thrust_levels)}")
    print(f" I={cfg.I_kgm2} kg*m^2, L={cfg.L_arm_m} m, delta_max=+/-{cfg.delta_max_deg} deg, "
          f"loop={cfg.loop_rate_hz:.0f} Hz")
    print(f" initial tilt={cfg.initial_tilt_deg} deg, disturbance={cfg.disturbance_torque_Nm} N*m, "
          f"servo slew={cfg.servo_slew_dps} deg/s, lag={cfg.servo_lag_tau_s} s")
    print("-" * 74)
    if args.closed_loop:
        print(f" Optimizing gains with method = '{args.method}' against RocketFC's REAL "
              f"C++ TvcControl\n ({args.rocketfc}/tools/sim, phase={args.phase}) -- this "
              f"will take a few minutes...")
        cost_fn = functools.partial(closed_loop_backend.cost_closed_loop,
                                    rocketfc_root=args.rocketfc, phase=args.phase)
    else:
        print(f" Optimizing gains with method = '{args.method}' (worst-case over thrust)...")
        cost_fn = cost

    if args.method == "grid":
        gains, best_cost = optimize_grid(cfg, cost_fn=cost_fn)
    else:
        gains, best_cost = optimize_de(cfg, cost_fn=cost_fn)

    kp, ki, kd = gains

    print("-" * 74)
    print(" RESULTS")
    print(f"   Recommended gains : Kp = {kp:.4f}   Ki = {ki:.4f}   Kd = {kd:.4f}")
    print(f"   Worst-case cost   : {best_cost:.4g}")

    if args.closed_loop:
        _, m = closed_loop_backend.run_and_summarize(
            gains, cfg, rocketfc_root=args.rocketfc, phase=args.phase)
    else:
        # Final reported run: REAL time-varying thrust curve + disturbance (+ optional gust).
        final = simulate(gains, cfg, thrust_fn, seed=cfg.eval_seed,
                         duration=cfg.sim_duration_s, with_disturbance=True,
                         with_gust=cfg.enable_gust, noise=True)
        m = compute_metrics(final, cfg, burn_time)

    print(f"   Overshoot         : {m['overshoot_pct']:.1f} %")
    print(f"   Settling time     : {m['settling_s']:.2f} s  (to within +/-max(2% tilt, 0.5 deg))")
    print(f"   Steady-state err  : {m['sse_deg']:.3f} deg")
    print(f"   Peak gimbal defl. : {m['peak_delta_deg']:.2f} deg  (limit +/-{cfg.delta_max_deg:.1f})")
    print(f"   Gimbal saturated  : {'YES' if m['saturated'] else 'no'}")
    print(f"   Diverged          : {'YES' if m['diverged'] else 'no'}")

    if args.closed_loop:
        print("   Plots             : run `python ../RocketFC/tools/sim/dashboard.py` "
              "and Simulate with these gains for the full trajectory view "
              "(altitude, tilt, state timeline) -- make_plots() here only "
              "understands the rotation-only result shape.")
    else:
        make_plots(final, cfg, gains, thrust_fn, args.out)
        outdir = Path(args.out).resolve()
        print(f"   Plots saved to    : {outdir}\\tilt.png, gimbal.png, rate.png, robustness.png")
    print("=" * 74)
    # One-line, paste-into-Arduino summary:
    print(f"// PID gains from tvc_pid_tuner.py  ->  "
          f"float Kp = {kp:.4f}, Ki = {ki:.4f}, Kd = {kd:.4f};")


if __name__ == "__main__":
    main()
