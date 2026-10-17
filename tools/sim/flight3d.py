#!/usr/bin/env python3
"""3D flight view: watch the rocket actually fly, in a window you can orbit.

Same renderer the bench tool uses (vendored into viz3d/ -- orbit camera, lit
mesh, a nozzle that deflects with the TVC command, world +Z up and body +Z out
the nose with no axis remapping), but the vehicle now TRANSLATES: it climbs,
tips, drifts downrange and lands, leaving a trail behind it, against a
metre-scaled ground grid and an altitude ruler.

The mesh is the real fin-less airframe: four landing legs on the gimbal axes
swing down when the leg-release nichrome has cut their band, and when the
parachute comes out the nose cone pops off and hangs on the shock cord below
the canopy. In a sim run both follow the plant's TRUTH (a stuck latch shows
an open latch and no canopy); a real log has no truth, so the canopy is drawn
from the moment the flight computer confirmed it and the legs from the
nichrome turning on plus an assumed burn-through time -- the readouts say so.

Near the ground the drawing eases from its x3 exaggeration down to TRUE
scale, anchored at the real CG, so the leg pads touch the ground exactly where
the simulated feet do (ground.py): you see the rocket stand, bounce, get
lifted back off by the motor, or fall over, all straight from the physics.
After touchdown a banner gives landing_outcome.py's verdict -- "LANDED" or
"FAILED: <why>" -- and the timeline marks the fire, the ground contact and
the moment it stood or fell.

Four ways to run it:

    python tools/sim/flight3d.py FLIGHTS/flight_001.csv    a real log
    python tools/sim/flight3d.py --drop -1,1.5             just a landing: 1 m/s
                                                           down, 1.5 sideways
    python tools/sim/flight3d.py --sim 101:5               Monte Carlo flight 5
                                                           of seed 101 (add
                                                           --measured for the
                                                           after-measuring spreads)
    (or the dashboard's "3D view" button, which drives it live)

Controls:  the buttons left of the timeline: stop (back to the start),
           step back, play/pause, step forward, playback speed.
           Keys: space = play/pause, Home = stop, Left/Right = step,
           +/- = speed, F = follow/whole-flight camera, G = grid,
           T = trail, Esc = close. Drag = orbit, wheel = zoom.
           Click or drag the stage bar at the bottom to seek.
"""
import argparse
import bisect
import math
import os
import sys

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
if SIM_DIR not in sys.path:
    sys.path.insert(0, SIM_DIR)

import logfile  # noqa: E402

# Nose-to-tail span of the mesh in model.py's own units (BODY_Z0 .. NOSE_TIP_Z).
# The vehicle is scaled by (real length / this) so it sits correctly in a
# world measured in metres.
MODEL_LENGTH_UNITS = 4.05

# A 1.2 m rocket against a 48 m climb is a speck. Drawing it oversized is a
# deliberate, declared lie -- the overlay says "x3" so nobody reads the
# on-screen size as a real proportion. Position, attitude and gimbal angle
# are all still exactly true.
DEFAULT_EXAGGERATION = 3.0

# Mirrors dashboard.py's STATE_COLORS (same hues, pygame 0-255 ints) so the
# stage bar here, the stage strip under the dashboard's scrubber and the
# dashboard's ax_state ribbon all agree at a glance.
STATE_COLORS = {
    "IDLE": (204, 204, 204), "ARMED": (163, 201, 247), "BOOST": (255, 138, 61),
    "COAST": (255, 210, 61), "APOGEE": (199, 125, 255), "DESCENT": (95, 179, 245),
    "LANDING_BURN": (255, 93, 93), "DESCENT_CHUTE": (87, 204, 153),
    "TOUCHDOWN": (45, 106, 79), "ABORT": (208, 0, 0),
}

# Shortened for narrow bands; dropped entirely when even this won't fit.
STAGE_ABBREV = {
    "LANDING_BURN": "BURN", "DESCENT_CHUTE": "CHUTE", "TOUCHDOWN": "DOWN",
    "DESCENT": "DESC", "APOGEE": "APO",
}

# Same palette in GL's 0..1 floats, derived rather than hand-maintained as a
# second table that could drift out of step with the one above.
STATE_COLORS_F = {k: (r / 255.0, g / 255.0, b / 255.0)
                  for k, (r, g, b) in STATE_COLORS.items()}
_TRAIL_FALLBACK_F = (0.35, 0.85, 0.55)

# Roughly how many vertices the trail is decimated to. See Track's trail
# construction for why this matters so much in immediate-mode GL.
TRAIL_POINTS = 400

# The overlay is rebuilt at most this often while only the sample index is
# moving; camera moves and resizes still rebuild immediately so pinned labels
# never lag the geometry they point at.
OVERLAY_HZ = 30.0

# States where the motor is burning, for the flame (a real log only; a sim
# run carries the true thrust, which keeps burning after touchdown).
THRUSTING = ("BOOST", "LANDING_BURN")

# Landing close-up: below CLOSE_HI_M of CG height the drawing eases from the
# exaggeration down to true scale, reaching it at CLOSE_LO_M.
CLOSE_HI_M = 6.0
CLOSE_LO_M = 1.5

# Transport controls: step size and the playback speeds offered.
STEP_S = 0.05
SPEEDS = (0.25, 0.5, 1.0, 2.0, 4.0)
CTRL_W, CTRL_GAP = 30, 4

# Parachute rig, in REAL metres (drawn x exaggeration like the vehicle). The
# shock cord is drawn shorter than a real one would be, deliberately, so the
# canopy and the landing legs both fit in the follow camera at default zoom;
# canopy size comes from the drag area (see canopy_diameter_m).
CHUTE_CORD_M = 1.0
CHUTE_CD = 1.5               # typical for a hemispherical canopy
NOSE_ON_CORD = 0.45          # where the popped nose cone hangs, along the cord
# The cord and lines pay out over this first part of the inflation ramp.
CHUTE_PAYOUT_FRAC = 0.4
# While the chute is out, the follow camera aims this fraction of the rig
# height above the vehicle, so canopy and vehicle are framed together.
CHUTE_CAMERA_LIFT = 0.45


def canopy_diameter_m(cda_m2, cd=CHUTE_CD):
    """Canopy diameter from its drag area: CdA = Cd * pi * D^2 / 4."""
    return 2.0 * math.sqrt(max(cda_m2, 0.0) / (cd * math.pi))


def _ramp(t, t0, duration):
    """0 before t0, rising linearly to 1 over `duration` seconds."""
    return min(1.0, max(0.0, (t - t0) / max(duration, 1e-6)))


def config_gimbal_limit_deg(default=5.0):
    """cfg::GIMBAL_MAX_RAD in degrees, for the crosshair's full scale.

    Read straight out of src/config.h rather than by building and loading the
    DLL, so running this viewer standalone on a log stays instant. The
    dashboard passes the value from the config snapshot it has already read,
    which is the authoritative one for a swept build; this is the fallback.
    Same approach as SensorServoBenchTest/viz/rocket_viz.py::gimbal_limit_deg.
    """
    import re
    path = os.path.join(os.path.dirname(SIM_DIR), "..", "src", "config.h")
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            text = f.read()
        m = re.search(r"GIMBAL_MAX_RAD\s*=\s*([0-9.]+)f?\s*\*\s*DEG2RAD", text)
        if m:
            return float(m.group(1))
        m = re.search(r"GIMBAL_MAX_RAD\s*=\s*([0-9.]+)f?", text)
        if m:
            return math.degrees(float(m.group(1)))
    except OSError:
        pass
    return default


class Track:
    """A flight normalised for rendering: parallel lists indexed by sample.

    Sim rows and log rows carry different things -- a sim knows true
    position and attitude, a real log only has the flight computer's own
    estimate and no horizontal position at all -- so this is the one place
    that difference is resolved, and everything downstream just draws.
    """

    def __init__(self, rows, events=None, is_log=False, dead_reckon=False,
                 label="", landing_speed=None, legs_cut_s=None,
                 legs_swing_s=None, chute_inflation_s=None, outcome=None,
                 ground=None):
        # Timings used ONLY to animate a real log, which records when the
        # nichrome came on and when the canopy was confirmed but not how long
        # the band took to part. Defaults are vehicle.py's; the dashboard
        # passes its edited values.
        import vehicle
        dflt = vehicle.VehicleConfig()
        legs_cut_s = dflt.legs_cut_s if legs_cut_s is None else legs_cut_s
        legs_swing_s = dflt.legs_swing_s if legs_swing_s is None else legs_swing_s
        chute_inflation_s = (dflt.chute_inflation_s if chute_inflation_s is None
                             else chute_inflation_s)
        self.is_log = is_log
        self.label = label
        self.events = list(events or [])
        # landing_outcome.classify()'s verdict and plant.ground_summary() for a
        # sim run with the legs ground model; None for a log or "arrest".
        self.outcome = outcome
        self.ground = ground
        params = (ground or {}).get("params") or {}
        self.cg_h = params.get("landed_cg_height_m", dflt.landed_cg_height_m)
        self.leg_dims = (params.get("leg_foot_radius_m", dflt.leg_foot_radius_m),
                         self.cg_h,
                         params.get("leg_top_ring_from_tail_m",
                                    dflt.leg_top_ring_from_tail_m),
                         params.get("cg_from_nose_m", dflt.cg_from_nose_m))
        contact = (ground or {}).get("contact")
        self.contact_t = contact["t"] if contact else None
        self.fire_t = None
        for ev in self.events:
            code = ev.get("code")
            if getattr(code, "name", str(code)) == "FIRE_LANDING":
                self.fire_t = ev.get("t")
                break
        self.broken_t = (self.contact_t if outcome and
                         "LEGS_BROKEN" in outcome.get("factors", ()) else None)
        self.t = []
        self.pos = []          # (x, y, z) world metres
        self.quat = []         # (w, x, y, z) body -> world
        self.gimbal = []       # (gimx_deg, gimy_deg) ACTUAL deflection
        self.tilt = []
        self.vel = []
        self.state = []
        self.thrust = []       # true thrust (N) for a sim run; None for a log
        # The flight computer's OWN estimate, for the ghost. Only meaningful
        # for a sim run, where there is also a truth to compare it against --
        # a real log contains nothing but the estimate.
        self.est_alt = []
        self.est_quat = []
        self.has_estimate = not is_log
        # Deployment: how far the canopy is inflated and the legs are down
        # (0..1), plus what the flight computer is doing about it.
        self.chute_frac = []
        self.legs_frac = []
        self.latch_open = []     # chute latch servo at RELEASE
        self.canopy_ok = []      # flight computer has confirmed the canopy
        self.nichrome_on = []    # leg-release wire energized
        self.deploy_assumed = is_log

        if not rows:
            raise ValueError("no rows to render")

        xs = ys = None
        if is_log and dead_reckon:
            xs, ys = logfile.dead_reckon_xy(rows)
        self.dead_reckoned = bool(is_log and dead_reckon)

        # A log's deployment is inferred from its pyro/cont bits (see
        # src/hw/logger.h): find when the canopy was confirmed and when the
        # nichrome first came on, then animate from those moments.
        t_canopy = t_nichrome = None
        if is_log:
            for r in rows:
                if t_canopy is None and int(r.get("cont", 0)) & 1:
                    t_canopy = r["t"]
                if t_nichrome is None and int(r.get("pyro", 0)) & 4:
                    t_nichrome = r["t"]

        for i, r in enumerate(rows):
            self.t.append(r["t"])
            if is_log:
                x = xs[i] if xs else 0.0
                y = ys[i] if ys else 0.0
                self.pos.append((x, y, r.get("kf_alt", 0.0)))
                self.quat.append(r.get("quat_est", (1.0, 0.0, 0.0, 0.0)))
                self.gimbal.append((r.get("gimbal_x_deg", 0.0),
                                    r.get("gimbal_y_deg", 0.0)))
                self.vel.append(r.get("kf_vel", 0.0))
            else:
                self.pos.append((r.get("x_true", 0.0), r.get("y_true", 0.0),
                                 r.get("h_true", 0.0)))
                self.quat.append(r.get("quat_true", (1.0, 0.0, 0.0, 0.0)))
                # The ACTUAL servo-lagged deflection, not the command: this is
                # what the nozzle is physically doing, which is what a picture
                # of the nozzle should show.
                self.gimbal.append((math.degrees(r.get("gimbal_x_act", 0.0)),
                                    math.degrees(r.get("gimbal_y_act", 0.0))))
                self.vel.append(r.get("v_true", 0.0))
                self.est_alt.append(r.get("kf_alt", 0.0))
                self.est_quat.append(r.get("quat_est", (1.0, 0.0, 0.0, 0.0)))
            self.thrust.append(r.get("thrust_n"))
            self.tilt.append(r.get("tilt_deg", 0.0))
            self.state.append(r.get("state", ""))
            if is_log:
                pyro, cont = int(r.get("pyro", 0)), int(r.get("cont", 0))
                self.chute_frac.append(
                    _ramp(r["t"], t_canopy, chute_inflation_s)
                    if t_canopy is not None else 0.0)
                self.legs_frac.append(
                    _ramp(r["t"], t_nichrome + legs_cut_s, legs_swing_s)
                    if t_nichrome is not None else 0.0)
                self.latch_open.append(bool(pyro & 1))
                self.canopy_ok.append(bool(cont & 1))
                self.nichrome_on.append(bool(pyro & 4))
            else:
                self.chute_frac.append(r.get("chute_frac", 0.0))
                self.legs_frac.append(r.get("legs_frac", 0.0))
                self.latch_open.append(bool(r.get("chute_release", False)))
                self.canopy_ok.append(bool(r.get("chute_detected", False)))
                self.nichrome_on.append(bool(r.get("legs_burn", False)))

        self.t_max = self.t[-1]
        self.max_alt = max(p[2] for p in self.pos)
        self.stages = self._stages()
        # Decimated trail. A 13 s flight is ~6600 samples, and drawing that
        # many vertices through PyOpenGL's immediate mode costs more than the
        # entire rest of the frame -- every glVertex3f is a Python call. A few
        # hundred points is visually identical for a smooth trajectory.
        # Stage boundaries are force-included so a colour change never lands
        # on the wrong side of a decimated segment.
        stride = max(1, len(self.t) // TRAIL_POINTS)
        keep = set(range(0, len(self.t), stride))
        keep.add(len(self.t) - 1)
        for k in range(1, len(self.state)):
            if self.state[k] != self.state[k - 1]:
                keep.add(k - 1)
                keep.add(k)
        self.trail_idx = sorted(keep)
        self.trail_pos = [self.pos[k] for k in self.trail_idx]
        self.trail_col = [STATE_COLORS_F.get(self.state[k], _TRAIL_FALLBACK_F)
                          for k in self.trail_idx]
        self.landing_speed = (landing_speed if landing_speed is not None
                              else self._landing_speed())

    def _stages(self):
        """Contiguous runs of the same state, as (t_start, t_end, name)."""
        out = []
        start = self.t[0]
        cur = self.state[0]
        for i in range(1, len(self.t)):
            if self.state[i] != cur:
                out.append((start, self.t[i], cur))
                start = self.t[i]
                cur = self.state[i]
        out.append((start, self.t[-1], cur))
        return out

    def _landing_speed(self):
        """|vertical velocity| at the last sample still meaningfully above the
        ground -- i.e. how hard it arrived.

        Returns None, not 0.0, for a flight that never lands (a mid-air abort,
        a truncated log): "we don't know" and "it touched down perfectly" are
        very different answers and must not render the same.

        run.simulate() computes this authoritatively from the plant's own
        touchdown detection; the dashboard passes that in and this derivation
        is only the fallback for a bare log.
        """
        # Close to contact: the vehicle decelerates hard in the last few
        # centimetres, so sampling too high reads faster than it landed.
        GROUND = 0.05
        apogee_i = max(range(len(self.pos)), key=lambda i: self.pos[i][2])
        for i in range(len(self.pos) - 1, apogee_i, -1):
            if self.pos[i][2] > GROUND:
                # Found the last airborne sample; if nothing after it is on the
                # ground, the flight simply ends mid-air.
                return abs(self.vel[i]) if i < len(self.pos) - 1 else None
        return None

    def index_at(self, t):
        i = bisect.bisect_left(self.t, t)
        return max(0, min(len(self.t) - 1, i))

    def bounds(self):
        """(center, radius) of the whole trajectory, for the wide camera."""
        xs = [p[0] for p in self.pos]
        ys = [p[1] for p in self.pos]
        zs = [p[2] for p in self.pos]
        cx = (min(xs) + max(xs)) * 0.5
        cy = (min(ys) + max(ys)) * 0.5
        cz = (min(zs) + max(zs)) * 0.5
        rad = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs), 1.0)
        return (cx, cy, cz), rad


def track_from_log(path, dead_reckon=False):
    log = logfile.read_log(path)
    if not log["rows"]:
        raise ValueError("{}: no data rows".format(path))
    return Track(log["rows"], log["events"], is_log=True,
                 dead_reckon=dead_reckon, label=os.path.basename(path))


def track_from_sim(result, label="simulated flight"):
    s = result.get("summary", {})
    return Track(result["rows"], result["events"], is_log=False, label=label,
                 landing_speed=s.get("touchdown_speed_mps"),
                 outcome=s.get("landing_outcome"), ground=s.get("ground"))


class FlightView:
    """The pygame/OpenGL window.

    Deliberately NOT owning a main loop: the dashboard pumps it from Tk's
    after() timer so both windows share one scrub time with no IPC, and
    main() below pumps it from its own loop for standalone use.
    """

    def __init__(self, track, vehicle_length_m=1.2, size=(1100, 720),
                 exaggeration=DEFAULT_EXAGGERATION, title=None,
                 gimbal_limit_deg=None, chute_diameter_m=None):
        import pygame
        from OpenGL.GL import glViewport

        from viz3d import model as model_mod
        from viz3d import quatmath as qm
        from viz3d import scene as scene_mod

        self._pygame = pygame
        self._qm = qm
        self._scene = scene_mod
        self.track = track
        self.exaggeration = exaggeration
        self.size = size
        self.closed = False
        self.playing = False      # owned by the caller; drawn on the play button
        self.speed = 1.0          # likewise, for the speed buttons
        self.show_grid = True
        self.show_trail = True
        self.show_ghost = track.has_estimate
        self.follow = True
        self.gimbal_limit_deg = (gimbal_limit_deg if gimbal_limit_deg
                                 else config_gimbal_limit_deg())
        self._dragging = False
        self._scrubbing = False
        self._t = 0.0
        self._fps = 0.0
        self._frames = 0
        self._fps_t0 = None

        # Metres per model unit at true scale; render() multiplies in the
        # exaggeration (eased to 1 near the ground -- see _exag_at).
        self.m_per_unit = vehicle_length_m / MODEL_LENGTH_UNITS
        self.model_scale = self.m_per_unit * exaggeration
        self._exag_now = exaggeration
        if chute_diameter_m is None:
            import vehicle
            chute_diameter_m = canopy_diameter_m(
                vehicle.VehicleConfig().cda_chute_m2)
        # Parachute rig, drawn in world metres x the same exaggeration.
        self.chute_radius = 0.5 * chute_diameter_m * exaggeration
        self.chute_line_len = chute_diameter_m * exaggeration
        self.chute_cord_len = CHUTE_CORD_M * exaggeration

        pygame.init()
        pygame.display.set_caption(
            title or "RocketFC 3D flight view -- {}".format(track.label))
        pygame.display.set_mode(size, pygame.OPENGL | pygame.DOUBLEBUF |
                                pygame.RESIZABLE)
        scene_mod.init_gl(*size)
        self.model_mod = model_mod
        self.rocket = model_mod.RocketModel(self._leg_geom())

        # Follow and whole-flight want wildly different framings -- roughly 14 m
        # back for a 3.6 m drawn vehicle vs ~70 m to take in a 48 m climb -- so
        # each mode remembers its own distance. Switching modes restores that
        # mode's framing instead of leaving you zoomed to nothing, and zooming
        # writes back into whichever mode is active.
        center, rad = track.bounds()
        self._wide_center = center
        self._dist_follow = max(5.0, vehicle_length_m * exaggeration * 4.0)
        self._dist_wide = max(12.0, rad * 1.5)
        self.camera = scene_mod.Camera(target=track.pos[0],
                                       dist=self._dist_follow,
                                       max_dist=max(400.0, rad * 8.0))

        pygame.font.init()
        self._font = pygame.font.SysFont("consolas,couriernew,monospace", 15)
        self._font_b = pygame.font.SysFont("consolas,couriernew,monospace", 15,
                                           bold=True)
        self._font_s = pygame.font.SysFont("consolas,couriernew,monospace", 12)
        # Overlay caching. Rebuilding this surface and pushing a full-window
        # RGBA texture is by far the most expensive thing per frame (~6.7 ms
        # measured at 1100x720), and text does not need 60 Hz -- so the
        # surface is rebuilt only when something material changed, and the
        # cached texture is simply re-drawn on the frames in between. Same
        # reasoning as the bench HUD's module docstring.
        self._overlay = None
        self._tex_size = None       # size the texture is currently allocated at
        self._overlay_key = None    # camera/size/toggles: rebuild immediately
        self._overlay_soft = None   # sample index/fps: rebuild at OVERLAY_HZ
        self._overlay_t0 = 0.0
        from OpenGL.GL import glGenTextures
        self._tex = glGenTextures(1)
        self._viewport = glViewport  # kept for resize

    # -- time -------------------------------------------------------------
    @property
    def t(self):
        return self._t

    def set_time(self, t):
        self._t = max(0.0, min(self.track.t_max, t))

    # -- event pump -------------------------------------------------------
    def pump(self):
        """Drain pygame events. Returns a dict describing what the user did:
        {"closed": bool, "seek": float|None, "toggle_play": bool}.
        The caller owns the clock, so seeking is reported rather than applied
        directly -- that keeps the dashboard's scrubber authoritative."""
        pygame = self._pygame
        out = {"closed": False, "seek": None, "toggle_play": False,
               "stop": False, "step": 0, "speed": None}
        if self.closed:
            out["closed"] = True
            return out

        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                self.close()
                out["closed"] = True
                return out
            elif ev.type == pygame.VIDEORESIZE:
                self.size = (ev.w, ev.h)
                pygame.display.set_mode(self.size, pygame.OPENGL |
                                        pygame.DOUBLEBUF | pygame.RESIZABLE)
                self._scene.resize(*self.size)
                self._overlay = None
            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                hit = self._control_at(ev.pos)
                if hit is not None:
                    self._apply_control(hit, out)
                elif self._in_timeline(ev.pos):
                    self._scrubbing = True
                    out["seek"] = self._time_at_x(ev.pos[0])
                else:
                    self._dragging = True
            elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
                self._dragging = False
                self._scrubbing = False
            elif ev.type == pygame.MOUSEMOTION:
                if self._scrubbing:
                    out["seek"] = self._time_at_x(ev.pos[0])
                elif self._dragging:
                    self.camera.orbit(ev.rel[0], ev.rel[1])
            elif ev.type == pygame.MOUSEWHEEL:
                self.camera.zoom(ev.y)
                self._store_dist()
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    self.close()
                    out["closed"] = True
                    return out
                elif ev.key == pygame.K_f:
                    self._set_follow(not self.follow)
                elif ev.key == pygame.K_g:
                    self.show_grid = not self.show_grid
                elif ev.key == pygame.K_t:
                    self.show_trail = not self.show_trail
                elif ev.key == pygame.K_h:
                    # Inert for a log: there is no truth to compare the
                    # estimate against, so there is no ghost to show.
                    if self.track.has_estimate:
                        self.show_ghost = not self.show_ghost
                elif ev.key == pygame.K_SPACE:
                    out["toggle_play"] = True
                elif ev.key == pygame.K_HOME:
                    out["stop"] = True
                elif ev.key == pygame.K_LEFT:
                    out["step"] = -1
                elif ev.key == pygame.K_RIGHT:
                    out["step"] = 1
                elif ev.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                    out["speed"] = self._next_speed(+1)
                elif ev.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    out["speed"] = self._next_speed(-1)
        return out

    # -- transport controls -----------------------------------------------
    def _next_speed(self, direction):
        idx = min(range(len(SPEEDS)), key=lambda k: abs(SPEEDS[k] - self.speed))
        return SPEEDS[max(0, min(len(SPEEDS) - 1, idx + direction))]

    def _control_rects(self):
        """[(name, (x, y, w, h))] for the buttons left of the timeline."""
        _w, h = self.size
        x, y = 14, h - 46
        out = []
        for name in ("stop", "back", "play", "fwd"):
            out.append((name, (x, y, CTRL_W, 30)))
            x += CTRL_W + CTRL_GAP
        x += 6
        for sp in SPEEDS:
            out.append((sp, (x, y, 34, 30)))
            x += 34 + 2
        return out

    def _control_at(self, pos):
        for name, (x, y, w, h) in self._control_rects():
            if x <= pos[0] <= x + w and y <= pos[1] <= y + h:
                return name
        return None

    @staticmethod
    def _apply_control(name, out):
        if name == "stop":
            out["stop"] = True
        elif name == "back":
            out["step"] = -1
        elif name == "fwd":
            out["step"] = 1
        elif name == "play":
            out["toggle_play"] = True
        else:
            out["speed"] = name

    def _draw_controls(self, s):
        pygame = self._pygame
        idle, hot, ink = (34, 38, 46, 235), (70, 120, 200, 255), (226, 230, 238, 255)
        for name, (x, y, w, h) in self._control_rects():
            on = (name == "play" and self.playing) or (
                not isinstance(name, str) and abs(name - self.speed) < 1e-9)
            pygame.draw.rect(s, hot if on else idle, (x, y, w, h), border_radius=4)
            pygame.draw.rect(s, (70, 76, 90, 255), (x, y, w, h), 1, border_radius=4)
            cx, cy = x + w // 2, y + h // 2
            if name == "stop":
                pygame.draw.rect(s, ink, (cx - 6, cy - 6, 12, 12))
            elif name == "back":
                pygame.draw.rect(s, ink, (cx - 7, cy - 6, 3, 12))
                pygame.draw.polygon(s, ink, [(cx + 6, cy - 6), (cx + 6, cy + 6), (cx - 3, cy)])
            elif name == "fwd":
                pygame.draw.rect(s, ink, (cx + 4, cy - 6, 3, 12))
                pygame.draw.polygon(s, ink, [(cx - 6, cy - 6), (cx - 6, cy + 6), (cx + 3, cy)])
            elif name == "play":
                if self.playing:
                    pygame.draw.rect(s, ink, (cx - 6, cy - 7, 4, 14))
                    pygame.draw.rect(s, ink, (cx + 2, cy - 7, 4, 14))
                else:
                    pygame.draw.polygon(s, ink, [(cx - 5, cy - 7), (cx - 5, cy + 7), (cx + 7, cy)])
            else:
                txt = ("{:g}x".format(name) if name >= 1 else
                       {0.25: ".25x", 0.5: ".5x"}.get(name, "{:g}x".format(name)))
                surf = self._font_s.render(txt, True, ink)
                s.blit(surf, (cx - surf.get_width() // 2, cy - surf.get_height() // 2))

    # -- camera modes -----------------------------------------------------
    def _store_dist(self):
        if self.follow:
            self._dist_follow = self.camera.dist
        else:
            self._dist_wide = self.camera.dist

    def _set_follow(self, follow):
        self._store_dist()
        self.follow = follow
        if follow:
            self.camera.dist = self._dist_follow
            self.camera.target = self._cg_world(
                self.track.pos[self.track.index_at(self._t)])
        else:
            self.camera.dist = self._dist_wide
            self.camera.target = self._wide_center

    # -- timeline geometry ------------------------------------------------
    def _timeline_rect(self):
        w, h = self.size
        name, (x, _y, cw, _ch) = self._control_rects()[-1]
        x0 = x + cw + 12
        return (x0, h - 44, max(40, w - 14 - x0), 26)

    def _in_timeline(self, pos):
        x, y, tw, th = self._timeline_rect()
        return x <= pos[0] <= x + tw and y - 4 <= pos[1] <= y + th + 4

    def _time_at_x(self, px):
        x, _y, tw, _th = self._timeline_rect()
        frac = (px - x) / float(max(tw, 1))
        return max(0.0, min(1.0, frac)) * self.track.t_max

    # -- rendering --------------------------------------------------------
    def render(self, t=None):
        if self.closed:
            return
        if t is not None:
            self.set_time(t)
        from OpenGL.GL import (glPushMatrix, glPopMatrix, glMultMatrixf,
                               glTranslatef, glScalef)

        trk = self.track
        i = trk.index_at(self._t)
        pos = trk.pos[i]
        scene_mod = self._scene
        self.rocket.set_leg_geom(self._leg_geom())   # no-op unless the flight changed
        self._exag_now = self._exag_at(i)
        self.model_scale = self.m_per_unit * self._exag_now
        cg = self._cg_world(pos)

        chute_f = trk.chute_frac[i]
        # The plant collapses the canopy once the rocket is down (no drag);
        # stop drawing the rig then too. The nose stays off.
        rig_f = (0.0 if trk.contact_t is not None and self._t > trk.contact_t + 0.3
                 else chute_f)
        if self.follow:
            # Distance is whatever the user has zoomed to in this mode; only
            # the aim point tracks the vehicle -- lifted toward the canopy
            # while the chute is out, so both stay in frame.
            lift = CHUTE_CAMERA_LIFT * self._rig_height() * rig_f
            self.camera.target = (cg[0], cg[1], cg[2] + lift)

        scene_mod.begin_frame(self.camera)

        if self.show_grid:
            # Fine cells under the vehicle in follow mode, so the ground reads
            # as moving past; a coarser, wider mat when framing the whole
            # flight, so it doesn't turn into moire.
            if self.follow:
                scene_mod.draw_grid(half=12, step=2.0,
                                    center=(pos[0], pos[1]), major_every=5)
            else:
                scene_mod.draw_grid(half=10, step=10.0, center=(0.0, 0.0),
                                    major_every=5)
        scene_mod.draw_world_axes(origin=(0.0, 0.0, 0.0),
                                  length=max(3.0, trk.max_alt * 0.08),
                                  dropline=False)
        # Offset sideways from the pad: the vehicle flies up the x=y=0 line, so
        # a ruler drawn there would run straight through the rocket and drop
        # its labels on top of it.
        ruler_labels = scene_mod.draw_altitude_ruler(
            trk.max_alt, step=self._ruler_step(), at=(self._ruler_x(), 0.0),
            tick_len=max(0.6, trk.max_alt * 0.02))
        if self.show_trail:
            # Prefix of the decimated trail up to the current sample, plus the
            # vehicle's exact current position so the trail always meets it.
            k = bisect.bisect_right(trk.trail_idx, i)
            scene_mod.draw_trail(trk.trail_pos[:k] + [pos],
                                 colors=trk.trail_col[:k] + [
                                     STATE_COLORS_F.get(trk.state[i],
                                                        _TRAIL_FALLBACK_F)])
        scene_mod.draw_dropline(pos)
        self._draw_ground_patch(i)

        glPushMatrix()
        glTranslatef(*cg)
        glMultMatrixf(self._qm.gl_matrix(trk.quat[i]))
        glScalef(self.model_scale, self.model_scale, self.model_scale)
        glTranslatef(0.0, 0.0, -self.rocket.leg_geom.cg_z)
        gx, gy = trk.gimbal[i]
        broken = trk.broken_t is not None and self._t >= trk.broken_t
        self.rocket.draw_solid(gx, gy, legs_frac=trk.legs_frac[i],
                               nose_on=chute_f <= 0.0, legs_broken=broken)
        thr = trk.thrust[i] if trk.thrust else None
        if (thr > 0.5) if thr is not None else (trk.state[i] in THRUSTING):
            self._draw_flame(gx, gy)
        glPopMatrix()

        if rig_f > 0.0:
            self._draw_chute_rig(i, pos, rig_f)

        if self.show_ghost and trk.has_estimate:
            self._draw_ghost(i, pos)

        # One batched projection for every ruler label, rather than three GL
        # state readbacks apiece.
        pts = [p for p, _txt in ruler_labels]
        screen = scene_mod.project_many(pts)
        labels = [(s, txt) for s, (_p, txt) in zip(screen, ruler_labels)]
        self._draw_overlay(i, labels)
        self._pygame.display.flip()
        self._tick_fps()

    # -- landing close-up ----------------------------------------------------
    def _leg_geom(self):
        foot_r, cg_h, top, cg_nose = self.track.leg_dims
        return self.model_mod.LegGeom(foot_r, cg_h, top, cg_nose, self.m_per_unit)

    def _cg_world(self, pos):
        """pos is the plant's reference point (the feet's level when the rocket
        stands upright); the CG -- what the drawing rotates about -- is
        landed_cg_height_m above it."""
        return (pos[0], pos[1], pos[2] + self.track.cg_h)

    def _exag_at(self, i):
        h = self.track.pos[i][2] + self.track.cg_h
        f = max(0.0, min(1.0, (h - CLOSE_LO_M) / (CLOSE_HI_M - CLOSE_LO_M)))
        return 1.0 + (self.exaggeration - 1.0) * f

    def _draw_ground_patch(self, i):
        """The landing spot's sloped grass, drawn once the rocket is near it."""
        g = self.track.ground
        if not g or not g.get("anchor") or self._exag_now >= self.exaggeration:
            return
        from OpenGL.GL import (glDisable, glEnable, glColor4f, glBegin, glEnd,
                               glVertex3f, glDepthMask, GL_LIGHTING, GL_QUADS,
                               GL_LINES)
        s, d = math.radians(g["slope_deg"]), math.radians(g["slope_dir_deg"])
        nx, ny, nz = math.sin(s) * math.cos(d), math.sin(s) * math.sin(d), math.cos(s)
        ax, ay = g["anchor"]
        half = 2.0
        # The plane is the physics' ground everywhere; draw the part under
        # the rocket, which may have hopped or slid away from the anchor.
        px, py = self.track.pos[i][0], self.track.pos[i][1]

        def z_at(x, y):
            return -(nx * (x - ax) + ny * (y - ay)) / nz + 0.004

        corners = [(px - half, py - half), (px + half, py - half),
                   (px + half, py + half), (px - half, py + half)]
        glDisable(GL_LIGHTING)
        glDepthMask(False)
        glColor4f(0.30, 0.62, 0.30, 0.35)
        glBegin(GL_QUADS)
        for x, y in corners:
            glVertex3f(x, y, z_at(x, y))
        glEnd()
        glColor4f(0.42, 0.75, 0.42, 0.55)
        glBegin(GL_LINES)
        for k in range(9):
            o = -half + k * 0.5
            for (x0, y0), (x1, y1) in (((px + o, py - half), (px + o, py + half)),
                                       ((px - half, py + o), (px + half, py + o))):
                glVertex3f(x0, y0, z_at(x0, y0))
                glVertex3f(x1, y1, z_at(x1, y1))
        glEnd()
        glDepthMask(True)
        glEnable(GL_LIGHTING)

    def _rig_height(self):
        """Fully paid-out height of the parachute rig above the body top."""
        return (self.chute_cord_len + self.chute_line_len +
                0.7 * self.chute_radius)

    def _draw_chute_rig(self, i, pos, frac):
        """Shock cord from the top of the body tube, the popped nose cone
        hanging on it, and the canopy above -- straight up in world space.

        The plant has no canopy dynamics, so there is no lateral swing to
        draw; and the relative wind under a canopy comes from below, so up
        is the honest direction for the descent this is drawn for.
        """
        from OpenGL.GL import (glPushMatrix, glPopMatrix, glTranslatef,
                               glScalef, glDisable, glEnable, glColor4f,
                               glBegin, glEnd, glVertex3f, glLineWidth,
                               GL_LIGHTING, GL_LINES)
        trk = self.track
        m = self.model_mod
        s = self.model_scale
        # Attach point: top of the body tube, carried by the vehicle attitude.
        cg = self._cg_world(pos)
        top = self._qm.rotate(trk.quat[i],
                              (0.0, 0.0, (m.BODY_Z1 - self.rocket.leg_geom.cg_z) * s))
        ax, ay, az = cg[0] + top[0], cg[1] + top[1], cg[2] + top[2]

        payout = min(1.0, frac / CHUTE_PAYOUT_FRAC)
        cord = self.chute_cord_len * payout
        nose_h = (m.NOSE_TIP_Z - m.BODY_Z1) * s
        nose_base = az + NOSE_ON_CORD * cord
        conf = az + cord                       # where the shroud lines meet

        glDisable(GL_LIGHTING)
        glLineWidth(2.0)
        glColor4f(0.92, 0.92, 0.95, 1.0)  # shock cord: no trail uses white
        glBegin(GL_LINES)
        glVertex3f(ax, ay, az)
        glVertex3f(ax, ay, nose_base)
        if nose_base + nose_h < conf:
            glVertex3f(ax, ay, nose_base + nose_h)
            glVertex3f(ax, ay, conf)
        glEnd()
        glEnable(GL_LIGHTING)

        # The nose cone, tip up, hanging by its base.
        glPushMatrix()
        glTranslatef(ax, ay, nose_base)
        glScalef(s, s, s)
        glTranslatef(0.0, 0.0, -m.BODY_Z1)
        self.rocket.draw_nose()
        glPopMatrix()

        glPushMatrix()
        glTranslatef(ax, ay, conf)
        self.rocket.draw_chute(frac, self.chute_radius,
                               self.chute_line_len * payout)
        glPopMatrix()

    def _draw_ghost(self, i, true_pos):
        """The flight computer's own belief, drawn translucent beside the truth.

        Positioned at the TRUE x/y and the ESTIMATED altitude, because the
        altitude KF is vertical-only -- the flight computer has no horizontal
        estimate at all. Offsetting the ghost sideways would draw an opinion
        the firmware does not actually hold.
        """
        from OpenGL.GL import (glPushMatrix, glPopMatrix, glMultMatrixf,
                               glTranslatef, glScalef, glDepthMask)
        trk = self.track
        glPushMatrix()
        glTranslatef(true_pos[0], true_pos[1], trk.est_alt[i] + trk.cg_h)
        glMultMatrixf(self._qm.gl_matrix(trk.est_quat[i]))
        glScalef(self.model_scale, self.model_scale, self.model_scale)
        glTranslatef(0.0, 0.0, -self.rocket.leg_geom.cg_z)
        # Translucent without writing depth, so the solid vehicle stays
        # visible through it from any angle.
        glDepthMask(False)
        self.rocket.draw_ghost()
        glDepthMask(True)
        glPopMatrix()

    def _tick_fps(self):
        import time
        now = time.perf_counter()
        if self._fps_t0 is None:
            self._fps_t0 = now
            return
        self._frames += 1
        dt = now - self._fps_t0
        if dt >= 0.5:
            self._fps = self._frames / dt
            self._frames = 0
            self._fps_t0 = now

    def _ruler_x(self):
        """How far to the side of the flight path the altitude ruler stands."""
        return max(4.0, self.track.max_alt * 0.15)

    def _ruler_step(self):
        a = self.track.max_alt
        for step in (5.0, 10.0, 20.0, 50.0, 100.0):
            if a / step <= 12:
                return step
        return 200.0

    def _draw_flame(self, gimx_deg, gimy_deg):
        """A short translucent plume out of the nozzle, deflected with it.
        Cosmetic -- it is the cue that the motor is lit, nothing more."""
        import random

        from OpenGL.GL import (glPushMatrix, glPopMatrix, glTranslatef,
                               glRotatef, glDepthMask, glDisable, glEnable,
                               glColor4f, glBegin, glEnd, glVertex3f,
                               GL_LIGHTING, GL_TRIANGLE_FAN)
        m = self.model_mod
        flare = 1.0 + random.uniform(-0.12, 0.12)
        glPushMatrix()
        glTranslatef(0, 0, m.NOZZLE_Z0)
        glRotatef(gimx_deg, 1, 0, 0)
        glRotatef(gimy_deg, 0, 1, 0)
        glTranslatef(0, 0, -m.NOZZLE_Z0)
        glDisable(GL_LIGHTING)
        glDepthMask(False)
        z1 = self.rocket.nozzle_z1
        tip_z = z1 - 1.5 * flare
        for r, a, z in ((m.NOZZLE_R1 * 1.05, 0.55, tip_z),
                        (m.NOZZLE_R1 * 0.6, 0.85, z1 - 0.6 * flare)):
            glBegin(GL_TRIANGLE_FAN)
            glColor4f(1.0, 0.85, 0.45, a)
            glVertex3f(0, 0, z)
            glColor4f(1.0, 0.45, 0.12, 0.0)
            for k in range(13):
                ang = 2.0 * math.pi * k / 12.0
                glVertex3f(r * math.cos(ang), r * math.sin(ang), z1)
            glEnd()
        glDepthMask(True)
        glEnable(GL_LIGHTING)
        glPopMatrix()

    # -- 2D overlay -------------------------------------------------------
    def _draw_overlay(self, i, ruler_labels):
        """Rebuild the overlay only when it would actually look different,
        then draw the cached texture.

        Rebuilding the surface and pushing a full-window RGBA texture was
        measured at ~6.7 ms -- more than the entire rest of the frame. Text
        does not need 60 Hz, so it is rebuilt on a change of sample, camera
        pose, size or toggle (plus a ~25 Hz floor so the clock keeps moving)
        and merely re-drawn in between.
        """
        import time
        w, h = self.size
        cam = self.camera
        # Split deliberately: anything that moves the 3D geometry the labels
        # are pinned to must rebuild NOW or the labels visibly lag it, while
        # the readouts merely counting up can wait for the next slot.
        hard = (w, h, round(cam.az, 4), round(cam.el, 4), round(cam.dist, 3),
                round(cam.target[0], 3), round(cam.target[1], 3),
                round(cam.target[2], 3), self.follow, self.show_ghost,
                self.show_grid, self.show_trail, self.playing, self.speed)
        soft = (i, round(self._fps, 0))
        now = time.perf_counter()
        due = (now - self._overlay_t0) >= (1.0 / OVERLAY_HZ)
        if (self._overlay is None or hard != self._overlay_key or
                (soft != self._overlay_soft and due)):
            self._rebuild_overlay(i, ruler_labels)
            self._overlay_key = hard
            self._overlay_soft = soft
            self._overlay_t0 = now
            self._upload_overlay()
        self._blit_overlay()

    def _rebuild_overlay(self, i, ruler_labels):
        pygame = self._pygame
        w, h = self.size
        if self._overlay is None or self._overlay.get_size() != (w, h):
            self._overlay = pygame.Surface((w, h), pygame.SRCALPHA, 32)
            self._tex_size = None   # force a fresh allocation at the new size
        s = self._overlay
        s.fill((0, 0, 0, 0))
        trk = self.track

        # -- readouts panel --
        state = trk.state[i]
        land = ("{:6.2f} m/s".format(trk.landing_speed)
                if trk.landing_speed is not None else "    -- ")
        lines = [
            ("t", "{:6.2f} s".format(self._t)),
            ("alt", "{:6.1f} m".format(trk.pos[i][2])),
            ("vert v", "{:6.1f} m/s".format(trk.vel[i])),
            ("tilt", "{:6.1f} deg".format(trk.tilt[i])),
            ("chute", self._chute_status(i)),
            ("legs", self._legs_status(i)),
            ("landing v", land),
            # "--" until half a second of frames have actually been timed;
            # printing 0 before then reads as "it is running at zero FPS".
            ("fps", "{:6.0f}".format(self._fps) if self._fps > 0 else "    -- "),
        ]
        pw, ph = 216, 28 + len(lines) * 19 + 24
        pygame.draw.rect(s, (16, 18, 22, 210), (12, 12, pw, ph), border_radius=6)
        pygame.draw.rect(s, (60, 66, 78, 255), (12, 12, pw, ph), 1, border_radius=6)
        s.blit(self._font_b.render(state, True, STATE_COLORS.get(state,
                                                                (230, 230, 230))),
               (24, 20))
        y = 44
        for k, v in lines:
            s.blit(self._font.render(k, True, (150, 158, 172)), (24, y))
            s.blit(self._font.render(v, True, (226, 230, 238)), (96, y))
            y += 19
        note = ("true scale" if self._exag_now <= 1.0001
                else "scale x{:.1f}".format(self._exag_now))
        if trk.dead_reckoned:
            note += "  downrange dead-reckoned"
        if trk.deploy_assumed and (trk.legs_frac[-1] > 0 or trk.chute_frac[-1] > 0):
            note += "  * timing inferred"
        s.blit(self._font_s.render(note, True, (140, 148, 162)), (24, y + 2))

        # -- gimbal crosshair, top right --
        self._draw_gimbal_panel(s, i)

        # -- keys --
        keys = ("drag orbit | wheel zoom | F {} | space play | Home stop | "
                "<- -> step | +/- speed | G grid | T trail").format(
            "whole flight" if self.follow else "follow")
        if trk.has_estimate:
            keys += " | H ghost"
        keys += " | Esc"
        s.blit(self._font_s.render(keys, True, (128, 136, 150)), (14, h - 112))

        # -- altitude ruler labels, projected from 3D --
        for scr, txt in ruler_labels:
            if scr is None:
                continue
            s.blit(self._font_s.render(txt, True, (130, 140, 155)),
                   (scr[0] + 2, scr[1] - 7))

        self._draw_outcome_banner(s)
        self._draw_controls(s)
        self._draw_timeline(s, i)
        self._blit_overlay()

    def _outcome_shown(self):
        o = self.track.outcome
        if not o:
            return False
        t = o.get("t")
        return self._t >= (t if t is not None else self.track.t_max)

    def _draw_outcome_banner(self, s):
        """landing_outcome.py's verdict, once the flight has reached it."""
        if not self._outcome_shown():
            return
        pygame = self._pygame
        o = self.track.outcome
        ok = o.get("success")
        head = o["label"] if ok else "FAILED: " + o["label"]
        others = [f for f in o.get("factors", []) if f != o["code"]]
        sub = ("also: " + ", ".join(f.replace("_", " ").lower() for f in others)
               if others else "")
        w, _h = self.size
        t1 = self._font_b.render(head, True, (255, 255, 255))
        t2 = self._font_s.render(sub, True, (235, 235, 240)) if sub else None
        bw = max(t1.get_width(), t2.get_width() if t2 else 0) + 28
        bh = 34 + (18 if t2 else 0)
        x = (w - bw) // 2
        col = (34, 128, 68, 235) if ok else (168, 36, 36, 235)
        pygame.draw.rect(s, col, (x, 14, bw, bh), border_radius=6)
        s.blit(t1, (x + 14, 22))
        if t2:
            s.blit(t2, (x + 14, 42))

    def _chute_status(self, i):
        trk = self.track
        f = trk.chute_frac[i]
        if f >= 1.0:
            txt = "canopy OK" if trk.canopy_ok[i] else "canopy"
        elif f > 0.0:
            txt = "inflating"
        elif trk.latch_open[i]:
            txt = "latch open"      # released, nothing out (yet)
        else:
            txt = "stowed"
        return txt + (" *" if trk.deploy_assumed and f > 0.0 else "")

    def _legs_status(self, i):
        trk = self.track
        f = trk.legs_frac[i]
        if f >= 1.0:
            txt = "down"
        elif f > 0.0:
            txt = "deploying"
        elif trk.nichrome_on[i]:
            txt = "burning"         # nichrome on, band not cut yet
        else:
            txt = "stowed"
        return txt + (" *" if trk.deploy_assumed and f > 0.0 else "")

    def _draw_gimbal_panel(self, s, i):
        """Gimbal deflection crosshair, ported from the bench tool's
        hud.py::_panel_gimbal: a square whose edge is the gimbal limit, with a
        dot at the current (Y, X) deflection.

        NOTE the full-alpha colours throughout. pygame.draw REPLACES alpha
        rather than blending it, so mixing alphas here punches see-through
        bands into the panel that read as separate boxes instead of one
        square with a cross. This is a correctness constraint, not a style.
        """
        pygame = self._pygame
        w, _h = self.size
        gx, gy = self.track.gimbal[i]
        lim = max(1e-3, self.gimbal_limit_deg)

        cross = 108
        pw = 190
        ph = 30 + cross + 44
        px0 = w - pw - 12
        py0 = 12
        pygame.draw.rect(s, (16, 18, 22, 210), (px0, py0, pw, ph), border_radius=6)
        pygame.draw.rect(s, (60, 66, 78, 255), (px0, py0, pw, ph), 1,
                         border_radius=6)
        s.blit(self._font_b.render("GIMBAL", True, (196, 204, 218)),
               (px0 + 12, py0 + 8))

        cx, cy = px0 + pw // 2, py0 + 30 + cross // 2
        r = cross // 2
        pygame.draw.rect(s, (12, 13, 16, 255), (cx - r, cy - r, cross, cross))
        pygame.draw.line(s, (52, 57, 66, 255), (cx - r, cy), (cx + r, cy))
        pygame.draw.line(s, (52, 57, 66, 255), (cx, cy - r), (cx, cy + r))
        pygame.draw.rect(s, (58, 63, 72, 255), (cx - r, cy - r, cross, cross),
                         width=1)

        dx = cx + int(max(-1.0, min(1.0, gy / lim)) * r)
        dy = cy - int(max(-1.0, min(1.0, gx / lim)) * r)
        pygame.draw.line(s, (120, 180, 255, 255), (cx, cy), (dx, dy), 2)
        pygame.draw.circle(s, (120, 180, 255, 255), (dx, dy), 5)

        s.blit(self._font.render("X {:+5.2f}  Y {:+5.2f}".format(gx, gy), True,
                                 (226, 230, 238)), (px0 + 12, cy + r + 6))
        s.blit(self._font_s.render("full scale +/-{:.1f} deg".format(lim), True,
                                   (146, 154, 166)), (px0 + 12, cy + r + 26))

    def _draw_timeline(self, s, i):
        """The stage bar: one coloured, labelled band per flight phase."""
        pygame = self._pygame
        x, y, tw, th = self._timeline_rect()
        t_max = max(self.track.t_max, 1e-6)

        pygame.draw.rect(s, (12, 14, 18, 200), (x - 2, y - 2, tw + 4, th + 4),
                         border_radius=4)
        for t0, t1, name in self.track.stages:
            bx = x + int(tw * (t0 / t_max))
            bw = max(1, int(tw * ((t1 - t0) / t_max)))
            pygame.draw.rect(s, STATE_COLORS.get(name, (110, 110, 110)),
                             (bx, y, bw, th))
            label = STAGE_ABBREV.get(name, name)
            surf = self._font_s.render(label, True, (18, 20, 24))
            if surf.get_width() + 6 <= bw:
                s.blit(surf, (bx + (bw - surf.get_width()) // 2,
                              y + (th - surf.get_height()) // 2))

        # event ticks above the bar
        for ev in self.track.events:
            et = ev.get("t", 0.0)
            ex = x + int(tw * (et / t_max))
            pygame.draw.line(s, (240, 240, 245), (ex, y - 6), (ex, y - 1))

        # landing markers: F = fire, C = ground contact, then the verdict
        trk = self.track
        marks = [(trk.fire_t, "F", (255, 140, 60)),
                 (trk.contact_t, "C", (200, 200, 210))]
        if trk.outcome:
            ok = trk.outcome.get("success")
            marks.append((trk.outcome.get("t"), "OK" if ok else "X",
                          (60, 190, 100) if ok else (230, 60, 60)))
        placed = []
        for mt, txt, col in marks:
            if mt is None:
                continue
            mx = x + int(tw * (mt / t_max))
            # Stack a label above any neighbour it would overlap (the verdict
            # often lands on the contact itself).
            row = sum(1 for px_ in placed if abs(px_ - mx) < 18)
            placed.append(mx)
            pygame.draw.polygon(s, col, [(mx - 5, y - 12), (mx + 5, y - 12), (mx, y - 4)])
            lab = self._font_s.render(txt, True, col)
            s.blit(lab, (mx - lab.get_width() // 2, y - 26 - 13 * row))

        px = x + int(tw * (self._t / t_max))
        pygame.draw.line(s, (255, 255, 255), (px, y - 8), (px, y + th + 8), 2)

    def _upload_overlay(self):
        """Push the rebuilt surface into the texture.

        Allocated once per size with glTexImage2D, then updated in place with
        glTexSubImage2D -- reallocating a 1100x720 RGBA texture every frame
        was a measurable part of the old frame cost, and only happens on a
        resize now. Called only when the surface was actually rebuilt.
        """
        from OpenGL.GL import (glBindTexture, glTexParameteri, glTexImage2D,
                               glTexSubImage2D, GL_TEXTURE_2D,
                               GL_TEXTURE_MIN_FILTER, GL_TEXTURE_MAG_FILTER,
                               GL_NEAREST, GL_RGBA, GL_UNSIGNED_BYTE)
        w, h = self.size
        # tobytes(), not the deprecated tostring().
        data = self._pygame.image.tobytes(self._overlay, "RGBA", False)
        glBindTexture(GL_TEXTURE_2D, self._tex)
        if self._tex_size != (w, h):
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA,
                         GL_UNSIGNED_BYTE, data)
            self._tex_size = (w, h)
        else:
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, GL_RGBA,
                            GL_UNSIGNED_BYTE, data)

    def _blit_overlay(self):
        """Draw the cached overlay texture over the scene. Cheap: no surface
        work and no upload, just one textured quad."""
        from OpenGL.GL import (glMatrixMode, glPushMatrix, glPopMatrix,
                               glLoadIdentity, glOrtho, glDisable, glEnable,
                               glBindTexture, glColor4f, glBegin, glEnd,
                               glTexCoord2f, glVertex2f, GL_PROJECTION,
                               GL_MODELVIEW, GL_DEPTH_TEST, GL_LIGHTING,
                               GL_TEXTURE_2D, GL_QUADS)
        w, h = self.size
        if self._tex_size is None:
            return  # nothing uploaded yet

        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        glOrtho(0, w, h, 0, -1, 1)      # y down, matching pygame's surface
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()

        glDisable(GL_DEPTH_TEST)
        glDisable(GL_LIGHTING)
        glEnable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, self._tex)
        glColor4f(1, 1, 1, 1)
        glBegin(GL_QUADS)
        glTexCoord2f(0, 0); glVertex2f(0, 0)
        glTexCoord2f(1, 0); glVertex2f(w, 0)
        glTexCoord2f(1, 1); glVertex2f(w, h)
        glTexCoord2f(0, 1); glVertex2f(0, h)
        glEnd()
        glDisable(GL_TEXTURE_2D)
        glEnable(GL_LIGHTING)
        glEnable(GL_DEPTH_TEST)

        glPopMatrix()
        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)

    def close(self):
        if not self.closed:
            self.closed = True
            try:
                self._pygame.display.quit()
            except Exception:
                pass


def track_from_mc(spec, measured=False):
    """Fly one Monte Carlo flight ("SEED:I") exactly as mc_batch.py would and
    return its Track."""
    import mc_batch
    import run as run_mod
    import sweep
    import vehicle
    import core
    seed, i = (int(x) for x in spec.split(":"))
    dll = core.default_dll()
    base = vehicle.default_vehicle_config(dll)
    disp = list(sweep.DISPERSION_FIELDS)
    if measured:
        tight = {"mass_descent_kg": 0.01, "ignition_delay_s": 0.05}
        disp = [(a, tight.get(a, w)) for a, w in disp]
    vc, _d, flight_seed, _i = mc_batch.build_jobs(base, disp, seed, i + 1, [])[i]
    result = run_mod.simulate(vc, mode="land", dll_path=dll, seed=flight_seed)
    return track_from_sim(result, label="MC seed {} flight {}{}".format(
        seed, i, " (measured)" if measured else ""))


def track_from_drop(vz=-1.0, vx=0.0, height=0.02, roll_deg=45.0, wind=0.0):
    """Just the landing: the rocket released with its feet `height` metres
    above the grass (so it touches at the speed you asked for) moving at
    (vx, 0, vz) m/s, legs down, motor out, no flight
    computer -- the same ground physics and verdict as a full flight. A clean
    way to watch what a given touchdown speed does (roll_deg 45 = moving
    straight at the gap between two legs, the weak direction)."""
    import dataclasses
    import landing_outcome
    import plant as plant_mod
    import vehicle
    vc = dataclasses.replace(vehicle.VehicleConfig(), ground_model="legs",
                             wind_mps=wind)
    pl = plant_mod.Plant(vc)
    st = pl.state
    st.ascent_ignite_t = -10.0          # long after launch: motor off
    r = math.radians(roll_deg) / 2.0
    st.q = plant_mod.Q(math.cos(r), 0.0, 0.0, math.sin(r))
    st.pos = plant_mod.V3(0.0, 0.0, height)
    st.vel = plant_mod.V3(vx, 0.0, vz)
    pl.legs_frac = 1.0
    rows = []
    dt = 0.002
    while not st.landed and st.t < 20.0:
        q = st.q
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (q.x * q.x + q.y * q.y)))))
        state = "DESCENT" if st.touchdown_t is None else "TOUCHDOWN"
        rows.append({"t": st.t, "state": state, "h_true": st.pos.z,
                     "v_true": st.vel.z, "x_true": st.pos.x, "y_true": st.pos.y,
                     "quat_true": (q.w, q.x, q.y, q.z), "quat_est": (q.w, q.x, q.y, q.z),
                     "kf_alt": st.pos.z, "tilt_deg": tilt, "legs_frac": 1.0,
                     "thrust_n": 0.0})
        pl.step(dt, 0.0, 0.0)
    summary = {"touchdown_t": st.touchdown_t, "land_ignite_t": 0.0, "hop": False,
               "abort_reason": None, "ground": pl.ground_summary(),
               "touchdown_speed_mps": abs(vz)}
    summary["landing_outcome"] = landing_outcome.classify(summary)
    return track_from_sim({"rows": rows, "events": [], "summary": summary},
                          label="drop test {:+.1f} down, {:+.1f} sideways".format(vz, vx))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", nargs="?",
                    help="a FLIGHTS/flight_NNN.csv (or any file in that format)")
    ap.add_argument("--sim", metavar="SEED:I",
                    help="instead of a log, fly Monte Carlo flight I of SEED "
                         "(the same draws as mc_batch.py / landing_report.py)")
    ap.add_argument("--drop", metavar="VZ[,VX]",
                    help="instead of a flight, just the landing: touching down "
                         "at VZ m/s (negative = down) and VX m/s sideways, "
                         "e.g. --drop -1  (stands)  or  --drop -1,1.5  (tips)")
    ap.add_argument("--measured", action="store_true",
                    help="with --sim: the after-measuring spreads "
                         "(mass +-1 %%, ignition delay +-5 %%)")
    ap.add_argument("--dead-reckon", action="store_true",
                    help="reconstruct downrange position by double-integrating "
                         "accel (drifts -- see logfile.dead_reckon_xy)")
    ap.add_argument("--length", type=float, default=1.2,
                    help="vehicle length in metres, for scale (default 1.2)")
    ap.add_argument("--exaggerate", type=float, default=DEFAULT_EXAGGERATION,
                    help="draw the vehicle this many times oversize so it "
                         "stays visible against the altitude scale")
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed")
    ap.add_argument("--fps", type=int, default=120,
                    help="frame rate cap (default 120; 0 = uncapped)")
    args = ap.parse_args()

    if args.drop:
        v = [float(x) for x in args.drop.split(",")]
        track = track_from_drop(vz=v[0], vx=v[1] if len(v) > 1 else 0.0)
    elif args.sim:
        track = track_from_mc(args.sim, measured=args.measured)
    elif args.log:
        track = track_from_log(args.log, dead_reckon=args.dead_reckon)
    else:
        ap.error("give a log file or --sim SEED:I")
    view = FlightView(track, vehicle_length_m=args.length,
                      exaggeration=args.exaggerate)
    view.playing = True
    view.speed = args.speed

    import pygame
    clock = pygame.time.Clock()
    t = 0.0
    while not view.closed:
        ev = view.pump()
        if ev["closed"]:
            break
        if ev["toggle_play"]:
            view.playing = not view.playing
            if view.playing and t >= track.t_max:
                t = 0.0                      # play again from the start
        if ev["stop"]:
            view.playing, t = False, 0.0
        if ev["step"]:
            view.playing = False
            t = max(0.0, min(track.t_max, t + ev["step"] * STEP_S))
        if ev["speed"] is not None:
            view.speed = ev["speed"]
        if ev["seek"] is not None:
            t = ev["seek"]
            view.playing = False
        dt = clock.tick(args.fps) / 1000.0
        if view.playing:
            t += dt * view.speed
            if t >= track.t_max:
                t, view.playing = track.t_max, False   # hold the final frame
        view.render(t)
    pygame.quit()
    return 0


if __name__ == "__main__":
    sys.exit(main())
