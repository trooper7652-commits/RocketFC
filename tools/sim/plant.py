"""6-DOF rigid-body plant for the closed-loop RocketFC simulator.

This is the "real world" the flight computer flies in: true position,
velocity, attitude and rates, advanced by thrust (deflected by the actual --
servo-lagged -- gimbal angle), gravity, aerodynamic drag/normal-force/damping,
and the same ~12 g ground-contact impulse model tools/synth_flight.py uses.
The flight computer never sees this directly; sensors.py samples it into
noisy sensor-frame readings, which are what actually gets stepped through
FlightCore.

Conventions match src/core/quat.h exactly: Hamilton quaternion, w-first,
q.rotate(v) is body->world. Body +Z points toward the NOSE (see the mounting
comment in src/config.h). World frame: z = up, x/y horizontal.

Because everything here is TRUE state (not an estimate), the sim's own
trajectory display can show the honest full 3D path -- the "dead-reckoned,
drifts" caveat in the plan applies to real SD-card logs, which only have
attitude + accel and must reconstruct position; here horizontal position is
ground truth, tracked because a gimbal deflection really does impart sideways
thrust and the vehicle really does drift downrange, same as a real rocket.
"""
import math
import os
import sys

# burn_table.py (the Motor/.eng loader) lives in tools/, one level up from
# tools/sim/ -- reuse it rather than writing a second thrust-curve reader.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import burn_table as bt

RHO = 1.225  # kg/m^3, sea level


# ---------------------------------------------------------------------------
# Minimal quaternion/vector math, ported from src/core/quat.h so the plant's
# rotation kinematics match the flight code's conventions exactly.
# ---------------------------------------------------------------------------
class V3:
    __slots__ = ("x", "y", "z")

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z

    def __add__(self, r):
        return V3(self.x + r.x, self.y + r.y, self.z + r.z)

    def __sub__(self, r):
        return V3(self.x - r.x, self.y - r.y, self.z - r.z)

    def __mul__(self, s):
        return V3(self.x * s, self.y * s, self.z * s)

    def dot(self, r):
        return self.x * r.x + self.y * r.y + self.z * r.z

    def cross(self, r):
        return V3(self.y * r.z - self.z * r.y, self.z * r.x - self.x * r.z,
                  self.x * r.y - self.y * r.x)

    def norm(self):
        return math.sqrt(self.dot(self))

    def as_tuple(self):
        return (self.x, self.y, self.z)


class Q:
    __slots__ = ("w", "x", "y", "z")

    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
        self.w, self.x, self.y, self.z = w, x, y, z

    @staticmethod
    def identity():
        return Q(1.0, 0.0, 0.0, 0.0)

    def normalize(self):
        n = math.sqrt(self.w * self.w + self.x * self.x + self.y * self.y +
                      self.z * self.z)
        if n > 1e-9:
            self.w /= n; self.x /= n; self.y /= n; self.z /= n
        else:
            self.w, self.x, self.y, self.z = 1.0, 0.0, 0.0, 0.0

    def rotate(self, v):
        # body -> world
        qv = V3(self.x, self.y, self.z)
        t = qv.cross(v) * 2.0
        return v + t * self.w + qv.cross(t)

    def rotate_inv(self, v):
        # world -> body
        return Q(self.w, -self.x, -self.y, -self.z).rotate(v)

    def integrate(self, omega_body, dt):
        # Same first-order body-rate integration as Quat::integrate in quat.h.
        wq = Q(0.0, omega_body.x, omega_body.y, omega_body.z)
        dq = Q(
            self.w * wq.w - self.x * wq.x - self.y * wq.y - self.z * wq.z,
            self.w * wq.x + self.x * wq.w + self.y * wq.z - self.z * wq.y,
            self.w * wq.y - self.x * wq.z + self.y * wq.w + self.z * wq.x,
            self.w * wq.z + self.x * wq.y - self.y * wq.x + self.z * wq.w)
        self.w += 0.5 * dt * dq.w
        self.x += 0.5 * dt * dq.x
        self.y += 0.5 * dt * dq.y
        self.z += 0.5 * dt * dq.z
        self.normalize()


class PlantState:
    def __init__(self):
        self.t = 0.0
        self.pos = V3(0.0, 0.0, 0.0)   # world, m (z = up = altitude AGL)
        self.vel = V3(0.0, 0.0, 0.0)   # world, m/s
        self.q = Q.identity()          # body -> world
        self.omega = V3(0.0, 0.0, 0.0)  # body rates, rad/s
        self.specific_force_body = V3(0.0, 0.0, 9.80665)  # what an accelerometer
                                                           # reads at rest (see step())
        self.landed = False
        self.touchdown_t = None
        self.ascent_ignite_t = None
        self.landing_ignite_t = None
        self.ascent_burnout_t = None
        self.apogee_t = None
        self.apogee_h = 0.0


class Plant:
    """Advances PlantState by dt given the current commanded gimbal angles
    (already run through servo.py's rate-limit/lag model by the caller) and
    pyro fire commands from the flight core."""

    def __init__(self, vcfg):
        self.v = vcfg
        self.ascent_motor = bt.Motor(vcfg.ascent_eng_path)
        self.landing_motor = bt.Motor(vcfg.landing_eng_path)
        self.state = PlantState()
        self.chute_deployed = False
        self.chute_open_t = None
        self._area_ref = math.pi * (vcfg.diameter_m * 0.5) ** 2
        self._z_cp_rel_cg = vcfg.cg_from_nose_m - vcfg.cp_from_nose_m
        self._gimbal_arm_m = (vcfg.gimbal_pivot_from_nose_m -
                              vcfg.cg_from_nose_m)
        # Reference thrust the disturbance torque is quoted at (tvc_pid_tuner.py's
        # model): 0 means "auto" -> use this motor's average thrust over its
        # burn, same convention tvc_pid_tuner.py's main() uses.
        avg_thrust = (self.ascent_motor.total_impulse /
                     self.ascent_motor.burn_time)
        self._ref_thrust_n = (vcfg.disturbance_ref_thrust_n
                              if vcfg.disturbance_ref_thrust_n > 0
                              else max(avg_thrust, 1e-3))
        # Pad mass is a DERIVED quantity -- dry (descent) mass plus whatever
        # ascent propellant hasn't burned yet -- not an independent number,
        # even though config.h separately declares MASS_PAD_KG for
        # documentation/sanity-check purposes. Anchor on mass_descent_kg (the
        # constant the landing-burn table is actually built from) plus this
        # motor's real propellant mass, exactly like tools/synth_flight.py's
        # own `m_pad = MASS_DESCENT + ascent.prop_kg` -- so the sim's ascent
        # performance matches the validated replay/synth trajectory instead
        # of silently drifting from it if vcfg.mass_pad_kg (a [MEASURE]
        # placeholder) hasn't been reconciled with the placeholder motor yet.
        self._pad_mass_kg = vcfg.mass_descent_kg + self.ascent_motor.prop_kg

    def ignite_ascent(self, t):
        self.state.ascent_ignite_t = t

    def ignite_landing(self, t):
        self.state.landing_ignite_t = t

    def deploy_chute(self, t):
        if not self.chute_deployed:
            self.chute_deployed = True
            self.chute_open_t = t

    def mass_and_thrust(self, t):
        """Returns (mass_kg, thrust_N) at time t, accounting for whichever
        motor(s) have been ignited and how much propellant has burned."""
        m = self._pad_mass_kg
        thrust = 0.0
        if self.state.ascent_ignite_t is not None:
            tb = t - self.state.ascent_ignite_t
            thrust += self.ascent_motor.thrust(tb)
            m -= self.ascent_motor.mass_burned(tb)
            if (self.state.ascent_burnout_t is None and
                    tb > self.ascent_motor.burn_time):
                self.state.ascent_burnout_t = t
        if self.state.landing_ignite_t is not None:
            tb = t - self.state.landing_ignite_t
            thrust += self.landing_motor.thrust(tb)
            m -= self.landing_motor.mass_burned(tb)
        return max(m, 0.05), thrust

    def _wind(self, t):
        v = self.v
        wx = v.wind_mps
        if v.gust_time_s > 0 and v.gust_time_s <= t < v.gust_time_s + v.gust_dur_s:
            wx += v.gust_mps
        return V3(wx, 0.0, 0.0)

    def step(self, dt, gimbal_x_rad, gimbal_y_rad):
        """Advance one physics tick. gimbal_x_rad/gimbal_y_rad are the ACTUAL
        (servo-modeled) deflections, not the raw command."""
        v = self.v
        s = self.state
        if s.landed:
            return s

        m, thrust = self.mass_and_thrust(s.t)
        # Inertia scales with instantaneous mass (simplification: fixed mass
        # distribution shape as propellant burns -- see vehicle.py's note on
        # inertia_kgm2 for how to refine).
        inertia = v.inertia_kgm2 * (m / self._pad_mass_kg)

        # ---- thrust vector in body frame ----
        # Torque about body X/Y should be directly proportional to
        # gimbalX/Y (see src/core/control.h: "gimbalX() -> torque cmd about
        # body X"). With the gimbal pivot at -L (aft of CG, since body +Z
        # points toward the nose) and r x F giving torque_x = L*Fy,
        # torque_y = -L*Fx, that means Fy = T*sin(gimbalX), Fx = -T*sin(gimbalY).
        fx_body = -thrust * math.sin(gimbal_y_rad)
        fy_body = thrust * math.sin(gimbal_x_rad)
        fz_body = thrust * math.cos(gimbal_x_rad) * math.cos(gimbal_y_rad)
        thrust_body = V3(fx_body, fy_body, fz_body)
        thrust_world = s.q.rotate(thrust_body)

        # ---- aerodynamics ----
        wind = self._wind(s.t)
        v_rel_world = s.vel - wind
        v_rel_body = s.q.rotate_inv(v_rel_world)
        speed = v_rel_body.norm()

        cda = v.cda_m2
        if self.chute_deployed:
            k = min(1.0, (s.t - self.chute_open_t) / max(v.chute_inflation_s, 1e-6))
            cda = v.cda_m2 + k * v.cda_chute_m2
        drag_body_z = -0.5 * RHO * cda * v_rel_body.z * abs(v_rel_body.z)
        drag_body = V3(0.0, 0.0, drag_body_z)
        drag_world = s.q.rotate(drag_body)

        # Aerodynamic normal force at the CP, from the transverse (crossflow)
        # component of relative airspeed -- see vehicle.py's normal_force_coeff
        # docstring: this is what makes a CP-ahead-of-CG (fin-less) airframe
        # aerodynamically unstable and a CP-aft one self-righting, from the
        # SAME formula (only the sign of z_cp_rel_cg differs).
        v_t = V3(v_rel_body.x, v_rel_body.y, 0.0)
        normal_force_body = v_t * (-0.5 * RHO * speed * self._area_ref *
                                   v.normal_force_coeff)
        aero_torque_body = V3(0.0, 0.0, self._z_cp_rel_cg).cross(normal_force_body)
        # Rotational aero damping (opposes body rate directly).
        aero_torque_body = aero_torque_body - s.omega * v.aero_damping_nms

        gravity_world = V3(0.0, 0.0, -9.80665)
        G0 = 9.80665

        # specific_force_world is what an accelerometer actually measures:
        # everything EXCEPT gravity (thrust, drag, and -- while resting on
        # something -- the ground's normal-force reaction). True acceleration
        # is specific force PLUS gravity (matches ahrs.h's own convention:
        # worldVerticalAccel() = rotate(specific_force).z - g0). Tracking
        # this separately from true acceleration matters here specifically
        # because "at rest" means true accel = 0 but specific force = +1 g --
        # exactly what a real accelerometer reads sitting on a table, and
        # what tools/synth_flight.py's own ground-contact comment calls out:
        # the sensor must SEE the pad/impact reaction, not a silent v=0.
        specific_force_world = (thrust_world + drag_world) * (1.0 / m)

        # Rests on the pad indefinitely before ignition (thrust=0, so the
        # ground reaction exactly cancels gravity, i.e. specific force = +g,
        # true acceleration = 0), and for up to 1 s after in case the ascent
        # motor's early thrust briefly can't overcome weight (matches
        # tools/synth_flight.py's on_ground window, but anchored to the
        # actual ignition time rather than sim start -- the pad-sit duration
        # before arming/launch is otherwise unbounded).
        if s.ascent_ignite_t is None:
            on_ground = s.pos.z <= 0.0
        else:
            on_ground = s.pos.z <= 0.0 and s.t < s.ascent_ignite_t + 1.0
        if on_ground and specific_force_world.z + gravity_world.z < 0:
            specific_force_world = V3(specific_force_world.x,
                                      specific_force_world.y, -gravity_world.z)

        # ---- ground contact after touchdown ----
        if s.touchdown_t is not None:
            if s.vel.z < -1e-3:
                # Legs/ground kill the remaining descent velocity at ~12 g;
                # the accelerometer sees that arrest PLUS gravity.
                az = min(-s.vel.z / dt, 12.0 * G0)
                specific_force_world = V3(0.0, 0.0, az - gravity_world.z)
            else:
                specific_force_world = V3(0.0, 0.0, -gravity_world.z)  # resting: +1 g
                s.vel = V3(0.0, 0.0, 0.0)

        accel_world = specific_force_world + gravity_world
        s.specific_force_body = s.q.rotate_inv(specific_force_world)

        s.vel = s.vel + accel_world * dt
        s.pos = s.pos + s.vel * dt
        if (s.pos.z <= 0.0 and s.touchdown_t is None and
                s.ascent_ignite_t is not None and
                s.t > s.ascent_ignite_t + 1.0):
            s.touchdown_t = s.t
            s.pos = V3(s.pos.x, s.pos.y, 0.0)

        if (s.apogee_t is None and s.ascent_ignite_t is not None and
                s.vel.z < 0 and s.t - s.ascent_ignite_t > 1.0):
            s.apogee_t, s.apogee_h = s.t, s.pos.z

        # ---- rotation ----
        # Both disturbance torques are PRODUCED by thrust (misalignment / CG
        # offset / motor-swirl-induced roll), so they scale with it and are
        # exactly zero at rest -- a rocket sitting on the pad with the motor
        # unlit experiences no thrust-coupled disturbance, only aero (still
        # air, so also ~zero) and control (also zero pre-launch, TVC inactive).
        dist_scale = thrust / self._ref_thrust_n
        torque_body = aero_torque_body + V3(
            v.disturbance_torque_nm * dist_scale,
            0.0,
            v.roll_disturbance_torque_nm * dist_scale)
        torque_body = torque_body + V3(
            self._gimbal_arm_m * fy_body, -self._gimbal_arm_m * fx_body, 0.0)
        alpha = torque_body * (1.0 / max(inertia, 1e-6))
        s.omega = s.omega + alpha * dt
        s.q.integrate(s.omega, dt)

        s.t += dt
        if s.touchdown_t is not None and s.t > s.touchdown_t + 0.5 and \
                abs(s.vel.z) < 1e-3:
            s.landed = True
        return s
