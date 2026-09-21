"""Gimbal servo model: commanded angle -> actual angle, with the same
rate-limit + first-order-lag model as ../../tvc-sim/tvc_pid_tuner.py's
ServoModel (ported rather than reimplemented, so both tools agree on what a
"realistic servo" means).

The linkage mapping (SERVO_A/B_US_PER_GIMBAL_DEG etc.) is LINEAR, so a servo
rate limit / lag expressed in gimbal-radians is equivalent to the same limit
expressed in microseconds -- applying it here, before the microsecond
conversion, means one model serves both the physics (torque needs the actual
gimbal angle) and the logging/display (actual servo microseconds, via the
bridged toUs() mapping in core.FlightCore).
"""


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class ServoModel:
    """One gimbal axis: rate-limited, lagged, mechanically clamped.

    slew_rad_s: max |d(angle)/dt| (rad/s) -- the servo's own speed derated for
        the gimbal linkage. Primary source of TVC instability if too slow.
    tau_s: first-order lag time constant (s). 0 disables lag.
    max_rad: mechanical gimbal travel limit (+/-), independent of the
        firmware's own GIMBAL_MAX_RAD software clamp (this is the real stop).
    """

    def __init__(self, slew_rad_s, tau_s, max_rad, dt):
        self.slew = slew_rad_s
        self.tau = tau_s
        self.max_rad = max_rad
        self.dt = dt
        self.angle = 0.0  # current ACTUAL gimbal angle (rad)

    def reset(self, angle=0.0):
        self.angle = angle

    def step(self, commanded_rad):
        cmd = clamp(commanded_rad, -self.max_rad, self.max_rad)
        max_move = self.slew * self.dt
        target = clamp(cmd, self.angle - max_move, self.angle + max_move)
        if self.tau > 1e-9:
            self.angle += (target - self.angle) * (self.dt / self.tau)
        else:
            self.angle = target
        self.angle = clamp(self.angle, -self.max_rad, self.max_rad)
        return self.angle


class GimbalServos:
    """Both gimbal axes (A/X, B/Y) as one convenience wrapper."""

    def __init__(self, slew_rad_s, tau_s, max_rad, dt):
        self.x = ServoModel(slew_rad_s, tau_s, max_rad, dt)
        self.y = ServoModel(slew_rad_s, tau_s, max_rad, dt)

    def reset(self):
        self.x.reset()
        self.y.reset()

    def step(self, cmd_x_rad, cmd_y_rad):
        return self.x.step(cmd_x_rad), self.y.step(cmd_y_rad)
