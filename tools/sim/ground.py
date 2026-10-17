"""Ground contact for the landing: legs, grass, and a body that can fall over.

Used by plant.py when VehicleConfig.ground_model == "legs" (the default);
"arrest" keeps the old fixed ~12 g vertical stop and never tips.

The rocket touches the ground only through a handful of points fixed in the
body frame (all relative to the CG, body +Z toward the nose):

  - 4 feet on the gimbal axes, leg_foot_radius_m out from the centreline and
    landed_cg_height_m below the CG -- so standing upright, the feet are
    exactly on the ground. Each foot is where one leg's three carbon rods meet.
    Feet only carry load once the legs are fully swung down (legs_frac >= 1):
    a leg that hasn't locked folds instead of holding the rocket up.
  - a ring of points around the tail, so a rocket whose legs aren't down
    lands on its tail edge (and falls over, as it would for real);
  - two rings along the body plus the nose tip, so a rocket that falls over
    comes to rest lying on its side instead of sinking through the ground.

Each point that is below the ground gets a spring-damper push back out along
the ground's normal and a friction force along the ground, capped at mu * N.
Friction is smoothed below FRICTION_V0 so a resting foot doesn't chatter.
Everything is in plain floats -- this runs at 4 substeps per 2 ms tick for
every landing, and V3 objects would dominate the cost.

The ground is a plane through z = 0 at the landing spot, tilted by a small
random slope (set_slope), the grass-field slope the user chose to model.
"""
import math

FRICTION_V0 = 0.02  # m/s: below this slip speed friction ramps linearly


def rot_matrix(q):
    """Body->world rotation matrix rows from a w-first quaternion (same
    convention as plant.Q.rotate)."""
    w, x, y, z = q.w, q.x, q.y, q.z
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)),
        (2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)),
        (2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)),
    )


class Ground:
    def __init__(self, vcfg):
        self.k = vcfg.ground_k_npm
        self.c = vcfg.ground_c_nspm
        self.mu = vcfg.ground_mu
        self.foot_r = vcfg.leg_foot_radius_m
        self.cg_h = vcfg.landed_cg_height_m
        rb = 0.5 * vcfg.diameter_m
        tail_z = -(vcfg.length_m - vcfg.cg_from_nose_m)
        nose_z = vcfg.cg_from_nose_m
        self.tail_z, self.nose_z, self.body_r = tail_z, nose_z, rb

        h = self.cg_h
        r = self.foot_r
        self.feet = [(r, 0.0, -h), (0.0, r, -h), (-r, 0.0, -h), (0.0, -r, -h)]
        body = []
        # Body stations: the tail, two along the tube, and the nose shoulder
        # region; 8 points around each.
        for z in (tail_z, tail_z + 0.4 * (nose_z - tail_z),
                  tail_z + 0.75 * (nose_z - tail_z)):
            for k in range(8):
                a = k * math.pi / 4.0
                body.append((rb * math.cos(a), rb * math.sin(a), z))
        body.append((0.0, 0.0, nose_z))
        self.body = body
        # Farthest any point can be from the CG: below this CG height nothing
        # can be touching, so the plant skips contact entirely in flight.
        self.reach = max(math.sqrt(px * px + py * py + pz * pz)
                         for px, py, pz in self.feet + self.body)
        self.set_slope(0.0, 0.0)
        self.anchor = (0.0, 0.0)

    def set_slope(self, slope_deg, dir_deg):
        """Tilt the ground plane down toward compass direction dir_deg."""
        self.slope_deg = slope_deg
        self.slope_dir_deg = dir_deg
        s, d = math.radians(slope_deg), math.radians(dir_deg)
        # Unit normal leaning toward the downhill direction.
        self.n = (math.sin(s) * math.cos(d), math.sin(s) * math.sin(d), math.cos(s))

    def height_of(self, px, py, pz):
        """Signed height of a world point above the ground plane."""
        nx, ny, nz = self.n
        ax, ay = self.anchor
        return nx * (px - ax) + ny * (py - ay) + nz * pz

    def contact(self, cg, vel, R, w_world, feet_active):
        """Total contact force (world, N) and torque about the CG (world, N*m)
        for the body at CG position `cg` with CG velocity `vel` (tuples),
        rotation matrix R and world angular rate w_world.

        Returns (fx, fy, fz, tx, ty, tz, min_height, touching)."""
        nx, ny, nz = self.n
        ax, ay = self.anchor
        cx, cy, cz = cg
        vx, vy, vz = vel
        wx, wy, wz = w_world
        r0, r1, r2 = R
        k, c, mu = self.k, self.c, self.mu
        fx = fy = fz = tx = ty = tz = 0.0
        min_h = 1e9
        touching = False
        pts = self.feet + self.body if feet_active else self.body
        for px, py, pz in pts:
            # World offset of the point from the CG.
            ox = r0[0] * px + r0[1] * py + r0[2] * pz
            oy = r1[0] * px + r1[1] * py + r1[2] * pz
            oz = r2[0] * px + r2[1] * py + r2[2] * pz
            hgt = nx * (cx + ox - ax) + ny * (cy + oy - ay) + nz * (cz + oz)
            if hgt < min_h:
                min_h = hgt
            if hgt >= 0.0:
                continue
            # Point velocity = CG velocity + w x offset.
            pvx = vx + wy * oz - wz * oy
            pvy = vy + wz * ox - wx * oz
            pvz = vz + wx * oy - wy * ox
            vn = pvx * nx + pvy * ny + pvz * nz
            fn = -k * hgt - c * vn
            if fn <= 0.0:
                continue  # the ground pushes, never pulls
            touching = True
            # Slip velocity along the ground, and friction against it.
            sx, sy, sz = pvx - vn * nx, pvy - vn * ny, pvz - vn * nz
            slip = math.sqrt(sx * sx + sy * sy + sz * sz)
            if slip > 1e-9:
                ff = mu * fn * min(1.0, slip / FRICTION_V0) / slip
                gx, gy, gz = -ff * sx, -ff * sy, -ff * sz
            else:
                gx = gy = gz = 0.0
            px_, py_, pz_ = fn * nx + gx, fn * ny + gy, fn * nz + gz
            fx += px_; fy += py_; fz += pz_
            tx += oy * pz_ - oz * py_
            ty += oz * px_ - ox * pz_
            tz += ox * py_ - oy * px_
        return fx, fy, fz, tx, ty, tz, min_h, touching


def side_drag(vcfg, rel_vel_world, axis_world):
    """Crossflow drag on the standing/falling body from the wind: the part of
    the relative air velocity perpendicular to the body axis, on the whole
    side area (length x diameter). Only used after ground contact -- in flight
    the plant's slender-body normal-force model already covers this.
    Returns the world force (tuple)."""
    rx, ry, rz = rel_vel_world
    ex, ey, ez = axis_world
    along = rx * ex + ry * ey + rz * ez
    px, py, pz = rx - along * ex, ry - along * ey, rz - along * ez
    sp = math.sqrt(px * px + py * py + pz * pz)
    k = -0.5 * 1.225 * vcfg.crossflow_cd * vcfg.length_m * vcfg.diameter_m * sp
    return (k * px, k * py, k * pz)
