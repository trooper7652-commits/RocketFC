# VENDORED from SensorServoBenchTest/viz/model.py on 2026-09-21.
#
# Copied rather than imported across projects so RocketFC stays self-contained
# and clonable on its own (the same reasoning, in the opposite direction, as
# SensorServoBenchTest/sync_core.py vendoring the flight headers into there).
#
# Unlike sync_core.py's copies this is NOT auto-synced and IS edited here:
# the bench rig's vehicle never translates, so the flight viewer needs a
# moving origin and a metres-scaled ground grid. Fixes that belong to both
# should be made in both; the two lineages are expected to drift. They have:
# this copy draws the real fin-less airframe with its landing legs, a
# detachable nose cone and the parachute; the bench copy keeps its four
# axis-marker fins.
"""
The rocket mesh, generated in code.

Built directly in BODY coordinates with the nose along +Z, matching the frame
declared in RocketFC's src/config.h. Nothing here remaps axes, which is the
point: if the model looks wrong on screen, the fault is in the sensor data or
the mounting matrix, never in this file.

The airframe has no fins. Its four landing legs sit on the body ±X and ±Y
axes, so they double as visible markers for the two gimbal axes (the job the
bench copy's fins do). Each leg is three carbon rods meeting at a foot pad,
as built: two from the lower leg ring (hinged just above the tail) and one
strut from the upper leg ring. Stowed they lie against the body pointing at
the nose; they swing down and out to the foot position the ground physics
uses (ground.py), so in the 3D view's true-scale landing close-up the pads
touch the ground exactly where the simulated feet do. The nozzle is a separate display list because it is drawn with the
commanded gimbal deflection applied, letting you see the TVC response in 3D
rather than only as numbers. The nose cone is separate too, so it can be drawn
popped off and hanging on the parachute's shock cord.

Legs and canopy change shape as they deploy. Rebuilding them every frame in
immediate mode would cost more than the rest of the vehicle, so each is
compiled once per quantised deployment step (DEPLOY_STEPS) and cached; a
stowed or fully-deployed vehicle is a single glCallList.

Fixed-function OpenGL and display lists: no shaders, no vertex buffers, nothing
that depends on a modern driver path. This runs anywhere.
"""

import math

from OpenGL.GL import *

# --- proportions (arbitrary units; the camera frames them) -------------------
BODY_R = 0.34
BODY_Z0 = -1.50          # tail
BODY_Z1 = 1.45           # shoulder, where the nose cone starts
NOSE_TIP_Z = 2.55

NOZZLE_Z0 = BODY_Z0
NOZZLE_Z1 = BODY_Z0 - 0.62
NOZZLE_R0 = 0.20
NOZZLE_R1 = 0.32

# Landing legs, one per body ±X/±Y axis: 3 carbon rods each. The lower pair
# hinges on the lower leg ring just above the tail; the strut hangs from the
# upper leg ring. Where the foot ends up (LegGeom) comes from the vehicle's
# leg_foot_radius_m / landed_cg_height_m / leg_top_ring_from_tail_m.
LEG_HINGE_Z = BODY_Z0 + 0.12
LEG_HINGE_R = BODY_R + 0.03
ROD_SPREAD = 0.13        # tangential half-spacing of the lower rod pair at the ring
ROD_R = 0.028
STRUT_R = 0.022
PAD_R = 0.11
PAD_HALF_T = 0.025

# Parachute: 8 gores, alternating colours like a real canopy.
CHUTE_GORES = 8
CHUTE_RINGS = 6

# Deployment shapes are cached per quantised step (see module docstring).
DEPLOY_STEPS = 40

# Body-axis indicator rods. Kept short on purpose: they exist to name the axes,
# and a long +Z rod would dominate the silhouette and force the camera so far
# back that the vehicle itself became small.
AXIS_STUB_LEN = 0.85
AXIS_STUB_Z_LEN = 0.45
AXIS_STUB_R = 0.032

SEGS = 24

# --- colors ------------------------------------------------------------------
C_BODY = (0.82, 0.83, 0.86)
C_BODY_INNER = (0.30, 0.31, 0.34)   # open tube top, seen once the nose is off
C_NOSE = (0.62, 0.66, 0.74)
C_LEG = (0.20, 0.21, 0.23)          # carbon rods
C_LEG_X = (0.86, 0.36, 0.24)        # +X leg tinted the old fin orange: legs mark the axes
C_LEG_BROKEN = (0.92, 0.12, 0.12)   # legs that hit faster than the crash speed
C_PAD = (0.58, 0.59, 0.63)          # light: a dark pad read as a hole
C_NOZZLE = (0.30, 0.31, 0.34)
C_AXIS_X = (0.90, 0.30, 0.30)
C_AXIS_Y = (0.35, 0.80, 0.40)
C_AXIS_Z = (0.40, 0.60, 0.95)
C_GHOST = (0.45, 0.75, 1.00)
C_CANOPY_A = (0.95, 0.95, 0.97)
C_CANOPY_B = (0.93, 0.42, 0.20)
C_LINES = (0.20, 0.21, 0.24)


def _set_color(rgb, alpha):
    r, g, b = rgb
    glColor4f(r, g, b, alpha)
    # Fixed-function lighting ignores glColor unless COLOR_MATERIAL is on, which
    # scene.py enables; the specular term is set once there.
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, (r, g, b, alpha))


def _frustum(z0, z1, r0, r1, segs=SEGS):
    """Tapered tube side wall with correct outward normals."""
    dz, dr = z1 - z0, r1 - r0
    ln = math.hypot(dz, dr)
    if ln < 1e-9:
        return
    nr, nz = dz / ln, -dr / ln  # outward normal in the (radial, z) plane
    glBegin(GL_QUAD_STRIP)
    for i in range(segs + 1):
        a = 2.0 * math.pi * i / segs
        ca, sa = math.cos(a), math.sin(a)
        glNormal3f(nr * ca, nr * sa, nz)
        glVertex3f(r0 * ca, r0 * sa, z0)
        glVertex3f(r1 * ca, r1 * sa, z1)
    glEnd()


def _disc(z, r, nz, segs=SEGS):
    """Flat cap facing along nz (+1 or -1)."""
    glBegin(GL_TRIANGLE_FAN)
    glNormal3f(0, 0, nz)
    glVertex3f(0, 0, z)
    rng = range(segs + 1) if nz < 0 else range(segs, -1, -1)
    for i in rng:
        a = 2.0 * math.pi * i / segs
        glVertex3f(r * math.cos(a), r * math.sin(a), z)
    glEnd()


def _quad(p0, p1, p2, p3):
    """One flat quad, normal derived from its own winding."""
    ux, uy, uz = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    vx, vy, vz = (p3[0] - p0[0], p3[1] - p0[1], p3[2] - p0[2])
    nx, ny, nz = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
    ln = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
    glBegin(GL_QUADS)
    glNormal3f(nx / ln, ny / ln, nz / ln)
    for p in (p0, p1, p2, p3):
        glVertex3f(*p)
    glEnd()


def _align_z(v):
    """glRotatef so the local +Z axis points along v (any length)."""
    vx, vy, vz = v
    ln = math.sqrt(vx * vx + vy * vy + vz * vz)
    if ln < 1e-12:
        return 0.0
    c = max(-1.0, min(1.0, vz / ln))
    ax, ay = -vy, vx                  # +Z x v
    if ax * ax + ay * ay < 1e-18:
        if c < 0:
            glRotatef(180.0, 1, 0, 0)
    else:
        glRotatef(math.degrees(math.acos(c)), ax, ay, 0.0)
    return ln


def _rod(p0, p1, r, segs=8, caps=False):
    """A round rod from p0 to p1 (body coordinates)."""
    glPushMatrix()
    glTranslatef(*p0)
    ln = _align_z((p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]))
    _frustum(0.0, ln, r, r, segs)
    if caps:
        _disc(0.0, r, -1.0, segs)
        _disc(ln, r, 1.0, segs)
    glPopMatrix()


def _axis_stub(color, alpha, length=AXIS_STUB_LEN):
    """A short rod along +Z; the caller rotates it onto the wanted body axis."""
    _set_color(color, alpha)
    _frustum(0.0, length, AXIS_STUB_R, AXIS_STUB_R, 10)
    _disc(length, AXIS_STUB_R, 1.0, 10)
    # A little cone tip so the positive direction is unmistakable.
    _frustum(length, length + 0.13, AXIS_STUB_R * 2.4, 0.0, 10)
    _disc(length, AXIS_STUB_R * 2.4, -1.0, 10)


def _emit_body(alpha, uniform=None, with_axes=True):
    """Body tube and (optionally) the X/Y body-axis stubs. The nose cone, the
    +Z stub and the legs are separate so they can move independently."""
    body = uniform or C_BODY

    _set_color(body, alpha)
    _frustum(BODY_Z0, BODY_Z1, BODY_R, BODY_R)
    _disc(BODY_Z0, BODY_R, -1.0)
    # Top of the tube: hidden under the nose cone, seen once it pops off.
    _set_color(uniform or C_BODY_INNER, alpha)
    _disc(BODY_Z1, BODY_R, 1.0)

    if with_axes and uniform is None:
        # +X stub: rotate +Z onto +X.
        glPushMatrix()
        glRotatef(90, 0, 1, 0)
        _axis_stub(C_AXIS_X, alpha)
        glPopMatrix()
        # +Y stub: rotate +Z onto +Y.
        glPushMatrix()
        glRotatef(-90, 1, 0, 0)
        _axis_stub(C_AXIS_Y, alpha)
        glPopMatrix()


def _emit_nose(alpha, uniform=None):
    _set_color(uniform or C_NOSE, alpha)
    _frustum(BODY_Z1, NOSE_TIP_Z, BODY_R, 0.0)
    _disc(BODY_Z1, BODY_R, -1.0)        # its base, seen once it is off


def _emit_nozzle(alpha, uniform=None, z1=NOZZLE_Z1):
    _set_color(uniform or C_NOZZLE, alpha)
    _frustum(NOZZLE_Z0, z1, NOZZLE_R0, NOZZLE_R1)
    _disc(z1, NOZZLE_R1, -1.0)


class LegGeom:
    """Deployed foot position in model units, from the vehicle in metres.

    m_per_unit is the model's true scale (vehicle length / model length);
    the CG sits cg_from_nose_m below the nose tip. The foot stands
    landed_cg_height_m below the CG and leg_foot_radius_m out."""

    def __init__(self, foot_r_m=0.25, cg_h_m=0.60, top_ring_m=0.25,
                 cg_from_nose_m=0.66, m_per_unit=1.2 / 4.05):
        u = m_per_unit
        self.cg_z = NOSE_TIP_Z - cg_from_nose_m / u
        self.foot = (foot_r_m / u, self.cg_z - cg_h_m / u)
        self.top_z = BODY_Z0 + top_ring_m / u
        dx, dz = self.foot[0] - LEG_HINGE_R, self.foot[1] - LEG_HINGE_Z
        self.length = math.hypot(dx, dz)
        self.deploy = math.atan2(dx, dz)   # swing from stowed (+Z) to the foot
        self.key = (round(foot_r_m, 4), round(cg_h_m, 4), round(top_ring_m, 4),
                    round(cg_from_nose_m, 4), round(u, 5))


def _emit_legs(frac, geom, broken=False):
    """All four legs at deployment fraction `frac` (0 stowed .. 1 down)."""
    th = geom.deploy * max(0.0, min(1.0, frac))
    st, ct = math.sin(th), math.cos(th)
    L = geom.length
    foot = (LEG_HINGE_R + L * st, 0.0, LEG_HINGE_Z + L * ct)
    # The pad turns from flat against the skin (normal +X) to level (normal
    # -Z) over the swing, so it neither sticks out stowed nor lands tilted.
    ph = math.radians(90.0 * max(0.0, min(1.0, frac)))
    pad_n = (math.cos(ph), 0.0, -math.sin(ph))
    pad_c = (foot[0] + pad_n[0] * PAD_HALF_T, 0.0, foot[2] + pad_n[2] * PAD_HALF_T)
    # A broken leg's lower rods buckle outward at mid-length.
    kink = 0.10 if broken else 0.0

    for k in range(4):
        glPushMatrix()
        glRotatef(90.0 * k, 0, 0, 1)
        _set_color(C_LEG_BROKEN if broken else (C_LEG_X if k == 0 else C_LEG), 1.0)
        for side in (-1.0, 1.0):
            base = (LEG_HINGE_R, side * ROD_SPREAD, LEG_HINGE_Z)
            if kink:
                mid = (0.5 * (base[0] + foot[0]) + kink, 0.5 * base[1],
                       0.5 * (base[2] + foot[2]))
                _rod(base, mid, ROD_R)
                _rod(mid, foot, ROD_R)
            else:
                _rod(base, foot, ROD_R)
        _rod((BODY_R, 0.0, geom.top_z), foot, STRUT_R)

        _set_color(C_PAD, 1.0)
        _rod((pad_c[0] - pad_n[0] * PAD_HALF_T, 0.0, pad_c[2] - pad_n[2] * PAD_HALF_T),
             (pad_c[0] + pad_n[0] * PAD_HALF_T, 0.0, pad_c[2] + pad_n[2] * PAD_HALF_T),
             PAD_R, segs=14, caps=True)
        glPopMatrix()


def _emit_chute(frac, radius, line_len):
    """Canopy + shroud lines in their own frame: origin at the point where
    the lines meet the shock cord, +Z up. Sizes are in the caller's units.

    Partly inflated it is a narrow, tall bag; fully open a shallow dome --
    the shape change is what reads as "opening" at a glance.
    """
    f = max(0.0, min(1.0, frac))
    r = radius * (0.18 + 0.82 * f)
    h = radius * (1.6 - 0.9 * f)
    z0 = line_len
    rim = [(r * math.cos(2.0 * math.pi * g / CHUTE_GORES),
            r * math.sin(2.0 * math.pi * g / CHUTE_GORES))
           for g in range(CHUTE_GORES + 1)]

    # Gores, one colour each, built ring by ring from the skirt to the apex.
    for g in range(CHUTE_GORES):
        _set_color(C_CANOPY_A if g % 2 == 0 else C_CANOPY_B, 0.95)
        a0 = 2.0 * math.pi * g / CHUTE_GORES
        a1 = 2.0 * math.pi * (g + 1) / CHUTE_GORES
        glBegin(GL_QUAD_STRIP)
        for j in range(CHUTE_RINGS + 1):
            phi = 0.5 * math.pi * j / CHUTE_RINGS
            cr, z = r * math.cos(phi), z0 + h * math.sin(phi)
            # Ellipsoid normal: (cos phi / r, sin phi / h), radial then up.
            nr, nz = math.cos(phi) / max(r, 1e-6), math.sin(phi) / max(h, 1e-6)
            nl = math.hypot(nr, nz) or 1.0
            for a in (a0, a1):
                glNormal3f(nr / nl * math.cos(a), nr / nl * math.sin(a), nz / nl)
                glVertex3f(cr * math.cos(a), cr * math.sin(a), z)
        glEnd()

    # Shroud lines from every gore seam down to the confluence point.
    glDisable(GL_LIGHTING)
    glColor4f(C_LINES[0], C_LINES[1], C_LINES[2], 1.0)
    glLineWidth(1.5)
    glBegin(GL_LINES)
    for x, y in rim[:-1]:
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(x, y, z0)
    glEnd()
    glEnable(GL_LIGHTING)


class RocketModel:
    """Compiled display lists. Build once, after the GL context exists."""

    def __init__(self, leg_geom=None):
        self._solid = glGenLists(1)
        glNewList(self._solid, GL_COMPILE)
        _emit_body(1.0)
        glEndList()

        self._nose = glGenLists(1)
        glNewList(self._nose, GL_COMPILE)
        _emit_nose(1.0)
        glEndList()

        self._zstub = glGenLists(1)
        glNewList(self._zstub, GL_COMPILE)
        _axis_stub(C_AXIS_Z, 1.0, AXIS_STUB_Z_LEN)
        glEndList()

        self._ghost = glGenLists(1)
        glNewList(self._ghost, GL_COMPILE)
        _emit_body(0.30, uniform=C_GHOST, with_axes=False)
        _emit_nose(0.30, uniform=C_GHOST)
        glEndList()

        self._nozzle = glGenLists(1)
        self._legs = {}     # (quantised deployment step, broken) -> display list
        self._chutes = {}   # (step, radius, line_len) -> display list
        self.leg_geom = None
        self.set_leg_geom(leg_geom or LegGeom())

    def set_leg_geom(self, geom):
        """Re-shape the legs (and fit the nozzle above the feet) for a vehicle
        with different leg dimensions; a no-op when nothing changed."""
        if self.leg_geom is not None and self.leg_geom.key == geom.key:
            return
        self.leg_geom = geom
        for lst in self._legs.values():
            glDeleteLists(lst, 1)
        self._legs = {}
        # The drawn nozzle is longer than the real one; never let it reach
        # below the feet, or a standing rocket would show it through the ground.
        self.nozzle_z1 = max(NOZZLE_Z1, geom.foot[1] + 0.06)
        glNewList(self._nozzle, GL_COMPILE)
        _emit_nozzle(1.0, z1=self.nozzle_z1)
        glEndList()

    @staticmethod
    def _step(frac):
        return int(round(max(0.0, min(1.0, frac)) * DEPLOY_STEPS))

    def _cached(self, cache, key, emit):
        lst = cache.get(key)
        if lst is None:
            lst = glGenLists(1)
            glNewList(lst, GL_COMPILE)
            emit()
            glEndList()
            cache[key] = lst
        return lst

    def draw_solid(self, gimx_deg=0.0, gimy_deg=0.0, legs_frac=0.0, nose_on=True,
                   legs_broken=False):
        glCallList(self._solid)
        if nose_on:
            glCallList(self._nose)
        # The +Z stub always names the body axis from the top of whatever is
        # there: the nose tip, or the open tube once the nose is off.
        glPushMatrix()
        glTranslatef(0, 0, (NOSE_TIP_Z if nose_on else BODY_Z1) - 0.05)
        glCallList(self._zstub)
        glPopMatrix()

        q = self._step(legs_frac)
        g = self.leg_geom
        glCallList(self._cached(self._legs, (q, legs_broken),
                                lambda: _emit_legs(q / DEPLOY_STEPS, g, legs_broken)))

        glPushMatrix()
        # The gimbal pivots at the tail. gimbalX is a torque command about body
        # X, which is produced by deflecting the nozzle about that same axis.
        glTranslatef(0, 0, NOZZLE_Z0)
        glRotatef(gimx_deg, 1, 0, 0)
        glRotatef(gimy_deg, 0, 1, 0)
        glTranslatef(0, 0, -NOZZLE_Z0)
        glCallList(self._nozzle)
        glPopMatrix()

    def draw_nose(self):
        """The nose cone alone, in body units with its base at BODY_Z1 --
        for drawing it popped off on the shock cord."""
        glCallList(self._nose)

    def draw_chute(self, frac, radius, line_len):
        """Canopy + shroud lines; origin at the line confluence, +Z up."""
        q = self._step(frac)
        key = (q, round(radius, 4), round(line_len, 4))
        glCallList(self._cached(
            self._chutes, key,
            lambda: _emit_chute(q / DEPLOY_STEPS, radius, line_len)))

    def draw_ghost(self):
        glCallList(self._ghost)
