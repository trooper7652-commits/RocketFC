# VENDORED from SensorServoBenchTest/viz/model.py on 2026-09-21.
#
# Copied rather than imported across projects so RocketFC stays self-contained
# and clonable on its own (the same reasoning, in the opposite direction, as
# SensorServoBenchTest/sync_core.py vendoring the flight headers into there).
#
# Unlike sync_core.py's copies this is NOT auto-synced and IS edited here:
# the bench rig's vehicle never translates, so the flight viewer needs a
# moving origin and a metres-scaled ground grid. Fixes that belong to both
# should be made in both; the two lineages are expected to drift.
"""
The rocket mesh, generated in code.

Built directly in BODY coordinates with the nose along +Z, matching the frame
declared in RocketFC's src/config.h. Nothing here remaps axes, which is the
point: if the model looks wrong on screen, the fault is in the sensor data or
the mounting matrix, never in this file.

The four fins sit on the body ±X and ±Y axes, so they double as visible markers
for the two gimbal axes. The nozzle is a separate display list because it is
drawn with the commanded gimbal deflection applied, letting you see the TVC
response in 3D rather than only as numbers.

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

FIN_ROOT_Z0 = BODY_Z0    # fin root runs from the tail forward
FIN_ROOT_Z1 = -0.35
FIN_TIP_R = 0.86
FIN_TIP_Z0 = BODY_Z0
FIN_TIP_Z1 = -1.02       # swept leading edge
FIN_HALF_T = 0.025

NOZZLE_Z0 = BODY_Z0
NOZZLE_Z1 = BODY_Z0 - 0.62
NOZZLE_R0 = 0.20
NOZZLE_R1 = 0.32

# Body-axis indicator rods. Kept short on purpose: they exist to name the axes,
# and a long +Z rod would dominate the silhouette and force the camera so far
# back that the vehicle itself became small.
AXIS_STUB_LEN = 0.85
AXIS_STUB_Z_LEN = 0.45
AXIS_STUB_R = 0.032

SEGS = 24

# --- colors ------------------------------------------------------------------
C_BODY = (0.82, 0.83, 0.86)
C_NOSE = (0.62, 0.66, 0.74)
C_FIN = (0.86, 0.36, 0.24)
C_NOZZLE = (0.30, 0.31, 0.34)
C_AXIS_X = (0.90, 0.30, 0.30)
C_AXIS_Y = (0.35, 0.80, 0.40)
C_AXIS_Z = (0.40, 0.60, 0.95)
C_GHOST = (0.45, 0.75, 1.00)


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


def _fin():
    """
    One fin lying in the body XZ plane, extruded to a thin plate.

    Drawn in the +X direction; the caller rotates it about Z for the other
    three. Profile, from the tail forward: root leading edge sweeps out and
    back to the tip.
    """
    t = FIN_HALF_T
    # Planform corners, as (radius, z).
    prof = [
        (BODY_R, FIN_ROOT_Z0),
        (BODY_R, FIN_ROOT_Z1),
        (FIN_TIP_R, FIN_TIP_Z1),
        (FIN_TIP_R, FIN_TIP_Z0),
    ]
    near = [(r, -t, z) for r, z in prof]
    far = [(r, t, z) for r, z in prof]

    _quad(*[(x, y, z) for x, y, z in reversed(near)])  # -Y face
    _quad(*far)                                        # +Y face
    # Edge band around the planform.
    for i in range(4):
        j = (i + 1) % 4
        _quad(near[i], near[j], far[j], far[i])


def _axis_stub(color, alpha, length=AXIS_STUB_LEN):
    """A short rod along +Z; the caller rotates it onto the wanted body axis."""
    _set_color(color, alpha)
    _frustum(0.0, length, AXIS_STUB_R, AXIS_STUB_R, 10)
    _disc(length, AXIS_STUB_R, 1.0, 10)
    # A little cone tip so the positive direction is unmistakable.
    _frustum(length, length + 0.13, AXIS_STUB_R * 2.4, 0.0, 10)
    _disc(length, AXIS_STUB_R * 2.4, -1.0, 10)


def _emit_body(alpha, uniform=None, with_axes=True):
    """Body tube, nose cone, fins and (optionally) the body-axis stubs."""
    body = uniform or C_BODY
    nose = uniform or C_NOSE
    fin = uniform or C_FIN

    _set_color(body, alpha)
    _frustum(BODY_Z0, BODY_Z1, BODY_R, BODY_R)
    _disc(BODY_Z0, BODY_R, -1.0)

    _set_color(nose, alpha)
    _frustum(BODY_Z1, NOSE_TIP_Z, BODY_R, 0.0)

    _set_color(fin, alpha)
    for i in range(4):
        glPushMatrix()
        glRotatef(90.0 * i, 0, 0, 1)
        _fin()
        glPopMatrix()

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
        # +Z stub, continuing straight out of the nose.
        glPushMatrix()
        glTranslatef(0, 0, NOSE_TIP_Z - 0.05)
        _axis_stub(C_AXIS_Z, alpha, AXIS_STUB_Z_LEN)
        glPopMatrix()


def _emit_nozzle(alpha, uniform=None):
    _set_color(uniform or C_NOZZLE, alpha)
    _frustum(NOZZLE_Z0, NOZZLE_Z1, NOZZLE_R0, NOZZLE_R1)
    _disc(NOZZLE_Z1, NOZZLE_R1, -1.0)


class RocketModel:
    """Compiled display lists. Build once, after the GL context exists."""

    def __init__(self):
        self._solid = glGenLists(1)
        glNewList(self._solid, GL_COMPILE)
        _emit_body(1.0)
        glEndList()

        self._ghost = glGenLists(1)
        glNewList(self._ghost, GL_COMPILE)
        _emit_body(0.30, uniform=C_GHOST, with_axes=False)
        glEndList()

        self._nozzle = glGenLists(1)
        glNewList(self._nozzle, GL_COMPILE)
        _emit_nozzle(1.0)
        glEndList()

    def draw_solid(self, gimx_deg=0.0, gimy_deg=0.0):
        glCallList(self._solid)
        glPushMatrix()
        # The gimbal pivots at the tail. gimbalX is a torque command about body
        # X, which is produced by deflecting the nozzle about that same axis.
        glTranslatef(0, 0, NOZZLE_Z0)
        glRotatef(gimx_deg, 1, 0, 0)
        glRotatef(gimy_deg, 0, 1, 0)
        glTranslatef(0, 0, -NOZZLE_Z0)
        glCallList(self._nozzle)
        glPopMatrix()

    def draw_ghost(self):
        glCallList(self._ghost)
