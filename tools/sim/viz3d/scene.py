# VENDORED from SensorServoBenchTest/viz/scene.py on 2026-09-21.
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
World rendering: camera, ground grid, world axis triad.

THE ONE THING THAT MATTERS HERE is the camera's up vector. RocketFC works in a
world frame where +Z is up, so this scene uses gluLookAt(..., up=(0, 0, 1)).
That means the attitude quaternion goes straight into glMultMatrixf with NO
axis remapping anywhere in this program, and the coloured axes you see on
screen are literally the axes named in src/config.h.

Do not "fix" this to the +Y-up convention that three.js and glTF use. Swapping
frames to suit a graphics convention is exactly the class of mistake the bench
tool this came from exists to catch, and doing it here would make it lie.

Flight-viewer differences from the bench original: the world is measured in
METRES and the vehicle MOVES, so the grid takes a cell size/extent and the
axis triad takes the origin to draw from (the pad) rather than reading a
module-level VEHICLE_ORIGIN constant.
"""

import math

from OpenGL.GL import *
from OpenGL.GLU import *

# Defaults suit the bench's ~4-unit vehicle; flight3d.py passes metre-scaled
# values sized to the actual trajectory.
GRID_HALF = 6         # grid extends +/- this many cells
GRID_STEP = 0.5

# Where the vehicle sits when nothing else says otherwise. In the bench tool
# this was the one true vehicle position; here it is only the pad/origin
# fallback, since the flight viewer moves the vehicle every frame.
VEHICLE_ORIGIN = (0.0, 0.0, 0.0)

C_BG = (0.09, 0.10, 0.12, 1.0)
C_GRID = (0.22, 0.24, 0.28)
C_GRID_MAJOR = (0.32, 0.35, 0.40)
C_WORLD_X = (0.85, 0.25, 0.25)
C_WORLD_Y = (0.30, 0.75, 0.35)
C_WORLD_Z = (0.35, 0.55, 0.95)


class Camera:
    """Orbit camera: drag to rotate, wheel to zoom."""

    def __init__(self, target=VEHICLE_ORIGIN, dist=11.5, max_dist=40.0):
        self.az = math.radians(35.0)    # around world Z
        self.el = math.radians(14.0)    # above the horizon
        # Framed so the whole vehicle plus its axis indicators stay in view at
        # any attitude, including nose-down. The flight viewer overrides both
        # target and distance every frame in follow mode.
        self.dist = dist
        self.target = target
        # A flight spans tens of metres, far past the bench's 40-unit cap.
        self.max_dist = max_dist

    def orbit(self, dx_px, dy_px):
        self.az -= dx_px * 0.008
        self.el += dy_px * 0.008
        # Stop just short of the poles: at exactly +/-90 the up vector and the
        # view direction become parallel and gluLookAt degenerates.
        lim = math.radians(89.0)
        self.el = max(-lim, min(lim, self.el))

    def zoom(self, steps):
        self.dist *= math.pow(0.88, steps)
        self.dist = max(1.0, min(self.max_dist, self.dist))

    def eye(self):
        ce = math.cos(self.el)
        return (self.target[0] + self.dist * ce * math.cos(self.az),
                self.target[1] + self.dist * ce * math.sin(self.az),
                self.target[2] + self.dist * math.sin(self.el))

    def apply(self):
        ex, ey, ez = self.eye()
        gluLookAt(ex, ey, ez, self.target[0], self.target[1], self.target[2],
                  0.0, 0.0, 1.0)  # world +Z is up. See the module docstring.


def init_gl(width, height):
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LESS)
    glEnable(GL_NORMALIZE)      # keeps lighting right under the model rotation
    glShadeModel(GL_SMOOTH)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glEnable(GL_LINE_SMOOTH)
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)

    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    # A single directional key light from high and to the side, plus enough
    # ambient that faces turned away are shaded rather than black.
    glLightfv(GL_LIGHT0, GL_POSITION, (0.4, -0.8, 1.0, 0.0))
    glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.95, 0.95, 0.92, 1.0))
    glLightfv(GL_LIGHT0, GL_AMBIENT, (0.0, 0.0, 0.0, 1.0))
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, (0.38, 0.39, 0.42, 1.0))
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE)

    glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, (0.0, 0.0, 0.0, 1.0))
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0)

    glClearColor(*C_BG)
    resize(width, height)


def resize(width, height):
    height = max(1, height)
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, width / float(height), 0.1, 200.0)
    glMatrixMode(GL_MODELVIEW)


def begin_frame(camera):
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    camera.apply()


def draw_grid(half=GRID_HALF, step=GRID_STEP, center=(0.0, 0.0), major_every=4):
    """Ground plane at world Z = 0, so 'up' is unambiguous on screen.

    half/step are in world units (metres, for the flight viewer). `center`
    lets the grid follow the vehicle downrange so it never runs out from
    under a rocket that has drifted, while staying snapped to the step so the
    lines read as ground passing by rather than a mat sliding along with you.
    """
    glDisable(GL_LIGHTING)
    extent = half * step
    cx = round(center[0] / step) * step
    cy = round(center[1] / step) * step
    glBegin(GL_LINES)
    for i in range(-half, half + 1):
        p = i * step
        major = (i % major_every == 0)
        glColor3f(*(C_GRID_MAJOR if major else C_GRID))
        glVertex3f(cx + p, cy - extent, 0.0)
        glVertex3f(cx + p, cy + extent, 0.0)
        glVertex3f(cx - extent, cy + p, 0.0)
        glVertex3f(cx + extent, cy + p, 0.0)
    glEnd()
    glEnable(GL_LIGHTING)


WORLD_AXIS_LEN = 2.6


def draw_world_axes(origin=VEHICLE_ORIGIN, length=WORLD_AXIS_LEN, dropline=True):
    """
    World X / Y / Z triad, drawn from `origin` in the same red/green/blue
    order the body-axis stubs on the model use. Matching hues is intentional:
    with the vehicle upright and level, each body stub lies along its world
    arrow.

    The flight viewer anchors this at the PAD rather than at the vehicle, so
    it doubles as a fixed reference the rocket visibly departs from.
    """
    ox, oy, oz = origin
    glDisable(GL_LIGHTING)
    glLineWidth(2.0)
    glBegin(GL_LINES)
    glColor3f(*C_WORLD_X)
    glVertex3f(ox, oy, oz); glVertex3f(ox + length, oy, oz)
    glColor3f(*C_WORLD_Y)
    glVertex3f(ox, oy, oz); glVertex3f(ox, oy + length, oz)
    glColor3f(*C_WORLD_Z)
    glVertex3f(ox, oy, oz); glVertex3f(ox, oy, oz + length)
    glEnd()

    if dropline and oz != 0.0:
        # A dropline to the ground plane, so height above the grid reads as
        # deliberate rather than as the model floating by accident.
        glColor3f(*C_GRID_MAJOR)
        glLineWidth(1.0)
        glEnable(GL_LINE_STIPPLE)
        glLineStipple(2, 0x5555)
        glBegin(GL_LINES)
        glVertex3f(ox, oy, oz)
        glVertex3f(ox, oy, 0.0)
        glEnd()
        glDisable(GL_LINE_STIPPLE)
    glEnable(GL_LIGHTING)


def draw_dropline(point):
    """Dashed vertical line from `point` down to the ground plane -- the
    flight viewer's read on 'how high is it, and where over the ground'."""
    x, y, z = point
    glDisable(GL_LIGHTING)
    glColor3f(*C_GRID_MAJOR)
    glLineWidth(1.0)
    glEnable(GL_LINE_STIPPLE)
    glLineStipple(2, 0x5555)
    glBegin(GL_LINES)
    glVertex3f(x, y, z)
    glVertex3f(x, y, 0.0)
    glEnd()
    glDisable(GL_LINE_STIPPLE)
    glEnable(GL_LIGHTING)


def world_axis_label_points(origin=VEHICLE_ORIGIN, length=WORLD_AXIS_LEN):
    """3D points where the world axis labels belong, for the 2D overlay pass."""
    ox, oy, oz = origin
    d = length + 0.18
    return [
        ((ox + d, oy, oz), "world +X", C_WORLD_X),
        ((ox, oy + d, oz), "world +Y", C_WORLD_Y),
        ((ox, oy, oz + d), "world +Z  (up)", C_WORLD_Z),
    ]


def project(point):
    """
    Screen position of a world point under the CURRENT matrices, or None if it
    is behind the camera. Used to hang 2D text labels off 3D geometry.

    Returns coordinates with the origin at the TOP-left, matching pygame,
    whereas OpenGL's window origin is bottom-left -- hence the flip.
    """
    model = glGetDoublev(GL_MODELVIEW_MATRIX)
    proj = glGetDoublev(GL_PROJECTION_MATRIX)
    view = glGetIntegerv(GL_VIEWPORT)
    try:
        x, y, z = gluProject(point[0], point[1], point[2], model, proj, view)
    except Exception:
        return None
    if not (0.0 <= z <= 1.0):
        return None
    return (x, view[3] - y)


# ---------------------------------------------------------------------------
# Flight-viewer additions (not in the bench original, which never climbs).
# ---------------------------------------------------------------------------
C_RULER = (0.40, 0.44, 0.52)
C_TRAIL = (0.35, 0.85, 0.55)


def altitude_ruler_ticks(max_alt, step=10.0):
    """Tick heights for the altitude ruler, as a list of metres."""
    if step <= 0 or max_alt <= 0:
        return []
    n = int(max_alt / step) + 1
    return [i * step for i in range(n + 1)]


def draw_altitude_ruler(max_alt, step=10.0, at=(0.0, 0.0), tick_len=1.2):
    """A vertical ruler beside the pad with a tick every `step` metres, so
    altitude is readable off the scene itself and not only off the HUD.
    Returns the (point, text) pairs for the 2D label pass."""
    x, y = at
    ticks = altitude_ruler_ticks(max_alt, step)
    if not ticks:
        return []
    glDisable(GL_LIGHTING)
    glColor3f(*C_RULER)
    glLineWidth(1.0)
    glBegin(GL_LINES)
    glVertex3f(x, y, 0.0)
    glVertex3f(x, y, ticks[-1])
    for h in ticks:
        glVertex3f(x, y, h)
        glVertex3f(x + tick_len, y, h)
    glEnd()
    glEnable(GL_LIGHTING)
    return [((x + tick_len * 1.25, y, h), "{:g} m".format(h)) for h in ticks]


def draw_trail(points, color=C_TRAIL, width=2.0):
    """Where the vehicle has already been, as a line strip in world space."""
    if len(points) < 2:
        return
    glDisable(GL_LIGHTING)
    glColor3f(*color)
    glLineWidth(width)
    glBegin(GL_LINE_STRIP)
    for p in points:
        glVertex3f(p[0], p[1], p[2])
    glEnd()
    glLineWidth(1.0)
    glEnable(GL_LIGHTING)
