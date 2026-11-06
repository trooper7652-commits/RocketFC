"""
World rendering: camera, ground grid, world axis triad.

THE ONE THING THAT MATTERS HERE is the camera's up vector. RocketFC works in a
world frame where +Z is up, so this scene uses gluLookAt(..., up=(0, 0, 1)).
That means the attitude quaternion goes straight into glMultMatrixf with NO
axis remapping anywhere in this program, and the coloured axes you see on
screen are literally the axes named in src/config.h.

Do not "fix" this to the +Y-up convention that three.js and glTF use. Swapping
frames to suit a graphics convention is exactly the class of mistake this bench
tool exists to catch, and doing it here would make the tool lie.
"""

import math

from OpenGL.GL import *
from OpenGL.GLU import *

GRID_HALF = 6         # grid extends +/- this many cells
GRID_STEP = 0.5

# The vehicle is drawn floating above the ground plane, rotating about this
# point, and the world axis triad is drawn from the SAME point. Sharing an
# origin is the whole reason the triad is there: with the rocket upright and
# level, each coloured body stub lies along its matching world arrow, and any
# mismatch is visible directly rather than by mental arithmetic.
VEHICLE_ORIGIN = (0.0, 0.0, 2.0)

C_BG = (0.09, 0.10, 0.12, 1.0)
C_GRID = (0.22, 0.24, 0.28)
C_GRID_MAJOR = (0.32, 0.35, 0.40)
C_WORLD_X = (0.85, 0.25, 0.25)
C_WORLD_Y = (0.30, 0.75, 0.35)
C_WORLD_Z = (0.35, 0.55, 0.95)


class Camera:
    """Orbit camera: drag to rotate, wheel to zoom."""

    def __init__(self):
        self.az = math.radians(35.0)    # around world Z
        self.el = math.radians(14.0)    # above the horizon
        # Framed so the whole vehicle plus its axis indicators stay in view at
        # any attitude, including nose-down.
        self.dist = 11.5
        self.target = VEHICLE_ORIGIN

    def orbit(self, dx_px, dy_px):
        self.az -= dx_px * 0.008
        self.el += dy_px * 0.008
        # Stop just short of the poles: at exactly +/-90 the up vector and the
        # view direction become parallel and gluLookAt degenerates.
        lim = math.radians(89.0)
        self.el = max(-lim, min(lim, self.el))

    def zoom(self, steps):
        self.dist *= math.pow(0.88, steps)
        self.dist = max(2.5, min(40.0, self.dist))

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


def draw_grid():
    """Ground plane at world Z = 0, so 'up' is unambiguous on screen."""
    glDisable(GL_LIGHTING)
    extent = GRID_HALF * GRID_STEP
    glBegin(GL_LINES)
    for i in range(-GRID_HALF, GRID_HALF + 1):
        p = i * GRID_STEP
        major = (i % 4 == 0)
        glColor3f(*(C_GRID_MAJOR if major else C_GRID))
        glVertex3f(p, -extent, 0.0)
        glVertex3f(p, extent, 0.0)
        glVertex3f(-extent, p, 0.0)
        glVertex3f(extent, p, 0.0)
    glEnd()
    glEnable(GL_LIGHTING)


WORLD_AXIS_LEN = 2.6


def draw_world_axes():
    """
    World X / Y / Z triad, drawn from VEHICLE_ORIGIN in the same red/green/blue
    order the body-axis stubs on the model use. Matching hues is intentional:
    with the vehicle upright and level, each body stub lies along its world
    arrow.
    """
    ox, oy, oz = VEHICLE_ORIGIN
    glDisable(GL_LIGHTING)
    glLineWidth(2.0)
    glBegin(GL_LINES)
    glColor3f(*C_WORLD_X)
    glVertex3f(ox, oy, oz); glVertex3f(ox + WORLD_AXIS_LEN, oy, oz)
    glColor3f(*C_WORLD_Y)
    glVertex3f(ox, oy, oz); glVertex3f(ox, oy + WORLD_AXIS_LEN, oz)
    glColor3f(*C_WORLD_Z)
    glVertex3f(ox, oy, oz); glVertex3f(ox, oy, oz + WORLD_AXIS_LEN)
    glEnd()

    # A dropline to the ground plane, so the vehicle's height above the grid
    # reads as deliberate rather than as the model floating by accident.
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


def world_axis_label_points():
    """3D points where the world axis labels belong, for the 2D overlay pass."""
    ox, oy, oz = VEHICLE_ORIGIN
    d = WORLD_AXIS_LEN + 0.18
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
