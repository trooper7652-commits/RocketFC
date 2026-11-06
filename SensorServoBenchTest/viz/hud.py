"""
The 2D overlay: numbers, panels, labels, and the board's console output.

Everything is drawn onto one pygame surface, uploaded as a single texture and
blitted over the 3D view in an orthographic pass. The surface is rebuilt at
~20 Hz rather than every frame -- text does not need 60 Hz, and re-uploading a
full-window RGBA texture that often is the one thing here that could actually
cost frame time on integrated graphics. It is also rebuilt immediately whenever
the camera moves, so the labels pinned to 3D axes never lag behind them.
"""

import time

import pygame
from OpenGL.GL import *

import quatmath as qm

REBUILD_HZ = 20.0

BG_PANEL = (18, 20, 24, 205)
BORDER = (58, 63, 72, 230)
FG = (226, 230, 238, 255)
DIM = (146, 154, 166, 255)
GOOD = (110, 205, 130, 255)
WARN = (238, 186, 90, 255)
BAD = (236, 110, 100, 255)
ACCENT = (120, 180, 255, 255)

C_X = (230, 90, 90, 255)
C_Y = (95, 200, 110, 255)
C_Z = (110, 155, 245, 255)

PAD = 10
LINE = 17
TITLE_H = 6 + LINE + 4      # vertical space a panel title occupies
PANEL_W = 250
SIDE_W = 238


def panel_h(rows, titled=True):
    """Height a panel needs for `rows` text lines. Deriving this rather than
    hand-tuning per panel is what stops the last line clipping out of view when
    a panel gains a row."""
    return (TITLE_H if titled else 6) + rows * LINE + 8


class Hud:
    def __init__(self):
        pygame.font.init()
        self._font = pygame.font.SysFont("consolas,couriernew,monospace", 14)
        self._font_b = pygame.font.SysFont("consolas,couriernew,monospace", 14,
                                           bold=True)
        self._font_t = pygame.font.SysFont("segoeui,arial,sans", 17, bold=True)
        self._tex = glGenTextures(1)
        self._surf = None
        self._size = (0, 0)
        self._last_build = 0.0
        self._last_cam = None

    # -- public -----------------------------------------------------------
    def draw(self, size, ctx):
        """Rebuild if due, then blit. `ctx` is the dict assembled by rocket_viz."""
        now = time.monotonic()
        cam = (round(ctx["cam"].az, 4), round(ctx["cam"].el, 4),
               round(ctx["cam"].dist, 3))
        due = (now - self._last_build) >= (1.0 / REBUILD_HZ)
        if self._surf is None or self._size != size or due or cam != self._last_cam:
            self._build(size, ctx)
            self._last_build = now
            self._last_cam = cam
        self._blit(size)

    # -- surface construction ---------------------------------------------
    def _build(self, size, ctx):
        w, h = size
        if self._surf is None or self._size != size:
            self._surf = pygame.Surface(size, pygame.SRCALPHA, 32)
            self._size = size
        s = self._surf
        s.fill((0, 0, 0, 0))

        smp = ctx["sample"]
        self._panel_header(s, ctx)

        # Panels stack downward from their column top, each returning its own
        # bottom edge, so adding or removing a row never needs a second edit.
        left_x, right_x, gap = 12, w - SIDE_W - 12, 8
        y = 46
        y = self._panel_sensors(s, left_x, y, smp, ctx)
        y = self._panel_axis(s, left_x, y + gap, smp)
        self._panel_ghost(s, left_x, y + gap, smp, ctx)

        y = self._panel_gimbal(s, right_x, 46, smp)
        self._panel_keys(s, right_x, y + gap, ctx)

        console_h = panel_h(7)
        self._console(s, left_x, h - console_h - 12,
                      w - SIDE_W - 36, ctx)
        self._axis_labels(s, ctx)

    def _box(self, s, x, y, w, h, title=None):
        pygame.draw.rect(s, BG_PANEL, (x, y, w, h), border_radius=5)
        pygame.draw.rect(s, BORDER, (x, y, w, h), width=1, border_radius=5)
        if title:
            s.blit(self._font_b.render(title, True, ACCENT), (x + PAD, y + 6))
            return y + 6 + LINE + 4
        return y + 6

    def _row(self, s, x, y, label, value, color=FG, vx=92):
        """One label/value line. `vx` is the value column, widened by callers
        whose labels are long enough to otherwise run into it."""
        s.blit(self._font.render(label, True, DIM), (x, y))
        s.blit(self._font.render(value, True, color), (x + vx, y))
        return y + LINE

    # -- panels -----------------------------------------------------------
    def _panel_header(self, s, ctx):
        w = self._size[0]
        pygame.draw.rect(s, BG_PANEL, (0, 0, w, 34))
        pygame.draw.line(s, BORDER, (0, 34), (w, 34))
        s.blit(self._font_t.render("SensorServoBenchTest", True, FG), (12, 7))

        rate = ctx["rate_hz"]
        expected = ctx["expected_hz"]
        # For a live board, a rate well below the firmware's send rate means
        # dropped serial or a stalled loop -- worth surfacing as prominently as
        # the attitude itself. A replay's rate is set by playback speed, so
        # expected_hz is 0 there and the number is reported without judgment.
        if expected <= 0:
            col, note = DIM, "{:.0f} Hz".format(rate)
        elif rate <= 0.1:
            col, note = BAD, "NO DATA"
        elif rate < expected * 0.6:
            col, note = WARN, "{:.0f} Hz (low)".format(rate)
        else:
            col, note = GOOD, "{:.0f} Hz".format(rate)
        txt = "{}   {}".format(ctx["source_label"], note)
        img = self._font_b.render(txt, True, col)
        s.blit(img, (w - img.get_width() - 12, 9))

    def _panel_sensors(self, s, x, y, smp, ctx):
        h = panel_h(8)
        ty = self._box(s, x, y, PANEL_W, h, "SENSORS / AHRS")
        tx = x + PAD
        ty = self._row(s, tx, ty, "tilt", "{:6.2f} deg".format(smp.tilt_deg),
                       GOOD if smp.tilt_deg < 5 else FG)
        ty = self._row(s, tx, ty, "quat",
                       "{:+.3f} {:+.3f}".format(smp.q[0], smp.q[1]))
        ty = self._row(s, tx, ty, "",
                       "{:+.3f} {:+.3f}".format(smp.q[2], smp.q[3]))
        ty = self._row(s, tx, ty, "accel m/s2", "{:+6.2f} {:+6.2f} {:+6.2f}".format(
            smp.accel[0], smp.accel[1], smp.accel[2]))
        ty = self._row(s, tx, ty, "gyro dps", "{:+6.1f} {:+6.1f} {:+6.1f}".format(
            smp.gyro[0], smp.gyro[1], smp.gyro[2]))
        ty = self._row(s, tx, ty, "baro alt", "{:6.2f} m".format(smp.baro_alt))

        imu = "OK" if smp.imu_ok else "FAIL"
        baro = "OK" if smp.baro_ok else "FAIL"
        s.blit(self._font.render("health", True, DIM), (tx, ty))
        s.blit(self._font.render("imu " + imu, True, GOOD if smp.imu_ok else BAD),
               (tx + 92, ty))
        s.blit(self._font.render("baro " + baro, True, GOOD if smp.baro_ok else BAD),
               (tx + 162, ty))
        ty += LINE

        label = smp.state if smp.state else "tvc " + smp.tvc_mode_name
        col = WARN if smp.tvc_mode == 2 else FG
        self._row(s, tx, ty, "mode", label, col)
        return y + h

    def _panel_axis(self, s, x, y, smp):
        """Which body axis gravity is along -- the fastest IMU_R_SB check."""
        h = panel_h(4)
        ty = self._box(s, x, y, PANEL_W, h, "AXIS CHECK")
        tx = x + PAD
        axis, mag = qm.dominant_body_axis(smp.accel)
        upright = (axis == "+Z")
        ty = self._row(s, tx, ty, "gravity along", "body {}".format(axis),
                       GOOD if upright else WARN, vx=112)
        ty = self._row(s, tx, ty, "magnitude", "{:5.2f} m/s2".format(mag), vx=112)
        if upright:
            msg1, msg2, col = "consistent with", "the rocket standing nose up", DIM
        else:
            msg1, msg2, col = "if it IS nose up now,", "IMU_R_SB is wrong", WARN
        s.blit(self._font.render(msg1, True, col), (tx, ty))
        s.blit(self._font.render(msg2, True, col), (tx, ty + LINE))
        return y + h

    def _panel_ghost(self, s, x, y, smp, ctx):
        h = panel_h(3)
        ty = self._box(s, x, y, PANEL_W, h, "AHRS vs ACCEL (ghost)")
        tx = x + PAD
        div = ctx["ghost_div_deg"]
        col = GOOD if div < 3.0 else (WARN if div < 10.0 else BAD)
        ty = self._row(s, tx, ty, "tilt differs", "{:5.2f} deg".format(div), col,
                       vx=104)
        s.blit(self._font.render("ghost = gravity only:", True, DIM), (tx, ty))
        s.blit(self._font.render("its heading is meaningless", True, DIM),
               (tx, ty + LINE))
        return y + h

    def _panel_gimbal(self, s, x, y, smp):
        cross = 96
        h = panel_h(4) + cross + 22
        ty = self._box(s, x, y, SIDE_W, h, "GIMBAL / SERVOS")
        tx = x + PAD
        ty = self._row(s, tx, ty, "gimbal X", "{:+6.2f} deg".format(smp.gimx_deg))
        ty = self._row(s, tx, ty, "gimbal Y", "{:+6.2f} deg".format(smp.gimy_deg))
        ty = self._row(s, tx, ty, "servo A", "{:6.0f} us".format(smp.servo_a_us))
        ty = self._row(s, tx, ty, "servo B", "{:6.0f} us".format(smp.servo_b_us))

        # Deflection crosshair, scaled so the box edge is the gimbal limit.
        # Full alpha throughout: pygame.draw REPLACES alpha rather than
        # blending it, so mixing alphas here produces stray see-through bands
        # that read as separate boxes instead of one square with a cross.
        size = cross
        cx, cy = x + SIDE_W // 2, ty + 8 + size // 2
        r = size // 2
        pygame.draw.rect(s, (12, 13, 16, 255), (cx - r, cy - r, size, size))
        pygame.draw.line(s, (52, 57, 66, 255), (cx - r, cy), (cx + r, cy))
        pygame.draw.line(s, (52, 57, 66, 255), (cx, cy - r), (cx, cy + r))
        pygame.draw.rect(s, BORDER, (cx - r, cy - r, size, size), width=1)

        lim = max(1e-3, GIMBAL_LIMIT_DEG)
        px = cx + int(max(-1.0, min(1.0, smp.gimy_deg / lim)) * r)
        py = cy - int(max(-1.0, min(1.0, smp.gimx_deg / lim)) * r)
        pygame.draw.line(s, ACCENT, (cx, cy), (px, py), 2)
        pygame.draw.circle(s, ACCENT, (px, py), 4)
        s.blit(self._font.render("full scale +/-{:.0f} deg".format(lim), True, DIM),
               (tx, cy + r + 4))
        return y + h

    def _panel_keys(self, s, x, y, ctx):
        live = ctx["can_send"]
        keys = [("drag", "orbit"), ("wheel", "zoom"), ("G", "ghost on/off")]
        if live:
            keys += [("C", "cal (hold still)"), ("Z", "baro zero"),
                     ("1 / 2", "tvc off / calc"), ("X", "axis check"),
                     ("/", "type a command")]
        elif ctx["is_replay"]:
            keys += [("SPACE", "pause"), (", .", "seek 5 s"),
                     ("- +", "speed")]
        keys += [("ESC", "quit")]

        h = panel_h(len(keys) + (1 if live else 0))
        ty = self._box(s, x, y, SIDE_W, h, "KEYS")
        tx = x + PAD
        for k, v in keys:
            s.blit(self._font.render(k, True, ACCENT), (tx, ty))
            s.blit(self._font.render(v, True, DIM), (tx + 62, ty))
            ty += LINE
        if live:
            s.blit(self._font.render("tvc drive: type in full", True, WARN),
                   (tx, ty))

    def _console(self, s, x, y, w, ctx):
        """Firmware text output. This program owns the COM port, so without
        this panel the board's replies would be invisible."""
        lines = ctx["console"]
        rows = 6
        h = panel_h(rows + 1)
        ty = self._box(s, x, y, w, h, "BOARD OUTPUT")
        prompt = ctx.get("prompt")
        for ln in list(lines)[-rows:]:
            s.blit(self._font.render(ln[:140], True, DIM), (x + PAD, ty))
            ty += LINE
        if prompt is not None:
            s.blit(self._font_b.render("> " + prompt + "_", True, GOOD),
                   (x + PAD, y + h - LINE - 8))

    def _axis_labels(self, s, ctx):
        """Text pinned to the 3D axis tips, positioned by scene.project()."""
        for pos, text, color in ctx["labels"]:
            if pos is None:
                continue
            col = (int(color[0] * 255), int(color[1] * 255), int(color[2] * 255), 255) \
                if isinstance(color[0], float) else color
            img = self._font_b.render(text, True, col)
            s.blit(img, (int(pos[0]) + 4, int(pos[1]) - img.get_height() // 2))

    # -- upload + blit ----------------------------------------------------
    def _blit(self, size):
        w, h = size
        data = pygame.image.tostring(self._surf, "RGBA", False)

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
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA,
                     GL_UNSIGNED_BYTE, data)
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


# Filled in by rocket_viz at startup from the vendored config.h, so the
# crosshair's full-scale matches the controller's real clamp.
GIMBAL_LIMIT_DEG = 7.0
