#!/usr/bin/env python3
"""
RocketFC bench visualizer -- draw the rocket's orientation in 3D, live.

    python rocket_viz.py --demo                 no hardware; check the graphics
    python rocket_viz.py --list-ports           find your Teensy's COM port
    python rocket_viz.py --port COM7            the bench test
    python rocket_viz.py --port COM7 --record bench.csv
    python rocket_viz.py --csv bench.csv        replay a recording
    python rocket_viz.py --csv ../../RocketFC/FLIGHTS/flight_007.csv

Requires: pip install pyserial pygame PyOpenGL
"""

import argparse
import collections
import os
import re
import sys

import numpy as np

# Run correctly whether invoked as `python rocket_viz.py` from viz/ or as
# `python viz/rocket_viz.py` from the project root.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import quatmath as qm  # noqa: E402
import sources  # noqa: E402

WINDOW_TITLE = "SensorServoBenchTest -- rocket orientation"
DEFAULT_SIZE = (1280, 800)

# Safe commands reachable from a single keypress. `tvc drive` is deliberately
# NOT here: the one command that physically moves the gimbal should take a
# deliberate act, so it must be typed in full at the "/" prompt.
QUICK_KEYS = {
    "c": "cal",
    "z": "zero",
    "1": "tvc off",
    "2": "tvc calc",
    "x": "axis",
    "s": "status",
}


def gimbal_limit_deg():
    """Read GIMBAL_MAX_RAD out of the vendored config.h so the HUD crosshair's
    full-scale matches the controller's actual clamp instead of a guess."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "src", "config.h")
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            text = f.read()
        m = re.search(r"GIMBAL_MAX_RAD\s*=\s*([0-9.]+)f?\s*\*\s*DEG2RAD", text)
        if m:
            return float(m.group(1))
        m = re.search(r"GIMBAL_MAX_RAD\s*=\s*([0-9.]+)f?", text)
        if m:
            return float(m.group(1)) * 180.0 / np.pi
    except OSError:
        pass
    return 7.0


def build_source(args, console):
    if args.demo:
        return sources.DemoSource()
    if args.csv:
        return sources.CsvSource(args.csv, speed=args.speed)
    try:
        return sources.SerialSource(args.port, args.baud,
                                    on_text=lambda t: console.append(t))
    except ImportError:
        sys.exit("pyserial is not installed.\n"
                 "Run:  pip install pyserial pygame PyOpenGL")
    except Exception as exc:
        sys.exit(
            "could not open {}: {}\n\n"
            "Things to check:\n"
            "  - is the Teensy plugged in? `python rocket_viz.py --list-ports`\n"
            "  - is the Arduino Serial Monitor still open? Only one program can\n"
            "    hold a COM port at a time -- close it and try again.\n"
            "  - is that really the right port number?".format(args.port, exc))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--port", help="serial port of the Teensy, e.g. COM7")
    src.add_argument("--csv", help="replay a bench recording or a RocketFC flight log")
    src.add_argument("--demo", action="store_true",
                     help="synthetic motion; no hardware needed")
    ap.add_argument("--list-ports", action="store_true",
                    help="list serial ports and exit")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--record", metavar="FILE",
                    help="also write every received sample to FILE (CSV)")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="initial playback speed for --csv (default 1.0)")
    ap.add_argument("--width", type=int, default=DEFAULT_SIZE[0])
    ap.add_argument("--height", type=int, default=DEFAULT_SIZE[1])
    ap.add_argument("--frames", type=int, default=0, metavar="N",
                    help="render N frames and exit; for checking the install "
                         "without sitting and watching it")
    ap.add_argument("--screenshot", metavar="FILE",
                    help="save the final frame as a PNG (implies --frames 60 "
                         "if --frames is not given)")
    args = ap.parse_args()

    if args.list_ports:
        for line in sources.list_ports():
            print(line)
        return 0

    if not (args.demo or args.csv or args.port):
        ap.error("pick one of --port, --csv or --demo "
                 "(try --demo first: it needs no hardware)")

    # Imported here, after argument parsing, so --list-ports and --help still
    # work on a machine that has not installed the graphics packages yet.
    try:
        import pygame
        from OpenGL.GL import (glPushMatrix, glPopMatrix, glMultMatrixf,
                               glTranslatef, glDepthMask)
    except ImportError as exc:
        sys.exit("missing a graphics package ({}).\n"
                 "Run:  pip install pygame PyOpenGL".format(exc))

    import hud as hud_mod
    import model as model_mod
    import scene as scene_mod

    console = collections.deque(maxlen=200)
    source = build_source(args, console)

    recorder = None
    if args.record:
        import recorder as recorder_mod
        recorder = recorder_mod.Recorder(args.record)
        console.append("[viz] recording to {}".format(args.record))

    pygame.init()
    size = (args.width, args.height)
    pygame.display.set_caption(WINDOW_TITLE)
    try:
        pygame.display.set_mode(size, pygame.OPENGL | pygame.DOUBLEBUF |
                                pygame.RESIZABLE)
    except pygame.error as exc:
        sys.exit("could not create an OpenGL window: {}\n"
                 "Update your graphics driver, or try a different machine."
                 .format(exc))

    hud_mod.GIMBAL_LIMIT_DEG = gimbal_limit_deg()
    scene_mod.init_gl(*size)
    rocket = model_mod.RocketModel()
    overlay = hud_mod.Hud()
    camera = scene_mod.Camera()
    clock = pygame.time.Clock()

    show_ghost = True
    prompt = None          # not None while typing a command
    dragging = False
    # Only a live board has a rate worth judging: a replay's sample rate is set
    # by playback speed, not by the health of anything.
    expected_hz = 0.0 if args.csv else 50.0
    console.append("[viz] source: {}".format(source.label))
    if args.demo:
        console.append("[viz] demo mode -- synthetic motion, no hardware.")

    frame_limit = args.frames or (60 if args.screenshot else 0)

    running = True
    frames = 0
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.VIDEORESIZE:
                size = (ev.w, ev.h)
                pygame.display.set_mode(size, pygame.OPENGL | pygame.DOUBLEBUF |
                                        pygame.RESIZABLE)
                scene_mod.resize(*size)
            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                dragging = True
            elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
                dragging = False
            elif ev.type == pygame.MOUSEMOTION and dragging:
                camera.orbit(ev.rel[0], ev.rel[1])
            elif ev.type == pygame.MOUSEWHEEL:
                camera.zoom(ev.y)
            elif ev.type == pygame.KEYDOWN:
                # Note whether a command was being typed BEFORE dispatching:
                # while typing, letters must reach the prompt rather than fire
                # their shortcuts.
                was_typing = prompt is not None
                prompt, running = handle_key(ev, prompt, source, console)
                if not was_typing and ev.key == pygame.K_g:
                    show_ghost = not show_ghost

        smp = source.latest()
        if smp is None:
            # Serial connected but nothing parsed yet: show a neutral vehicle
            # rather than a blank window, so "no data" is still legible.
            smp = sources.Sample()
        elif recorder is not None:
            recorder.add(smp)

        ghost_q = qm.quat_from_accel(smp.accel) if np.linalg.norm(smp.accel) > 0.5 \
            else qm.IDENTITY
        ghost_div = qm.tilt_angle_between_deg(smp.q, ghost_q)

        # --- 3D pass ---
        scene_mod.begin_frame(camera)
        scene_mod.draw_grid()
        scene_mod.draw_world_axes()

        ox, oy, oz = scene_mod.VEHICLE_ORIGIN
        glPushMatrix()
        glTranslatef(ox, oy, oz)
        glMultMatrixf(qm.gl_matrix(smp.q))
        rocket.draw_solid(smp.gimx_deg, smp.gimy_deg)
        glPopMatrix()

        if show_ghost:
            glPushMatrix()
            glTranslatef(ox, oy, oz)
            glMultMatrixf(qm.gl_matrix(ghost_q))
            # Translucent: draw without writing depth so the solid model stays
            # visible through it from any angle.
            glDepthMask(False)
            rocket.draw_ghost()
            glDepthMask(True)
            glPopMatrix()

        labels = [(scene_mod.project(p), t, c)
                  for p, t, c in scene_mod.world_axis_label_points()]

        # --- 2D pass ---
        overlay.draw(size, {
            "sample": smp,
            "cam": camera,
            "rate_hz": source.rate_hz,
            "expected_hz": expected_hz,
            "source_label": source.label,
            "ghost_div_deg": ghost_div,
            "console": console,
            "labels": labels,
            "prompt": prompt,
            "can_send": isinstance(source, sources.SerialSource),
            "is_replay": isinstance(source, sources.CsvSource),
        })

        pygame.display.flip()
        clock.tick(60)

        frames += 1
        if frame_limit and frames >= frame_limit:
            if args.screenshot:
                save_screenshot(args.screenshot, size)
                print("wrote {}".format(args.screenshot))
            print("rendered {} frames at {:.0f} FPS; source rate {:.0f} Hz"
                  .format(frames, clock.get_fps(), source.rate_hz))
            running = False

    source.close()
    if recorder is not None:
        recorder.close()
        print("recorded {} rows to {}".format(recorder.rows, recorder.path))
    pygame.quit()
    return 0


def save_screenshot(path, size):
    """Read the framebuffer back and write a PNG. OpenGL's origin is at the
    bottom-left, so the rows come back upside down and have to be flipped."""
    import pygame
    from OpenGL.GL import (glReadPixels, GL_RGBA, GL_UNSIGNED_BYTE,
                           GL_FRONT, glReadBuffer)
    w, h = size
    glReadBuffer(GL_FRONT)
    data = glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE)
    surf = pygame.image.fromstring(bytes(data), (w, h), "RGBA")
    pygame.image.save(pygame.transform.flip(surf, False, True), path)


def handle_key(ev, prompt, source, console):
    """
    Returns (prompt, running).

    While `prompt` is a string the user is typing a command, so keystrokes go
    into it rather than triggering shortcuts.
    """
    import pygame

    live = isinstance(source, sources.SerialSource)

    if prompt is not None:
        if ev.key == pygame.K_ESCAPE:
            return None, True
        if ev.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
            cmd = prompt.strip()
            if cmd:
                console.append("> " + cmd)
                if not source.send(cmd):
                    console.append("[viz] cannot send commands to this source.")
            return None, True
        if ev.key == pygame.K_BACKSPACE:
            return prompt[:-1], True
        if ev.unicode and ev.unicode.isprintable():
            return prompt + ev.unicode, True
        return prompt, True

    if ev.key == pygame.K_ESCAPE:
        return None, False
    if ev.key == pygame.K_SLASH and live:
        return "", True

    name = pygame.key.name(ev.key)
    if live and name in QUICK_KEYS:
        cmd = QUICK_KEYS[name]
        console.append("> " + cmd)
        source.send(cmd)
        return None, True

    # Replay transport controls.
    if isinstance(source, sources.CsvSource):
        if ev.key == pygame.K_SPACE:
            source.toggle_pause()
        elif ev.key == pygame.K_COMMA:
            source.seek(-5.0)
        elif ev.key == pygame.K_PERIOD:
            source.seek(5.0)
        elif ev.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
            source.set_speed(source.speed / 1.5)
        elif ev.key in (pygame.K_EQUALS, pygame.K_PLUS, pygame.K_KP_PLUS):
            source.set_speed(source.speed * 1.5)

    return None, True


if __name__ == "__main__":
    sys.exit(main())
