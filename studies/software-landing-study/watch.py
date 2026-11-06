#!/usr/bin/env python3
"""Watch a recorded study flight in RocketFC's 3D view.

    py watch.py            the successful landing (0.3 m/s, upright)
    py watch.py failure    a flight from the same batch that hit too hard
    py watch.py best       best setup so far (S3 + D2 + GPS, fire 1 m higher):
                           0.24 m/s down, 0.27 m/s sideways

These were flown with the study's candidate firmware (drift-cancelling loop +
D2 tilt interlock, see patches/final_S3_D2.patch) and a 10 % stronger ascent
motor, which RocketFC itself doesn't have -- so they are replayed from the
recording rather than re-flown.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "RocketFC", "tools", "sim"))
os.chdir(os.path.join(HERE, "..", "RocketFC", "tools", "sim"))

import flight3d  # noqa: E402


def load(name):
    data = json.load(open(os.path.join(HERE, "example_{}.json".format(name)), encoding="utf-8"))
    for r in data["rows"]:
        for k in ("quat_true", "quat_est"):
            r[k] = tuple(r[k])
    track = flight3d.track_from_sim(data, label=data["note"])
    return track


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "success"
    track = load(name)
    print(track.outcome["label"] if track.outcome else "")
    view = flight3d.FlightView(track)
    view.playing = True
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
                t = 0.0
        if ev["stop"]:
            view.playing, t = False, 0.0
        if ev["step"]:
            view.playing = False
            t = max(0.0, min(track.t_max, t + ev["step"] * flight3d.STEP_S))
        if ev["speed"] is not None:
            view.speed = ev["speed"]
        if ev["seek"] is not None:
            t, view.playing = ev["seek"], False
        dt = clock.tick(120) / 1000.0
        if view.playing:
            t += dt * view.speed
            if t >= track.t_max:
                t, view.playing = track.t_max, False
        view.render(t)
    pygame.quit()


if __name__ == "__main__":
    main()
