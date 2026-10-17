#!/usr/bin/env python3
"""Checks for the legs/grass ground model (ground.py) and the landing verdict
(landing_outcome.py), with no flight computer in the loop: the plant is put
just above the ground in a chosen state, left alone with the motor off, and
the outcome compared with what the physics should give.

Run:  python tools/sim/test_landing_outcome.py
"""
import dataclasses
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import landing_outcome  # noqa: E402
import plant as plant_mod  # noqa: E402
import vehicle  # noqa: E402

failures = 0


def check(cond, msg):
    global failures
    status = "PASS" if cond else "FAIL"
    print("  {}: {}".format(status, msg))
    if not cond:
        failures += 1


def drop(vx=0.0, vz=-1.0, roll_deg=0.0, tilt_deg=0.0, legs_frac=1.0, wind=0.0,
         dt=0.002, max_s=15.0, **cfg):
    """Release the rocket with its feet 1 cm above level ground at velocity
    (vx, 0, vz), rolled `roll_deg` about its axis (0 = a leg points along +x)
    and leaning `tilt_deg` toward +x. Returns (outcome, plant)."""
    vc = dataclasses.replace(vehicle.VehicleConfig(), ground_model="legs",
                             wind_mps=wind, **cfg)
    pl = plant_mod.Plant(vc)
    s = pl.state
    s.ascent_ignite_t = -10.0          # ascent long over: motor off, past the pad window
    s.t = 0.0
    r, t = math.radians(roll_deg) / 2.0, math.radians(tilt_deg) / 2.0
    roll = plant_mod.Q(math.cos(r), 0.0, 0.0, math.sin(r))
    lean = plant_mod.Q(math.cos(t), 0.0, math.sin(t), 0.0)   # about +y: nose toward +x
    q = plant_mod.Q(
        lean.w * roll.w - lean.x * roll.x - lean.y * roll.y - lean.z * roll.z,
        lean.w * roll.x + lean.x * roll.w + lean.y * roll.z - lean.z * roll.y,
        lean.w * roll.y - lean.x * roll.z + lean.y * roll.w + lean.z * roll.x,
        lean.w * roll.z + lean.x * roll.y - lean.y * roll.x + lean.z * roll.w)
    s.q = q
    s.pos = plant_mod.V3(0.0, 0.0, 0.01)
    s.vel = plant_mod.V3(vx, 0.0, vz)
    pl.legs_frac = legs_frac
    while not s.landed and s.t < max_s:
        pl.step(dt, 0.0, 0.0)
    summary = {"touchdown_t": s.touchdown_t, "land_ignite_t": 0.0, "hop": False,
               "abort_reason": None, "ground": pl.ground_summary()}
    return landing_outcome.classify(summary), pl


def tip_threshold(roll_deg, lo=0.2, hi=3.0, **cfg):
    """Smallest sideways speed (bisection, 2 cm/s) that knocks it over."""
    for _ in range(8):
        mid = 0.5 * (lo + hi)
        o, _ = drop(vx=mid, vz=-0.3, roll_deg=roll_deg, **cfg)
        if o["code"] == "TIPPED_OVER":
            hi = mid
        else:
            lo = mid
    return 0.5 * (lo + hi)


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")  # labels use the ≈ sign
    print("ground model / landing outcome")

    o, pl = drop(vz=-1.0)
    check(o["code"] == "SUCCESS", "upright 1 m/s drop stands ({})".format(o["label"]))

    # Energy estimate for tipping over the edge between two feet, with the
    # plant's own mass and inertia at contact.
    c = pl.contact
    vc = vehicle.VehicleConfig()
    _, v_edge, _ = landing_outcome.edge_limits(vc.leg_foot_radius_m,
                                               vc.landed_cg_height_m,
                                               c["mass_kg"], c["inertia_kgm2"])
    # Feet that catch (high friction) are what the estimate assumes.
    between = tip_threshold(45.0, ground_mu=2.0)
    toward = tip_threshold(0.0, ground_mu=2.0)
    check(abs(between / v_edge - 1.0) < 0.20,
          "between legs tips at {:.2f} m/s vs estimate {:.2f}".format(between, v_edge))
    check(abs(toward / (v_edge * math.sqrt(2.0)) - 1.0) < 0.20,
          "toward a leg tips at {:.2f} m/s vs estimate {:.2f}".format(
              toward, v_edge * math.sqrt(2.0)))
    grass = tip_threshold(45.0)
    check(abs(grass / v_edge - 1.0) < 0.20,
          "grass (mu 0.6), between legs tips at {:.2f} m/s vs estimate {:.2f}".format(
              grass, v_edge))
    print("    (sideways tip speeds: between legs {:.2f}, toward a leg {:.2f}, "
          "grass between legs {:.2f} m/s)".format(between, toward, grass))

    o, _ = drop(vx=2.5, vz=-0.5, roll_deg=45.0)
    check(o["code"] == "TIPPED_OVER" and o["primary"] == "sideways",
          "2.5 m/s sideways tips over, blamed on sideways speed ({})".format(o["label"]))

    o, _ = drop(vz=-1.0, legs_frac=0.5)
    check(o["code"] == "LEGS_NOT_DOWN", "legs half swung -> LEGS_NOT_DOWN ({})".format(o["label"]))

    o, _ = drop(vz=-4.0)
    check(o["code"] == "LEGS_BROKEN", "4 m/s contact -> LEGS_BROKEN ({})".format(o["label"]))
    # Falling 6 cm per tick must still be caught by the contact model, not
    # slip past it (it once did, and a 30 m/s crash read as a success).
    big, pl30 = drop(vz=-30.0)
    check(pl30.contact is not None and big["code"] == "LEGS_BROKEN"
          and "LIFTED_OFF" not in big["factors"],
          "30 m/s crash is caught at contact -> LEGS_BROKEN, no lift-off ({})".format(big["label"]))
    o2 = landing_outcome.classify({"touchdown_t": 0.0, "land_ignite_t": 0.0, "hop": False,
                                   "abort_reason": None, "ground": _.ground_summary()},
                                  crash_speed=5.0)
    check(o2["code"] == "SUCCESS", "same flight with a 5 m/s crash limit stands ({})".format(o2["label"]))

    o, _ = drop(vz=-0.5, roll_deg=45.0, wind=12.0)
    check(o["code"] == "TIPPED_OVER" and o["primary"] == "wind",
          "landing into 12 m/s wind tips, blamed on wind ({})".format(o["label"]))
    o, _ = drop(vz=-0.5, roll_deg=45.0, gust_mps=12.0, gust_time_s=1.5, gust_dur_s=5.0)
    check(o["code"] == "BLOWN_OVER",
          "standing, then a 12 m/s gust -> BLOWN_OVER ({})".format(o["label"]))
    o, _ = drop(vz=-0.5, roll_deg=45.0, wind=3.0)
    check(o["code"] == "SUCCESS", "3 m/s wind: still standing ({})".format(o["label"]))

    o, _ = drop(vz=-1.0, tilt_deg=25.0, roll_deg=45.0)
    check(o["code"] == "TIPPED_OVER", "landing leaned 25 deg between legs falls ({})".format(o["label"]))
    o, _ = drop(vz=-1.0, tilt_deg=5.0, roll_deg=45.0)
    check(o["code"] == "SUCCESS", "landing leaned 5 deg recovers ({})".format(o["label"]))

    # At rest the contact must not pump energy in.
    o, pl = drop(vz=-1.0, max_s=4.0)
    s = pl.state
    vmax = 0.0
    for _ in range(2500):
        s.landed = False   # keep stepping past the "it has settled" stop
        pl.step(0.002, 0.0, 0.0)
        vmax = max(vmax, s.vel.norm(), s.omega.norm() * 0.6)
    check(vmax < 0.02, "standing still for 5 s stays still (max speed {:.4f} m/s)".format(vmax))

    print("{} failure(s)".format(failures))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
