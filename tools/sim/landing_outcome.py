"""Did the landing succeed, and if not, why?

classify(summary) reads run.simulate()'s summary (with the "ground" block that
plant.ground_summary() adds under ground_model "legs") and returns:

    {"code": "SUCCESS" | <failure code>, "success": bool, "label": str,
     "t": time of the deciding moment (for the 3D view's timeline marker),
     "factors": [every failure code that applies, in time order],
     "contributors": {name: {"value", "limit", "ratio", "text"}},
     "primary": the contributor that pushed it over hardest (or None)}

Success = a powered landing that ends upright and intact: the landing motor
lit, nothing aborted, no hop, the legs were down, the contact speed was under
crash_speed_mps, and it was still standing settle_s after burnout. Failure
codes, in time order (the first one that applies is the headline):

  NO_POWERED_LANDING  aborted (tilt, Kalman health, dud e-match...), the fire
                      window was missed, or a chute-test flight
  HOPPED              stopped in mid-air, climbed and fell back (run.py's hop)
  LEGS_NOT_DOWN       touched down before the legs had swung down and locked
  LEGS_BROKEN         vertical contact speed above crash_speed_mps. A rule,
                      not physics: the simulated legs never break, so the same
                      flights can be re-judged at any crash speed.
  LIFTED_OFF          the F15 was still pushing harder than the weight and
                      lifted it back off the ground (> 0.25 m)
  TIPPED_OVER         fell over during the landing (tilt passed 45 deg)
  BLOWN_OVER          came to rest standing, then the wind pushed it over
  NOT_SETTLED         still moving when the sim gave up (12 s after contact)
  NO_TOUCHDOWN        the sim ended before it reached the ground

For a fall, `contributors` compares each cause at the moment of contact with
the size that would tip the rocket on its own (rigid legs, feet that catch;
an energy estimate, so treat it as a guide -- the physics in ground.py makes
the actual call):

  sideways  CG sideways speed vs. the speed whose kinetic energy lifts the CG
            over the edge between two feet (between-legs is the weak
            direction: that edge is only r/sqrt(2) from the centre)
  rotation  tipping rate at contact, as the CG speed it causes
  tilt      lean at contact vs. the static tip angle in that direction
  thrust    landing thrust still on, as a fraction of the weight
  slope     ground slope vs. the static tip angle
  wind      wind's tipping moment vs. the weight's restoring moment
"""
import math

G0 = 9.80665
RHO = 1.225
LIFTOFF_M = 0.25        # same threshold as run.py's mid-air hop
FALLEN_DEG = 45.0

CODES = ("SUCCESS", "NO_POWERED_LANDING", "HOPPED", "LEGS_NOT_DOWN",
         "LEGS_BROKEN", "LIFTED_OFF", "TIPPED_OVER", "BLOWN_OVER",
         "NOT_SETTLED", "NO_TOUCHDOWN")


def _rot(q):
    w, x, y, z = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def edge_limits(r, h, m, inertia):
    """(edge distance, sideways speed limit, static tip angle in rad) for
    tipping straight over the edge between two feet."""
    d = r / math.sqrt(2.0)
    rise = math.hypot(d, h) - h
    i_pivot = inertia + m * (d * d + h * h)
    v_edge = math.sqrt(2.0 * G0 * rise * i_pivot / (m * h * h))
    return d, v_edge, math.atan2(d, h)


def _off_edge(azimuth, feet_azimuth):
    """Angle (rad) between a horizontal direction and the nearest edge normal
    of the square of feet; 0 = straight at an edge, +-45 deg = at a foot."""
    a = math.degrees(azimuth - feet_azimuth) % 90.0 - 45.0
    return math.radians(a)


def contributors(ground):
    """Each tipping cause at contact vs. its own limit (see module doc)."""
    c = ground.get("contact")
    if not c:
        return {}
    p = ground["params"]
    m = c["mass_kg"]
    r, h = p["leg_foot_radius_m"], p["landed_cg_height_m"]
    d, v_edge, th_edge = edge_limits(r, h, m, c["inertia_kgm2"])
    R = _rot(c["quat"])
    feet_az = math.atan2(R[1][0], R[0][0])
    out = {}

    vx, vy, vz = c["vel"]
    vh = math.hypot(vx, vy)
    lim = v_edge / max(math.cos(_off_edge(math.atan2(vy, vx), feet_az)), 1e-6) \
        if vh > 1e-6 else v_edge
    out["sideways"] = {"value": vh, "limit": lim, "ratio": vh / lim,
                       "text": "sideways {:.1f} m/s vs ≈{:.2f} m/s limit".format(vh, lim)}

    wb = c["omega"]
    wx = R[0][0] * wb[0] + R[0][1] * wb[1] + R[0][2] * wb[2]
    wy = R[1][0] * wb[0] + R[1][1] * wb[1] + R[1][2] * wb[2]
    rx, ry = wy * h, -wx * h          # CG velocity from rotating about the feet
    vr = math.hypot(rx, ry)
    lim = v_edge / max(math.cos(_off_edge(math.atan2(ry, rx), feet_az)), 1e-6) \
        if vr > 1e-6 else v_edge
    out["rotation"] = {"value": math.degrees(math.hypot(wx, wy)), "limit": lim,
                       "ratio": vr / lim,
                       "text": "rotating {:.0f}°/s at contact (≈{:.2f} m/s at the CG vs {:.2f})"
                               .format(math.degrees(math.hypot(wx, wy)), vr, lim)}

    tilt = math.acos(max(-1.0, min(1.0, R[2][2])))
    if tilt > 1e-6:
        cd = max(math.cos(_off_edge(math.atan2(R[1][2], R[0][2]), feet_az)), 1e-6)
        lim = math.atan2(d, h * cd)
    else:
        lim = th_edge
    out["tilt"] = {"value": math.degrees(tilt), "limit": math.degrees(lim),
                   "ratio": tilt / lim,
                   "text": "tilted {:.0f}° at contact vs ≈{:.0f}° tip angle"
                           .format(math.degrees(tilt), math.degrees(lim))}

    t_ratio = c["thrust_n"] / (m * G0)
    out["thrust"] = {"value": c["thrust_n"], "limit": m * G0, "ratio": t_ratio,
                     "text": "motor still pushing {:.0f} N ({:.1f}× weight) at contact"
                             .format(c["thrust_n"], t_ratio)}

    sl = math.radians(ground.get("slope_deg", 0.0))
    out["slope"] = {"value": math.degrees(sl), "limit": math.degrees(th_edge),
                    "ratio": sl / th_edge,
                    "text": "ground slope {:.1f}° vs {:.0f}° tip angle"
                            .format(math.degrees(sl), math.degrees(th_edge))}

    # Strongest wind from touchdown on (a gust can arrive after it stands).
    w = max(abs(c.get("wind_mps", 0.0)), ground.get("max_wind_mps", 0.0))
    area = p["length_m"] * p["diameter_m"]
    z_c = h + (p["cg_from_nose_m"] - 0.5 * p["length_m"])
    moment = 0.5 * RHO * p["crossflow_cd"] * area * w * w * z_c
    out["wind"] = {"value": w, "limit": None, "ratio": moment / (m * G0 * d),
                   "text": "wind {:.1f} m/s pushing on the standing rocket".format(w)}
    return out


def _abort_text(summary):
    reason = summary.get("abort_reason")
    phase = summary.get("abort_state")
    txt = reason.replace("_", " ").lower() if reason else "abort"
    return "aborted ({}{})".format(txt, " in " + phase if phase else "")


def classify(summary, crash_speed=None, mode="land"):
    """See the module docstring. crash_speed overrides the crash speed the
    flight was simulated with (for sweeps; no re-flight needed)."""
    g = summary.get("ground")
    factors = []
    td_t = summary.get("touchdown_t")
    ab_t = summary.get("abort_t")
    aborted = summary.get("abort_reason") and (td_t is None or ab_t is None or ab_t < td_t)

    first = {"code": None, "label": None, "t": None}

    def add(code, label, t):
        factors.append(code)
        if first["code"] is None:
            first.update(code=code, label=label, t=t)

    # -- before touchdown --
    if mode != "land":
        add("NO_POWERED_LANDING", "No powered landing: chute-test flight", ab_t)
    elif aborted:
        add("NO_POWERED_LANDING", "No powered landing: " + _abort_text(summary), ab_t)
    elif summary.get("land_ignite_t") is None:
        add("NO_POWERED_LANDING", "No powered landing: landing motor never lit", td_t)
    if summary.get("hop"):
        add("HOPPED", "Hopped: stopped {:.1f} m up, climbed to {:.1f} m and fell back".format(
            summary.get("hop_stop_h") or 0.0, summary.get("hop_apex_h") or 0.0), td_t)

    contrib = {}
    primary = None
    if "NO_POWERED_LANDING" in factors:
        pass  # came down under the chute: not a landing attempt to judge
    elif td_t is None:
        add("NO_TOUCHDOWN", "Never touched down (the sim ended first)", None)
    elif g and g.get("contact"):
        c = g["contact"]
        p = g["params"]
        crash = p["crash_speed_mps"] if crash_speed is None else crash_speed
        vz = abs(c["vel"][2])
        ct = c["t"]
        # -- at touchdown --
        if c["legs_frac"] < 1.0:
            add("LEGS_NOT_DOWN", "Legs not down at touchdown ({:.0f}% swung)".format(
                100.0 * c["legs_frac"]), ct)
        if vz > crash:
            add("LEGS_BROKEN", "Legs broken: hit at {:.1f} m/s > {:.1f} m/s limit".format(
                vz, crash), ct)
        # -- after touchdown --
        if g.get("liftoff_max_m", 0.0) > LIFTOFF_M:
            add("LIFTED_OFF", "Motor lifted it back off after touchdown ({:.2f} m up)".format(
                g["liftoff_max_m"]), ct)
        contrib = contributors(g)
        if contrib:
            primary = max(contrib, key=lambda k: contrib[k]["ratio"])
        fell = g.get("fall_t") is not None
        if fell:
            blown = (g.get("rest_upright_t") is not None
                     and g["rest_upright_t"] < g["fall_t"])
            if blown:
                primary = "wind"
                add("BLOWN_OVER", "Blown over after standing: " + contrib["wind"]["text"],
                    g["fall_t"])
            else:
                add("TIPPED_OVER", "Tipped over: " + contrib[primary]["text"], g["fall_t"])
        elif g.get("end") == "timeout":
            add("NOT_SETTLED", "Still moving 12 s after touchdown", ct)

    if first["code"] is None:
        c = g["contact"] if g else None
        vz = abs(c["vel"][2]) if c else (summary.get("touchdown_speed_mps") or 0.0)
        hs = math.hypot(c["vel"][0], c["vel"][1]) if c else 0.0
        first.update(code="SUCCESS",
                     label="LANDED: upright, {:.1f} m/s down, {:.1f} m/s sideways".format(vz, hs),
                     t=(g or {}).get("rest_upright_t") or td_t)
    return {"code": first["code"], "success": first["code"] == "SUCCESS",
            "label": first["label"], "t": first["t"], "factors": factors,
            "contributors": contrib, "primary": primary}
