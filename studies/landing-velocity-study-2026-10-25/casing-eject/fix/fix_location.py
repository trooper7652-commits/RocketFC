"""Fix study, steps 3-4: ballast location (point mass) and steady wind for the
E12/E12 dev-sweep winner. Usage: fix_location.py <ballast g> <margin m>"""
import json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import lvlib as L
import fix_sweep as F
sys.path.insert(0, os.path.join(HERE, "ws", "base", "tools", "sim"))
sys.path.insert(0, os.path.join(HERE, "ws", "base", "tools"))
import vehicle  # noqa: E402
import burn_table as bt  # noqa: E402
LOCATIONS = {"balance point": None, "nose (0.10 m)": 0.10, "low (1.10 m, motor mount)": 1.10}


def point_mass_overrides(m_b, x_b, landing_eng):
    """cg / inertia / standing CG height with a point ballast mass at x_b (m from the nose)."""
    v = vehicle.VehicleConfig()
    pad0 = bt.config_value("MASS_PAD_KG", os.path.join(HERE, "ws", "c1_eject", "src", "config.h"))
    desc0 = bt.config_value("MASS_DESCENT_KG", os.path.join(HERE, "ws", "c1_eject", "src", "config.h"))
    cg0, h0 = v.cg_from_nose_m, v.landed_cg_height_m
    x_b = cg0 if x_b is None else x_b
    cg = (desc0 * cg0 + m_b * x_b) / (desc0 + m_b)
    inertia = (vehicle.estimate_inertia_kgm2(pad0, v.length_m) + pad0 * (cg - cg0) ** 2
               + m_b * (x_b - cg) ** 2)
    m_end = desc0 - bt.Motor(landing_eng).prop_kg - F.E12_EJ
    h = h0 - m_b * (x_b - cg0) / (m_end + m_b)
    return {"cg_from_nose_m": round(cg, 4), "inertia_kgm2": round(inertia, 5), "landed_cg_height_m": round(h, 4)}


if __name__ == "__main__":
    g, mg = int(sys.argv[1]), float(sys.argv[2])
    ws = F.E12_WS[g]
    c = json.load(open(os.path.join(HERE, "combos.json")))["c1_e12_e12"]
    P75 = {"ascent_eng_path": c["ascent_eng"], "landing_eng_path": c["landing_eng"], "legs_cut_s": 0.75,
           "ascent_casing_eject_kg": F.E12_EJ}
    FW = {"LEGS_DELAY_MS": "0.0f", "BURN_TABLE_MARGIN_M": "%.1ff" % mg}
    R = {"winner, ballast as uniform mass (sweep)": L.run("f_c1_b%d_m%.1f" % (g, mg), ws=ws, plant=P75, fw=FW)}
    for lab, x in LOCATIONS.items():
        ov = point_mass_overrides(g / 1000.0, x, c["landing_eng"])
        print("  %-26s %s" % (lab, ov))
        R["%d g at %s" % (g, lab)] = L.run("f_loc_b%d_m%.1f_%s" % (g, mg, lab.split()[0]), ws=ws,
                                           plant=dict(P75, **ov), fw=FW)
    L.table(R, "winner, ballast as uniform mass (sweep)")
    print()
    W = {}
    for w in (0, 3, 5):
        W["casing aboard, wind %d" % w] = L.run("w_c1_%d" % w, ws=c["ws"], fw={"LEGS_DELAY_MS": "0.0f"},
                                                plant={k: v for k, v in P75.items() if k != "ascent_casing_eject_kg"}, wind=w)
        W["fix, wind %d" % w] = L.run("f_wind_b%d_m%.1f_w%d" % (g, mg, w), ws=ws, plant=P75, fw=FW, wind=w)
    L.table(W, "casing aboard, wind 0")
