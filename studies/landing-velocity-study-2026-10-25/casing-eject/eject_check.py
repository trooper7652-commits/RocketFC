"""Calm nominal flight per eject workspace: mass step at thrust onset, ignition confirmed."""
import importlib, json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
if __name__ == "__main__":
    name, kg = sys.argv[1], float(sys.argv[2])
    sim = os.path.join(HERE, "ws", name, "tools", "sim")
    sys.path.insert(0, sim); os.chdir(sim)
    import vehicle, run, plant, core
    combos = json.load(open(os.path.join(HERE, "combos.json")))
    for eject in (0.0, kg):
        v = vehicle.default_vehicle_config(core.default_dll())
        if name.startswith("c1"):
            c = combos["c1_e12_e12"]
            v.ascent_eng_path = c["ascent_eng"].replace("c1_e12_e12", name)
            v.landing_eng_path = c["landing_eng"].replace("c1_e12_e12", name)
        v.ascent_casing_eject_kg = eject
        r = run.simulate(v, seed=1, record=False)
        s = r["summary"]
        pl = plant.Plant(v); pl.ignite_ascent(0.0); pl.ignite_landing(20.0)
        m0, _ = pl.mass_and_thrust(19.999); m1, _ = pl.mass_and_thrust(20.001)
        ev = [e["code"].name for e in r["events"]]
        print("%s eject %.4f | mass %.4f -> %.4f at thrust onset | apogee %.1f m | fire v %.1f m/s | contact %.2f m/s | %s | %s"
              % (name, eject, m0, m1, s["apogee_h"], -s["fire_v_true"], s["contact_vspeed_mps"],
                 s["landing_outcome"]["verdict"] if "verdict" in s["landing_outcome"] else s["landing_outcome"].get("success"),
                 "IGNITION_CONFIRMED" in ev and "DUD" not in " ".join(ev)))
