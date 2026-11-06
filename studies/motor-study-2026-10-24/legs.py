"""Leg timing on the final setup (F10, S3 + 1 m margin), dev seeds 101-103 (600 flights)."""
import json, sys
sys.path.insert(0, ".")
import drv
M = json.load(open("motors/motors.json"))
FW = {"DRIFT_GAIN": "2.0f", "BURN_TABLE_MARGIN_M": "1.0f"}
for delay in ("1000.0f", "0.0f"):
    fw = dict(FW, LEGS_DELAY_MS=delay)
    for cut in (1.5, 2.0, 2.5, 3.0):
        rows = drv.run("legs_F10_d%s_c%.1f" % (delay[:-3], cut), ws="m_F10", fw=fw,
                       plant={"ascent_eng_path": M["F10"]["eng"], "legs_cut_s": cut})
        need = drv.legs_need(rows)
        pw = [s for _, s in need]
        down = sum(1 for x, _ in need if x >= cut) / max(1, len(need))
        on = drv.q([s["legs_on_t"] - s["fire_t"] for s in pw], .5)
        m = drv.metrics(rows)
        print("LEGS_DELAY_MS %-6s band cut %.1f s: nichrome on at fire+%.2f s | legs down at contact %5.1f%% | success %5.2f%%"
              % (delay[:-1], cut, on, 100 * (1 - m["legs_late"]), 100 * m["rate"][0]), flush=True)
