"""Robustness study, analysis only: what the fix's remaining holdout failures are,
and how success depends on the crash speed the legs survive (re-classified)."""
import collections, json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "ws", "c1_ej_b80", "tools", "sim"))
import landing_outcome as LO  # noqa: E402
if __name__ == "__main__":
    for lab in ("h_fix_b80", "h_c1_best"):
        rows = [r for r in json.load(open(os.path.join(HERE, "results", lab + ".json")))["rows"] if "error" not in r["summary"]]
        n = len(rows)
        fails = [r["summary"]["landing_outcome"] for r in rows if not r["summary"]["landing_outcome"]["success"]]
        first = collections.Counter(o["code"] for o in fails)
        anyf = collections.Counter(f for o in fails for f in set(o["factors"]))
        alone = collections.Counter(o["factors"][0] for o in fails if len(set(o["factors"])) == 1)
        print("== %s: %d failures of %d (%.1f %%)" % (lab, len(fails), n, 100.0 * len(fails) / n))
        print("| Failure | first cause | involved in | only cause |")
        print("|---|---|---|---|")
        for code, k in anyf.most_common():
            print("| %s | %.1f %% | %.1f %% | %.1f %% |" % (code, 100.0 * first[code] / n, 100.0 * k / n, 100.0 * alone[code] / n))
        prim = collections.Counter(o.get("primary") for o in fails if o["code"] == "TIPPED_OVER")
        if prim:
            print("   tipped over, main contributor:", dict(prim))
        line = []
        for cs in (2.0, 2.5, 3.0, 3.5, 4.0, 5.0):
            k = sum(LO.classify(r["summary"], crash_speed=cs)["success"] for r in rows)
            line.append("%.1f m/s: %.1f %%" % (cs, 100.0 * k / n))
        print("   success vs crash speed the legs survive: " + " | ".join(line) + "\n")
