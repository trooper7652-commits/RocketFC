"""to_csv.py <mc_batch json> <out csv.gz> [job name] [--note text]: compact per-flight CSV + one summary line."""
import gzip, json, sys, os, statistics as st
COLS = ["seed", "i", "success", "code", "abort_reason", "abort_state", "apogee_h", "contact_vspeed_mps",
        "touchdown_hspeed_mps", "touchdown_t", "fire_t", "fire_h_true", "land_ignite_t", "abort_t",
        "chute_releases", "chute_release_t", "chute_release_h", "chute_open_t", "chute_detected_t",
        "max_tilt_deg", "gsat_frac", "post_ign_max_h", "post_ign_max_tilt", "wind_mps", "ignition_delay_disp"]
def row(r):
    s = r["summary"]; lo = s.get("landing_outcome") or {}
    d = dict(seed=r["seed"], i=r["i"], success=int(bool(lo.get("success"))), code=lo.get("code"),
             wind_mps=r["dispersions"].get("wind_mps"), ignition_delay_disp=r["dispersions"].get("ignition_delay_s"))
    if r.get("error"): d["code"] = "ERROR"
    for c in COLS:
        if c not in d: d[c] = s.get(c)
    if s.get("ground") and (s["ground"].get("contact") or {}).get("wind_mps") is not None:
        d["wind_mps"] = s["ground"]["contact"]["wind_mps"]
    return d
def fmt(v):
    if v is None: return ""
    if isinstance(v, float): return f"{v:.4g}"
    return str(v)
src, out = sys.argv[1], sys.argv[2]; name = sys.argv[3] if len(sys.argv) > 3 else os.path.basename(out).split(".")[0]
D = json.load(open(src)); rows = [row(r) for r in D["rows"]]
with gzip.open(out, "wt", newline="") as f:
    f.write(",".join(COLS) + "\n")
    for d in rows: f.write(",".join(fmt(d[c]) for c in COLS) + "\n")
n = len(rows); ok = sum(d["success"] for d in rows)
ap = sorted(d["apogee_h"] for d in rows if d["apogee_h"] is not None)
codes = {}
for d in rows: codes[d["code"]] = codes.get(d["code"], 0) + 1
p99 = ap[min(len(ap) - 1, int(0.99 * len(ap)))] if ap else float("nan")
flag = " **APOGEE>30.48m**" if ap and ap[-1] > 30.48 else ""
mix = " ".join(f"{k}:{v}" for k, v in sorted(codes.items(), key=lambda kv: -kv[1]))
meta = D["meta"]; args = f"fw={json.dumps(meta['fw_overrides'])} plant={json.dumps(meta['plant'])} wind={meta['wind']}"
line = (f"| {name} | {n} | {100*ok/n:.1f} % | {st.median(ap):.2f} / {p99:.2f} / {ap[-1]:.2f}{flag} | {mix} | {args} |")
print(line)
with open(os.path.join(os.path.dirname(out), "summary.md"), "a") as f: f.write(line + "\n")
