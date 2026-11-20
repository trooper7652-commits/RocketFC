"""PROOF-ONLY patches for the scratch copies (never ported to RocketFC).
  summary  every copy: first chute release time + canopy-freed time in the summary
  latch    copy C: servo-travel latch model (latch-dud-study-2026-11-07 "repass")
usage: python proof_patch.py <copy dir> summary|latch ...
"""
import os, sys

def sub(p, old, new):
    t = open(p, encoding="utf-8", newline="").read()
    nl = "\r\n" if "\r\n" in t else "\n"
    t2 = t.replace("\r\n", "\n")
    assert t2.count(old) == 1, (p, old[:70], t2.count(old))
    open(p, "w", encoding="utf-8", newline="").write(t2.replace(old, new).replace("\n", nl))

ws = sys.argv[1]
run = os.path.join(ws, "tools", "sim", "run.py")
veh = os.path.join(ws, "tools", "sim", "vehicle.py")
for what in sys.argv[2:]:
    if what == "summary":
        sub(run, "    chute_releases = 0\n", "    chute_releases = 0\n    chute_release_t = None  # PROOF\n")
        sub(run, "                    chute_releases += 1\n",
            "                    chute_releases += 1\n"
            "                    if chute_release_t is None:\n"
            "                        chute_release_t = t  # PROOF\n")
        sub(run, '        "chute_releases": chute_releases,\n',
            '        "chute_releases": chute_releases,\n'
            '        "chute_release_t": chute_release_t, "chute_open_t": pl.chute_open_t,  # PROOF\n')
    elif what == "latch":
        sub(veh, "    chute_stuck_releases: int = 0\n", """    chute_stuck_releases: int = 0
    # PROOF: servo-travel latch (0 = off = count-based). repass rule: a retry
    # frees the canopy when the latch comes back up through latch_release_deg
    # after being >= margin below it (latch-dud-study-2026-11-07).
    latch_rate_dps: float = 0.0
    latch_lock_deg: float = 40.0
    latch_open_deg: float = 180.0
    latch_release_deg: float = 80.0
    latch_wig_low_deg: float = 55.0
    latch_wig_high_deg: float = 110.0
    latch_repass_margin_deg: float = 5.0
""")
        sub(run, "                    if chute_releases > vcfg.chute_stuck_releases:\n",
            "                    if vcfg.latch_rate_dps <= 0.0 and chute_releases > vcfg.chute_stuck_releases:\n")
        sub(run, "    chute_releases = 0\n", """    chute_releases = 0
    latch_deg = None; latch_tries = 0; latch_armed = False; latch_try_t = []  # PROOF
""")
        sub(run, "            # ---- servo sign round-trip (see module docstring) ----\n", """            # ---- PROOF: servo-travel latch ----
            if vcfg.latch_rate_dps > 0.0 and chute_releases > 0:
                if latch_deg is None:
                    latch_deg = vcfg.latch_lock_deg
                tgt = {0: vcfg.latch_lock_deg, 1: vcfg.latch_open_deg,
                       2: vcfg.latch_wig_low_deg, 3: vcfg.latch_wig_high_deg}[out["chute_servo"]]
                stp = vcfg.latch_rate_dps * dt
                latch_deg += max(-stp, min(stp, tgt - latch_deg))
                R = vcfg.latch_release_deg
                tried = False
                if latch_tries == 0:
                    if latch_deg >= R:
                        latch_tries, tried = 1, True
                elif latch_deg < R - vcfg.latch_repass_margin_deg:
                    latch_armed = True
                elif latch_armed and latch_deg >= R:
                    latch_armed, tried = False, True
                    latch_tries += 1
                if tried:
                    latch_try_t.append(t)
                    if latch_tries > vcfg.chute_stuck_releases:
                        pl.deploy_chute(t)

            # ---- servo sign round-trip (see module docstring) ----
""")
        sub(run, '        "chute_releases": chute_releases,\n',
            '        "chute_releases": chute_releases,\n        "latch_try_t": latch_try_t,  # PROOF\n')
    print("patched", what, ws)
