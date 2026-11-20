"""STUDY-ONLY summary fields for scratch copies (never ported; read-only, the
flight is unchanged). Adds to run.py's summary:
  chute_release_t/_h/_v  first CHUTE_RELEASE: time, true height, true vz
  chute_open_t           canopy actually out (plant)
  gsat_frac              share of powered, airborne steps with |gimbal cmd| >= 98 % of GIMBAL_MAX
  post_ign_max_h         max true height after landing-motor thrust onset
  post_ign_max_tilt      max estimated tilt after thrust onset, before contact
usage: python sim_fields.py <copy dir>
"""
import os, sys

def sub(p, old, new):
    t = open(p, encoding="utf-8").read()
    assert t.count(old) == 1, (p, old[:70], t.count(old))
    open(p, "w", encoding="utf-8").write(t.replace(old, new))

run = os.path.join(sys.argv[1], "tools", "sim", "run.py")
sub(run, "    chute_releases = 0\n",
    "    chute_releases = 0\n"
    "    chute_release_t = chute_release_h = chute_release_v = None  # STUDY\n"
    "    gs_n = gs_sat = 0; post_ign_max_h = None; post_ign_max_tilt = None  # STUDY\n")
sub(run, "                    chute_releases += 1\n",
    "                    chute_releases += 1\n"
    "                    if chute_release_t is None:  # STUDY\n"
    "                        chute_release_t, chute_release_h, chute_release_v = t, s.pos.z, s.vel.z\n")
sub(run, "            prev_v_true = s.vel.z\n",
    "            prev_v_true = s.vel.z\n"
    "            if launched and thrust_now > 0.0 and pl.state.touchdown_t is None and s.pos.z > 0.05:  # STUDY\n"
    "                gs_n += 1\n"
    "                if max(abs(out['gimbal_x']), abs(out['gimbal_y'])) >= 0.98 * snap['gimbalMaxRad']:\n"
    "                    gs_sat += 1\n"
    "            if pl.state.landing_ignite_t is not None and t >= pl.state.landing_ignite_t:  # STUDY\n"
    "                post_ign_max_h = max(post_ign_max_h or -1e9, s.pos.z)\n"
    "                if pl.state.touchdown_t is None:\n"
    "                    post_ign_max_tilt = max(post_ign_max_tilt or 0.0, out['tilt_deg'])\n")
sub(run, '        "chute_releases": chute_releases,\n',
    '        "chute_releases": chute_releases,\n'
    '        "chute_release_t": chute_release_t, "chute_release_h": chute_release_h,  # STUDY\n'
    '        "chute_release_v": chute_release_v, "chute_open_t": pl.chute_open_t,\n'
    '        "gsat_frac": (gs_sat / gs_n) if gs_n else None,\n'
    '        "post_ign_max_h": post_ign_max_h, "post_ign_max_tilt": post_ign_max_tilt,\n')
print("patched", run)
mcb = os.path.join(sys.argv[1], "tools", "sim", "mc_batch.py")
sub(mcb, 'mode=mode, dll_path=dll, seed=seed, record=False)',
    'mode=mode, dll_path=dll, seed=seed, record=False,\n'
    '                             dud_landing_igniter=bool(os.environ.get("STUDY_DUD")))  # STUDY')
t = open(mcb).read()
if "\nimport os" not in t:
    open(mcb, "w").write("import os  # STUDY\n" + t)
print("patched", mcb)
