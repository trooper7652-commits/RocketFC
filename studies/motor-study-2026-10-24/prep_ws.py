"""Rebuild the study workspaces in a work folder (default: next to this script):
ws/base = RocketFC as-is, ws/fix = RocketFC + firmware/bridge hunks of
ideas_full.patch (LF copy, sim fixes already in RocketFC) + the extra sim
fields, then ws/m_<motor> via setup_motors.py. Results in results/ stay valid
only if RocketFC still matches rocketfc_state_at_pause.txt.

    python prep_ws.py [work_dir]
"""
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WORK = os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else HERE
ROCKETFC = os.path.join(os.path.dirname(HERE), "RocketFC")
PY = sys.executable
SKIP = shutil.ignore_patterns(".git", "build", "*.dll", "__pycache__")


def edit(path, old, new):
    t = open(path, encoding="utf-8").read().replace("\r\n", "\n")
    assert t.count(old) == 1, (path, old[:60])
    open(path, "w", encoding="utf-8", newline="\n").write(t.replace(old, new))


for w in ("base", "fix"):
    dst = os.path.join(WORK, "ws", w)
    if os.path.isdir(dst):
        sys.exit(dst + " already exists; delete ws/ first")
    shutil.copytree(ROCKETFC, dst, ignore=SKIP)

fix = os.path.join(WORK, "ws", "fix")
subprocess.run(["patch", "-p1", "-i", os.path.join(HERE, "ideas_fw_lf.patch")], cwd=fix, check=True)
sim = os.path.join(fix, "tools", "sim")
edit(os.path.join(sim, "vehicle.py"),
     "    settle_s: float = 2.0              # must stand still this long after burnout\n",
     "    settle_s: float = 2.0              # must stand still this long after burnout\n"
     "\n"
     "    # ---- Simulated GPS velocity (study; 0 Hz = no GPS) ----\n"
     "    gps_hz: float = 0.0                # u-blox M9/M10-class: 10 Hz\n"
     "    gps_vel_sigma: float = 0.1         # [ESTIMATE] horizontal velocity noise (m/s)\n"
     "    gps_latency_s: float = 0.1         # [ESTIMATE] measurement age when it arrives (s)\n"
     "\n"
     "    # ---- New-ideas study switches (defaults = earlier behaviour) ----\n"
     "    gimbal_offset_x_deg: float = 0.0   # actual gimbal = command + this (trim error)\n"
     "    gimbal_offset_y_deg: float = 0.0\n"
     "    wind_oracle: bool = False          # feed the TRUE wind to the firmware (WIND_MATCH bound)\n"
     "    wind_oracle_scale: float = 1.0     # ...times this (estimate scale error)\n"
     "    wind_oracle_rot_deg: float = 0.0   # ...rotated by this (direction error)\n"
     "    wind_oracle_add: float = 0.0       # ...plus this along the wind (m/s)\n")
edit(os.path.join(sim, "vehicle.py"),
     "    landing_thrust_scale: float = 1.0    # multiplies landing thrust (impulse scales too)\n",
     "    landing_thrust_scale: float = 1.0    # multiplies landing thrust (impulse scales too)\n"
     "    ascent_thrust_scale: float = 1.0     # multiplies ascent thrust (motor-to-motor spread)\n")
edit(os.path.join(sim, "plant.py"),
     "        self.ascent_motor = bt.Motor(vcfg.ascent_eng_path)\n",
     "        _as = getattr(vcfg, \"ascent_thrust_scale\", 1.0)\n"
     "        self.ascent_motor = (bt.Motor(vcfg.ascent_eng_path) if _as == 1.0\n"
     "                             else bt.Motor(vcfg.ascent_eng_path, scale=_as))\n")
# setup_motors.py builds ws/m_<motor> from ws/fix next to itself.
if WORK != HERE:
    for f in ("drv.py", "setup_motors.py", "screen.py", "grid.py", "robust.py", "robust_gimbal.py", "holdout.py"):
        shutil.copy(os.path.join(HERE, f), WORK)
    shutil.copytree(os.path.join(HERE, "mdb"), os.path.join(WORK, "mdb"), dirs_exist_ok=True)
subprocess.run([PY, "-I", os.path.join(WORK, "setup_motors.py")], cwd=WORK, check=True)
print("workspaces ready in", os.path.join(WORK, "ws"))
