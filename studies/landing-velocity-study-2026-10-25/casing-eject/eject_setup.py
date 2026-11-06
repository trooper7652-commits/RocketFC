"""Casing-eject study: build the scratch workspaces and patch in the model.

The landing motor's exhaust pushes the spent ascent casing out of the tail at
landing thrust onset. Each workspace is a copy of an earlier study workspace
with
  - tools/sim: VehicleConfig.ascent_casing_eject_kg (0 = casing stays aboard,
    bit-identical), the CG/inertia/ground-geometry shift at eject, and an
    optional kick bracket (ascent_casing_kick_ns);
  - tools/burn_table.py: --eject-kg (the mass drops at thrust onset, the
    delay phase keeps the full mass);
  - optionally src/core/burn_table.h regenerated with --eject-kg.
"""
import os
import shutil
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
PY = r"C:\Users\troop\AppData\Local\Programs\Python\Python312\python.exe"
F10_CASING = 0.0830 - 0.0400            # AeroTech F10 total - propellant (kg)
E12_CASING = 0.0594 - 0.0303 + 0.010    # spent E12-RCJ + R/C case (earlier study's mass) + 24->29 adapter
# name: (source workspace, landing .eng relative to the workspace, eject kg or None = keep the old table)
WORKSPACES = {
    "base_eject_old": ("base", None, None),
    "base_eject": ("base", None, F10_CASING),
    "c1_eject": ("c1_e12_e12", os.path.join("tools", "motors", "landing.eng"), E12_CASING),
}


def sub(path, old, new):
    t = open(path, encoding="utf-8").read()
    assert t.count(old) == 1, (path, old[:60])
    open(path, "w", encoding="utf-8", newline="\n").write(t.replace(old, new))


def patch_vehicle(p):
    sub(p, """    landing_time_scale: float = 1.0      # stretches the landing burn, impulse kept
""", """    landing_time_scale: float = 1.0      # stretches the landing burn, impulse kept
    # Casing-eject study: the landing motor's exhaust pushes the spent ascent
    # casing out of the tail at landing thrust onset. 0 = it stays aboard.
    ascent_casing_eject_kg: float = 0.0
    ascent_casing_cg_from_nose_m: float = 0.0  # 0 = tail minus half the ascent .eng length
    ascent_casing_kick_ns: float = 0.0   # net impulse change from the eject, over the first 0.15 s
""")


def patch_plant(p):
    sub(p, "import math\n", "import dataclasses\nimport math\n")
    sub(p, """        self._pad_mass_kg = vcfg.mass_descent_kg + self.ascent_motor.prop_kg
""", """        self._pad_mass_kg = vcfg.mass_descent_kg + self.ascent_motor.prop_kg

        # Casing eject (casing-eject study): the mass drop is in
        # mass_and_thrust(); _eject_casing() moves the CG and trims the inertia.
        self._eject_kg = getattr(vcfg, "ascent_casing_eject_kg", 0.0)
        self._ejected = False
        self._inertia_cut = 0.0
        self._kick_k = 0.0
        self._geo = vcfg
        if self._eject_kg > 0:
            self._casing_x = (getattr(vcfg, "ascent_casing_cg_from_nose_m", 0.0) or
                              vcfg.length_m - 0.5 * _eng_length_m(vcfg.ascent_eng_path))
            kick = getattr(vcfg, "ascent_casing_kick_ns", 0.0)
            if kick:
                self._kick_k = kick / max(self.landing_motor.impulse(KICK_S), 1e-6)
""")
    sub(p, """            if self._land_k == 1.0:
                thrust += self.landing_motor.thrust(tb)
                m -= self.landing_motor.mass_burned(tb)
""", """            if self._land_k == 1.0:
                lt = self.landing_motor.thrust(tb)
                if self._kick_k and 0.0 <= tb < KICK_S:
                    lt *= 1.0 + self._kick_k
                thrust += lt
                m -= self.landing_motor.mass_burned(tb)
""")
    sub(p, """        return max(m, 0.05), thrust

""", """            if self._eject_kg > 0 and tb >= 0.0:
                m -= self._eject_kg
        return max(m, 0.05), thrust

    def _eject_casing(self):
        \"\"\"The landing exhaust has pushed the spent ascent casing out of the
        tail: the CG moves toward the nose, the inertia loses the casing's
        parallel-axis share, and the ground model gets the lighter rocket's
        standing geometry (CG higher on the legs).\"\"\"
        self._ejected = True
        v, s = self.v, self.state
        mc = self._eject_kg
        m_after = self.mass_and_thrust(s.t)[0]
        m_before = m_after + mc
        d = self._casing_x - v.cg_from_nose_m      # casing aft of the CG (m)
        shift = mc * d / m_after
        cg = v.cg_from_nose_m - shift
        self._gimbal_arm_m = v.gimbal_pivot_from_nose_m - cg
        self._z_cp_rel_cg = cg - v.cp_from_nose_m
        i_before = v.inertia_kgm2 * (m_before / self._pad_mass_kg)
        i_after = i_before - mc * d * d - m_after * shift * shift
        self._inertia_cut = v.inertia_kgm2 * (m_after / self._pad_mass_kg) - i_after
        if self.ground is not None:
            m_end = v.mass_descent_kg - self.landing_motor.prop_kg - mc
            shift_l = mc * d / m_end
            self._geo = dataclasses.replace(
                v, cg_from_nose_m=v.cg_from_nose_m - shift_l,
                landed_cg_height_m=v.landed_cg_height_m + shift_l)
            g = ground_mod.Ground(self._geo)
            g.set_slope(self.ground.slope_deg, self.ground.slope_dir_deg)
            g.anchor = self.ground.anchor
            self.ground = g

""")
    sub(p, """        m, thrust = self.mass_and_thrust(s.t)
        # Inertia scales""", """        if (self._eject_kg > 0 and not self._ejected and
                s.landing_ignite_t is not None and s.t >= s.landing_ignite_t):
            self._eject_casing()
        m, thrust = self.mass_and_thrust(s.t)
        # Inertia scales""")
    sub(p, "        inertia = v.inertia_kgm2 * (m / self._pad_mass_kg)\n",
        "        inertia = v.inertia_kgm2 * (m / self._pad_mass_kg) - self._inertia_cut\n")
    sub(p, """        v = self.v
        st = dict(self._g)""", """        v = self._geo
        st = dict(self._g)""")
    sub(p, """class PlantState:""", """KICK_S = 0.15   # casing-eject kick bracket: spread over the first 0.15 s of thrust


def _eng_length_m(path):
    \"\"\"Motor length from a RASP .eng header line (mm -> m).\"\"\"
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.strip() and not line.lstrip().startswith(";"):
                return float(line.split()[2]) / 1000.0
    raise ValueError("no .eng header in " + path)


class PlantState:""")


def patch_burn_table(p):
    sub(p, """def sim_burn(motor, m_descent, cda, delay_s, h0, v_down, dt=0.002):""",
        """def sim_burn(motor, m_descent, cda, delay_s, h0, v_down, dt=0.002, eject_kg=0.0):""")
    sub(p, """        m = m_descent - motor.mass_burned(tb)
        drag_a""", """        m = m_descent - motor.mass_burned(tb) - (eject_kg if tb >= 0.0 else 0.0)
        drag_a""")
    sub(p, """def find_h_ignite(motor, m, cda, delay_s, v_down):""",
        """def find_h_ignite(motor, m, cda, delay_s, v_down, eject_kg=0.0):""")
    sub(p, """        vi, _, _ = sim_burn(motor, m, cda, delay_s, h, v_down, dt=0.004)""",
        """        vi, _, _ = sim_burn(motor, m, cda, delay_s, h, v_down, dt=0.004, eject_kg=eject_kg)""")
    sub(p, """        vi, _, _ = sim_burn(motor, m, cda, delay_s, h, v_down, dt=0.002)""",
        """        vi, _, _ = sim_burn(motor, m, cda, delay_s, h, v_down, dt=0.002, eject_kg=eject_kg)""")
    sub(p, """    ap.add_argument("--vmin\"""", """    ap.add_argument("--eject-kg", type=float, default=0.0,
                    help="spent ascent casing pushed out at landing thrust onset (kg)")
    ap.add_argument("--vmin\"""")
    sub(p, """    dv = motor.total_impulse / args.mass - G * motor.burn_time""",
        """    ej = args.eject_kg
    dv = motor.total_impulse / (args.mass - ej) - G * motor.burn_time""")
    sub(p, """        h, vi = find_h_ignite(motor, args.mass, args.cda, delay_s, v)""",
        """        h, vi = find_h_ignite(motor, args.mass, args.cda, delay_s, v, eject_kg=ej)""")
    sub(p, """//   mass at descent: {args.mass:.3f} kg   CdA""",
        """//   mass at descent: {args.mass:.3f} kg (minus {ej:.4f} kg casing at thrust onset)   CdA""")
    sub(p, """            vi, _, peak = sim_burn(motor, args.mass, args.cda, delay_s, h0, vd)""",
        """            vi, _, peak = sim_burn(motor, args.mass, args.cda, delay_s, h0, vd, eject_kg=ej)""")
    sub(p, """                vi, _, _ = sim_burn(m2, args.mass, args.cda, dly, h0, vd)""",
        """                vi, _, _ = sim_burn(m2, args.mass, args.cda, dly, h0, vd, eject_kg=ej)""")


if __name__ == "__main__":
    for name, (src, eng, eject) in WORKSPACES.items():
        ws = os.path.join(HERE, "ws", name)
        if os.path.isdir(ws):
            print(name, "exists, skipped")
            continue
        shutil.copytree(os.path.join(HERE, "ws", src), ws,
                        ignore=shutil.ignore_patterns("build", "*.dll", "__pycache__"))
        patch_vehicle(os.path.join(ws, "tools", "sim", "vehicle.py"))
        patch_plant(os.path.join(ws, "tools", "sim", "plant.py"))
        patch_burn_table(os.path.join(ws, "tools", "burn_table.py"))
        line = "old table kept"
        if eject is not None:
            cmd = [PY, os.path.join("tools", "burn_table.py"), "--eject-kg", "%.4f" % eject]
            if eng:
                cmd += ["--eng", eng]
            r = subprocess.run(cmd, cwd=ws, capture_output=True, text=True)
            if r.returncode:
                raise SystemExit(r.stderr)
            out = r.stdout.splitlines()
            line = " | ".join(l for l in out if "delta-v" in l or "Target apogee" in l)
        print(name, "from", src, "| eject", eject, "|", line)
