"""Robustness knob: a constant gimbal zero error (servo trim / thrust misalignment)."""
from patchlib import edit

edit("tools/sim/vehicle.py",
"""    normal_force_translates: bool = False  # aero normal force also pushes the CG (and the
""",
"""    gimbal_offset_x_deg: float = 0.0   # actual gimbal = command + this (trim error)
    gimbal_offset_y_deg: float = 0.0
    normal_force_translates: bool = False  # aero normal force also pushes the CG (and the
""")
edit("tools/sim/run.py",
"""            pl.step(dt, act_x, act_y)
""",
"""            pl.step(dt, act_x + math.radians(vcfg.gimbal_offset_x_deg),
                    act_y + math.radians(vcfg.gimbal_offset_y_deg))
""")
