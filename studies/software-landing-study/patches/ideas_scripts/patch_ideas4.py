"""Sim fidelity switch: let the airframe's normal force push the rocket (and
show up on the accelerometer), not just turn it."""
from patchlib import edit

edit("tools/sim/vehicle.py",
"""    pad_hold: bool = False             # pad cradle/clamp: no rotation before ascent ignition
""",
"""    pad_hold: bool = False             # pad cradle/clamp: no rotation before ascent ignition
    normal_force_translates: bool = False  # aero normal force also pushes the CG (and the
                                           # accelerometer sees it); False = torque only
""")
edit("tools/sim/plant.py",
"""        aero_torque_body = aero_torque_body - s.omega * v.aero_damping_nms
""",
"""        aero_torque_body = aero_torque_body - s.omega * v.aero_damping_nms
        if (getattr(v, "normal_force_translates", False) and
                s.ascent_ignite_t is not None and s.touchdown_t is None):
            drag_world = drag_world + s.q.rotate(normal_force_body)
""")
