from patchlib import edit as _e


class P:
    edit = staticmethod(_e)


P.edit("tools/sim/vehicle.py",
"""    wind_oracle: bool = False          # feed the TRUE wind to the firmware (WIND_MATCH bound)
""",
"""    wind_oracle: bool = False          # feed the TRUE wind to the firmware (WIND_MATCH bound)
    wind_oracle_scale: float = 1.0     # ...times this (estimate scale error)
    wind_oracle_rot_deg: float = 0.0   # ...rotated by this (direction error)
    wind_oracle_add: float = 0.0       # ...plus this along the wind (m/s)
""")
P.edit("tools/sim/run.py",
"""            if getattr(vcfg, "wind_oracle", False):
                _w = pl._wind(t)
                fc.wind_est(_w.x, _w.y)
""",
"""            if getattr(vcfg, "wind_oracle", False):
                _w = pl._wind(t)
                _sp = math.hypot(_w.x, _w.y)
                _k = vcfg.wind_oracle_scale + (vcfg.wind_oracle_add / _sp if _sp > 1e-6 else 0.0)
                _r = math.radians(vcfg.wind_oracle_rot_deg)
                fc.wind_est(_k * (_w.x * math.cos(_r) - _w.y * math.sin(_r)),
                            _k * (_w.x * math.sin(_r) + _w.y * math.cos(_r)))
""")
