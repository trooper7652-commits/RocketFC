"""What a live bench flight should go through. Times are seconds after the
Launch command (when the simulated ascent motor lights); a fault with
duration 0 is off.
"""
import dataclasses
import json


@dataclasses.dataclass
class Scenario:
    mode: str = "chute"                 # "chute" (CHUTE_TEST) or "land" (FULL_LANDING)
    seed: int = 1                       # sensor noise + field slope
    # -- wind --
    wind_mps: float = 0.0               # steady, along +x
    gust_mps: float = 0.0
    gust_at_s: float = 0.0
    gust_s: float = 0.0
    # -- motors --
    dud_landing_igniter: bool = False   # fire command issued, motor never lights
    landing_thrust_pct: float = 100.0
    ignition_delay_s: float = -1.0      # e-match -> thrust; < 0 = vehicle default (0.62 s)
    # -- chute --
    chute_stuck_releases: int = 0       # the first N latch releases don't free the canopy
    # -- sensor faults --
    imu_dropout_at_s: float = 0.0       # no frames at all (link loss -> IMU fail)
    imu_dropout_s: float = 0.0
    imu_frozen_at_s: float = 0.0        # the same IMU sample repeated (frozen-data check)
    imu_frozen_s: float = 0.0
    baro_offset_m: float = 0.0          # added to the baro altitude
    baro_offset_at_s: float = 0.0
    baro_offset_s: float = 0.0
    baro_noise_m: float = 0.0           # extra Gaussian noise (std, m)
    baro_noise_at_s: float = 0.0
    baro_noise_s: float = 0.0
    baro_dropout_at_s: float = 0.0      # no new baro samples
    baro_dropout_s: float = 0.0
    # -- hardware --
    live_legs: bool = False             # really power the leg nichrome (bench, opt-in)
    max_flight_s: float = 40.0          # stop this long after launch regardless

    def window(self, name, t_rel):
        """True while fault `name` (e.g. "imu_dropout") is active at t_rel."""
        at = getattr(self, name + "_at_s")
        dur = getattr(self, name + "_s")
        return dur > 0 and t_rel is not None and at <= t_rel < at + dur

    def faults(self):
        """Short human list of what's injected (for titles and reports)."""
        out = []
        if self.wind_mps:
            out.append("wind {:g} m/s".format(self.wind_mps))
        if self.gust_mps and self.gust_s > 0:
            out.append("gust {:g} m/s @ {:g} s".format(self.gust_mps, self.gust_at_s))
        if self.dud_landing_igniter:
            out.append("dud igniter")
        if self.landing_thrust_pct != 100.0:
            out.append("landing thrust {:g} %".format(self.landing_thrust_pct))
        if self.ignition_delay_s >= 0:
            out.append("ignition delay {:g} s".format(self.ignition_delay_s))
        if self.chute_stuck_releases:
            out.append("{} stuck release(s)".format(self.chute_stuck_releases))
        for name, label in (("imu_dropout", "IMU dropout"), ("imu_frozen", "frozen IMU"),
                            ("baro_offset", "baro offset {:g} m".format(self.baro_offset_m)),
                            ("baro_noise", "baro noise {:g} m".format(self.baro_noise_m)),
                            ("baro_dropout", "baro dropout")):
            if getattr(self, name + "_s") > 0:
                out.append("{} @ {:g}-{:g} s".format(
                    label, getattr(self, name + "_at_s"),
                    getattr(self, name + "_at_s") + getattr(self, name + "_s")))
        if self.live_legs:
            out.append("LIVE LEGS")
        return out or ["nominal"]

    def to_dict(self):
        return dataclasses.asdict(self)

    @classmethod
    def from_dict(cls, d):
        names = {f.name for f in dataclasses.fields(cls)}
        return cls(**{k: v for k, v in d.items() if k in names})

    def save(self, path):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, indent=1)

    @classmethod
    def load(cls, path):
        with open(path, encoding="utf-8") as f:
            return cls.from_dict(json.load(f))
