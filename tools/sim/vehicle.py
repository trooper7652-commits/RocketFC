"""Vehicle/environment/sensor parameters for the closed-loop simulator.

Every field here is a plain, independently-editable number with a unit, a
[MEASURE] or [ESTIMATE] tag, and a comment on how to get the real value --
same convention as src/config.h and ../../tvc-sim/tvc_pid_tuner.py's Config
block. Nothing here is derived/hidden behind a property: the GUI binds one
control per field, and save_json()/load_json() round-trip the whole set, so
values can be replaced one at a time as the airframe gets built and measured
without touching code.

Two fields (mass_pad_kg, mass_descent_kg, cda_m2) default from the FIRMWARE's
own src/config.h via the bridge's config snapshot rather than being retyped
here, so the sim's default plant and the flight computer's own assumptions
never silently drift apart. Override them freely once you have your own
numbers -- they're independent fields after construction, not a live link.
"""
import dataclasses
import json
import math
import os

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
MOTORS_DIR = os.path.join(os.path.dirname(SIM_DIR), "motors")
DEFAULT_ENG = os.path.join(MOTORS_DIR, "Estes_F15.eng")


def estimate_inertia_kgm2(mass_kg, length_m):
    """Uniform-rod first cut: I ~= (1/12) * m * L^2. Documented starting point
    only -- refine with a bifilar (two-string) pendulum measurement:
        I = (m * g * d^2 * T^2) / (16 * pi^2 * Ls)
    where d = half the string separation, Ls = string length, T = period of a
    small torsional oscillation. See ../../tvc-sim/README.md for the method;
    once measured, just type the number into inertia_kgm2 below -- it is a
    plain field, not recomputed from mass/length automatically.
    """
    return (1.0 / 12.0) * mass_kg * length_m * length_m


@dataclasses.dataclass
class VehicleConfig:
    # ---- Mass & geometry ---------------------------------------------------
    # [MEASURE] Defaults mirror src/config.h so the sim starts from what the
    # flight computer itself already assumes; see default_vehicle_config().
    mass_pad_kg: float = 1.10          # on the pad, everything loaded (kg)
    mass_descent_kg: float = 0.95      # after ascent burnout, F15 still loaded (kg)

    length_m: float = 1.20             # [ESTIMATE] overall airframe length (m):
                                       #   nose tip to tail. Measure directly.
    diameter_m: float = 0.075          # [ESTIMATE] body tube OD (m); only used
                                       #   for the aero reference area below.

    # Distances measured from the NOSE TIP, along the body axis (m). Balance
    # the built rocket on a knife edge to find the CG; measure the gimbal
    # pivot and an assumed CP location with a tape measure from the nose.
    cg_from_nose_m: float = 0.66              # [MEASURE] balance point
    gimbal_pivot_from_nose_m: float = 1.15    # [MEASURE] thrust application point
    cp_from_nose_m: float = 0.50              # [ESTIMATE] see note below

    # This airframe has no fins in the CAD (SpaceX Assembly.iam) -- it is a
    # fin-less body relying on ACTIVE TVC for stability, so the center of
    # pressure is expected to sit AHEAD of the CG (cp_from_nose_m <
    # cg_from_nose_m, i.e. aerodynamically unstable open-loop, same reason
    # every real landing-booster needs active control). If you add fins,
    # cp_from_nose_m should move aft of cg_from_nose_m and this sign flips --
    # the sim doesn't assume either way, it just uses the numbers you give it.

    # [MEASURE/ESTIMATE] Transverse moment of inertia about the CG (kg*m^2).
    # THE most important number -- the whole attitude response scales with
    # it. Default is the uniform-rod estimate below; refine with a bifilar
    # pendulum (see estimate_inertia_kgm2's docstring) and just type the
    # measured value in here -- this field is not recomputed automatically.
    inertia_kgm2: float = dataclasses.field(
        default_factory=lambda: estimate_inertia_kgm2(1.10, 1.20))

    # ---- Aerodynamics --------------------------------------------------------
    cda_m2: float = 0.0040             # [MEASURE/estimate] drag Cd*A, no chute (m^2)
    cda_chute_m2: float = 0.45         # [ESTIMATE] drag Cd*A under canopy (m^2)
                                       #   (~0.45 -> ~5.5 m/s terminal at this mass)
    chute_inflation_s: float = 0.4     # canopy inflation ramp time (s)
    # Failure injection: how many spring-latch releases fail before the chute
    # actually comes out (0 = the first release works). Exercises the flight
    # computer's accelerometer-confirmed re-cycle (src/core/chute_deploy.h).
    chute_stuck_releases: int = 0

    # ---- Landing legs ------------------------------------------------------
    # The flight computer powers a nichrome wire from fire + LEGS_DELAY_MS
    # until touchdown; the legs are out once the wire has burned through the
    # rubber band and they have swung down. The sim doesn't change the
    # physics for this -- it reports how long before ground contact the legs
    # were out (run.py's legs_margin_s; negative = too late).
    legs_cut_s: float = 1.5            # [MEASURE] nichrome on -> band parts (s);
                                       #   time it on the bench: `pyrotest legs`
    legs_swing_s: float = 0.3          # [ESTIMATE] band parts -> legs fully down (s)
    # Destabilizing/restoring aero torque coefficient: torque = dynamic
    # pressure * reference area * (cp_from_nose_m - cg_from_nose_m) * angle of
    # attack (small-angle). [ESTIMATE] -- this is a coarse model; treat sim
    # aero-instability results qualitatively, not as a precise margin number.
    aero_damping_nms: float = 0.02     # [ESTIMATE] rotational aero damping (N*m*s/rad)

    # Constant disturbance torques the TVC loop must reject, same model as
    # ../../tvc-sim/tvc_pid_tuner.py: a thrust-misalignment/CG-offset torque
    # that scales with instantaneous thrust (it is PRODUCED by the thrust, so
    # it vanishes at burnout), plus a small independent roll (body Z) torque
    # since nothing in this airframe actively controls roll.
    disturbance_torque_nm: float = 0.05    # [ESTIMATE] about body X, worst-case bias
    disturbance_ref_thrust_n: float = 0.0  # torque quoted at this thrust (N); 0 = auto (mean thrust)
    roll_disturbance_torque_nm: float = 0.01  # [ESTIMATE] small constant torque about body Z
    normal_force_coeff: float = 2.0    # [ESTIMATE] CN_alpha, per-radian slender-body normal-
                                       #   force slope (2.0/rad is a standard rough default);
                                       #   treat aero-instability results qualitatively

    # ---- Motors --------------------------------------------------------------
    ascent_eng_path: str = DEFAULT_ENG   # RASP .eng file, ascent motor
    landing_eng_path: str = DEFAULT_ENG  # RASP .eng file, landing (F15) motor
    ignition_delay_s: float = 0.5        # [MEASURE on ThrustStand] e-match -> thrust (s)

    # ---- Gimbal / servo --------------------------------------------------
    # NOTE: firmware's own GIMBAL_MAX_RAD (software clamp) and the
    # SERVO_*_SIGN / *_CENTER_US / *_US_PER_GIMBAL_DEG mapping come from
    # src/config.h via the bridge (core.FlightCore.config_snapshot) -- they
    # are not duplicated here. These are the physical servo characteristics
    # the firmware has no knowledge of:
    servo_slew_dps: float = 400.0       # [MEASURE] max servo slew rate (deg/s);
                                        #   a "0.10 s/60deg" servo ~= 600 deg/s,
                                        #   derate for the gimbal linkage
    servo_lag_tau_s: float = 0.008      # [MEASURE] first-order lag time constant (s);
                                        #   bench-measure the step response

    # ---- Environment -----------------------------------------------------
    wind_mps: float = 0.0               # steady horizontal wind (m/s)
    gust_mps: float = 0.0               # optional gust magnitude (m/s)
    gust_time_s: float = 0.0            # gust start time (s), 0 = disabled
    gust_dur_s: float = 0.5             # gust duration (s)

    # ---- Sensor noise/bias -------------------------------------------------
    # Defaults mirror tools/synth_flight.py's model so a sim run and the
    # existing replay-case generator agree on what "realistic sensors" means.
    accel_noise_thrust: float = 0.8     # accel noise std-dev while thrust > 1 N (m/s^2)
    accel_noise_quiet: float = 0.12     # accel noise std-dev otherwise (m/s^2)
    gyro_noise_thrust_dps: float = math.degrees(0.005)   # synth_flight.py's sg while T>1N
    gyro_noise_quiet_dps: float = math.degrees(0.0015)   # synth_flight.py's sg otherwise
    gyro_bias_dps: tuple = (0.3, -0.2, 0.15)      # constant per-axis gyro bias (deg/s)
    accel_bias_mps2: tuple = (0.03, -0.02, 0.04)  # constant per-axis accel bias (m/s^2)
    baro_noise_pa: float = 2.0          # barometer pressure noise std-dev (Pa)
    baro_lag_s: float = 0.025           # barometer response lag (s)

    def to_json(self, path):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(dataclasses.asdict(self), f, indent=2, sort_keys=True)

    @classmethod
    def from_json(cls, path):
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        for tup_field in ("gyro_bias_dps", "accel_bias_mps2"):
            if tup_field in data:
                data[tup_field] = tuple(data[tup_field])
        return cls(**data)


def default_vehicle_config(dll_path=None):
    """VehicleConfig with mass_pad_kg / mass_descent_kg / cda_m2 pulled live
    from the firmware's own src/config.h (via the bridge), so the sim's
    starting point matches what the flight computer already assumes."""
    cfg = VehicleConfig()
    if dll_path is not None:
        import core as _core  # local import: avoids a hard dependency for
                              # callers that only want the dataclass shape.
        snap = _core.FlightCore.config_snapshot(dll_path)
        cfg.mass_pad_kg = snap["massPadKg"]
        cfg.mass_descent_kg = snap["massDescentKg"]
        cfg.cda_m2 = snap["cdaM2"]
        cfg.ignition_delay_s = snap["ignitionDelayMs"] / 1000.0
        cfg.inertia_kgm2 = estimate_inertia_kgm2(cfg.mass_pad_kg, cfg.length_m)
    return cfg
