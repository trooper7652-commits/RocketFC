"""Sensor model: PlantState (truth) -> noisy IMU/baro readings, in the SENSOR
frame -- exactly mirroring tools/synth_flight.py's noise model (same accel/
gyro sigmas that scale with thrust, same ~25 ms baro lag) so a sim run and the
existing replay-case generator agree on what "realistic sensors" means.

Readings are generated in the sensor frame and mapped into the body frame via
the bridged cfg::IMU_R_SB (see core.FlightCore's caller in run.py), the same
way the real firmware's sensors.h does -- so a wrong mount matrix or a wrong
servo sign shows up in the sim as a diverging control loop, not as something
silently corrected for it.
"""
import math
import random

P0_PA = 101325.0


class SensorModel:
    def __init__(self, vcfg, seed=42):
        self.v = vcfg
        self.rng = random.Random(seed)
        self._baro_lag_alt = 0.0

    def imu(self, specific_force_body, omega_body, dt, thrust_n):
        """specific_force_body: true specific force in the BODY frame (m/s^2),
        i.e. what an ideal accelerometer bolted to the body would read (thrust
        reaction minus gravity's world-frame contribution is NOT subtracted --
        an accelerometer measures specific force, matching worldVerticalAccel's
        own convention in src/core/ahrs.h). omega_body: true body rates (rad/s).
        Returns ((ax,ay,az), (gx,gy,gz)) with noise + bias applied, as the
        SENSOR would report it (before any IMU_R_SB mount-matrix mapping).
        """
        v = self.v
        sig_a = v.accel_noise_thrust if thrust_n > 1.0 else v.accel_noise_quiet
        sig_g = math.radians(v.gyro_noise_thrust_dps if thrust_n > 1.0
                             else v.gyro_noise_quiet_dps)
        bx, by, bz = v.accel_bias_mps2
        gbx, gby, gbz = (math.radians(d) for d in v.gyro_bias_dps)
        ax = specific_force_body.x + bx + self.rng.gauss(0, sig_a)
        ay = specific_force_body.y + by + self.rng.gauss(0, sig_a)
        az = specific_force_body.z + bz + self.rng.gauss(0, sig_a)
        gx = omega_body.x + gbx + self.rng.gauss(0, sig_g)
        gy = omega_body.y + gby + self.rng.gauss(0, sig_g)
        gz = omega_body.z + gbz + self.rng.gauss(0, sig_g)
        return (ax, ay, az), (gx, gy, gz)

    def baro(self, true_alt_m, dt_since_last_sample):
        """One barometer sample: applies the lag filter, then returns a noisy
        pressure reading (Pa) for the given TRUE altitude AGL."""
        v = self.v
        alpha = dt_since_last_sample / max(v.baro_lag_s, 1e-6)
        alpha = min(alpha, 1.0)
        self._baro_lag_alt += (true_alt_m - self._baro_lag_alt) * alpha
        p = P0_PA * (1.0 - min(self._baro_lag_alt, 40000.0) / 44330.0) ** 5.2553
        return p + self.rng.gauss(0, v.baro_noise_pa)
