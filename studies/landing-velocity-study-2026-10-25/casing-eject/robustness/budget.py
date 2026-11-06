"""Part 1: error budget. One uncertainty removed at a time, paired on dev seeds 101-103."""
import sys
sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
import lvlib as L

AIRFRAME = {"inertia_kgm2": 0, "cg_from_nose_m": 0, "gimbal_pivot_from_nose_m": 0,
            "servo_slew_dps": 0, "servo_lag_tau_s": 0}
QUIET = {"accel_noise_thrust": 0.0, "accel_noise_quiet": 0.0, "gyro_noise_thrust_dps": 0.0,
         "gyro_noise_quiet_dps": 0.0, "baro_noise_pa": 0.0, "gyro_bias_dps": [0.0, 0.0, 0.0],
         "accel_bias_mps2": [0.0, 0.0, 0.0]}
NONE = dict(AIRFRAME, mass_descent_kg=0, ignition_delay_s=0, cda_m2=0)

if __name__ == "__main__":
    R = {}
    R["base (measured)"] = L.run("b_base")
    R["- mass spread"] = L.run("b_nomass", disp={"mass_descent_kg": 0, "ignition_delay_s": 0.05})
    R["- delay spread"] = L.run("b_nodelay", disp={"mass_descent_kg": 0.01, "ignition_delay_s": 0})
    R["- drag spread"] = L.run("b_nodrag", disp=dict(L.MEAS, cda_m2=0))
    R["- airframe guesses"] = L.run("b_noairframe", disp=dict(L.MEAS, **AIRFRAME))
    R["- wind"] = L.run("b_nowind", wind=0)
    R["- sensor noise/bias"] = L.run("b_nosensors", plant=QUIET)
    R["+ F15 spread (sigma 1.7 %)"] = L.run("b_f15spread", ext=["landing_thrust_scale:0.97:1.03"])
    R["perfect knowledge"] = L.run("b_perfect", disp=NONE, plant=QUIET, wind=0)
    print()
    L.table(R, "base (measured)")
