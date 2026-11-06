import os
import ideas

H = {'pad_hold': True, 'normal_force_translates': True}
FWE = dict(ideas.IMU, BOOST_DRIFT_GAIN='1.0f', WIND_MATCH='true', WIND_EST='true')
F15 = os.path.join(ideas.SCR, 'sw', 'ideas', 'tools', 'motors', 'Estes_F15.eng')
base = ideas.run('nf_imu_best_hold', ideas.IMU, plant=H)
est = ideas.run('nf_B3_est_s0.8', FWE, plant=H)

print('--- fire margin with the wind-matched ascent (measured spreads)')
for m in ('0.0f', '0.5f', '1.5f', '2.0f'):
    rows = ideas.run('mg_est_m' + m[:-1], dict(FWE, BURN_TABLE_MARGIN_M=m), plant=H)
    ideas.show('est, margin ' + m, rows)
    print('   paired vs est margin 1.0: rescued %d lost %d p=%.3f' % ideas.paired(est, rows))

print("--- today's spreads (mass/e-match delay not measured)")
tb = ideas.run('td_base', ideas.IMU, plant=H, disp={})
te = ideas.run('td_est', FWE, plant=H, disp={})
ideas.show('today base', tb)
ideas.show('today est', te)
print('   paired: rescued %d lost %d p=%.3f' % ideas.paired(tb, te))

print("--- today's apogee (no stronger ascent motor)")
ab = ideas.run('ap100_base', ideas.IMU, plant=dict(H, ascent_eng_path=F15))
ae = ideas.run('ap100_est', FWE, plant=dict(H, ascent_eng_path=F15))
ideas.show('today apogee base', ab)
ideas.show('today apogee est', ae)
print('   paired: rescued %d lost %d p=%.3f' % ideas.paired(ab, ae))

print("--- GPS (hardware) with the realistic sim, for reference")
g = ideas.run('nf_gps_best_hold', ideas.GPS, plant=H)
ideas.show('GPS best, hold', g)
print('   paired vs IMU best hold: rescued %d lost %d p=%.3f' % ideas.paired(base, g))
ge = ideas.run('nf_gps_est', dict(ideas.GPS, BOOST_DRIFT_GAIN='1.0f', WIND_MATCH='true', WIND_EST='true'), plant=H)
ideas.show('GPS + est', ge)
print('   paired vs IMU est: rescued %d lost %d p=%.3f' % ideas.paired(est, ge))

print('--- vertical-channel oracle bounds (what knowing X exactly would be worth)')
for label, disp in [('or_delay0', {'mass_descent_kg': 0.01, 'ignition_delay_s': 0.0}),
                    ('or_cda0', {'mass_descent_kg': 0.01, 'ignition_delay_s': 0.05, 'cda_m2': 0.0}),
                    ('or_delay0_cda0', {'mass_descent_kg': 0.01, 'ignition_delay_s': 0.0, 'cda_m2': 0.0})]:
    rows = ideas.run(label, FWE, plant=H, disp=disp)
    ideas.show(label, rows)
    print('   paired vs est: rescued %d lost %d p=%.3f' % ideas.paired(est, rows))
