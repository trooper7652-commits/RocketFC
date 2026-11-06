import math
import ideas

H = {'pad_hold': True, 'normal_force_translates': True}
FWE = dict(ideas.IMU, BOOST_DRIFT_GAIN='1.0f', WIND_MATCH='true', WIND_EST='true')
CNA = ['normal_force_coeff:0.7:1.3']
GOFF = ['gimbal_offset_x_deg:-1:1', 'gimbal_offset_y_deg:-1:1']
cases = [
    ('cna30', {}, CNA),
    ('noise2x', {'accel_noise_thrust': 1.6}, None),
    ('goff05', {'gimbal_offset_x_deg': 0.5, 'gimbal_offset_y_deg': 0.5}, GOFF),
    ('all3', {'accel_noise_thrust': 1.6, 'gimbal_offset_x_deg': 0.5, 'gimbal_offset_y_deg': 0.5}, CNA + GOFF),
]


def west(rows):
    err = []
    for r in rows:
        s = r['summary']
        we, wt = s['bo_wind_est'], s['bo_wind_true']
        sp = math.hypot(*wt)
        if sp > 0.5:
            ang = math.degrees(math.atan2(we[1], we[0]) - math.atan2(wt[1], wt[0]))
            err.append((math.hypot(*we) / sp, abs((ang + 180) % 360 - 180)))
    return ('   est/true magnitude p10/50/90: %.2f %.2f %.2f' % tuple(ideas.q([e[0] for e in err], p) for p in (.1, .5, .9))
            + '  direction error p50/90: %.1f %.1f deg' % tuple(ideas.q([e[1] for e in err], p) for p in (.5, .9)))


for name, pl, ext in cases:
    p = dict(H, **pl)
    base = ideas.run('rb_base_' + name, ideas.IMU, plant=p, ext=ext)
    est = ideas.run('rb_est_' + name, FWE, plant=p, ext=ext)
    ideas.show('base ' + name, base)
    ideas.show('est  ' + name, est)
    print('   paired est vs base: rescued %d lost %d p=%.3f' % ideas.paired(base, est))
    print(west(est))
