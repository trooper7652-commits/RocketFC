import json
import math
import os
import ideas

H = {'pad_hold': True, 'normal_force_translates': True}
FWE = dict(ideas.IMU, BOOST_DRIFT_GAIN='1.0f', WIND_MATCH='true', WIND_EST='true')


def load(label):
    return json.load(open(os.path.join(ideas.study.RES, label + '.json')))['rows']


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


for prior in ('2.0f', '1.0f'):
    fw = dict(FWE, WIND_EST_PRIOR_MPS=prior)
    for noise, base_label, pl in [('1x', 'nf_imu_best_hold', H),
                                  ('2x', 'rb_base_noise2x', dict(H, accel_noise_thrust=1.6))]:
        label = 'sh_p%s_n%s' % (prior[:-1], noise)
        rows = ideas.run(label, fw, plant=pl, ws='ideas2')
        ideas.show(label, rows)
        print('   paired vs %s: rescued %d lost %d p=%.4f' % ((base_label,) + ideas.paired(load(base_label), rows)))
        print(west(rows))
