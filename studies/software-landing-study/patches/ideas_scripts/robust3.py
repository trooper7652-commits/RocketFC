"""Stress cases for the wind+offset estimator (sw/ideas3), paired with the same
cases without any wind matching (rb_base_* / nf_imu_best_hold, workspace ideas)."""
import json
import math
import os
import sys
import ideas

H = {'pad_hold': True, 'normal_force_translates': True}
FW3 = dict(ideas.IMU, BOOST_DRIFT_GAIN='1.0f', WIND_MATCH='true', WIND_EST='true',
           WIND_EST_PRIOR_MPS='2.0f', WIND_EST_OFFSET_PRIOR_DEG='1.0f')
CNA = ['normal_force_coeff:0.7:1.3']
GOFF = ['gimbal_offset_x_deg:-1:1', 'gimbal_offset_y_deg:-1:1']
cases = [
    ('nominal', 'nf_imu_best_hold', {}, None),
    ('cna30', 'rb_base_cna30', {}, CNA),
    ('noise2x', 'rb_base_noise2x', {'accel_noise_thrust': 1.6}, None),
    ('goff05', 'rb_base_goff05', {'gimbal_offset_x_deg': 0.5, 'gimbal_offset_y_deg': 0.5}, GOFF),
    ('all3', 'rb_base_all3', {'accel_noise_thrust': 1.6, 'gimbal_offset_x_deg': 0.5,
                              'gimbal_offset_y_deg': 0.5}, CNA + GOFF),
]
tag = sys.argv[1] if len(sys.argv) > 1 else 'r3'
fw = dict(FW3, **dict(a.split('=') for a in sys.argv[2:]))


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


ONLY = os.environ.get('CASES', '').split(',') if os.environ.get('CASES') else None
for name, base_label, pl, ext in cases:
    if ONLY and name not in ONLY:
        continue
    rows = ideas.run('%s_%s' % (tag, name), fw, plant=dict(H, **pl), ext=ext, ws='ideas3')
    ideas.show('%s %s' % (tag, name), rows)
    print('   paired vs %s: rescued %d lost %d p=%.4f' % ((base_label,) + ideas.paired(load(base_label), rows)))
    print(west(rows), flush=True)
