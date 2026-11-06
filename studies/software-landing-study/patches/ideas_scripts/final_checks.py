import json, math, os
import ideas

H = {'pad_hold': True, 'normal_force_translates': True}
FWS = dict(ideas.IMU, BOOST_DRIFT_GAIN='1.0f', WIND_MATCH='true', WIND_EST='true', WIND_EST_PRIOR_MPS='1.0f')
CNA = ['normal_force_coeff:0.7:1.3']
GOFF = ['gimbal_offset_x_deg:-1:1', 'gimbal_offset_y_deg:-1:1']
def load(l): return json.load(open(os.path.join(ideas.study.RES, l + '.json')))['rows']
cases = [
    ('cna30', None, {}, CNA),
    ('goff05', None, {'gimbal_offset_x_deg': 0.5, 'gimbal_offset_y_deg': 0.5}, GOFF),
    ('goff02', 'make', {'gimbal_offset_x_deg': 0.2, 'gimbal_offset_y_deg': 0.2}, GOFF),
    ('all3', None, {'accel_noise_thrust': 1.6, 'gimbal_offset_x_deg': 0.5, 'gimbal_offset_y_deg': 0.5}, CNA + GOFF),
    ('all3_g02', 'make', {'accel_noise_thrust': 1.6, 'gimbal_offset_x_deg': 0.2, 'gimbal_offset_y_deg': 0.2}, CNA + GOFF),
]
for name, mk, pl, ext in cases:
    p = dict(H, **pl)
    base = ideas.run('rb_base_' + name, ideas.IMU, plant=p, ext=ext) if mk else load('rb_base_' + name)
    rows = ideas.run('shp1_' + name, FWS, plant=p, ext=ext, ws='ideas2')
    ideas.show('base ' + name, base)
    ideas.show('shrink p1 ' + name, rows)
    print('   paired: rescued %d lost %d p=%.4f' % ideas.paired(base, rows), flush=True)
