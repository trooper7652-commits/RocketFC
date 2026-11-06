"""One-shot holdout for the headline idea, fresh seeds 501-510 (2,000 flights per row).
Pre-registered bar: the wind-matched ascent must rescue significantly more landings
than it loses (sign test p < 0.05) vs. the same setup without it, and not raise aborts."""
import sys
import ideas

margin = sys.argv[1] if len(sys.argv) > 1 else '1.0f'
H = {'pad_hold': True, 'normal_force_translates': True}
FWE = dict(ideas.IMU, BOOST_DRIFT_GAIN='1.0f', WIND_MATCH='true', WIND_EST='true', WIND_EST_PRIOR_MPS='1.0f',
           BURN_TABLE_MARGIN_M=margin)
SEEDS = tuple(range(501, 511))
for spreads, disp in [('measured', ideas.MEAS), ('today', {})]:
    b = ideas.run('hold_%s_base' % spreads, ideas.IMU, plant=H, disp=disp, seeds=SEEDS)
    e = ideas.run('hold_%s_est_m%s' % (spreads, margin[:-1]), FWE, plant=H, disp=disp, seeds=SEEDS, ws='ideas2')
    ideas.show('HOLDOUT %s base' % spreads, b)
    ideas.show('HOLDOUT %s wind-matched' % spreads, e)
    print('   paired: rescued %d lost %d p=%.4f' % ideas.paired(b, e))
