# 01 — Research: how others time a landing burn they can't throttle

Collected 2026-10-12. Status labels: **FLOWN** (worked on a real vehicle), **FLOWN-PARTIAL**,
**GAME** (tested in Kerbal Space Program, a physics game, not reality), **SIMULATED**,
**PROPOSED/IN DEVELOPMENT**. "Unverified" means I could only see a secondary summary
(search-engine snippet or press article), not the primary source. Nothing here is invented;
where a page was blocked (403) I say so.

Plain-language terms used below:
- **Suicide burn / hoverslam**: light the engine as late as possible so you reach zero speed exactly
  at the ground, instead of slowing early and hovering down.
- **Predictor–corrector**: repeatedly *predict* where you'll end up if you act now ("if I light at
  this instant, what will my touchdown speed be?"), then *correct* the plan.
- **Throttle margin**: planning to need less than full thrust, so spare thrust can absorb surprises.
- **Dead time / lag**: a delay between reality and what a sensor (or the filter) reports.

## Summary table

| # | Source | Status | Vehicle / motor (throttleable?) | How the burn is timed | Inputs | Maps to RocketFC? |
|---|---|---|---|---|---|---|
| 1 | BPS.space Scout F (Joe Barnard) | **FLOWN** (1st success Jul 2022) | Model rocket, Estes **F15** (no) | Pure timing **abandoned**; 2020: TVC side-to-side wiggle to cut vertical thrust; 2022: **ceramic paddles in the exhaust** ("throttle control") | Flight-computer state (velocity, attitude, altitude) | Proves timing alone wasn't enough with F15 variation; success needed post-ignition thrust modulation (hardware) |
| 2 | BPS.space Echo drone drops | Tested, "nearly landed" (secondary) | F15 (no) | Drop from 30 m, light motor, settle | same | Same vehicle class; failures were legs/vibration, not timing |
| 3 | Aryan Kapoor / JRD Propulsion (2024) | **FLOWN-PARTIAL** (landed upright, stage didn't separate) | Two stacked solids, ±7° gimbal (no) | "Descent motor ignited when sensors indicate the proper altitude" | Altimeter | Same idea as our table; no code/numbers found |
| 4 | UConn AIAA "Propulsive Landing" | IN DEVELOPMENT | TVC model rocket | Unknown (repo behind SSO) | — | — |
| 5 | Ravin Joshi (ISEF abstract) | **SIMULATED** (97.4 % sim success); hardware built, not landed | Solid + TVC (no) | "Artificial throttling": vector thrust away from vertical, circle to cancel sideways push | sim state | Needs large vehicle tilt; our 5° gimbal alone can't do it |
| 6 | Ryan Westcott (ISEF abstracts) | Built + ground-tested; landing result not stated | 4 one-axis-gimballed solids (no) | Throttle by canting motors | — | Hardware approach |
| 7 | kOS `dd_hoverslam.ks` (jberkman, MIT) | **GAME** ("risky but works") | KSP liquid engine (**yes**) | Start when needed decel (v²/2h + g) = full thrust; then **closed-loop throttle** (×1.01), hover fallback | radar alt, speed, thrust, g | Start rule = our table's physics; the rest needs throttle |
| 8 | MechJeb2 landing autopilot | **GAME** (widely used) | KSP (**yes**) | Forward simulation + speed limit with **10–20 % margins**: v_max = 0.9·√(2(a−g)h) | sim state | Margin philosophy; a solid can't hold a throttle margin |
| 9 | kRPC Python suicide-burn script (KSP forum) | **GAME** (unverified: page 403, seen via search summary) | KSP (yes) | Iterate burn start time until predicted end-of-burn altitude is "slightly positive" | sim state | Predictor–corrector on ignition time = our candidate A |
| 10 | Apollo LM descent guidance (Klumpp, MIT/Draper R-695 1971; Automatica 1974) | **FLOWN** (Apollo 11–17) | LM descent engine, throttleable 10–60 % + fixed max | Braking phase at fixed max throttle; thrust uncertainty ≈ +1/−2 %; trajectory **biased so throttle-down happens early even for a low-thrust motor** → margin kept | IMU + landing radar | Bias the plan toward the side you can still correct; with no throttle, our only bias lever is the aim point |
| 11 | Falcon 9 landing (Blackmore, NAE Frontiers 2016) | **FLOWN** | Merlin, T/W > 1 at min throttle (yes, limited) | "The rocket has to hit zero velocity at exactly zero altitude"; onboard convex optimization (CVXGEN), recomputed as disturbances act | GPS/IMU/radar | Closest professional analog to the hop cliff — but keeps throttle authority during the burn |
| 12 | G-FOLD (Açıkmeşe & Ploen, JGCD 2007; Masten Xombie tests 2012–13) | **FLOWN** (throttleable VTVL) | Throttle bounds required | Real-time convex trajectory optimization | full state | Not applicable (no throttle); concept of feasibility envelope only |
| 13 | ZEM/ZEV guidance (Battin; Guo, Hawkins, Wie 2013) | SIMULATED / analysis | Continuous thrust | Predict "zero-effort miss" (outcome if you do nothing) and correct | full state | The prediction idea maps to "predicted touchdown speed if fired now" |
| 14 | Smith predictor (O.J.M. Smith, 1957) | Industrial practice | — | Predict the plant through a known dead time using a model | model + delay | = candidate B (and A through the ignition delay) |
| 15 | Complementary/Kalman baro–inertial fusion (Higgins, IEEE TAES 1975) | Practice | — | Accel for fast changes, baro for slow truth | accel, baro | Basis of the KF and of candidate B2 |
| 16 | **Mars Pathfinder RAD rockets** (1997) | **FLOWN** | Solid retro-rockets (no) | Fired at 98 m, 61.2 m/s; **designed to stop 2–22 m (nominal 12 m) ABOVE the surface**, then bridle cut → free fall into airbags; hit at 18 m/s (airbags) | radar altimeter | Real "risk-aware aim point": put the error band on the side your landing system tolerates |
| 17 | MER RAD/TIRS (2004) | **FLOWN** | Solid RAD + transverse solids (no) | Onboard software computes RAD ignition (~90 m) from radar altitude & velocity (details unverified) | radar, IMU, DIMES camera | Predictor-based ignition decision, plus horizontal-velocity correction |
| 18 | Soyuz soft-landing engines | **FLOWN** (hundreds) | Six solid motors (no) | Fired ≈ 0.7–1 m above ground by an altimeter (gamma "Kaktus": unverified); touchdown < 2 m/s (ESA) | proximity altimeter | Works because the burn is tiny and triggered at ~1 m; not our 3.45 s burn from 30–70 m |

## Data on motor and igniter variability (for realistic dispersions)

- **ThrustCurve.org, Estes F15**: 49.6 N·s, 14.4 N average, 25.3 N peak, 3.5 s, 60 g propellant,
  black powder, NAR certification 2012-11-25. Two certification-derived RASP curves exist with
  **different shapes**: Coker (2013, "same totals as the averaged NAR static tests", 0→25.3 N peak at
  0.42 s) and Smart (2013, faster rise, 26.75 N peak at 0.39 s, thrust from 0.06 s). Our repo's
  `Estes_F15.eng` is the Coker curve with a simplified, slightly higher tail (50.4 N·s, +1.6 %).
  → The plant and the solver both use our file; the cert curves are realistic "truth" perturbations.
- **Certification limits** (NFPA 1125, 2001 edition, as paraphrased by ThrustCurve): ≥ 10 samples,
  total-impulse standard deviation ≤ 6.7 %, average thrust within ±20 %.
- **F15-specific spread**: a search-engine summary of the NAR F15 certification PDF reported
  12 motors, total impulse SD ≈ 0.83 N·s (1.7 %), peak thrust 23–31.5 N (SD ≈ 20 %).
  **Unverified**: the PDF is blocked to direct download (403), and one 2013 contributor wrote that
  the PDF then contained E16 data by mistake. Treat 1.7 % as optimistic and 6.7 % as the
  certification bound.
- **E-match / ignition delay**: an e-match head fires in milliseconds (J-Tek sheet: minimum firing
  pulse 10 ms at 12 V, all-fire 0.75 A). The motor's own light-up (pyrogen → propellant → pressure)
  dominates; a RocketryForum poster reports ~140 ms typical for Estes igniters in black-powder
  motors (anecdotal). **No controlled data found** — `IGNITION_DELAY_MS` and its spread must be
  measured on the ThrustStand. The sim's 500 ms ± 15 % (uniform) is an assumption, not data.
- **MS5611 barometer**: OSR 2048 conversion 4.13 ms typical (4.54 max), 0.018 mbar RMS
  (~0.15 m). So the chip itself adds ~4–5 ms; the sim's 25 ms lag must stand for bay venting
  and filtering — also a `[MEASURE]` item.
- **Baro error from airflow**: aviation references describe position error and "over-reading in a
  descent" from static-port lag; rocketry sources are anecdotal. No quantitative source found, so
  it is tested as a sensitivity range (±5–10 % of dynamic pressure; at 20 m/s that's ±1–2 m!).

## What this means for a timing-only solver on RocketFC

1. **Nobody has demonstrated precise landings by open-loop timing of a long-burn solid motor alone.**
   BPS.space tried exactly that with the same F15 and abandoned it ("the ignition time and exact
   thrust numbers simply weren't repeatable enough"); the successful 2022 flight added exhaust
   paddles, and even then was "a little more slam than hover". Flown solid-retro landers either
   absorb the error with hardware (Pathfinder airbags, Soyuz seats and a very short last-metre burn)
   or correct it after ignition (MER transverse rockets, BPS paddles).
2. **What transfers to a pure timing solver** (no hardware change):
   - *Predict through every known delay* (Smith predictor): the KF/baro lag before deciding, and the
     ignition delay after deciding (candidates B and A).
   - *Put the error band on the cheap side of the cost curve* (Apollo's biased trajectory,
     Pathfinder's "stop 2–22 m up", MechJeb's margins, the newsvendor "critical fractile" rule).
     With an asymmetric cost, the best aim is a *quantile* of the error distribution, not its middle
     (candidate C).
   - *Use the actual vertical thrust component*: thrust along a tilted body loses cos(tilt)
     (candidate D).
3. **What would really fix it needs approval** (outside the solver): post-ignition impulse dumping
   (BPS 2020 TVC wiggle; Joshi's simulated circular vectoring). With a ±5° gimbal, gimbal angle alone
   wastes ≤ 0.4 % (cos 5°); a useful dump needs the *vehicle* to tilt 15–30°, i.e. a guidance/control
   change. Logged as candidate E: only to be implemented after the user approves.

## Sources
- Hackaday, "BPS.Space Succesfully Lands A Model Rocket" (2022-08-05): https://hackaday.com/2022/08/05/bps-space-succesfully-lands-a-model-rocket/
- Hackaday, "So Close To Landing A Model Rocket On Its Tail" (2020-11-27): https://hackaday.com/2020/11/27/so-close-to-landing-a-model-rocket-on-its-tail/
- Space Explored, BPS landing video (2022-08-01): https://spaceexplored.com/2022/08/01/bps-model-rocket-landing-video/
- BPS.space on X (2022-07-29, quoted via search result, not fetched): https://twitter.com/bps_space/status/1553103061135945728
- Polytechforum (Echo drone drop, secondary): https://www.polytechforum.com/model-rocketry/dropped-from-a-drone-this-model-rocket-nearly-lands-on-its-1628224-.htm
- Kapoor / JRD: https://popsci.com/science/model-rocket-landing (403), https://hackster.io/news/right-back-where-we-started-from-649b1bd7f87c (403), https://fabbaloo.com/news/design-of-the-week-propulsive-landing-rocket
- UConn: https://aiaa.engr.rso.uconn.edu/pl ; UT Austin: https://thedailytexan.com/2023/01/25/its-not-rocket-science-ut-students-plan-to-bring-rockets-back-down-to-earth/
- ISEF abstracts: https://abstracts.societyforscience.org/Home/PrintPdf/22455 (Joshi), /17242 and /19309 (Westcott)
- kOS hoverslam: https://github.com/jberkman/kOSScripts (dd_hoverslam.ks)
- MechJeb2: https://github.com/MuMech/MechJeb2 (MechJebModuleLandingAutopilot.cs)
- kRPC suicide burn thread (403): https://forum.kerbalspaceprogram.com/topic/188920-python-based-suicide-burn-autopilot-using-the-krpc-mod
- Klumpp, Apollo Lunar-Descent Guidance, MIT/Draper R-695 (1971): https://ntrs.nasa.gov/citations/19740044219 ; https://www.ibiblio.org/apollo/Documents/Apollo%20Lunar%20Descent%20Guidance.pdf
- Blackmore, "Autonomous Precision Landing of Space Rockets", NAE Frontiers of Engineering 2016: https://www.nationalacademies.org/read/23659/chapter/10
- Açıkmeşe & Ploen, JGCD 30(5) 2007; G-FOLD/Xombie flight tests: https://nescacademy.nasa.gov/video/eda2b96bddf945629be2c9d2e3bc87231d
- ZEM/ZEV: https://arxiv.org/pdf/2003.02182 (survey/extension citing Guo, Hawkins, Wie 2013)
- Smith predictor: https://en.wikipedia.org/wiki/Smith_predictor
- Higgins 1975: https://allaboutcircuits.com/uploads/articles/A_comparison_of_complementary_and_kalman_filtering.pdf
- Mars Pathfinder EDL: https://pds.nasa.gov/data/mpfl-m-imp-2-edr-v1.0/mpim_0002/document/mission.htm ; https://pds.nasa.gov/data/mpfl-m-asimet-3-rdr-surf-v1.0/mpam_0001/document/edler_ds.htm
- MER EDL (algorithm details unverified): https://ntrs.nasa.gov/api/citations/20040037788/downloads/20040037788.pdf
- Soyuz landing: https://www.esa.int/Science_Exploration/Human_and_Robotic_Exploration/PromISSe/Way_back_to_Earth ; https://spaceflightnow.com/station/exp29/exp29land.html
- Newsvendor / critical fractile: https://en.wikipedia.org/wiki/Newsvendor_model
- ThrustCurve F15: https://www.thrustcurve.org/motors/Estes/F15/ ; certification info: https://www.thrustcurve.org/info/certification.html
- NAR F15 cert PDF (403 to direct fetch): https://www.nar.org/wp-content/uploads/2014/08/F15.pdf
- E-match spec: https://electricmatch.com/upload/JTEK_TECHNICAL_INFORMATION_SHEET_SPECIFICATIONS.PDF ; igniter delay anecdote (~140 ms): a RocketryForum post surfaced by search among these threads — exact thread not verified: https://www.rocketryforum.com/threads/airstart-motor-ignition.161576/ , https://www.rocketryforum.com/threads/second-stage-ignition.162088/
- MS5611 datasheet: https://www.embeddedadventures.com/datasheets/ms5611.pdf
