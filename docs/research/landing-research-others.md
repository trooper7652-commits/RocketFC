# How other self-landing model rockets handled the landing burn

*Researched 2026-10-17 for RocketFC. "Flown" means a real flight; "sim only" means a simulation or a
design that hasn't flown. Earlier, broader research (Apollo, Falcon 9, Mars Pathfinder, Soyuz, KSP
autopilots, motor-variability data) is in `landing-solver-study\notes\01_research.md`.*

## The short answer
**No flown model rocket has landed a long-burn solid motor on ignition timing alone.**

The flown solution is to light the motor with **surplus thrust** (plan for less braking than the motor
gives), then **remove the surplus during the burn** in closed loop on a braking curve:
- **BPS.space** first did it by swinging the TVC side to side (2020);
- then with ceramic exhaust paddles that cut thrust (2022).

The second thing that kept sinking flights was **sideways position/velocity control** at touchdown.

## Projects

### BPS.space (Joe Barnard): flown
- **Echo (2019).** Drone drops from 30 m with an Estes F15, using pure timed ignition. It "nearly landed".
  Leg deployment vibrated the gyro and it touched down tilted. Planned fixes were better filtering and
  retiming ignition for a "higher hover".
- **Scout E (2020).** The plan was to ignite "at the moment that brings vertical velocity to zero at
  touchdown". It was abandoned because the delay from the ignition signal to peak thrust varied from flight
  to flight. Instead, the TVC was swung side to side so part of the thrust went sideways (a "throttle" via
  cos-loss). The rocket tipped over at touchdown from excessive horizontal movement, traced to a weak GPS
  signal (antenna placement) and a possible Kalman filter bug in the position/velocity estimate.
- **Scout F (2022).** Ceramic paddles ("throttle arms") move into the exhaust to cut thrust.
  - Flight 2 (May 2022): "rough position control on ascent" put it too far from the pad.
  - Flight 3: came in tilted after a wobbly ascent and "the guidance system couldn't get us back to the
    landing pad", though "throttle control is dialed in", with the plume length visibly changing.
  - **First successful propulsive landing: July 2022**, "a little more slam than hover; because of ignition
    variation and a fixed burn time, some landings need to touch down harder than others even with
    throttle control."
  - No later success rate has been published.
  - Scout F uses carbon-rod legs released by nichrome melting a rubber band (like RocketFC's legs), a
    machined aluminium TVC mount, and GPS for position.

### Project Horizon ("Eagle", "Hussar"): flown
- YouTube [@Project-Horizon](https://www.youtube.com/@Project-Horizon) videos:
  - "I Propulsively Landed a Model Rocket! – Eagle"
  - "This Model Rocket Lands Itself Better Now" (after leg upgrades)
  - "Propulsive Landing Model Rocket Attempts"
  - "Developing Reliable TVC Control – Hussar"
- Press: it took about **32 test flights** to land (The Awesomer, ~2023).
- **Method not public.** The channel states it does not share software, CAD files, simulations or other
  implementation details. Video descriptions couldn't be retrieved here. Watching "Propulsive Landing Model
  Rocket Attempts", or asking in the comments, is the only route to its burn-timing approach.

### JRD Propulsion (Aryan Kapoor): flown, partial
- Two stacked solids, about ±7° gimbal. The descent motor is ignited "when sensors indicate the proper
  altitude" (the same idea as RocketFC's table).
- Latest attempt landed upright, but the lower stage didn't separate.

### Open-source designs: sim only, none flown
- **[JamesGossy/TVC_Rocket_Firmware](https://github.com/JamesGossy/TVC_Rocket_Firmware)** ("in the style
  of BPS.space Scout and Project Horizon's Eagle"). Close to RocketFC: two F15-0s, BMI088/ICM-42688 IMU,
  MS5611/BMP390 baro, u-blox GPS, about 0.9 kg dry.
  - **Ignition rule:** light the landing motor once braking at **3.5 m/s²** would just stop the rocket at
    the ground, looking ahead by the igniter delay. That is deliberately less braking than the motor can
    give, so the paddles have room both ways.
  - **If tipped far over**, it waits for the tumble to swing back before igniting.
  - Paddles throttle to 60 % and follow a braking curve (open when falling too fast, close when too slow).
  - The gimbal (±10°) leans against sideways drift.
  - Velocity comes from the accelerometer, corrected by GPS.
  - **Its simulation:** 95 % good landings over 2,000 flights, and 98 % in wind under 2 m/s. **Without
    paddles the best was 42 %, because igniter timing (±180 ms) dominates.** Raising the CG by 5 cm drops
    it below 60 %, and tipping over is the main failure.
- **[chaseyarnall173/Self-Landing-Rocket](https://github.com/chaseyarnall173/Self-Landing-Rocket)**
  (not built). AeroTech F52 up and F25W down, fired about 15 m up.
  - Paddles shape a burn that starts strong and fades to near the rocket's weight at the end.
  - Assumes a 0.30 ± 0.08 s igniter delay.
  - 93 % clean landings in a 300-flight simulation.
- [CloudStas/Sliverflame-TVC_Rocket](https://github.com/CloudStas/Sliverflame-TVC_Rocket): F15s, not built,
  burn trigger unspecified.

### Academic
- **ISAE-SUPAERO, EuroGNC 2022** (Bourliatoux, Riera, Lustosa): "Definition of a Landing Strategy for a
  Model-Scale Reusable Rocket".
  - Solid, non-throttleable retro-landing: a passive aerodynamic descent, then a TVC-controlled touchdown
    using nonlinear model-predictive control.
  - Simulation only. Says no program had achieved touchdown with solid retro-propulsion.
- Ravin Joshi (ISEF): "artificial throttling" by vectoring away from vertical and circling to cancel the
  sideways push; 97.4 % in simulation.

## What this means for RocketFC (within its current limits)
| Lesson from others | RocketFC status |
|---|---|
| Throttle the motor after ignition (paddles / TVC wiggle). **This is the flown fix.** | Out of scope by your decision. |
| Lean against sideways drift during the burn (BPS's #2 failure) | Our **S3 drift loop**: essential in the sim study |
| Wait for a tumble to swing back before lighting | Our **D2** rule: small help |
| Measure the igniter delay. Every source names it as the dominant uncertainty. | Still `[MEASURE]`. ThrustStand, 10+ firings. |
| A burn that ends near the rocket's weight (energy match) | Our "fire at 17–21 m/s" finding |
| **GPS for sideways velocity** (BPS, both sim designs) | **Tested in the sim (2026-10-18):** with S3 + D2 + apogee x1.10, GPS cut the touchdown sideways speed from 2.4 to 0.6 m/s (median). Success after measuring rose from 0.55 % to 1.3 % (5.1 % in calm wind). Details: `software-landing-study\report.md`. |

## Software-only levers others have used (researched 2026-10-18)
What changed in software alone, with no new thrust hardware, and where RocketFC stands:

| Lever | Who / evidence | RocketFC status |
|---|---|---|
| **"Artificial throttling" by tilting the thrust away from vertical, circling the tilt to cancel the sideways push and tightening the circle near the ground** | BPS Scout E, flown 2020 (swung the existing TVC side to side; later replaced by paddles). Ravin Joshi ISEF, sim only: "utilizing control systems that are already present", 97.4 % in his sim after genetic-algorithm tuning, no physical landing yet. | **Tested 2026-10-18: no gain** (holdout: 0 landings rescued, 0 lost). The F15's ~14 N is 1.6× the weight, so 20–25° only removes 6–9 % of the thrust, the spare thrust shows up too late, and coning pushes sideways speed past the tip limit. See `software-landing-study\report.md`. Original estimate: Physics with a ±5° gimbal: 15° of body tilt sheds up to ~1.9 m/s over the burn (about the igniter-delay error), but the gimbal can only swing the body around so fast (≥1.4 s/rev), so it swings ±0.9 m/s sideways, right at the tip limit. It would have to shed early and straighten before touchdown. BPS abandoned it for paddles, which says it wasn't enough there. |
| Cancel sideways drift during the burn | BPS (GPS), JamesGossy sim | Tested: S3 drift loop, + GPS |
| Wait for the rocket to straighten before lighting | JRD flight (lit late while off-vertical, TVC straightened it); JamesGossy sim | Tested: D2 |
| Tune the burn/ignition calculation from flight logs | BPS: "retiming ignition for a higher hover" (Echo); "landed only 2 meters off the target altitude, flight computer got a bit too aggressive with its burn calculations" | Fire-margin sweep tested: it only trades hops for lift-offs. Real tuning needs real logs, i.e. measure. |
| Filter IMU vibration (leg deploy) | BPS Echo: leg deployment shook the gyro and it touched down tilted | Not modelled in the sim. Worth a bench test: log the gyro while the band burns and the legs snap open. |
| Fix the estimator (GPS/Kalman bugs) | BPS Scout E tipped from a GPS signal/Kalman issue | Covered by replay tests. Baro error near the ground from exhaust/flow is unquantified anywhere; log raw pressure in a burn. |
| Gain-scheduled LQR/LQI TVC (schedule on thrust/time/altitude) | Academic: arXiv 2303.16983 (solid-motor TVC testbed, LQI, gain-scheduled); 2026 landing-burn LQR paper (throttled vehicle) | Fixed PID landing gains. The gain sweep barely moved landing results; an LQR would mainly help tight attitude/drift tracking. |
| Re-plan using thrust measured in flight | 2026 solid-propellant lunar lander paper (sim): re-optimises onboard from in-flight thrust estimates | Only useful together with tilt-throttling (the only post-ignition "throttle" in software) |
| Nonlinear MPC for the TVC touchdown | ISAE-SUPAERO EuroGNC 2022, sim only | Not tried. Heavy for a Teensy, and no flown evidence. |
| Faster/more consistent igniter | Forum physics: bridge-wire heating ∝ current², so more current gives a faster light. No evidence for a software "pre-heat" pulse. | Hardware/measurement, not software |

Extra sources:
- [Joshi ISEF abstract](https://abstracts.societyforscience.org/Home/PrintPdf/22455)
- [arXiv 2303.16983 (TVC + state estimation for small solid launchers)](https://arxiv.org/pdf/2303.16983)
- [Gain-scheduled LQR landing (Processes 2026)](https://doi.org/10.3390/pr14152458)
- [Solid-propellant lunar lander re-optimisation (2026)](https://link.springer.com/article/10.1007/s42405-026-01247-y)
- [Review of SRM VTVL control algorithms](https://www.researchgate.net/publication/365874186_A_Review_of_Various_Control_Algorithms_Used_for_a_Solid_Rocket_Motor_to_Achieve_Vertical_Takeoff_and_Vertical_Landing)
- [BPS "burn calculations" post](https://www.facebook.com/bps.space/videos/propulsive-landing-test-2-rocket-landed-only-2-meters-off-the-target-altitude-fl/703263196680198/)
- [Hackster: JRD landing](https://hackster.io/news/right-back-where-we-started-from-649b1bd7f87c)
- [PopSci: JRD](https://popsci.com/science/model-rocket-landing)
- [UConn Propulsive Landing](https://aiaa.engr.rso.uconn.edu/pl)
- [RocketryForum: igniters & e-matches](https://www.rocketryforum.com/threads/ingiters-and-e-matches.44085/)
- [RocketryForum: barometer installation](https://www.rocketryforum.com/goto/post?id=26090)

## Sources
- Hackaday: [So Close To Landing A Model Rocket On Its Tail](https://hackaday.com/2020/11/27/so-close-to-landing-a-model-rocket-on-its-tail/) (2020)
- Hackaday: [BPS.Space Successfully Lands A Model Rocket](https://hackaday.com/2022/08/05/bps-space-succesfully-lands-a-model-rocket/) (2022)
- BPS.space on X:
  - [Scout F first landing](https://twitter.com/bps_space/status/1553103061135945728)
  - [Flight 2](https://x.com/bps_space/status/1530612626945806336)
  - [Flight 3](https://x.com/bps_space/status/1545881708452466688)
  - [Echo drop test](https://x.com/bps_space/status/1133058789572251649)
- [Space Explored](https://spaceexplored.com/2022/08/01/bps-model-rocket-landing-video/), [The Drive](https://www.thedrive.com/news/watch-this-homebuilt-model-rocket-land-like-a-real-spacex-falcon-9), [Flite Test podcast](https://www.flitetest.com/articles/podcast-landing-model-rockets-ft-bps-space), [Polytechforum (Echo)](https://www.polytechforum.com/model-rocketry/dropped-from-a-drone-this-model-rocket-nearly-lands-on-its-1628224-.htm)
- Project Horizon videos:
  - [Eagle landing](https://www.youtube.com/watch?v=56R21lG72Ek)
  - [Lands Itself Better Now](https://www.youtube.com/watch?v=I9hSAVelYLk)
  - [Landing attempts](https://www.youtube.com/watch?v=xWQDZPSco-E)
  - [Hussar TVC](https://www.youtube.com/watch?v=t9nslflu2A4)
- [The Awesomer: Mini Falcon 9 Rocket Lands After Take-off](https://theawesomer.com/mini-falcon-9-rocket-lands-after-take-off/719721/)
- [Fabbaloo: JRD Propulsion](https://fabbaloo.com/news/design-of-the-week-propulsive-landing-rocket)
- [EuroGNC 2022 paper page](https://eurognc.ceas.org/archive/EuroGNC2022/html/CEAS-GNC-2022-020.html)
- [Joshi ISEF abstract](https://abstracts.societyforscience.org/Home/PrintPdf/22455)
- [Project Insight (Elon University)](https://new.express.adobe.com/webpage/cjzely4LFtGNu)
