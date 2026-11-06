"""B3b: shrink the wind estimate toward 0 while it is uncertain (Bayesian prior),
with the sideways-force noise measured in flight. Applied to sw/ideas2."""
import patchlib
import os
patchlib.WS = os.path.join(patchlib.SCR, "sw", "ideas2")
from patchlib import edit

edit("src/config.h",
"""constexpr float WIND_EST_MIN_SPEED = 4.0f;       // m/s airspeed before samples count
""",
"""constexpr float WIND_EST_MIN_SPEED = 4.0f;       // m/s airspeed before samples count
// Prior spread of the wind (m/s): while the estimate is still noisy it is pulled
// toward 0 by its own uncertainty, so the boost doesn't lean on vibration noise.
// The noise itself is measured in flight (successive-sample differences). 0 = off.
constexpr float WIND_EST_PRIOR_MPS = 0.0f;
""")
edit("src/core/flight_core.h",
"""    const float wgt = k * k;
    windSx_ += wgt * wx;
    windSy_ += wgt * wy;
    windSw_ += wgt;
    windX_ = cfg::WIND_EST_SCALE * windSx_ / windSw_;
    windY_ = cfg::WIND_EST_SCALE * windSy_ / windSw_;
  }
""",
"""    const float wgt = k * k;
    windSx_ += wgt * wx;
    windSy_ += wgt * wy;
    windSw_ += wgt;
    float shrink = 1.0f;
    if (cfg::WIND_EST_PRIOR_MPS > 0.0f) {
      // Per-sample sideways-force noise variance from successive differences.
      if (windNn_ > 0) {
        const float ddx = nx - windPnx_, ddy = ny - windPny_;
        windNv_ += 0.25f * (ddx * ddx + ddy * ddy);
        ++windNn_;
      } else {
        windNn_ = 1;
      }
      windPnx_ = nx;
      windPny_ = ny;
      const float nv = windNn_ > 1 ? windNv_ / (float)(windNn_ - 1) : 1.0f;
      const float pv = cfg::WIND_EST_PRIOR_MPS * cfg::WIND_EST_PRIOR_MPS;
      shrink = windSw_ / (windSw_ + nv / pv);
    }
    windX_ = cfg::WIND_EST_SCALE * shrink * windSx_ / windSw_;
    windY_ = cfg::WIND_EST_SCALE * shrink * windSy_ / windSw_;
  }
""")
edit("src/core/flight_core.h",
"""  float windSx_ = 0.0f, windSy_ = 0.0f, windSw_ = 0.0f;
""",
"""  float windSx_ = 0.0f, windSy_ = 0.0f, windSw_ = 0.0f;
  float windPnx_ = 0.0f, windPny_ = 0.0f, windNv_ = 0.0f;
  int windNn_ = 0;
""")
