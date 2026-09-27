// OvershootBias.hpp
//
// deliberate asymmetry. under-brake early, approach full authority late
//
// over-braking early is unrecoverable as energy cannot be added back.
// under-braking early is recoverable, the authority is still there later in
// the coast
//
// the multiplier used to absorb forward-Euler truncation error as well, which
// biased predictions in a known direction. RK4 removed that bias, so the
// schedule now represents the risk asymmetry and nothing else
//
#pragma once

namespace airbrake {

class OvershootBias {
public:
    // progress is normalised coast progress, altitude / target_apogee, clamped
    // to [0,1]. 0 near burnout, 1 at the target
    static float factor(float progress);

    static float apply(float deployment_raw, float progress);
};

}  // namespace airbrake
