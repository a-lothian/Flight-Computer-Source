// DeploymentCorrector.hpp
//
// the FLOWN control law
//
// one predictor call per tick. take the apogee error at the current
// deployment, turn it into a deployment correction using a locally measured
// sensitivity, and apply a fraction of that correction. the closed loop does
// the root finding across successive ticks, no per-tick root find is attempted
//
// at 100 Hz over a 15 to 20 s coast there are 1500 to 2000 ticks. converging a
// root every 10 ms is unnecessary and does not fit the compute budget, one
// prediction per tick does
//
// sensitivity = d(apogee)/d(deployment), and it varies strongly through the
// coast. early on the brakes have seconds to act, near apogee they have almost
// none, so a fixed gain would be sluggish at one end and oscillatory at the
// other. it is re-measured by finite difference every N ticks instead, which
// costs one extra predictor call on those ticks only
//
#pragma once
#include <cstdint>
#include "airbrake/ApogeePredictor.hpp"

namespace airbrake {

struct CorrectorResult {
    float deployment_raw;     // [0,1] requested deployment, pre-bias
    float apogee_error_m;     // predicted - target, at the CURRENT deployment
    float sensitivity;        // m of apogee per unit of deployment, negative
    bool  sensitivity_stale;  // true if this tick reused a cached value
    bool  finite;             // false means the caller treats it as an internal fault
    uint16_t predictor_steps; // worst case across the calls made this tick
};

class DeploymentCorrector {
public:
    DeploymentCorrector() = default;
    void init(const ApogeePredictor* predictor,
              float target_apogee_agl_m,
              float gain_fraction,
              uint8_t sensitivity_refresh_ticks,
              float sensitivity_probe_delta);

    // clears the cached sensitivity and the tick counter. call on mode changes
    // so a resumed loop re-measures instead of trusting a pre-fault value
    void reset();

    CorrectorResult update(float altitude_agl_m,
                           float velocity_up_mps,
                           float speed_mps,
                           float mass_kg,
                           float current_deployment);

private:
    const ApogeePredictor* predictor_ = nullptr;
    float   target_apogee_agl_m_ = 0.0f;
    float   gain_fraction_ = 0.0f;
    float   probe_delta_ = 0.0f;
    uint8_t refresh_ticks_ = 0;
    uint8_t ticks_since_refresh_ = 0;
    float   cached_sensitivity_ = 0.0f;
    bool    have_sensitivity_ = false;
};

}  // namespace airbrake
