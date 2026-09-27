// DeploymentCorrector.cpp

// Alters deployment value to minimise projected apogee error
//
// how high will I go at this point in time?
// what can I do to get that projection closer to the target?
//
// sensitivity = apogee / deployment
// (if I change deployment x amount, apogee changes by y)
// 
// sensitivity is calculated as a slope between two apogee predictions
// (at_high, at_low, found with current_deployment +/- probe_delta_)
//
// sensitivity is computed every refresh_ticks_ to save compute
// consecutive ticks are likely close together temporally and
// therefore sensitivity would not update meaningfully.
// 

#include "airbrake/DeploymentCorrector.hpp"
#include "airbrake/AirbrakeConfig.hpp"
#include "airbrake/Validate.hpp"

namespace airbrake {

void DeploymentCorrector::init(const ApogeePredictor* predictor,
                                float target_apogee_agl_m,
                                float gain_fraction,
                                uint8_t sensitivity_refresh_ticks,
                                float sensitivity_probe_delta) {
    predictor_ = predictor;
    target_apogee_agl_m_ = target_apogee_agl_m;
    gain_fraction_ = gain_fraction;
    refresh_ticks_ = sensitivity_refresh_ticks;
    probe_delta_ = sensitivity_probe_delta;
    reset();
}

void DeploymentCorrector::reset() {
    have_sensitivity_ = false;
    ticks_since_refresh_ = 0;
    cached_sensitivity_ = 0.0f;
}

CorrectorResult DeploymentCorrector::update(float altitude_agl_m,
                                            float velocity_up_mps,
                                            float speed_mps,
                                            float mass_kg,
                                            float current_deployment) {
    CorrectorResult result{};
    result.deployment_raw = current_deployment;
    result.finite = false;

    if (predictor_ == nullptr) {
        return result;
    }
    // compute as locals + validate before updating internal state
    const ApogeePrediction at_current =
        predictor_->predictAtDeployment(altitude_agl_m, velocity_up_mps, speed_mps,
                                        mass_kg, current_deployment);

    uint16_t worst_steps = at_current.steps;

    if (!(at_current.finite && validate::isFinite(at_current.apogee_agl_m))) {
        result.predictor_steps = worst_steps;
        return result;
    }

    const float apogee_error_m = at_current.apogee_agl_m - target_apogee_agl_m_;

    // refresh on first tick
    bool refresh_now = false;
    if (!have_sensitivity_) {
        refresh_now = true;
    } else {
        ticks_since_refresh_ = static_cast<uint8_t>(ticks_since_refresh_ + 1u);
        if (ticks_since_refresh_ >= refresh_ticks_) {
            refresh_now = true;
        }
    }

    float sensitivity = cached_sensitivity_;
    bool sensitivity_ok = have_sensitivity_;

    if (refresh_now) {
        // probe points are clamped [0-1]
        // span is recomputed after points areclamped to avoid
        // clamped values at either end introducing error

        bool clamp_ok = true;
        const float probe_high =
            validate::clampChecked(current_deployment + probe_delta_,
                                   config::kDeployMin, config::kDeployMax,
                                   config::kDeployMax, &clamp_ok);
        const float probe_low =
            validate::clampChecked(current_deployment - probe_delta_,
                                   config::kDeployMin, config::kDeployMax,
                                   config::kDeployMin, &clamp_ok);

        const float probe_span = probe_high - probe_low;

        const ApogeePrediction at_high =
            predictor_->predictAtDeployment(altitude_agl_m, velocity_up_mps, speed_mps,
                                            mass_kg, probe_high);
        const ApogeePrediction at_low =
            predictor_->predictAtDeployment(altitude_agl_m, velocity_up_mps, speed_mps,
                                            mass_kg, probe_low);

        if (at_high.steps > worst_steps) { worst_steps = at_high.steps; }
        if (at_low.steps > worst_steps)  { worst_steps = at_low.steps; }

        // reject predictions if step limited, as they are not correct apogee
        const bool probes_usable =
            at_high.finite && at_low.finite && clamp_ok &&
            !at_high.step_limit_hit && !at_low.step_limit_hit &&
            validate::isFinite(at_high.apogee_agl_m) &&
            validate::isFinite(at_low.apogee_agl_m) &&
            probe_span > 0.0f;

        if (probes_usable) {
            const float measured =
                (at_high.apogee_agl_m - at_low.apogee_agl_m) / probe_span;
            if (validate::isFinite(measured)) {
                sensitivity = measured;
                sensitivity_ok = true;
            }
        }

        ticks_since_refresh_ = 0;
    }

    result.apogee_error_m = apogee_error_m;
    result.sensitivity = sensitivity;
    result.sensitivity_stale = !refresh_now;
    result.predictor_steps = worst_steps;

    if (!sensitivity_ok) {
        return result;
    }

    // near apogee brakes have little effect -> sensitivity may round to 0
    // guard for div/0 + sensitivity should always be negative

    float correction = 0.0f;
    if (sensitivity <= -config::kMinSensitivity_mPerUnit) {
        correction = -gain_fraction_ * apogee_error_m / sensitivity;
    }

    bool command_ok = true;
    const float requested = validate::clampChecked(current_deployment + correction,
                                                   config::kDeployMin, config::kDeployMax,
                                                   config::kNeutral, &command_ok);

    result.deployment_raw = requested;
    result.finite = command_ok && validate::isFinite(requested);

    // cache sensitivity with valid value
    if (result.finite && sensitivity_ok) {
        cached_sensitivity_ = sensitivity;
        have_sensitivity_ = true;
    }

    return result;
}

}  // namespace airbrake
