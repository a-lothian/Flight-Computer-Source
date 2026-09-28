// AirbrakeController.cpp 
//
// full orchestration
//
// contains an update order that prevents a one-tick NaN from poisioning 
// the controller's internal state
//
#include "airbrake/AirbrakeController.hpp"
#include "airbrake/AirbrakeConfig.hpp"
#include "airbrake/OvershootBias.hpp"
#include "airbrake/Validate.hpp"

namespace airbrake {

namespace {

// A finite dt outside the usable window is clamped and flagged instead of being
// treated as a bad tick
//
// the data is still usable afterwards, and degrading on it would cost control 
// authority for nothing
//
// A non-finite dt is a bad tick.
//
float usableDeltaTime(float dt_s) {
    if (!validate::isFinite(dt_s)) {
        return config::kNominalDt_s;
    }
    if (dt_s < config::kDtMin_s) {
        return config::kDtMin_s;
    }
    if (dt_s > config::kDtClampMax_s) {
        return config::kDtClampMax_s;
    }
    return dt_s;
}

}  // namespace

bool AirbrakeController::configure(const FlightConfig& cfg) {
    const uint16_t config_flags = validate::checkConfig(cfg);
    if (config_flags != FAULT_NONE) {
        configured_ = false;
        return false;
    }

    cfg_ = cfg;

    table_.init(kFlightCdGrid, cfg_.cd_scale);
    predictor_.init(&table_, config::kReferenceArea_m2, cfg_.launch_elevation_msl_m);
    corrector_.init(&predictor_, cfg_.target_apogee_agl_m,
                    config::kCorrectorGainFraction,
                    config::kSensitivityRefreshTicks,
                    config::kSensitivityProbeDelta);
    limiter_.init(cfg_.deploy_rate_per_s, cfg_.retract_rate_per_s,
                  config::kDeployMin, config::kDeployMax);
    limiter_.snapTo(config::kNeutral);
    faults_.init(config::kBridgeTicks, config::kRecoveryTicks, config::kDegradedMaxTicks);

    last_command_ = config::kNeutral;
    configured_ = true;
    return true;
}

ControllerOutput AirbrakeController::update(const FlightState& state, float dt_s) {
    ControllerOutput out{};
    out.mode = Mode::NotConfigured;
    out.deployment_cmd = config::kNeutral;

    if (!configured_) {
        out.fault_flags_this_tick = FAULT_NOT_CONFIGURED;
        out.fault_flags = FAULT_NOT_CONFIGURED;
        return out;
    }

    const float delta_time_s = usableDeltaTime(dt_s);

    // validate before updating internals
    uint16_t tick_flags = validate::checkState(state, dt_s);

    // A clamped dt is flagged but is not itself a bad tick
    const uint16_t bad_input_mask = FAULT_INPUT_NONFINITE | FAULT_INPUT_RANGE;
    const bool bad_input = (tick_flags & bad_input_mask) != 0;

    // compute into locals
    float command = limiter_.value();
    float unlimited = limiter_.value();
    float raw = limiter_.value();

    if (faults_.isLatched()) {
        // avionics' decided to perminantly shut off this subsystem
        // retract to neutral for rest of flight
        // retract through servo rate limited to maintain position knowledge

        command = limiter_.update(config::kNeutral, delta_time_s);
        unlimited = config::kNeutral;
        raw = config::kNeutral;
        out.mode = Mode::Latched;
    } else if (state.inhibit) {
        // avionics' temporary gating flag
        // retract to neutral through servo limiter
        command = limiter_.update(config::kNeutral, delta_time_s);
        unlimited = config::kNeutral;
        raw = config::kNeutral;
        // clear cache and recalculate state when restarting
        corrector_.reset();
        out.mode = Mode::Inhibited;
    } else {
        if (bad_input) {
            faults_.recordFault(FaultSource::Upstream, tick_flags, state, dt_s, cfg_);
        } else {
            faults_.recordGood();
        }

        const Mode tier = faults_.mode();

        if (tier == Mode::Normal) {
            const CorrectorResult correction =
                corrector_.update(state.altitude_agl_m, state.velocity_up_mps,
                                  state.speed_mps, cfg_.mass_kg, limiter_.value());

            out.predicted_apogee_agl_m = correction.apogee_error_m + cfg_.target_apogee_agl_m;
            out.apogee_error_m = correction.apogee_error_m;
            out.sensitivity = correction.sensitivity;
            out.predictor_steps = correction.predictor_steps;

            if (correction.predictor_steps >= config::kMaxIntegrationSteps) {
                tick_flags |= FAULT_PREDICTOR_LIMIT;
            }

            if (correction.finite) {
                // under-brake early approach full authority late
                // over-braking early cannot be undone but under-braking early can
                const float progress = state.altitude_agl_m / cfg_.target_apogee_agl_m;
                raw = correction.deployment_raw;
                unlimited = OvershootBias::apply(raw, progress);
                command = limiter_.update(unlimited, delta_time_s);
            } else {
                // a nonfinite result from finite inputs is my own bug not the estimator's
                // nothing physical recovers from it, so latch
                tick_flags |= FAULT_INTERNAL_NONFINITE;
                faults_.recordFault(FaultSource::Internal, tick_flags, state, dt_s, cfg_);
                command = limiter_.update(config::kNeutral, delta_time_s);
                unlimited = config::kNeutral;
                raw = config::kNeutral;
            }
        } else if (tier == Mode::Bridging) {
            // one bad tick at 100 hz is 10 ms, during which the vehicle has
            // barely moved and the previous command came from valid data
            //
            // retracting would cost apogee accuracy for no safety gain
            command = limiter_.value();
            unlimited = command;
            raw = command;
        } else {
            command = limiter_.update(config::kNeutral, delta_time_s);
            unlimited = config::kNeutral;
            raw = config::kNeutral;
        }

        out.mode = faults_.mode();
    }

    // validate
    bool output_clamped = false;
    command = validate::guardCommand(command, &output_clamped);
    if (output_clamped) {
        tick_flags |= FAULT_OUTPUT_CLAMPED;
        limiter_.snapTo(command);
    }

    // update state
    last_command_ = command;

    out.deployment_cmd = command;
    out.deployment_unlimited = unlimited;
    out.deployment_raw = raw;
    out.fault_flags_this_tick = tick_flags;
    out.fault_flags = static_cast<uint16_t>(faults_.stickyFlags() | tick_flags);
    out.consecutive_bad_ticks = faults_.consecutiveBad();

    return out;
}

void AirbrakeController::abort() {
    // one-way system lock out
    faults_.latch();
}

void AirbrakeController::notifyMechanicalRetract() {
    // the hardware failsafe physically retracted the brakes
    // update internal state to match and system lock out
    limiter_.snapTo(config::kNeutral);
    last_command_ = config::kNeutral;
    faults_.latch();
}

void AirbrakeController::reset() {
    // GROUND AND TEST USE ONLY. 
    // clears the 'perminant' latch which should NEVER
    // be done in-flight
    faults_.reset();
    faults_.init(config::kBridgeTicks, config::kRecoveryTicks, config::kDegradedMaxTicks);
    corrector_.reset();
    limiter_.snapTo(config::kNeutral);
    last_command_ = config::kNeutral;
}

float AirbrakeController::commandedDeployment() const {
    return limiter_.value();
}

Mode AirbrakeController::mode() const {
    if (!configured_) {
        return Mode::NotConfigured;
    }
    return faults_.mode();
}

bool AirbrakeController::isConfigured() const {
    return configured_;
}

const FaultSnapshot& AirbrakeController::faultSnapshot() const {
    return faults_.snapshot();
}

bool AirbrakeController::selfTest() const {
    // ground test designed to test for a corrupted flash image
    // every inverse lookup should see Cd non-decreasing with deployment
    // for all values
    if (!CdTable::validateGrid(kFlightCdGrid)) {
        return false;
    }

    if (!configured_) {
        return false;
    }

    // run a short fixed sequence in local scope to verify behaviour
    AirbrakeController probe;
    if (!probe.configure(cfg_)) {
        return false;
    }

    FlightState state{};
    state.altitude_agl_m = 1500.0f;
    state.velocity_up_mps = 220.0f;
    state.speed_mps = 220.0f;
    state.inhibit = false;

    for (uint8_t tick = 0; tick < 10u; ++tick) {
        const ControllerOutput out = probe.update(state, config::kNominalDt_s);
        const bool usable =
            validate::inRange(out.deployment_cmd, config::kDeployMin, config::kDeployMax) &&
            out.mode == Mode::Normal &&
            (out.fault_flags_this_tick & FAULT_INTERNAL_NONFINITE) == 0;
        if (!usable) {
            return false;
        }
    }

    return true;
}

}  // namespace airbrake
