// ApogeePredictor.cpp 
// RK4 coast integration to apogee at given deployment
//
// accounts non-updward velocity via tilt ratio
//
#include "airbrake/ApogeePredictor.hpp"
#include "airbrake/Atmosphere.hpp"
#include "airbrake/Validate.hpp"

namespace airbrake {

namespace {

// guard tilt range. velocity_up near zero would blow this up,
// and it can never physically be below 1 as total speed cannot
// be smaller than its own vertical component
constexpr float kMinTiltRatio = 1.0f;
constexpr float kMaxTiltRatio = 100.0f;

struct State {
    float altitude;
    float velocity_up;
};

struct StateDeriv {
    float d_altitude;
    float d_velocity_up;
};

// Everything the derivative needs that does not change across an integration.
// Bundled so the RK4 stages stay readable rather than threading eight arguments.

// struct for neatness
// values do not change across an integration
struct IntegrationContext {
    const CdTable* table;
    float launch_elevation_msl_m;
    float half_area_over_mass;  // 0.5 * area * tilt_ratio / mass, hoisted
    float tilt_ratio;
    // deployment is held for the whole integration while Mach decays, so only
    // the resolved axis bracket is carried
    CdTable::AxisBracket deploy_bracket;
    bool  fixed_cd;
    float fixed_cd_value;
};

// Cd is re-looked-up here on every stage because Mach decays through the coast
StateDeriv derivative(const State& state, const IntegrationContext& ctx) {
    const float altitude_msl_m = state.altitude + ctx.launch_elevation_msl_m;
    const float gravity = Atmosphere::gravity_mps2(altitude_msl_m);
    const float density = Atmosphere::density_kgm3(altitude_msl_m);

    float cd;
    if (ctx.fixed_cd) {
        cd = ctx.fixed_cd_value;
    } else {
        // Mach is taken on total airspeed not the vertical component
        // using only the vertical part would under-predict drag 
        // and over-predict apogee, biasing towards over-braking
        const float total_speed = ctx.tilt_ratio * state.velocity_up;
        const float mach = Atmosphere::mach(total_speed, altitude_msl_m);
        cd = ctx.table->cdAtBracketedDeployment(mach, ctx.deploy_bracket);
    }

    // half_area_over_mass already folds in 0.5 * area * tilt_ratio / mass so
    // only multiplies happen here and no divide enters the hot loop
    // divide take ~14 cycles on the cortex-M4F FPv4-SP
    const float drag_accel =
        density * cd * ctx.half_area_over_mass * state.velocity_up * state.velocity_up;

    StateDeriv result;
    result.d_altitude = state.velocity_up;
    result.d_velocity_up = -gravity - drag_accel;
    return result;
}

// One four-stage RK4 step of arbitrary size.
State rk4Step(const State& start, float step_s, const IntegrationContext& ctx) {
    const float half_step = 0.5f * step_s;

    const StateDeriv k1 = derivative(start, ctx);

    const State s2 = { start.altitude + half_step * k1.d_altitude,
                       start.velocity_up + half_step * k1.d_velocity_up };
    const StateDeriv k2 = derivative(s2, ctx);

    const State s3 = { start.altitude + half_step * k2.d_altitude,
                       start.velocity_up + half_step * k2.d_velocity_up };
    const StateDeriv k3 = derivative(s3, ctx);

    const State s4 = { start.altitude + step_s * k3.d_altitude,
                       start.velocity_up + step_s * k3.d_velocity_up };
    const StateDeriv k4 = derivative(s4, ctx);

    const float sixth_step = step_s / 6.0f;
    State result;
    result.altitude = start.altitude +
        sixth_step * (k1.d_altitude + 2.0f * k2.d_altitude + 2.0f * k3.d_altitude + k4.d_altitude);
    result.velocity_up = start.velocity_up +
        sixth_step * (k1.d_velocity_up + 2.0f * k2.d_velocity_up + 2.0f * k3.d_velocity_up + k4.d_velocity_up);
    return result;
}

bool stateFinite(const State& s) {
    return validate::isFinite(s.altitude) && validate::isFinite(s.velocity_up);
}

// Shared by predictAtDeployment (table Cd) and predictAtFixedCd (constant Cd).
ApogeePrediction integrate(const CdTable* table_, float reference_area_m2_,
                           float launch_elevation_msl_m_, float integration_step_s_,
                           float altitude_agl_m,
                           float velocity_up_mps,
                           float speed_mps,
                           float mass_kg,
                           float deployment,
                           bool fixed_cd,
                           float fixed_cd_value) {
    ApogeePrediction result{};
    result.finite = false;

    // Positive-form input validation
    // NaN == false against everything
    const bool inputs_ok =
        validate::isFinite(mass_kg) && mass_kg > 0.0f &&
        validate::isFinite(altitude_agl_m) &&
        validate::isFinite(velocity_up_mps) &&
        validate::isFinite(speed_mps) &&
        (fixed_cd ? validate::isFinite(fixed_cd_value) : (table_ != nullptr));

    if (!inputs_ok) {
        return result;
    }

    // captured once and held for the whole prediction so the 1D
    // system stays consistent as Mach decays
    bool clamp_ok = true;
    float tilt_ratio = 1.0f;
    if (velocity_up_mps > 0.0f) {
        tilt_ratio = speed_mps / velocity_up_mps;
    }
    tilt_ratio = validate::clampChecked(tilt_ratio, kMinTiltRatio, kMaxTiltRatio,
                                        kMinTiltRatio, &clamp_ok);

    IntegrationContext ctx;
    ctx.table = table_;
    ctx.launch_elevation_msl_m = launch_elevation_msl_m_;
    // loop invariant reciprocal hoisted out of the RK4 stages
    ctx.half_area_over_mass = 0.5f * reference_area_m2_ * tilt_ratio * (1.0f / mass_kg);
    ctx.tilt_ratio = tilt_ratio;
    ctx.fixed_cd = fixed_cd;
    ctx.fixed_cd_value = fixed_cd_value;
    if (!fixed_cd) {
        // table_ is non-null here; the fixed_cd == false branch of inputs_ok
        // already required it
        ctx.deploy_bracket = table_->deploymentBracket(deployment);
    }

    const float step_s = integration_step_s_;
    State current = { altitude_agl_m, velocity_up_mps };
    State previous = current;
    float time_s = 0.0f;
    uint16_t step = 0;
    bool all_finite = stateFinite(current);

    while (all_finite && current.velocity_up > 0.0f) {
        if (step >= config::kMaxIntegrationSteps) {
            result.step_limit_hit = true;
            break;
        }

        previous = current;
        current = rk4Step(previous, step_s, ctx);
        time_s += step_s;
        step += 1;

        all_finite = stateFinite(current);
    }

    result.steps = step;
    result.finite = all_finite && clamp_ok;

    if (!result.finite) {
        return result;
    }

    if (result.step_limit_hit) {
        // report how far it got at step limit
        result.apogee_agl_m = current.altitude;
        result.time_to_apogee_s = time_s;
        return result;
    }

    if (step == 0) {
        // already at or past apogee on entry
        result.apogee_agl_m = current.altitude;
        result.time_to_apogee_s = 0.0f;
        return result;
    }

    //
    // Land exactly on the velocity_up == 0 crossing with a final RK4 sub-step,
    // rather than interpolating between the two bracketing states. Linear
    // interpolation is only second-order accurate, so it would cap the whole
    // integration at second order however good the RK4 interior is, and it
    // would put a sawtooth into f(deployment) that the corrector measures its
    // sensitivity across.
    //
    // Apogee is a stationary point of altitude, so a residual error in the
    // sub-step length costs only its square in altitude. That is why a single
    // Newton refinement is enough to recover full fourth-order accuracy.
    //
    
    const float velocity_span = previous.velocity_up - current.velocity_up;
    float sub_step_s = 0.0f;
    if (velocity_span > 0.0f) {
        sub_step_s = (previous.velocity_up / velocity_span) * step_s;
    }
    sub_step_s = validate::clampChecked(sub_step_s, 0.0f, step_s, 0.0f, &clamp_ok);

    State terminal = rk4Step(previous, sub_step_s, ctx);

    if (stateFinite(terminal)) {
        const StateDeriv landing = derivative(terminal, ctx);
        if (landing.d_velocity_up < 0.0f) {
            sub_step_s += terminal.velocity_up / (-landing.d_velocity_up);
            sub_step_s = validate::clampChecked(sub_step_s, 0.0f, step_s, 0.0f, &clamp_ok);
            const State refined = rk4Step(previous, sub_step_s, ctx);
            if (stateFinite(refined)) {
                terminal = refined;
            }
        }
    }

    if (!stateFinite(terminal)) {
        result.finite = false;
        return result;
    }

    result.apogee_agl_m = terminal.altitude;
    result.time_to_apogee_s = time_s - step_s + sub_step_s;
    result.finite = clamp_ok;

    return result;
}

}  // namespace

void ApogeePredictor::init(const CdTable* table, float reference_area_m2, float launch_elevation_msl_m) {
    table_ = table;
    reference_area_m2_ = reference_area_m2;
    launch_elevation_msl_m_ = launch_elevation_msl_m;
}

ApogeePrediction ApogeePredictor::predictAtDeployment(float altitude_agl_m,
                                                       float velocity_up_mps,
                                                       float speed_mps,
                                                       float mass_kg,
                                                       float deployment) const {
    return integrate(table_, reference_area_m2_, launch_elevation_msl_m_,
                     integration_step_s_, altitude_agl_m, velocity_up_mps,
                     speed_mps, mass_kg, deployment, false, 0.0f);
}

ApogeePrediction ApogeePredictor::predictAtFixedCd(float altitude_agl_m,
                                                    float velocity_up_mps,
                                                    float speed_mps,
                                                    float mass_kg,
                                                    float cd) const {
    return integrate(table_, reference_area_m2_, launch_elevation_msl_m_,
                     integration_step_s_, altitude_agl_m, velocity_up_mps,
                     speed_mps, mass_kg, 0.0f, true, cd);
}

float ApogeePredictor::acceleration(float altitude_agl_m, float speed_mps,
                                    float mass_kg, float deployment) const {
    if (!(validate::isFinite(altitude_agl_m) && validate::isFinite(speed_mps) &&
          validate::isFinite(mass_kg) && mass_kg > 0.0f && table_ != nullptr)) {
        return 0.0f;
    }

    const float altitude_msl_m = altitude_agl_m + launch_elevation_msl_m_;
    const float gravity = Atmosphere::gravity_mps2(altitude_msl_m);
    const float density = Atmosphere::density_kgm3(altitude_msl_m);
    const float mach = Atmosphere::mach(speed_mps, altitude_msl_m);
    const float cd = table_->cd(mach, deployment);

    const float drag_accel =
        0.5f * density * cd * reference_area_m2_ * speed_mps * speed_mps / mass_kg;

    return -gravity - drag_accel;
}

// not a stub
// forces step_s to be validated, preventing it being forgotten elsewhere
void ApogeePredictor::setIntegrationStep(float step_s) {
    if (validate::isFinite(step_s) && step_s > 0.0f) {
        integration_step_s_ = step_s;
    }
}

float ApogeePredictor::integrationStep() const {
    return integration_step_s_;
}

}  // namespace airbrake
