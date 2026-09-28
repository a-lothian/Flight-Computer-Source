// airbrake_api.cpp
#include "airbrake_api.h"
#include "airbrake/AirbrakeController.hpp"

namespace {
// no constructor here, so nothing has to run before main and there is no link
// order to get wrong. Sits in .bss for the whole run; no heap, and no
// first-call setup check on every access. One of these per loaded copy of the
// library, so one per process in the harness and one per boot on firmware.

airbrake::AirbrakeController g_controller;

AB_Mode toAbMode(airbrake::Mode mode) {
    switch (mode) {
        case airbrake::Mode::Normal:   return AB_MODE_NORMAL;
        case airbrake::Mode::Bridging: return AB_MODE_BRIDGING;
        case airbrake::Mode::Degraded: return AB_MODE_DEGRADED;
        case airbrake::Mode::Inhibited:return AB_MODE_INHIBITED;
        case airbrake::Mode::Latched:  return AB_MODE_LATCHED;
        case airbrake::Mode::NotConfigured:
        default:                       return AB_MODE_NOT_CONFIGURED;
    }
}

// reports most servere thing that has happened
AB_Status toAbStatus(const airbrake::ControllerOutput& out) {
    if ((out.fault_flags_this_tick & AB_FAULT_INTERNAL_NONFINITE) != 0u) {
        return AB_ERR_INTERNAL;
    }
    switch (out.mode) {
        case airbrake::Mode::NotConfigured: return AB_ERR_NOT_CONFIGURED;
        case airbrake::Mode::Latched:       return AB_ERR_LATCHED;
        case airbrake::Mode::Degraded:      return AB_ERR_DEGRADED;
        case airbrake::Mode::Bridging:      return AB_ERR_BAD_INPUT;
        case airbrake::Mode::Normal:
        case airbrake::Mode::Inhibited:
        default:                            return AB_OK;
    }
}
}  // namespace

extern "C" {

AB_API int         ab_abi_version(void)          { return AB_ABI_VERSION; }
AB_API const char* ab_cd_table_provenance(void)  { return airbrake::kCdTableProvenance; }

AB_API AB_Status ab_configure(const AB_FlightConfig* cfg) {
    if (cfg == nullptr) {
        return AB_ERR_NULL_ARG;
    }

    airbrake::FlightConfig native{};
    native.mass_kg = cfg->mass_kg;
    native.target_apogee_agl_m = cfg->target_apogee_agl_m;
    native.launch_elevation_msl_m = cfg->launch_elevation_msl_m;
    native.cd_scale = cfg->cd_scale;
    native.deploy_rate_per_s = cfg->deploy_rate_per_s;
    native.retract_rate_per_s = cfg->retract_rate_per_s;

    return g_controller.configure(native) ? AB_OK : AB_ERR_BAD_INPUT;
}

AB_API AB_Status ab_self_test(void) {
    return g_controller.selfTest() ? AB_OK : AB_ERR_INTERNAL;
}

AB_API AB_Status ab_tick(const AB_TickInput* in, AB_TickOutput* out) {
    if (out == nullptr) {
        return AB_ERR_NULL_ARG;
    }
    // early return still leaves a finite, in-range command behind

    *out = AB_TickOutput{};
    out->deployment_cmd = 0.0f;

    if (in == nullptr) {
        return AB_ERR_NULL_ARG;
    }

    airbrake::FlightState state{};
    state.altitude_agl_m = in->altitude_agl_m;
    state.velocity_up_mps = in->velocity_up_mps;
    state.speed_mps = in->speed_mps;
    state.inhibit = (in->inhibit != 0);

    const airbrake::ControllerOutput result = g_controller.update(state, in->dt_s);

    out->deployment_cmd = result.deployment_cmd;
    out->deployment_unlimited = result.deployment_unlimited;
    out->deployment_raw = result.deployment_raw;
    out->predicted_apogee_agl_m = result.predicted_apogee_agl_m;
    out->apogee_error_m = result.apogee_error_m;
    out->sensitivity = result.sensitivity;
    out->mode = static_cast<unsigned char>(toAbMode(result.mode));
    out->fault_flags = result.fault_flags;
    out->fault_flags_this_tick = result.fault_flags_this_tick;
    out->consecutive_bad_ticks = result.consecutive_bad_ticks;
    out->predictor_steps = result.predictor_steps;

    return toAbStatus(result);
}

AB_API void      ab_abort(void)                    { g_controller.abort(); }
AB_API void      ab_notify_mechanical_retract(void){ g_controller.notifyMechanicalRetract(); }
AB_API void      ab_reset(void)                    { g_controller.reset(); }
AB_API float     ab_last_command(void)             { return g_controller.commandedDeployment(); }
AB_API AB_Mode   ab_mode(void)                     { return toAbMode(g_controller.mode()); }

AB_API AB_Status ab_fault_snapshot(AB_FaultSnapshot* out) {
    if (out == nullptr) {
        return AB_ERR_NULL_ARG;
    }
    *out = AB_FaultSnapshot{};

    const airbrake::FaultSnapshot& snapshot = g_controller.faultSnapshot();
    out->valid = snapshot.valid ? 1 : 0;
    out->tick_index = snapshot.tick_index;
    out->flags = snapshot.flags;
    out->source = static_cast<unsigned char>(snapshot.source);

    out->input.altitude_agl_m = snapshot.state.altitude_agl_m;
    out->input.velocity_up_mps = snapshot.state.velocity_up_mps;
    out->input.speed_mps = snapshot.state.speed_mps;
    out->input.dt_s = snapshot.dt_s;
    out->input.inhibit = snapshot.state.inhibit ? 1 : 0;

    out->config.mass_kg = snapshot.config.mass_kg;
    out->config.target_apogee_agl_m = snapshot.config.target_apogee_agl_m;
    out->config.launch_elevation_msl_m = snapshot.config.launch_elevation_msl_m;
    out->config.cd_scale = snapshot.config.cd_scale;
    out->config.deploy_rate_per_s = snapshot.config.deploy_rate_per_s;
    out->config.retract_rate_per_s = snapshot.config.retract_rate_per_s;

    return AB_OK;
}

}  // extern "C"
