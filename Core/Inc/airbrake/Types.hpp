// Types.hpp
//
// shared value types for the airbrake control core
// header only. no allocation, no exceptions, float32 only
//
#pragma once
#include <cstdint>

namespace airbrake {

// fault reporting ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

// bitmask, logged every tick. sticky within a flight and never cleared except
// by reset(), so post-flight telemetry shows everything that ever fired
enum FaultFlag : uint16_t {
    FAULT_NONE              = 0,
    FAULT_NOT_CONFIGURED    = 1u << 0,
    FAULT_INPUT_NONFINITE   = 1u << 1,  // NaN or +/-Inf in AB_TickInput
    FAULT_INPUT_RANGE       = 1u << 2,  // finite but physically implausible
    FAULT_DT_OUTLIER        = 1u << 3,  // dt was outside the window and clamped
    FAULT_INTERNAL_NONFINITE= 1u << 4,  // OUR BUG. latches immediately
    FAULT_PREDICTOR_LIMIT   = 1u << 5,  // integration hit the step cap
    FAULT_CD_SATURATED      = 1u << 6,  // required Cd outside the table range
    FAULT_CONFIG_INVALID    = 1u << 7,
    FAULT_OUTPUT_CLAMPED    = 1u << 8,  // final guard fired, should be impossible
};

// which of the two independent fault counters a fault increments.
// the asymmetry between them is deliberate
enum class FaultSource : uint8_t {
    Upstream = 0,  // bad input from the estimator, bridgeable and recoverable
    Internal = 1,  // non-finite produced from finite inputs, latches
};

// controller mode, strictly a function of the input sequence. no wall clock
// and no uninitialised memory, so golden vectors stay reproducible
enum class Mode : uint8_t {
    NotConfigured = 0,
    Normal        = 1,  // closed-loop control
    Bridging      = 2,  // few enough bad ticks to hold the last command
    Degraded      = 3,  // commanding neutral, still watching for recovery
    Inhibited     = 4,  // avionics asked for neutral this tick, recoverable
    Latched       = 5,  // neutral for the rest of the flight, ground reset only
};

// flight data ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

// per-tick vehicle state. no phase and no attitude, as flight-phase gating and
// the AURC 11.2.5, 11.2.6 abort conditions belong to the avionics' state
// machine and are NOT re-evaluated here
struct FlightState {
    float altitude_agl_m;   // above the pad
    float velocity_up_mps;  // earth-vertical component, signed
    float speed_mps;        // total airspeed magnitude, drives the drag term
    bool  inhibit;          // avionics' gating decision for THIS tick.
                            // true slews to neutral, recoverable next tick
};

// set once per flight or per Monte Carlo trial, before the first tick
struct FlightConfig {
    float mass_kg;                // coast-phase (burnout) mass
    float target_apogee_agl_m;
    float launch_elevation_msl_m; // maps AGL to MSL for the atmosphere model
    float cd_scale;               // multiplier on every Cd table lookup.
                                  // 1.0 in flight, swept in testing to
                                  // represent CFD uncertainty
};

// written once, on the first fault of a flight, then never overwritten.
// this is the reproduction case, the flags alone are not
struct FaultSnapshot {
    bool        valid;
    uint32_t    tick_index;
    FaultSource source;
    uint16_t    flags;
    FlightState state;   // the raw input, exactly as received
    float       dt_s;
    FlightConfig config;
};

// returned every tick. telemeter and log all of it
struct ControllerOutput {
    float    deployment_cmd;         // [0,1] THE command, always finite
    float    deployment_unlimited;   // [0,1] pre-rate-limit  (diagnostic)
    float    deployment_raw;         // [0,1] pre-bias        (diagnostic)
    float    predicted_apogee_agl_m;
    float    apogee_error_m;         // predicted - target
    float    sensitivity;            // d(apogee)/d(deployment), m per unit
    Mode     mode;
    uint16_t fault_flags;            // sticky
    uint16_t fault_flags_this_tick;  // non-sticky, for edge detection
    uint16_t consecutive_bad_ticks;
    uint16_t predictor_steps;        // worst-case timing evidence
};

}  // namespace airbrake
