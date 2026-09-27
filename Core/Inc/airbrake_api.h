// airbrake_api.h
//
// flat C airbrake interface. Allows the same source to be compiled to STM34F4
// flash and into a shared library loaded by JNA from the OpenRocket harness
//
// CALLING CONTRACT
//     Vehicle state-gating (launchpad, burning, coasting, etc.) and the AURC 11.2.5,
//      11.2.6 abort conditions belong the to avionics' state machine. This module
//      does NOT re-evaluate these. It validates its own inputs and guarentees
//      (finite, non-NaN) its own output.
//
//      Once the control loop starts, ab_tick() must be called EVERY control period
//      for as long as this module owns the actuator, including while inhibited.
//
//      To hold neutral recoverably, set AB_TickInput.inhibit = 1; when passed to
//      ab_tick(). Do not simply stop calling. The core maintains a model of where 
//      the actuator is, and the model goes stale when ticks are skipped or the 
//      command is overridden externally.
//
//
#ifndef AIRBRAKE_API_H
#define AIRBRAKE_API_H

#ifdef __cplusplus
extern "C" {
#endif

#define AB_ABI_VERSION 2

//
// the shared library is build with hidden default visibility so only the
// ab_* surface is exported. each entry point has to opt back in explicitly,
// which is done via the AB_API macro
//
// AB_API expands to nothing in the firmware build, which contains no
// dynamic symbol table to worry about
//

#if defined(_WIN32)
#  define AB_API __declspec(dllexport) // UNTESTED
#elif defined(__GNUC__) || defined(__clang__)
#  define AB_API __attribute__((visibility("default")))
#else
#  define AB_API
#endif

// status and mode ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

typedef enum {
    AB_OK                 = 0,
    AB_ERR_NULL_ARG       = 1,
    AB_ERR_NOT_CONFIGURED = 2,
    AB_ERR_BAD_INPUT      = 3,  // input rejected; command is neutral or held
    AB_ERR_DEGRADED       = 4,  // commanding neutral, watching for recovery
    AB_ERR_LATCHED        = 5,  // neutral for the rest of the flight
    AB_ERR_INTERNAL       = 6   // OUR BUG. Latched. Investigate before flight
} AB_Status;

typedef enum {
    AB_MODE_NOT_CONFIGURED = 0,
    AB_MODE_NORMAL         = 1,
    AB_MODE_BRIDGING       = 2,
    AB_MODE_DEGRADED       = 3,
    AB_MODE_INHIBITED      = 4,
    AB_MODE_LATCHED        = 5
} AB_Mode;

// mirrors airbrake::FaultFlag, keep in sync with Types.hpp and Java enum
#define AB_FAULT_NONE               0x0000u
#define AB_FAULT_NOT_CONFIGURED     0x0001u
#define AB_FAULT_INPUT_NONFINITE    0x0002u
#define AB_FAULT_INPUT_RANGE        0x0004u
#define AB_FAULT_DT_OUTLIER         0x0008u
#define AB_FAULT_INTERNAL_NONFINITE 0x0010u
#define AB_FAULT_PREDICTOR_LIMIT    0x0020u
#define AB_FAULT_CD_SATURATED       0x0040u
#define AB_FAULT_CONFIG_INVALID     0x0080u
#define AB_FAULT_OUTPUT_CLAMPED     0x0100u

// data ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

typedef struct {
    float mass_kg;                  // coast-phase mass
    float target_apogee_agl_m;
    float launch_elevation_msl_m;  // density needs MSL, estimator gives AGL
    float cd_scale;                // 1.0f in flight; swept in testing for CFD uncertainty
} AB_FlightConfig;

typedef struct {
    float altitude_agl_m;
    float velocity_up_mps;  // signed vertical component; drives altitude
    float speed_mps;        // TOTAL airspeed; drives drag
    float dt_s;
    int inhibit;            // 0 = control, 1 = hold neutral (recoverable)
} AB_TickInput;

typedef struct {
    float          deployment_cmd;         // [0,1] always finite
    float          deployment_unlimited;   // pre-rate-limit  (diagnostic)
    float          deployment_raw;         // pre-bias        (diagnostic)
    float          predicted_apogee_agl_m;
    float          apogee_error_m;
    float          sensitivity;            // d(apogee)/d(deployment)
    unsigned char  mode;                   // AB_Mode
    unsigned short fault_flags;            // sticky, OR of everything so far
    unsigned short fault_flags_this_tick;  // non-sticky
    unsigned short consecutive_bad_ticks;
    unsigned short predictor_steps;        // worst-case timing evidence       
} AB_TickOutput;

typedef struct {
    int             valid;
    unsigned int    tick_index;
    unsigned short  flags;
    unsigned char   source;        // 0 = upstream, 1 = internal
    AB_TickInput    input;         // the reproduction case
    AB_FlightConfig config;
} AB_FaultSnapshot;

// calls ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

// Guards against a stale .dll/.dylib loaded against newer Java bindings.
AB_API int         ab_abi_version(void);

// CFD run identifier compiled into the Cd table.
// Telemeter at boot so a flight image can be traced to a specific CFD dataset.
AB_API const char* ab_cd_table_provenance(void);

// Validates hard and refuses bad values. A NaN or zero mass caught on the
// ground costs nothing, and zero mass gives infinite acceleration through F/m.
AB_API AB_Status   ab_configure(const AB_FlightConfig* cfg);

// Runs built-in golden vectors through the whole chain against compiled-in
// expected values. Call once pre-flight and report the result in telemetry.
AB_API AB_Status   ab_self_test(void);

// The only per-iteration entry point. out must not be NULL. On every error
// path out->deployment_cmd is still written and is always finite and in [0,1].
AB_API AB_Status   ab_tick(const AB_TickInput* in, AB_TickOutput* out);

// One-way software abort. Slews to neutral and
// latches. Not clearable except by ab_reset().
AB_API void        ab_abort(void);

// Call this AFTER the hardware failsafe (AURC 11.1.3) has retracted the brakes.
// Latches for the rest of the flight
AB_API void        ab_notify_mechanical_retract(void);

// GROUND AND TEST USE ONLY. Clears the abort latch, so it is exactly the wrong
// call in flight. Preserves the configuration and clears all runtime state.
AB_API void        ab_reset(void);

// The command currently being held, the same value ab_tick() last wrote to
// out->deployment_cmd. Readable between ticks so telemetry does not have to
// cache it. This is also the core's belief about where the actuator is, so it
// follows a mechanical retract as well as a normal command.
AB_API float       ab_last_command(void);

// Current mode, the same value ab_tick() last wrote to out->mode. Readable
// between ticks for the same reason.
AB_API AB_Mode     ab_mode(void);

// The FIRST fault of the flight, kept so it can be reproduced on the ground.
// Later faults do not overwrite it, because the first one is usually what
// explains the ones after it. Carries the exact input and config that produced
// it, so the tick can be replayed offline.
//
// out must not be NULL. Returns AB_OK whenever the call itself worked, which
// includes the case where no fault has happened yet, so read out->valid rather
// than the status to find out whether there is anything in it.
AB_API AB_Status   ab_fault_snapshot(AB_FaultSnapshot* out);

#ifdef __cplusplus
}
#endif
#endif // AIRBRAKE_API_H
