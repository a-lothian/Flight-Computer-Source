// AirbrakeConfig.hpp
//
// compile time constants
// anything that varies per flight or per Monte Carlo trial belongs in
// FlightConfig, never here
//
#pragma once
#include <cstdint>

namespace airbrake {
namespace config {

// control rate ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

// fault timings are counted in TICKS and not seconds, as dt is itself a field
// that can arrive NaN. rederive these if the rate changes
inline constexpr float    kNominalRate_Hz   = 100.0f;
inline constexpr float    kNominalDt_s      = 1.0f / kNominalRate_Hz;

// vehicle ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

// bare-rocket reference area, from the CFD deliverable at deployment 0. the Cd
// table is generated with the growing per-deployment area already folded in,
// so every lookup pairs with THIS fixed area. a zero here would silently
// remove drag from the model entirely
inline constexpr float kReferenceArea_m2      = 0.021034f;
inline constexpr float kDefaultTargetApogee_m = 3048.0f; // 10,000 ft AGL

// actuator ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

inline constexpr float kDeployMin           = 0.0f;
inline constexpr float kDeployMax           = 1.0f;
// deploy and retract slew rates are measured per vehicle, so they arrive in
// FlightConfig rather than living here
inline constexpr float kNeutral             = 0.0f;      // AURC 11.1.3 neutral state

// predictor ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

// 0.2 s, not the 0.05 s originally assumed. measured RK4 error against a
// reference integration is under one float32 ulp of the apogee by h = 0.4 s,
// so 0.05 s bought no accuracy that survives single precision and cost four
// times the work per prediction
inline constexpr float    kIntegrationStep_s   = 0.2f;
// 400 steps at that size bounds one prediction at 80 s of coast, which covers
// the whole reachable state space with room to spare rather than clipping the
// energetic cases. a step-limited prediction is unusable, and the first tick
// after burnout is the longest one, so the budget has to cover more than the
// nominal coast
inline constexpr uint16_t kMaxIntegrationSteps = 400;

// corrector ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

// the four tuning constants below can be overridden at BUILD time so the
// dispersion harness can sweep them without editing this file, for example
// -DAIRBRAKE_TUNE_GAIN_FRACTION=0.35f. each default is stated here and is what
// an ordinary build compiles, so a flight image is bit-identical to one built
// before this mechanism existed unless someone passes an override explicitly.
// nothing reads these at runtime, they are still compile time constants
//
// the four interact, so change them together rather than one at a time
#ifndef AIRBRAKE_TUNE_REFRESH_TICKS
#define AIRBRAKE_TUNE_REFRESH_TICKS 8
#endif
#ifndef AIRBRAKE_TUNE_PROBE_DELTA
#define AIRBRAKE_TUNE_PROBE_DELTA 0.100217f
#endif
#ifndef AIRBRAKE_TUNE_GAIN_FRACTION
#define AIRBRAKE_TUNE_GAIN_FRACTION 0.628356f
#endif
#ifndef AIRBRAKE_TUNE_BIAS_FLOOR
#define AIRBRAKE_TUNE_BIAS_FLOOR 0.995577f
#endif

inline constexpr uint8_t kSensitivityRefreshTicks = AIRBRAKE_TUNE_REFRESH_TICKS;
inline constexpr float   kSensitivityProbeDelta   = AIRBRAKE_TUNE_PROBE_DELTA;
// floor on |d(apogee)/d(deployment)| before the correction divide. near apogee
// the brakes have almost no time left to act, so the sensitivity collapses
// toward zero and dividing by it would demand an enormous correction from a
// tiny error
inline constexpr float   kMinSensitivity_mPerUnit = 1.0f;

// fraction of the sensitivity-implied full correction applied per tick
inline constexpr float   kCorrectorGainFraction   = AIRBRAKE_TUNE_GAIN_FRACTION;

// overshoot bias authority floor at burnout. over-braking early cannot be
// undone, under-braking early still can, so authority is deliberately
// suppressed early in the coast
inline constexpr float   kBiasFloor               = AIRBRAKE_TUNE_BIAS_FLOOR;

// offline solver ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

// verification only, not flown
inline constexpr uint8_t kMaxSolverIterations = 40;
inline constexpr float   kApogeeTolerance_m   = 0.1f;

// fault policy ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

inline constexpr uint16_t kBridgeTicks       = 5;    // ~50 ms of hold-last-command
inline constexpr uint16_t kRecoveryTicks     = 50;   // ~500 ms of clean data to resume
inline constexpr uint16_t kDegradedMaxTicks  = 300;  // ~3 s, then latch
inline constexpr uint16_t kInternalFaultTicks= 1;    // internal NaN latches at once

// input validity ranges ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

// used by the positive-form checks in Validate
inline constexpr float kAltitudeMin_m = -500.0f,  kAltitudeMax_m = 20000.0f;
inline constexpr float kSpeedMin_mps  = 0.0f,     kSpeedMax_mps  = 1000.0f;
inline constexpr float kVelUpMin_mps  = -1000.0f, kVelUpMax_mps  = 1000.0f;
inline constexpr float kDtMin_s       = 1.0e-4f,  kDtMax_s       = 1.0f;
// a finite dt above this is clamped and flagged rather than treated as a bad
// tick, the data is still usable and degrading on it would cost control
// authority for nothing
inline constexpr float kDtClampMax_s  = 5.0f * kNominalDt_s;
inline constexpr float kMassMin_kg    = 0.1f,     kMassMax_kg    = 200.0f;
inline constexpr float kCdScaleMin    = 0.1f,     kCdScaleMax    = 5.0f;
// a zero rate would freeze the brakes where they are, including open
inline constexpr float kSlewRateMin_per_s = 0.05f, kSlewRateMax_per_s = 100.0f;

// physics ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

inline constexpr float kG0_mps2       = 9.80665f;
inline constexpr float kEarthRadius_m = 6371000.0f;

}  // namespace config
}  // namespace airbrake
