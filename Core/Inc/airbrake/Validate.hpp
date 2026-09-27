// Validate.hpp
//
// NaN safe validation primitives
//
// every check is written in positive form (assert what must be true) rather
// than rejecting what must be false. a negative form check passes NaN straight
// through, as every comparison against NaN is false
//
// do NOT use fminf / fmaxf / std::clamp anywhere in this project. they return
// the non-NaN operand, so a NaN gets laundered into a plausible number and the
// evidence is destroyed
//
#pragma once
#include <cstdint>
#include "airbrake/Types.hpp"

namespace airbrake {
namespace validate {

// true only if x is finite AND lo <= x <= hi. false for NaN, false for Inf
bool inRange(float x, float lo, float hi);

// true only if x is finite, so not NaN and not +/-Inf
bool isFinite(float x);

// clamp that reports rather than hides. a non-finite x returns fallback and
// sets *ok false. never use a bare fminf/fmaxf pair instead of this
float clampChecked(float x, float lo, float hi, float fallback, bool* ok);

// validates every field of a tick input, returns the OR of the FaultFlag bits
// that fired. 0 means clean
uint16_t checkState(const FlightState& s, float dt_s);

// validates a config. a failing config must leave the controller unconfigured
// so it can never reach flight
uint16_t checkConfig(const FlightConfig& c);

// final output guard, applied every tick even though it cannot happen. the
// caller turns this into an integer PWM value, and float to int conversion of
// a NaN is undefined behaviour
float guardCommand(float cmd, bool* clamped);

}  // namespace validate
}  // namespace airbrake
