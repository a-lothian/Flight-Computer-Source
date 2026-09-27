// RateLimiter.hpp
//
// slew limit and saturation. models what the actuator can physically do, so it
// also holds the core's belief about where the actuator currently is
//
// that belief is why a software abort and a mechanical retract are separate
// calls. an abort slews to neutral at max rate and the belief stays accurate,
// where the hardware failsafe retracts faster than the servo could and the
// belief has to be snapped instead
//
#pragma once

namespace airbrake {

class RateLimiter {
public:
    RateLimiter() = default;
    void init(float max_rate_per_s, float lower, float upper);

    // advance toward target, limited by max_rate * dt_s
    float update(float target, float dt_s);

    // force the internal belief with no slew. ONLY for when the actuator is
    // known to have moved by something other than our own commands
    void snapTo(float value);

    float value() const;

private:
    float max_rate_ = 0.0f;
    float lower_ = 0.0f;
    float upper_ = 1.0f;
    float value_ = 0.0f;
};

}  // namespace airbrake
