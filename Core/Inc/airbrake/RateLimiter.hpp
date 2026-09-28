// RateLimiter.hpp
//
// slew limit and saturation. models what the actuator can physically do, so it
// also holds the core's belief about where the actuator currently is
//

#pragma once

namespace airbrake {

class RateLimiter {
public:
    RateLimiter() = default;
    void init(float max_deploy_rate_per_s, float max_retract_rate_per_s, float lower, float upper);

    // advance toward target, limited by max_directional_rate * dt_s
    float update(float target, float dt_s);

    // force the internal belief with no slew. ONLY for when the actuator is
    // known to have moved by something other than our own commands
    void snapTo(float value);

    float value() const;

private:
    float max_deploy_rate_ = 0.0f;
    float max_retract_rate_ = 0.0f;
    float lower_ = 0.0f;
    float upper_ = 1.0f;
    float value_ = 0.0f;
};

}  // namespace airbrake
