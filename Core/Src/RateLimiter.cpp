// RateLimiter.cpp
#include "airbrake/RateLimiter.hpp"
#include "airbrake/Validate.hpp"

namespace airbrake {

void RateLimiter::init(float max_rate_per_s, float lower, float upper) {
    // validate: range is finite & rate is non-negative and finite

    if (!validate::isFinite(max_rate_per_s) || max_rate_per_s < 0.0f) {
        return;
    }
    if (!validate::isFinite(lower) || !validate::isFinite(upper)) {
        return;
    }
    max_rate_ = max_rate_per_s;
    lower_ = lower;
    upper_ = upper;
}

float RateLimiter::update(float target, float dt_s) {
    // validate before updating internal state

    if (!validate::isFinite(target) || !validate::isFinite(dt_s)) {
        return value_;
    }

    const float max_step = max_rate_ * dt_s;
    float next_value = value_;
    const float delta = target - value_;
    if (delta > max_step) {
        next_value = value_ + max_step;
    } else if (delta < -max_step) {
        next_value = value_ - max_step;
    } else {
        next_value = target;
    }

    bool ok = true;
    next_value = validate::clampChecked(next_value, lower_, upper_, value_, &ok);

    value_ = next_value;
    return value_;
}

void RateLimiter::snapTo(float value) {
    if (!validate::isFinite(value)) {
        return;
    }
    value_ = value;
}

float RateLimiter::value() const {
    return value_;
}

}  // namespace airbrake
