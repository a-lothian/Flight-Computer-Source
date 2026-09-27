// OvershootBias.cpp
#include "airbrake/OvershootBias.hpp"
#include "airbrake/AirbrakeConfig.hpp"
#include "airbrake/Validate.hpp"

namespace airbrake {

float OvershootBias::factor(float progress) {
    // nonfinite progress falls back to 0, overshoot is recoverable, undershoot is not
    bool ok = true;
    const float clamped_progress = validate::clampChecked(progress, 0.0f, 1.0f, 0.0f, &ok);
    return config::kBiasFloor + (1.0f - config::kBiasFloor) * clamped_progress * clamped_progress;
}

float OvershootBias::apply(float deployment_raw, float progress) {
    return deployment_raw * factor(progress);
}

}  // namespace airbrake
