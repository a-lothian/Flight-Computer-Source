// Validate.cpp 
// implementation of NaN safe validation primitives.
// Every check is written as a positive form (assert what must be true)
// as comparisons against NaN are always false

#include "airbrake/Validate.hpp"
#include "airbrake/AirbrakeConfig.hpp"
#include <cstring>

namespace airbrake {
namespace validate {

bool isFinite(float x) {

    // bit pattern check rather than std::isfinite avoids pulling libm
    // memcpy to uint32 is identical on all build targets

    // using x == x relies on IEEE semantics which break with -ffast-math
    // which shouldn't be used, but this stays correct incase someone does.
    
    uint32_t bits;
    std::memcpy(&bits, &x, sizeof(bits));
    const uint32_t exponent_mask = 0x7F800000u;
    return (bits & exponent_mask) != exponent_mask;
}

bool inRange(float x, float lo, float hi) {
    return isFinite(x) && x >= lo && x <= hi;
}

float clampChecked(float x, float lo, float hi, float fallback, bool* ok) {
    if (!isFinite(x)) {
        if (ok != nullptr) {
            *ok = false;
        }
        return fallback;
    }
    if (ok != nullptr) {
        *ok = true;
    }
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

uint16_t checkState(const FlightState& s, float dt_s) {
    uint16_t flags = FAULT_NONE;

    if (isFinite(s.altitude_agl_m)) {
        if (!(s.altitude_agl_m >= config::kAltitudeMin_m && s.altitude_agl_m <= config::kAltitudeMax_m)) {
            flags |= FAULT_INPUT_RANGE;
        }
    } else {
        flags |= FAULT_INPUT_NONFINITE;
    }

    if (isFinite(s.velocity_up_mps)) {
        if (!(s.velocity_up_mps >= config::kVelUpMin_mps && s.velocity_up_mps <= config::kVelUpMax_mps)) {
            flags |= FAULT_INPUT_RANGE;
        }
    } else {
        flags |= FAULT_INPUT_NONFINITE;
    }

    if (isFinite(s.speed_mps)) {
        if (!(s.speed_mps >= config::kSpeedMin_mps && s.speed_mps <= config::kSpeedMax_mps)) {
            flags |= FAULT_INPUT_RANGE;
        }
    } else {
        flags |= FAULT_INPUT_NONFINITE;
    }

    // dt must be finite and in range 0.0001 - 1 to be valid.
    
    if (isFinite(dt_s)) {
        if (!(dt_s >= config::kDtMin_s && dt_s <= config::kDtClampMax_s)) {
            flags |= FAULT_DT_OUTLIER;
        }
    } else {
        flags |= FAULT_INPUT_NONFINITE;
    }

    return flags;
}

uint16_t checkConfig(const FlightConfig& c) {
    uint16_t flags = FAULT_NONE;

    if (isFinite(c.mass_kg)) {
        if (!(c.mass_kg >= config::kMassMin_kg && c.mass_kg <= config::kMassMax_kg)) {
            flags |= FAULT_INPUT_RANGE | FAULT_CONFIG_INVALID;
        }
    } else {
        flags |= FAULT_INPUT_NONFINITE | FAULT_CONFIG_INVALID;
    }

    if (isFinite(c.target_apogee_agl_m)) {
        if (!(c.target_apogee_agl_m >= config::kAltitudeMin_m && c.target_apogee_agl_m <= config::kAltitudeMax_m)) {
            flags |= FAULT_INPUT_RANGE | FAULT_CONFIG_INVALID;
        }
    } else {
        flags |= FAULT_INPUT_NONFINITE | FAULT_CONFIG_INVALID;
    }

    if (isFinite(c.launch_elevation_msl_m)) {
        if (!(c.launch_elevation_msl_m >= config::kAltitudeMin_m && c.launch_elevation_msl_m <= config::kAltitudeMax_m)) {
            flags |= FAULT_INPUT_RANGE | FAULT_CONFIG_INVALID;
        }
    } else {
        flags |= FAULT_INPUT_NONFINITE | FAULT_CONFIG_INVALID;
    }

    if (isFinite(c.cd_scale)) {
        if (!(c.cd_scale >= config::kCdScaleMin && c.cd_scale <= config::kCdScaleMax)) {
            flags |= FAULT_INPUT_RANGE | FAULT_CONFIG_INVALID;
        }
    } else {
        flags |= FAULT_INPUT_NONFINITE | FAULT_CONFIG_INVALID;
    }

    return flags;
}

float guardCommand(float cmd, bool* clamped) {
    if (!isFinite(cmd)) {
        if (clamped != nullptr) {
            *clamped = true;
        }
        return config::kNeutral;
    }
    if (cmd < config::kDeployMin) {
        if (clamped != nullptr) {
            *clamped = true;
        }
        return config::kDeployMin;
    }
    if (cmd > config::kDeployMax) {
        if (clamped != nullptr) {
            *clamped = true;
        }
        return config::kDeployMax;
    }
    if (clamped != nullptr) {
        *clamped = false;
    }
    return cmd;
}

}  // namespace validate
}  // namespace airbrake
