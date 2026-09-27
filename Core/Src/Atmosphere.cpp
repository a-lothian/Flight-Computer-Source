// Atmosphere.cpp 
// ISA 1976 troposphere and lower stratosphere.
// Pressure and density come from the fitted piecewise polynomial in 
// AtmosphereData.cpp, temperature is exact ISA since it is piecewise 
// linear and needs no fit.
//
// polynomial fit to avoid heavy powf() call
//
#include <cmath>
#include "airbrake/Atmosphere.hpp"
#include "airbrake/AtmosphereData.hpp"
#include "airbrake/AirbrakeConfig.hpp"
#include "airbrake/Validate.hpp"

namespace airbrake {

namespace {

// force clamp within finite range instead of NaN later
float clampAltitude(float altitude_msl_m) {
    bool ok = false;
    return validate::clampChecked(altitude_msl_m, Atmosphere::kMinAltitude_m,
                                   Atmosphere::kMaxAltitude_m, 0.0f, &ok);
}

// Pick the segment whose [lo, hi) covers the clamped altitude
int segmentIndex(float altitude_msl_m) {
    for (int i = 0; i < kAtmosphereNumSegments - 1; ++i) {
        if (altitude_msl_m < kAtmosphereSegmentHi[i]) {
            return i;
        }
    }
    return kAtmosphereNumSegments - 1;
}

// Float32 horner polynomial evaluation, highest order coefficient first
//
// no powf() guarentees identical results across all IEEE targets provided
// compilation with -ffp-contract=off
float hornerEval(const float* coeffs, int count, float x) {
    float result = coeffs[0];
    for (int i = 1; i < count; ++i) {
        result = result * x + coeffs[i];
    }
    return result;
}

}  // namespace

float Atmosphere::temperature_K(float altitude_msl_m) {
    const float altitude = clampAltitude(altitude_msl_m);
    const float kSeaLevelTemperature_K = 288.15f;
    const float kLapseRate_Kperm = 0.0065f;
    const float kTropopauseAltitude_m = 11000.0f;
    const float kStratosphereTemperature_K = 216.65f;

    if (altitude < kTropopauseAltitude_m) {
        return kSeaLevelTemperature_K - kLapseRate_Kperm * altitude;
    }
    return kStratosphereTemperature_K;
}

float Atmosphere::pressure_Pa(float altitude_msl_m) {
    const float altitude = clampAltitude(altitude_msl_m);
    const int segment = segmentIndex(altitude);
    const float x = (altitude - kAtmosphereSegmentOffset[segment]) / kAtmosphereSegmentScale[segment];
    return hornerEval(kAtmospherePressureCoeffs[segment], kAtmospherePressureCoeffCount[segment], x);
}

float Atmosphere::density_kgm3(float altitude_msl_m) {
    const float altitude = clampAltitude(altitude_msl_m);
    const int segment = segmentIndex(altitude);
    const float x = (altitude - kAtmosphereSegmentOffset[segment]) / kAtmosphereSegmentScale[segment];
    return hornerEval(kAtmosphereDensityCoeffs[segment], kAtmosphereDensityCoeffCount[segment], x);
}

float Atmosphere::speedOfSound_mps(float altitude_msl_m) {
    const float kGamma = 1.4f;
    const float kGasConstant_JperkgK = 287.05287f;
    const float temperature = temperature_K(altitude_msl_m);
    return sqrtf(kGamma * kGasConstant_JperkgK * temperature);
}

float Atmosphere::mach(float speed_mps, float altitude_msl_m) {
    bool speed_ok = false;
    // negative speed has no physical meaning, clamp to 0
    const float speed = validate::clampChecked(speed_mps, 0.0f, config::kSpeedMax_mps, 0.0f, &speed_ok);
    const float speed_of_sound = speedOfSound_mps(altitude_msl_m);
    // speed of sounds should never be near 0 over the AURC altitude range
    // but guard against div/0
    if (speed_of_sound <= 0.0f) {
        return 0.0f;
    }
    return speed / speed_of_sound;
}

float Atmosphere::gravity_mps2(float altitude_msl_m) {
    const float altitude = clampAltitude(altitude_msl_m);
    const float ratio = config::kEarthRadius_m / (config::kEarthRadius_m + altitude);
    return config::kG0_mps2 * ratio * ratio;
}

}  // namespace airbrake
