// AtmosphereData.hpp
//
// declarations for the generated ISA polynomial fit
// populated by src/AtmosphereData.cpp, produced by tools/gen_atmosphere.py
//
#pragma once

namespace airbrake {

extern const int kAtmosphereNumSegments;

// segment boundaries in metres MSL, ascending, kAtmosphereNumSegments entries
extern const float kAtmosphereSegmentLo[];
extern const float kAtmosphereSegmentHi[];

// normalisation is x = (altitude_m - offset) / scale, mapping the segment to
// [-1, 1] before the polynomial is evaluated
extern const float kAtmosphereSegmentOffset[];
extern const float kAtmosphereSegmentScale[];

// per-segment coefficients, HIGHEST ORDER FIRST, for float32 Horner
// evaluation. index with the same segment index as the boundaries above
extern const float* const kAtmospherePressureCoeffs[];
extern const float* const kAtmosphereDensityCoeffs[];
extern const int kAtmospherePressureCoeffCount[];
extern const int kAtmosphereDensityCoeffCount[];

// worst case relative error of the fit, measured in float32 over the full
// valid altitude range. checked by unit test rather than taken on trust
extern const float kAtmosphereWorstRelErrorPressure;
extern const float kAtmosphereWorstRelErrorDensity;
extern const float kAtmosphereWorstRelError;

}  // namespace airbrake
