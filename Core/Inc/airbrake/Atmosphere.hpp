// Atmosphere.hpp
//
// ISA 1976 troposphere and lower stratosphere
//
// inputs are GEOMETRIC ALTITUDE ABOVE MEAN SEA LEVEL, never AGL. the caller
// adds FlightConfig::launch_elevation_msl_m
//
// no libm transcendentals. the canonical ISA density expression uses powf(),
// which is several hundred cycles on a Cortex-M4F and is not bit-identical
// between newlib and glibc, so host and target golden vectors would disagree.
// a fitted polynomial replaces it, checked against published ISA values
//
#pragma once

namespace airbrake {

class Atmosphere {
public:
    static float temperature_K(float altitude_msl_m);
    static float pressure_Pa(float altitude_msl_m);
    static float density_kgm3(float altitude_msl_m);
    static float speedOfSound_mps(float altitude_msl_m);
    static float mach(float speed_mps, float altitude_msl_m);
    static float gravity_mps2(float altitude_msl_m);

    // model validity, clamped to at the point of use. outside this the ISA
    // temperature term goes negative and the expression becomes a NaN factory
    static constexpr float kMinAltitude_m = -500.0f;
    static constexpr float kMaxAltitude_m = 20000.0f;

    // max absolute relative error of the approximation against exact ISA over
    // the valid range, asserted by unit test
    static constexpr float kApproxRelTolerance = 1.0e-4f;
};

}  // namespace airbrake
