// ApogeePredictor.hpp
//
// RK4 integration of the 1D vertical coast equations
//
// the state is (altitude, speed) and drag opposes motion along a vertical
// path, so off-vertical flight is not modelled. speed_mps is the TOTAL
// airspeed and drives the drag magnitude, velocity_up_mps drives the altitude
// integration
//
// passing only the vertical component would under-predict drag and so
// over-predict apogee, which biases the controller toward over-braking
//
#pragma once
#include <cstdint>
#include "airbrake/AirbrakeConfig.hpp"
#include "airbrake/CdTable.hpp"

namespace airbrake {

struct ApogeePrediction {
    float    apogee_agl_m;
    float    time_to_apogee_s;
    uint16_t steps;
    bool     step_limit_hit;  // bailed before v <= 0, result is NOT usable
    bool     finite;          // every intermediate stayed finite
};

class ApogeePredictor {
public:
    ApogeePredictor() = default;
    void init(const CdTable* table, float reference_area_m2, float launch_elevation_msl_m);

    // integrate to apogee assuming deployment is held for the rest of coast.
    // Cd is looked up again at every RK4 stage as Mach decays
    ApogeePrediction predictAtDeployment(float altitude_agl_m,
                                         float velocity_up_mps,
                                         float speed_mps,
                                         float mass_kg,
                                         float deployment) const;

    // Mach independent constant Cd, for analytic cross-checks only
    ApogeePrediction predictAtFixedCd(float altitude_agl_m,
                                      float velocity_up_mps,
                                      float speed_mps,
                                      float mass_kg,
                                      float cd) const;

    // instantaneous acceleration, exposed for unit test
    float acceleration(float altitude_agl_m, float speed_mps,
                       float mass_kg, float deployment) const;

    // override the RK4 step. flight code never calls this and leaves the step
    // at config::kIntegrationStep_s
    //
    // it exists because the order of convergence test is the only thing that
    // actually distinguishes RK4 from forward Euler. halving the step has to
    // shrink the error by about sixteen, and with a compile-time fixed step
    // there is no way to measure that, so "this is RK4" would be a claim
    // nobody can check
    //
    // a non-finite or non-positive value is ignored so a bad call cannot
    // silently stop the integration from advancing
    void setIntegrationStep(float step_s);

    float integrationStep() const;

private:
    const CdTable* table_ = nullptr;
    float reference_area_m2_ = 0.0f;
    float launch_elevation_msl_m_ = 0.0f;
    float integration_step_s_ = config::kIntegrationStep_s;
};

}  // namespace airbrake
