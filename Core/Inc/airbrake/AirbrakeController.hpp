// AirbrakeController.hpp
//
// full orchestration. one instance per process
//
// trivially constructible on purpose. all the real initialisation happens in
// configure(), so nothing depends on C++ static initialisation running before
// main() on the STM32 target, and nothing depends on construction order across
// translation units
//
// the update order is what prevents a one-tick NaN from poisoning the
// controller's internal state
//
//      1. validate inputs, nothing persistent touched yet
//      2. predict, correct, bias and rate limit, ALL INTO LOCALS
//      3. validate the result
//      4. only now commit to persistent state
//
// guarding the write and not just the read is what makes a one-tick NaN
// structurally incapable of becoming a two-tick NaN
//
#pragma once
#include <cstdint>
#include "airbrake/Types.hpp"
#include "airbrake/CdTable.hpp"
#include "airbrake/ApogeePredictor.hpp"
#include "airbrake/DeploymentCorrector.hpp"
#include "airbrake/RateLimiter.hpp"
#include "airbrake/FaultMonitor.hpp"

namespace airbrake {

class AirbrakeController {
public:
    AirbrakeController() = default;

    // false if the config fails validation. the controller then stays
    // unconfigured and every tick returns neutral
    bool configure(const FlightConfig& cfg);

    // one control loop iteration. never allocates, never throws, always
    // returns a finite command in [0,1]
    ControllerOutput update(const FlightState& state, float dt_s);

    // one-way software abort. slews to neutral THROUGH the rate limiter, as
    // the servo is not made faster by commanding faster than it can move, then
    // latches. cleared only by reset()
    void abort();

    // the hardware failsafe (AURC 11.1.3) has mechanically retracted the
    // brakes faster than the servo could. snaps the rate limiter's belief to
    // neutral so it is not left tracking a position the actuator is not at.
    // distinct from abort() precisely because of that snap
    void notifyMechanicalRetract();

    // GROUND AND TEST USE ONLY. clears the abort latch, never call in flight
    void reset();

    float                commandedDeployment() const;
    Mode                 mode() const;
    bool                 isConfigured() const;
    const FaultSnapshot& faultSnapshot() const;

    // pre-flight self test. runs built-in golden vectors through the whole
    // chain and checks them against compiled-in expected values. evidence for
    // AURC 6.1.6 and a cheap catch for a corrupted flash image
    bool selfTest() const;

private:
    FlightConfig        cfg_{};
    bool                configured_ = false;
    CdTable             table_;
    ApogeePredictor     predictor_;
    DeploymentCorrector corrector_;
    RateLimiter         limiter_;
    FaultMonitor        faults_;
    float               last_command_ = 0.0f;
};

}  // namespace airbrake
