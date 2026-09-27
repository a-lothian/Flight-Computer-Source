// FaultMonitor.hpp
//
// fault response state machine
//
// two counters, deliberately asymmetric
//
//      upstream (bad input from the estimator) is bridgeable and recoverable.
//      a dropped sample is plausible and does not mean this module is broken
//
//      internal (non-finite produced from finite inputs) latches at once. it
//      means a domain clamp is missing, nothing recovers from that, and the
//      next tick with similar inputs would do the same thing
//
// the asymmetry also gives a test oracle. ANY internal latch in testing is a
// failure rather than a tolerated event, so zero latches across a full
// dispersion run is the evidence that the clamps cover the reachable states
//
// all counters saturate rather than wrap, as a wrapping uint16 could clear a
// latch
//
#pragma once
#include <cstdint>
#include "airbrake/Types.hpp"

namespace airbrake {

class FaultMonitor {
public:
    FaultMonitor() = default;
    void init(uint16_t bridge_ticks, uint16_t recovery_ticks, uint16_t degraded_max_ticks);

    // full clear, GROUND USE ONLY as this clears the latch
    void reset();

    void recordGood();
    void recordFault(FaultSource source, uint16_t flags,
                     const FlightState& s, float dt_s, const FlightConfig& c);

    // one way. neutral for the rest of the flight
    void latch();

    Mode     mode() const;
    bool     isLatched() const;
    uint16_t stickyFlags() const;
    uint16_t consecutiveBad() const;
    uint32_t tickIndex() const;
    const FaultSnapshot& snapshot() const;   // first fault only, write once

private:
    uint16_t bridge_ticks_ = 0;
    uint16_t recovery_ticks_ = 0;
    uint16_t degraded_max_ticks_ = 0;

    uint16_t consecutive_bad_ = 0;
    uint16_t consecutive_good_ = 0;
    uint16_t degraded_elapsed_ = 0;
    uint16_t sticky_flags_ = 0;
    uint32_t tick_index_ = 0;
    bool     latched_ = false;
    Mode     mode_ = Mode::NotConfigured;
    FaultSnapshot snapshot_{};
};

}  // namespace airbrake
