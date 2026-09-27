// FaultMonitor.cpp
// fault response state machine
#include "airbrake/FaultMonitor.hpp"
#include <limits>

namespace airbrake {

namespace {

// increment a uint16_t, checking for overflow prior
// overflow could look like a fault clear
uint16_t saturatingInc(uint16_t value) {
    constexpr uint16_t kMax = std::numeric_limits<uint16_t>::max();
    if (value == kMax) {
        return kMax;
    }
    return static_cast<uint16_t>(value + 1u);
}

uint32_t saturatingInc(uint32_t value) {
    constexpr uint32_t kMax = std::numeric_limits<uint32_t>::max();
    if (value == kMax) {
        return kMax;
    }
    return value + 1u;
}

}  // namespace

void FaultMonitor::init(uint16_t bridge_ticks, uint16_t recovery_ticks, uint16_t degraded_max_ticks) {
    bridge_ticks_ = bridge_ticks;
    recovery_ticks_ = recovery_ticks;
    degraded_max_ticks_ = degraded_max_ticks;

    consecutive_bad_ = 0;
    consecutive_good_ = 0;
    degraded_elapsed_ = 0;
    sticky_flags_ = 0;
    tick_index_ = 0;
    latched_ = false;
    mode_ = Mode::Normal;
    snapshot_ = FaultSnapshot{};
}

// FOR GROUND TESTING ONLY. NOT TO BE CALLED MID-FLIGHT
void FaultMonitor::reset() {
    // full clear of error states
    consecutive_bad_ = 0;
    consecutive_good_ = 0;
    degraded_elapsed_ = 0;
    sticky_flags_ = 0;
    tick_index_ = 0;
    latched_ = false;
    mode_ = Mode::NotConfigured;
    snapshot_ = FaultSnapshot{};
}

void FaultMonitor::recordGood() {
    // latched is perminant
    if (latched_) {
        return;
    }

    tick_index_ = saturatingInc(tick_index_);
    consecutive_bad_ = 0;
    consecutive_good_ = saturatingInc(consecutive_good_);

    if (mode_ == Mode::Bridging) {
        // one good tick is enough to leave a bridging state
        mode_ = Mode::Normal;
        degraded_elapsed_ = 0;
    } else if (mode_ == Mode::Degraded) {
        // multiple good ticks required to clear a degraded state
        degraded_elapsed_ = saturatingInc(degraded_elapsed_);
        if (degraded_elapsed_ > degraded_max_ticks_) {
            latch();
            return;
        }
        if (consecutive_good_ >= recovery_ticks_) {
            mode_ = Mode::Normal;
            degraded_elapsed_ = 0;
            consecutive_bad_ = 0;
        }
    }
}

void FaultMonitor::recordFault(FaultSource source, uint16_t flags,
                               const FlightState& s, float dt_s, const FlightConfig& c) {
    tick_index_ = saturatingInc(tick_index_);
    consecutive_good_ = 0;
    consecutive_bad_ = saturatingInc(consecutive_bad_);
    sticky_flags_ = static_cast<uint16_t>(sticky_flags_ | flags);

    // snapshot the first fault which occurs in flight for reproduction
    // later faults should not overwrite
    if (!snapshot_.valid) {
        snapshot_.valid = true;
        snapshot_.tick_index = tick_index_;
        snapshot_.source = source;
        snapshot_.flags = flags;
        snapshot_.state = s;
        snapshot_.dt_s = dt_s;
        snapshot_.config = c;
    }

    if (source == FaultSource::Internal) {
        // nonfinite valud from finite innputs means an internal issue is
        // present. The produced state is unrecoverable, transition directly
        // to a degraded state and latch
        latch();
        return;
    }

    if (latched_) {
        return;
    }

    if (mode_ == Mode::Normal || mode_ == Mode::Bridging) {
        if (consecutive_bad_ <= bridge_ticks_) {
            mode_ = Mode::Bridging;
        } else {
            mode_ = Mode::Degraded;
            degraded_elapsed_ = 1;
        }
    } else if (mode_ == Mode::Degraded) {
        degraded_elapsed_ = saturatingInc(degraded_elapsed_);
        if (degraded_elapsed_ > degraded_max_ticks_) {
            latch();
        }
    }
}

void FaultMonitor::latch() {
    latched_ = true;
    mode_ = Mode::Latched;
}

Mode FaultMonitor::mode() const {
    return mode_;
}

bool FaultMonitor::isLatched() const {
    return latched_;
}

uint16_t FaultMonitor::stickyFlags() const {
    return sticky_flags_;
}

uint16_t FaultMonitor::consecutiveBad() const {
    return consecutive_bad_;
}

uint32_t FaultMonitor::tickIndex() const {
    return tick_index_;
}

const FaultSnapshot& FaultMonitor::snapshot() const {
    return snapshot_;
}

}  // namespace airbrake
