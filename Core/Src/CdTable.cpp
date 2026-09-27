// CdTable.cpp 
// bilinear forward lookup and Cd inverse over Mach, Deployment table
// no libm to avoid NaN generation / silent round to 0

#include "airbrake/CdTable.hpp"
#include "airbrake/Validate.hpp"
#include <cstddef>

namespace airbrake {

namespace {

// Finds the bracketing interval on an explicit ascending axis and the
// interpolation fraction within it. Search rather than arithmetic because the
// axis spacing is not assumed uniform, see the note on explicit axes in
// CdTable.hpp. Non-finite x is clamped to the axis start, same as any other
// out-of-range value.
CdTable::AxisBracket findBracket(const float* axis, uint8_t count, float x) {
    bool clamp_ok = false;
    const float clamped_x = validate::clampChecked(x, axis[0], axis[count - 1], axis[0], &clamp_ok);

    uint8_t low = 0;
    for (uint8_t i = 0; i < static_cast<uint8_t>(count - 1); ++i) {
        if (clamped_x >= axis[i]) {
            low = i;
        }
    }

    const float span = axis[low + 1] - axis[low];
    float t = 0.0f;
    if (span > 0.0f) {
        t = (clamped_x - axis[low]) / span;
    }
    bool frac_ok = false;

    CdTable::AxisBracket bracket;
    bracket.index_low = low;
    bracket.fraction = validate::clampChecked(t, 0.0f, 1.0f, 0.0f, &frac_ok);
    return bracket;
}

}  // namespace

void CdTable::init(const CdGrid& grid, float cd_scale) {
    grid_ = grid;
    cd_scale_ = cd_scale;
}

CdTable::AxisBracket CdTable::deploymentBracket(float deployment) const {
    if (!isInitialised()) {
        return AxisBracket{};
    }
    return findBracket(grid_.deploy_axis, grid_.n_deploy, deployment);
}

float CdTable::cd(float mach, float deployment) const {
    return cdAtBracketedDeployment(mach, deploymentBracket(deployment));
}

float CdTable::cdAtBracketedDeployment(float mach, AxisBracket deploy_bracket) const {
    if (!isInitialised()) {
        // no grid to look up; a fixed finite value beats dereferencing null
        return 1.0f;
    }

    const AxisBracket mach_bracket = findBracket(grid_.mach_axis, grid_.n_mach, mach);
    const uint8_t mach_low = mach_bracket.index_low;
    const float mach_fraction = mach_bracket.fraction;

    const uint8_t deploy_count = grid_.n_deploy;

    // the caller owns the deployment bracket, so a stale one built against a
    // larger grid would read past the row. Clamp instead of trusting it.
    const uint8_t highest_low = static_cast<uint8_t>(deploy_count - 2);
    const uint8_t deploy_low =
        deploy_bracket.index_low > highest_low ? highest_low : deploy_bracket.index_low;
    bool frac_ok = false;
    const float deploy_fraction =
        validate::clampChecked(deploy_bracket.fraction, 0.0f, 1.0f, 0.0f, &frac_ok);

    const float cd_low_low = grid_.cd[static_cast<size_t>(mach_low) * deploy_count + deploy_low];
    const float cd_low_high = grid_.cd[static_cast<size_t>(mach_low) * deploy_count + deploy_low + 1];
    const float cd_high_low = grid_.cd[static_cast<size_t>(mach_low + 1) * deploy_count + deploy_low];
    const float cd_high_high = grid_.cd[static_cast<size_t>(mach_low + 1) * deploy_count + deploy_low + 1];

    // interpolate along deployment first at each bracketing mach row, then
    // along mach between the two rows
    const float row_low = cd_low_low + deploy_fraction * (cd_low_high - cd_low_low);
    const float row_high = cd_high_low + deploy_fraction * (cd_high_high - cd_high_low);
    const float result = row_low + mach_fraction * (row_high - row_low);

    return result * cd_scale_;
}

float CdTable::cdAtMinDeploy(float mach) const {
    if (!isInitialised()) {
        return 1.0f;
    }
    return cd(mach, grid_.deploy_axis[0]);
}

float CdTable::cdAtMaxDeploy(float mach) const {
    if (!isInitialised()) {
        return 1.0f;
    }
    return cd(mach, grid_.deploy_axis[grid_.n_deploy - 1]);
}

float CdTable::deploymentForCd(float mach, float cd_required, bool* out_saturated) const {
    if (!isInitialised()) {
        if (out_saturated != nullptr) {
            *out_saturated = false;
        }
        return 0.0f;
    }

    // stack buffer for the collapsed 1D curve; no heap available. Sized for
    // the current 7-point deployment axis plus room for the outstanding
    // refinement above 75%, see the note on explicit axes in CdTable.hpp.
    constexpr uint8_t kMaxDeploySamples = 16;
    const uint8_t deploy_count = grid_.n_deploy;
    if (deploy_count > kMaxDeploySamples) {
        if (out_saturated != nullptr) {
            *out_saturated = false;
        }
        return grid_.deploy_axis[0];
    }

    const AxisBracket mach_bracket = findBracket(grid_.mach_axis, grid_.n_mach, mach);
    const uint8_t mach_low = mach_bracket.index_low;
    const float mach_fraction = mach_bracket.fraction;

    // collapse the mach dimension first into a single Cd(deployment) curve at
    // this mach, then invert that curve. Simpler and better conditioned than
    // searching the 2D grid directly.
    float curve[kMaxDeploySamples];
    for (uint8_t j = 0; j < deploy_count; ++j) {
        const float cd_low = grid_.cd[static_cast<size_t>(mach_low) * deploy_count + j];
        const float cd_high = grid_.cd[static_cast<size_t>(mach_low + 1) * deploy_count + j];
        curve[j] = (cd_low + mach_fraction * (cd_high - cd_low)) * cd_scale_;
    }

    const float min_cd = curve[0];
    const float max_cd = curve[deploy_count - 1];

    if (!validate::isFinite(cd_required) || cd_required < min_cd) {
        if (out_saturated != nullptr) {
            *out_saturated = true;
        }
        return grid_.deploy_axis[0];
    }
    if (cd_required > max_cd) {
        if (out_saturated != nullptr) {
            *out_saturated = true;
        }
        return grid_.deploy_axis[deploy_count - 1];
    }
    if (out_saturated != nullptr) {
        *out_saturated = false;
    }

    // binary search the bracketing interval; curve is non-decreasing in
    // deployment, enforced offline by validateGrid
    uint8_t low = 0;
    uint8_t high = static_cast<uint8_t>(deploy_count - 1);
    while (static_cast<uint8_t>(high - low) > 1) {
        const uint8_t mid = static_cast<uint8_t>(low + static_cast<uint8_t>((high - low) / 2));
        if (curve[mid] <= cd_required) {
            low = mid;
        } else {
            high = mid;
        }
    }

    // a zero-width Cd interval (flat region) leaves fraction at 0, which
    // returns the lower deployment of the flat region per contract
    const float cd_span = curve[high] - curve[low];
    float fraction = 0.0f;
    if (cd_span > 0.0f) {
        fraction = (cd_required - curve[low]) / cd_span;
    }
    bool frac_ok = false;
    fraction = validate::clampChecked(fraction, 0.0f, 1.0f, 0.0f, &frac_ok);

    const float deploy_span = grid_.deploy_axis[high] - grid_.deploy_axis[low];
    return grid_.deploy_axis[low] + fraction * deploy_span;
}

bool CdTable::machInRange(float mach) const {
    if (!isInitialised()) {
        return false;
    }
    return validate::inRange(mach, grid_.mach_axis[0], grid_.mach_axis[grid_.n_mach - 1]);
}

bool CdTable::isInitialised() const {
    return grid_.mach_axis != nullptr && grid_.deploy_axis != nullptr && grid_.cd != nullptr &&
           grid_.n_mach >= 2 && grid_.n_deploy >= 2;
}

bool CdTable::validateGrid(const CdGrid& grid) {
    if (grid.mach_axis == nullptr || grid.deploy_axis == nullptr || grid.cd == nullptr) {
        return false;
    }
    if (grid.n_mach < 2 || grid.n_deploy < 2) {
        return false;
    }

    // both axes strictly ascending, positive form: assert what must be true
    for (uint8_t i = 1; i < grid.n_mach; ++i) {
        if (!(grid.mach_axis[i] > grid.mach_axis[i - 1])) {
            return false;
        }
    }
    for (uint8_t j = 1; j < grid.n_deploy; ++j) {
        if (!(grid.deploy_axis[j] > grid.deploy_axis[j - 1])) {
            return false;
        }
    }

    // every cell finite and strictly positive, and non-decreasing along
    // deployment for each mach row, since the runtime inverse binary-searches
    // on exactly that assumption
    for (uint8_t i = 0; i < grid.n_mach; ++i) {
        float previous_cd = 0.0f;
        for (uint8_t j = 0; j < grid.n_deploy; ++j) {
            const float value = grid.cd[static_cast<size_t>(i) * grid.n_deploy + j];
            if (!validate::isFinite(value) || !(value > 0.0f)) {
                return false;
            }
            if (j > 0 && !(value >= previous_cd)) {
                return false;
            }
            previous_cd = value;
        }
    }

    return true;
}

}  // namespace airbrake
