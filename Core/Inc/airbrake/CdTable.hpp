// CdTable.hpp
//
// CFD derived (Mach, deployment) to Cd. bilinear forward interpolation plus a
// Cd-space inverse
//
// the grid carries EXPLICIT AXIS ARRAYS rather than (min, max, count). the
// delivered CFD table is uniform on both axes, but a deployment refinement
// above 75% is still outstanding, so explicit axes keep that arriving as a
// data change rather than an interface change
//
#pragma once
#include <cstdint>

namespace airbrake {

struct CdGrid {
    const float* mach_axis;    // ascending, n_mach entries
    const float* deploy_axis;  // ascending, n_deploy entries, spanning [0,1]
    const float* cd;           // row major, cd[i*n_deploy + j]
    uint8_t      n_mach;
    uint8_t      n_deploy;
};

class CdTable {
public:
    CdTable() = default;
    void init(const CdGrid& grid, float cd_scale);

    // a resolved position on one axis. the lower node, and how far along the
    // interval to the next one
    struct AxisBracket {
        uint8_t index_low = 0;
        float   fraction  = 0.0f;
    };

    // forward lookup, bilinear, scaled by cd_scale. clamps to the grid edges
    float cd(float mach, float deployment) const;

    // resolve a deployment once so it can be reused across many lookups. a
    // prediction holds deployment fixed while Mach decays, so the bracket is
    // the same for every stage of that integration. returns a zeroed bracket
    // when the table is not initialised
    AxisBracket deploymentBracket(float deployment) const;

    // forward lookup against an already resolved deployment, same result as
    // cd(). an index from a different grid is clamped rather than trusted
    float cdAtBracketedDeployment(float mach, AxisBracket deploy_bracket) const;

    float cdAtMinDeploy(float mach) const;
    float cdAtMaxDeploy(float mach) const;

    // inverse, binary search in Cd-space. assumes Cd is non-decreasing in
    // deployment at fixed Mach, which validateGrid() enforces.
    // out_saturated is true when cd_required is outside the achievable range
    float deploymentForCd(float mach, float cd_required, bool* out_saturated) const;

    bool machInRange(float mach) const;
    bool isInitialised() const;

    // offline check. axes strictly ascending, every value finite and positive,
    // Cd monotonic in deployment for every Mach row
    static bool validateGrid(const CdGrid& grid);

private:
    CdGrid grid_{};
    float  cd_scale_ = 1.0f;
};

// the flight table, defined in the generated CdTableData.cpp
extern const CdGrid  kFlightCdGrid;
extern const char    kCdTableProvenance[];   // CFD run ID, telemetered at boot

}  // namespace airbrake
