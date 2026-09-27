// AtmosphereData.cpp
//
// GENERATED, do not hand-edit. produced by tools/gen_atmosphere.py
//
// a fitted polynomial replaces the canonical ISA powf expression, which is
// several hundred cycles on a Cortex-M4F and is not bit-identical between
// newlib and glibc
//
// each altitude segment is normalised as x = (h - offset) / scale, mapping the
// segment onto [-1, 1], then evaluated with float32 Horner using coefficients
// HIGHEST ORDER FIRST. Atmosphere.cpp has to consume them in that order
#include "airbrake/AtmosphereData.hpp"

namespace airbrake {

const int kAtmosphereNumSegments = 3;

const float kAtmosphereSegmentLo[] = { -500.0f, 5000.0f, 11000.0f };
const float kAtmosphereSegmentHi[] = { 5000.0f, 11000.0f, 20000.0f };
const float kAtmosphereSegmentOffset[] = { 2250.0f, 8000.0f, 15500.0f };
const float kAtmosphereSegmentScale[] = { 2750.0f, 3000.0f, 4500.0f };

static const float kPressureCoeffs0[] = { 9.59422874f, -261.21994f, 3680.65552f, -26467.5566f, 77058.4531f };  // degree 4
static const float kPressureCoeffs1[] = { 11.2798204f, -243.553894f, 2714.88916f, -15450.3379f, 35599.8086f };  // degree 4
static const float kPressureCoeffs2[] = { 1.9358269f, -17.0022812f, 117.671097f, -662.746033f, 2802.46411f, -7898.83008f, 11131.3994f };  // degree 6
static const float kDensityCoeffs0[] = { -0.00142298534f, 0.0290641263f, -0.272962481f, 0.981431544f };  // degree 3
static const float kDensityCoeffs1[] = { 3.99060991e-05f, -0.00154067343f, 0.0248095095f, -0.184558094f, 0.525167167f };  // degree 4
static const float kDensityCoeffs2[] = { 3.1127598e-05f, -0.000273392303f, 0.00189212093f, -0.0106567862f, 0.0450629033f, -0.127011165f, 0.178990051f };  // degree 6

const float* const kAtmospherePressureCoeffs[] = { kPressureCoeffs0, kPressureCoeffs1, kPressureCoeffs2 };
const float* const kAtmosphereDensityCoeffs[]  = { kDensityCoeffs0, kDensityCoeffs1, kDensityCoeffs2 };
const int kAtmospherePressureCoeffCount[] = { 5, 5, 7 };
const int kAtmosphereDensityCoeffCount[]  = { 4, 5, 7 };

// worst case relative error of the float32 Horner evaluation against exact
// ISA 1976 values, measured over a 20001 point dense sample per
// segment. asserted against by unit test rather than taken on trust
const float kAtmosphereWorstRelErrorPressure = 1.06974289e-06f;
const float kAtmosphereWorstRelErrorDensity  = 6.99299335e-06f;
const float kAtmosphereWorstRelError = 6.99299335e-06f;

}  // namespace airbrake
