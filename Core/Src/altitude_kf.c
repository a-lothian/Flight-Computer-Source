/**
 * @file    altitude_kf.c
 * @brief   Altitude / vertical velocity Kalman filter. See altitude_kf.h.
 *
 * The maths is the standard linear Kalman filter for a
 * position-velocity state with acceleration as a control input:
 *   x = [h; v]      F = [1 dt; 0 1]      B = [dt^2/2; dt]      H = [1 0]
 * The 2x2 matrix products are written out by hand - no matrix library.
 */
#include "altitude_kf.h"
#include <math.h>

/* ---- Tuning -------------------------------------------------------- */

/**
 * Process noise: how much to distrust the vertical acceleration, as a
 * standard deviation in m/s^2.
 *   Bigger  -> uncertainty grows faster between baro readings, so the
 *              filter follows the baro more closely (more baro noise gets
 *              through, but less lag if the accel is wrong).
 *   Smaller -> smoother output, leans on the accelerometer more.
 */
#ifndef KF_ACCEL_STD
#define KF_ACCEL_STD   1.0f
#endif

/**
 * Measurement noise of the baro altitude, standard deviation in m.
 * Measure it: log the baro with the board sitting still for a minute and
 * take the standard deviation of the altitude readings.
 */
#ifndef KF_BARO_STD
#define KF_BARO_STD    0.5f
#endif

/**
 * Static-port error coefficient. Near the transonic region the pressure
 * at the port differs from true ambient by about Cp * dynamic pressure,
 * i.e. an altitude error of Cp * v^2 / (2 g). The filter treats that as
 * extra baro noise with standard deviation KF_BARO_CP * v^2 / (2 g), using
 * its own vertical velocity estimate for v.
 * 0.03 is deliberately conservative: last year's flight (different
 * airframe) showed ~100 m of baro error at ~300 m/s, i.e. Cp ~ 0.02.
 * If flight data or CFD for THIS rocket's port gives a better number,
 * use it here.
 */
#ifndef KF_BARO_CP
#define KF_BARO_CP     0.03f
#endif

/**
 * Baro readings further than this from the pad (m) cannot be real for this
 * rocket (apogee ~3.3 km) and are ignored without counting as a rejection,
 * so a failed sensor that returns garbage can never be force-accepted.
 */
#ifndef KF_BARO_MAX_AGL_M
#define KF_BARO_MAX_AGL_M  20000.0f
#endif

/**
 * Descent (baro-led) mode process noise, m/s^2: how quickly the filter
 * lets velocity change when it is not using the accelerometer. Must cover
 * parachute openings (drogue -> main slows ~30 m/s to ~6 m/s in a second or
 * two) without making the velocity too noisy. 3.0 is the compromise between
 * last year's real descent (quietest at 2) and OpenRocket's instantaneous
 * chute openings (best at 4); both are within ~1 m of their best at 3.
 */
#ifndef KF_DESCENT_ACCEL_STD
#define KF_DESCENT_ACCEL_STD  3.0f
#endif

/**
 * Baro outlier rejection.
 * A reading is rejected if it is further than KF_GATE_SIGMA standard
 * deviations of the expected innovation from the prediction, with a
 * minimum gate of KF_GATE_MIN_M metres (so normal noise never trips it
 * when the filter is very confident). At most KF_MAX_REJECTS readings are
 * rejected in a row; after that the next one is accepted so the filter
 * cannot lock itself out if its own estimate is the thing that's wrong.
 */
#ifndef KF_GATE_SIGMA
#define KF_GATE_SIGMA   5.0f
#endif
#ifndef KF_GATE_MIN_M
#define KF_GATE_MIN_M   10.0f
#endif
#ifndef KF_MAX_REJECTS
#define KF_MAX_REJECTS  16      /* ~0.5 s at 32 Hz */
#endif

/** Starting uncertainty (standard deviations). Small, because we know the
 *  rocket is sitting on the pad at 0 m, not moving. */
#ifndef KF_INIT_H_STD
#define KF_INIT_H_STD  0.5f     /* m   */
#endif
#ifndef KF_INIT_V_STD
#define KF_INIT_V_STD  0.1f     /* m/s */
#endif

/** 1 g in m/s^2, for the v^2 / 2g term. */
#define KF_G  9.80665f

/* -------------------------------------------------------------------- */
/**
 * Reset to "on the pad": h = 0, v = 0, small diagonal uncertainty, and
 * no uncertainty linking h and v yet (off-diagonal terms 0).
 */
void altitude_kf_init(AltitudeKF_t *kf)
{
    kf->h = 0.0f;
    kf->v = 0.0f;
    kf->P[0][0] = KF_INIT_H_STD * KF_INIT_H_STD;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = KF_INIT_V_STD * KF_INIT_V_STD;
    kf->rejects = 0;
    kf->descent = false;
}

/* -------------------------------------------------------------------- */
/**
 * Predict step.
 *
 * State: plain constant-acceleration kinematics over dt.
 *
 * Covariance: P = F P F' + Q, expanded for F = [1 dt; 0 1]:
 *   P00' = P00 + dt*(P01 + P10) + dt^2*P11 + Q00
 *   P01' = P01 + dt*P11 + Q01
 *   P10' = P10 + dt*P11 + Q01
 *   P11' = P11 + Q11
 * F moves velocity uncertainty into altitude uncertainty (an unknown
 * velocity error becomes an altitude error over time).
 *
 * Q is the uncertainty added by one step of accelerometer noise q:
 * an acceleration error a over dt causes a velocity error a*dt and an
 * altitude error a*dt^2/2, giving
 *   Q = q * [dt^4/4  dt^3/2 ; dt^3/2  dt^2]
 */
void altitude_kf_predict(AltitudeKF_t *kf, float a_vert, float dt)
{
    float dt2 = dt * dt;
    float std = KF_ACCEL_STD;

    /* Baro-led descent: the accelerometer projection can't be trusted, so
     * coast on velocity and let the baro do the correcting. */
    if (kf->descent) {
        a_vert = 0.0f;
        std    = KF_DESCENT_ACCEL_STD;
    }

    /* --- state --- */
    kf->h += kf->v * dt + 0.5f * a_vert * dt2;
    kf->v += a_vert * dt;

    /* --- covariance --- */
    float P00 = kf->P[0][0], P01 = kf->P[0][1];
    float P10 = kf->P[1][0], P11 = kf->P[1][1];

    float q   = std * std;
    float Q00 = 0.25f * dt2 * dt2 * q;
    float Q01 = 0.5f  * dt2 * dt  * q;
    float Q11 = dt2 * q;

    kf->P[0][0] = P00 + dt * (P01 + P10) + dt2 * P11 + Q00;
    kf->P[0][1] = P01 + dt * P11 + Q01;
    kf->P[1][0] = P10 + dt * P11 + Q01;
    kf->P[1][1] = P11 + Q11;
}

/* -------------------------------------------------------------------- */
/**
 * Update step (baro only measures altitude, so H = [1 0]).
 *
 *   y = z - h                       innovation (baro minus prediction)
 *   S = P00 + R                     expected spread of y
 *   K = [P00; P10] / S              Kalman gain
 *   x = x + K*y                     correct altitude AND velocity
 *   P = (I - K H) P                 uncertainty shrinks
 *
 * The gain automatically balances the two sources: if the prediction is
 * very uncertain (P00 >> R) then K0 -> 1 and the baro is trusted; if the
 * prediction is confident (P00 << R) then K0 -> 0 and the baro barely
 * moves it.
 */
bool altitude_kf_update(AltitudeKF_t *kf, float baro_alt_agl)
{
    /* Not a real reading: NaN/inf (e.g. powf of a negative pressure) or
     * impossibly far away. Ignored without touching the reject counter.
     * Written as !(x <= max) so NaN, for which every comparison is false,
     * is rejected too. */
    if (!(fabsf(baro_alt_agl) <= KF_BARO_MAX_AGL_M)) {
        return false;
    }

    /* Baro noise: sensor noise plus the speed-dependent static-port error */
    float port = KF_BARO_CP * kf->v * kf->v / (2.0f * KF_G);
    float R = KF_BARO_STD * KF_BARO_STD + port * port;

    /* How far off is the prediction? */
    float y = baro_alt_agl - kf->h;

    /* How far off would we expect it to be? (variance of y) */
    float S = kf->P[0][0] + R;

    /* Outlier gate: reject readings far outside what's expected, e.g.
     * an ejection-charge pressure spike (tens of metres for ~0.1 s). */
    float gate = KF_GATE_SIGMA * sqrtf(S);
    if (gate < KF_GATE_MIN_M) gate = KF_GATE_MIN_M;
    if (fabsf(y) > gate && kf->rejects < KF_MAX_REJECTS) {
        kf->rejects++;
        return false;
    }
    kf->rejects = 0;

    /* Kalman gain */
    float K0 = kf->P[0][0] / S;      /* share of y applied to altitude */
    float K1 = kf->P[1][0] / S;      /* share of y applied to velocity */

    /* Correct the state */
    kf->h += K0 * y;
    kf->v += K1 * y;

    /* Shrink the uncertainty: P = (I - K H) P, written out.
     * Uses the OLD P00 and P01 values, saved before overwriting. */
    float P00 = kf->P[0][0], P01 = kf->P[0][1];
    kf->P[0][0] = (1.0f - K0) * P00;
    kf->P[0][1] = (1.0f - K0) * P01;
    kf->P[1][0] -= K1 * P00;
    kf->P[1][1] -= K1 * P01;
    return true;
}
