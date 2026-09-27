/**
 * @file    altitude_kf.h
 * @brief   Kalman filter for vertical motion: altitude and vertical velocity.
 *
 * Combines two sensors with opposite strengths:
 *   Accelerometer (vertical accel): fast and smooth, but integrating it
 *     twice turns any small error into a growing altitude error.
 *   Barometer: never drifts, but is noisy, slow (32 Hz at best) and can
 *     give false spikes (ejection charges, transonic effects).
 *
 * The filter keeps an estimate of [altitude, velocity] plus a 2x2 matrix P
 * describing how uncertain that estimate is:
 *   Predict (every loop): move the state forward using physics and the
 *     vertical acceleration. Uncertainty grows.
 *   Update (each NEW baro reading): compare the prediction with the baro and
 *     correct both altitude and velocity. How much it trusts the baro vs its
 *     own prediction (the "Kalman gain") is worked out from the
 *     uncertainties. Uncertainty shrinks.
 *
 * Velocity is never measured directly, but it still gets corrected: the
 * filter learns (through P) how altitude errors relate to velocity errors.
 */
#ifndef ALTITUDE_KF_H
#define ALTITUDE_KF_H

#include <stdbool.h>

/**
 * @brief Kalman filter state.
 */
typedef struct {
    float h;            /**< altitude above the pad, m                        */
    float v;            /**< vertical velocity, m/s, +up                      */
    float P[2][2];      /**< covariance (uncertainty):
                             P[0][0] = variance of h (m^2)
                             P[1][1] = variance of v ((m/s)^2)
                             P[0][1] = P[1][0] = how h and v errors are linked */
    int   rejects;      /**< number of baro readings rejected in a row        */
    bool  descent;      /**< true = baro-led mode (after apogee). The predict
                             step ignores the vertical acceleration, because
                             the gyro-only attitude it depends on is
                             meaningless once the rocket tumbles under a
                             parachute. Set by sensor_fusion.c.             */
} AltitudeKF_t;

/**
 * @brief Reset: on the pad, altitude 0, velocity 0, small uncertainty.
 * @param kf  Filter to reset.
 */
void altitude_kf_init(AltitudeKF_t *kf);

/**
 * @brief Predict step: move the state forward in time.
 *
 * Constant-acceleration physics over one time step:
 *   h = h + v*dt + 0.5*a*dt^2
 *   v = v + a*dt
 * Then the uncertainty grows: P = F*P*F' + Q, where F = [1 dt; 0 1] and Q
 * models accelerometer noise (KF_ACCEL_STD).
 *
 * In descent mode (kf->descent) a_vert is ignored and the model becomes
 * "velocity roughly constant, changing by up to KF_DESCENT_ACCEL_STD": the
 * baro then drives both altitude and velocity.
 *
 * Call every loop, whether or not there is a new baro reading.
 *
 * @param kf      Filter state (updated).
 * @param a_vert  Vertical acceleration, m/s^2, +up, gravity removed.
 * @param dt      Time step, s.
 */
void altitude_kf_predict(AltitudeKF_t *kf, float a_vert, float dt);

/**
 * @brief Update step: correct the state with a baro altitude.
 *
 * 0. Readings that are not a number, or impossibly far from the pad
 *    (|alt| > KF_BARO_MAX_AGL_M, e.g. a failed I2C read), are ignored
 *    outright and do NOT count towards the forced acceptance below.
 * 1. Innovation y = baro - predicted altitude (how far off the prediction is).
 *    The baro's noise R grows with speed: the static port reads wrong by
 *    roughly Cp * (dynamic pressure), which is (KF_BARO_CP * v^2 / 2g) in
 *    metres. So near Mach 0.9 the filter leans on the accelerometer, and
 *    back on the baro as the rocket slows towards apogee.
 * 2. Outlier gate: if |y| is more than 5 standard deviations (and at least
 *    10 m), the reading is rejected - e.g. an ejection-charge pressure spike.
 *    After 16 rejections in a row (~0.5 s at 32 Hz) the next reading is
 *    accepted anyway, so the filter can't lock itself out if its own
 *    estimate is the thing that's wrong.
 * 3. Kalman gain K = P[:,0] / (P[0][0] + R): how much of the innovation to
 *    apply to altitude (K0) and velocity (K1).
 * 4. Correct: h += K0*y, v += K1*y. Shrink P.
 *
 * Only call when there is a genuinely NEW baro reading.
 *
 * @param kf            Filter state (updated).
 * @param baro_alt_agl  Baro altitude above the pad, m.
 * @return              true if the reading was used, false if rejected.
 */
bool altitude_kf_update(AltitudeKF_t *kf, float baro_alt_agl);

#endif /* ALTITUDE_KF_H */
