/**
 * @file    attitude.h
 * @brief   Attitude (orientation) estimation: accelerometer tilt, gyro
 *          integration and the complementary filter that combines them.
 *
 * BODY FRAME
 *   z = out of the rocket's nose, x and y = sideways. Fixed to the rocket,
 *   so it moves with it. Must be right-handed.
 *
 * REPRESENTATION
 *   The orientation is kept as a unit quaternion (body -> earth). Rockets
 *   spin fast about the nose (last year's flight reached ~940 deg/s, about
 *   19 deg per 20 ms loop). Euler angles integrated through their rate
 *   equations break down at that spin rate: on the real flight log the
 *   tilt ran away to ~90 deg during boost. A quaternion rotated by the exact
 *   rotation of each gyro sample has no such problem and no singularity.
 *
 *   For reporting, the quaternion is converted to the standard aerospace
 *   "ZYX" Euler angles (attitude_euler):
 *     phi   - rotation about body x  -> a lean
 *     theta - rotation about body y  -> a lean (the perpendicular direction)
 *     psi   - rotation about body z  -> spin about the long axis ("roll" in
 *             rocket terms)
 *   With z along the nose, a vertical rocket has phi = theta = 0.
 *
 * THE TWO SENSORS AND WHY BOTH ARE NEEDED
 *   Accelerometer: when still, it measures gravity, which shows which way
 *     is down -> absolute lean angles that never drift. But it is noisy,
 *     cannot see spin at all, and is completely fooled by any other
 *     acceleration (thrust, drag, parachute jolts).
 *   Gyroscope: measures rotation rate. Integrating it gives smooth angles
 *     that ignore acceleration, including spin. But any small bias adds up
 *     over time, so the angles slowly drift.
 *   The complementary filter uses the gyro for moment-to-moment tracking
 *   and the accelerometer to slowly cancel the drift - but only when the
 *   accelerometer can be trusted (on the ground, not in flight).
 */
#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdbool.h>

/**
 * @brief Orientation as a unit quaternion, rotating body -> earth
 *        (earth frame: z = up).
 */
typedef struct {
    float w, x, y, z;
} Quat_t;

/**
 * @brief Orientation as three Euler angles, radians. Reporting only.
 */
typedef struct {
    float phi;      /**< rotation about body x, rad (a lean)                */
    float theta;    /**< rotation about body y, rad (a lean)                */
    float psi;      /**< rotation about body z, rad (spin about the nose)   */
} Euler_t;

/**
 * @brief Lean from the direction of gravity.
 *
 * When the rocket is still, the accelerometer reads 1 g pointing away from
 * the ground. How that 1 g is split between the x, y and z axes shows how
 * the rocket is leaning:
 *   phi   = atan2(ay, az)
 *   theta = atan2(-ax, sqrt(ay^2 + az^2))
 *   psi   = 0 (spinning about the nose doesn't change where gravity points)
 *
 * Only valid when gravity is the ONLY thing the accelerometer feels.
 *
 * @param ax, ay, az  Accelerometer reading, body frame. Any unit (mg or
 *                    m/s^2) - only the ratios between axes matter.
 * @return            Orientation with that lean and zero spin.
 */
Quat_t attitude_from_accel(float ax, float ay, float az);

/**
 * @brief ZYX Euler angles of an orientation (for reporting).
 * @param q  Orientation.
 * @return   phi and psi in -pi..pi, theta in -pi/2..pi/2.
 */
Euler_t attitude_euler(const Quat_t *q);

/**
 * @brief Total angle between the nose and vertical.
 *
 * 0 = pointing straight up, 90 deg = horizontal, 180 deg = straight down.
 * Spin does not affect it.
 *
 * @param q  Current attitude.
 * @return   Tilt from vertical, rad, in the range 0..pi.
 */
float attitude_tilt(const Quat_t *q);

/**
 * @brief How much each body axis currently points up.
 *
 * The bottom row of the body -> earth rotation matrix: up[i] is the
 * vertical component of body axis i. up[2] is cos(tilt).
 *
 * @param q   Current attitude.
 * @param up  Output, 3 values.
 */
void attitude_up(const Quat_t *q, float up[3]);

/**
 * @brief Advance the orientation by one gyro sample (no correction).
 *
 * The body rotated by the vector (gx, gy, gz) * dt during the step. That
 * rotation is applied exactly (not as a small-angle approximation), so the
 * result stays correct however fast the rocket spins, provided the rate is
 * roughly constant over one step.
 *
 * @param q           Current attitude.
 * @param gx, gy, gz  Body rotation rates, rad/s, bias already removed.
 * @param dt          Time step, s.
 * @return            New attitude (re-normalised).
 */
Quat_t attitude_integrate_gyro(const Quat_t *q,
                               float gx, float gy, float gz, float dt);

/**
 * @brief Complementary filter: one full attitude update.
 *
 * 1. Integrate the gyro (attitude_integrate_gyro).
 * 2. If use_accel is true AND the total acceleration is within 10% of 1 g,
 *    turn the orientation a small step so that the measured gravity
 *    direction lines up with true vertical. The correction is about a
 *    horizontal axis only, so it fixes lean and never changes spin.
 *    The step size comes from a time constant (ATT_TAU_S, 1 s) so the
 *    filter behaves the same at any loop rate.
 *
 * WHY use_accel EXISTS: the 1 g check alone is not enough in flight. During
 * coast, drag deceleration passes through exactly 1 g while pointing DOWN
 * the nose. Trusting it would make the filter think the rocket is upside
 * down. sensor_fusion.c sets use_accel = false from launch until landing.
 *
 * @param q           Current attitude.
 * @param ax, ay, az  Accelerometer, m/s^2, body frame (units matter here:
 *                    the 1 g check compares against 9.80665).
 * @param gx, gy, gz  Gyro, rad/s, body frame, bias removed.
 * @param dt          Time step, s.
 * @param use_accel   true = accelerometer correction allowed (on the ground).
 * @return            New attitude.
 */
Quat_t attitude_update(const Quat_t *q,
                       float ax, float ay, float az,
                       float gx, float gy, float gz, float dt,
                       bool use_accel);

#endif /* ATTITUDE_H */
