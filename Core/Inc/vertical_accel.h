/**
 * @file    vertical_accel.h
 * @brief   Earth-frame vertical acceleration from the body-frame accelerometer.
 *
 * The accelerometer measures along the rocket's own axes. The altitude
 * filter needs acceleration along the EARTH's vertical axis. This module
 * uses the attitude estimate to project the body reading onto "up", then
 * removes gravity.
 */
#ifndef VERTICAL_ACCEL_H
#define VERTICAL_ACCEL_H

#include "attitude.h"

/**
 * @brief Vertical (earth-frame) acceleration with gravity removed.
 *
 * Each body axis contributes according to how much it currently points up
 * (the bottom row of the body-to-earth rotation matrix, attitude_up()):
 *   a_up = up[0]*ax + up[1]*ay + up[2]*az
 * In Euler terms that row is (-sin(theta), sin(phi)cos(theta),
 * cos(phi)cos(theta)); spin drops out, because spinning about the nose
 * doesn't change what points up.
 *
 * An accelerometer measures "specific force": sitting still it reads +1 g
 * upward, in free fall it reads 0. Subtracting 1 g converts that into real
 * acceleration.
 *
 * @param q           Attitude from the complementary filter.
 * @param ax, ay, az  Accelerometer, m/s^2, body frame.
 * @return            Vertical acceleration, m/s^2, +up.
 *                    ~0 still, ~+49 during a 6 g burn, ~-9.8 in free fall.
 */
float vertical_accel(const Quat_t *q, float ax, float ay, float az);

#endif /* VERTICAL_ACCEL_H */
