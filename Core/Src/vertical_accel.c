/**
 * @file    vertical_accel.c
 * @brief   Earth-frame vertical acceleration. See vertical_accel.h.
 */
#include "vertical_accel.h"

/** 1 g in m/s^2. Accelerometer input must be in m/s^2. */
#define G_VA  9.80665f

/**
 * Project the body-frame accelerometer onto earth "up" and remove gravity.
 *
 * The three coefficients are "how much of body x / y / z points up" for
 * the current attitude (bottom row of the body-to-earth rotation, see
 * attitude_up()). Upright this reduces to a_up = az.
 *
 * Any small constant error left over (e.g. accelerometer scale error) is
 * measured during the pad calibration and subtracted in sensor_fusion.c.
 */
float vertical_accel(const Quat_t *q, float ax, float ay, float az)
{
    float up[3];
    attitude_up(q, up);

    /* Specific force along earth vertical */
    float a_up = up[0] * ax + up[1] * ay + up[2] * az;

    /* The accelerometer reads +1 g upward when still (the ground pushing
     * back), so subtract gravity to get the real acceleration. */
    return a_up - G_VA;
}
