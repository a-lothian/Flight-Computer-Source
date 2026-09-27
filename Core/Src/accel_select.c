/**
 * @file    accel_select.c
 * @brief   Per-axis LSM6DSOX / H3LIS100DL selection. See accel_select.h.
 */
#include "accel_select.h"
#include <math.h>

/* ---- Settings ------------------------------------------------------ */

/**
 * Full-scale range the LSM6DSOX accelerometer is configured for, in mg.
 * MUST match the range set in the IMU driver (IMU.c). If the driver uses
 * +/-8 g, set 8000.0f here, etc. +/-16 g is recommended for a rocket.
 */
#ifndef LSM_FULL_SCALE_MG
#define LSM_FULL_SCALE_MG   16000.0f
#endif

/** Switch an axis to the H3LIS when the LSM6DSOX exceeds 90% of full scale
 *  (switch before it actually clips, since clipped readings are wrong). */
#define SWITCH_UP_MG        (0.9f * LSM_FULL_SCALE_MG)

/** Switch back to the LSM6DSOX when the H3LIS reads below 80% of full scale.
 *  The gap to SWITCH_UP_MG is the hysteresis band. */
#define SWITCH_DOWN_MG      (0.8f * LSM_FULL_SCALE_MG)

/* -------------------------------------------------------------------- */
/**
 * Reset every axis to the LSM6DSOX with no H3LIS offset.
 * Called from fusion_init().
 */
void accel_select_init(AccelSelect_t *s)
{
    for (int i = 0; i < 3; i++) {
        s->use_hg[i]    = false;
        s->hg_offset[i] = 0.0f;
    }
}

/* -------------------------------------------------------------------- */
/**
 * Store the H3LIS-to-LSM6DSOX offset for each axis.
 *
 * Both sensors see the same gravity on the pad, so the difference between
 * their averages is the H3LIS's error relative to the more accurate
 * LSM6DSOX. Adding it to every later H3LIS reading means the two sensors
 * agree at the moment of switching, so there's no step in acceleration
 * (which would show up as a kink in velocity).
 *
 * Note: at 780 mg per count the H3LIS offset can only be estimated roughly;
 * averaging 100 samples helps if its noise dithers between counts.
 */
void accel_select_calibrate(AccelSelect_t *s,
                            const float lsm_mean[3], const float hg_mean[3])
{
    for (int i = 0; i < 3; i++) {
        s->hg_offset[i] = lsm_mean[i] - hg_mean[i];
    }
}

/* -------------------------------------------------------------------- */
/**
 * Pick the reading for each axis, with hysteresis.
 *
 * The decision to switch UP looks at the LSM6DSOX (is it about to clip?).
 * The decision to switch DOWN looks at the H3LIS, because while an axis is
 * clipping the LSM6DSOX reading is stuck at its limit and says nothing
 * about the real acceleration.
 */
void accel_select(AccelSelect_t *s,
                  const float lsm[3], const float hg[3], float out[3])
{
    for (int i = 0; i < 3; i++) {
        /* H3LIS reading lined up with the LSM6DSOX */
        float hg_corrected = hg[i] + s->hg_offset[i];

        if (!s->use_hg[i] && fabsf(lsm[i]) > SWITCH_UP_MG) {
            s->use_hg[i] = true;             /* LSM6DSOX about to clip      */
        } else if (s->use_hg[i] && fabsf(hg_corrected) < SWITCH_DOWN_MG) {
            s->use_hg[i] = false;            /* safely back in LSM range    */
        }

        out[i] = s->use_hg[i] ? hg_corrected : lsm[i];
    }
}

/* -------------------------------------------------------------------- */
/**
 * true if any axis is on the H3LIS. Reported as FusionOutput.high_g.
 */
bool accel_select_high_g(const AccelSelect_t *s)
{
    return s->use_hg[0] || s->use_hg[1] || s->use_hg[2];
}
