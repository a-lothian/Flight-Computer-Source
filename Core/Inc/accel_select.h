/**
 * @file    accel_select.h
 * @brief   Per-axis choice between the LSM6DSOX and H3LIS100DL accelerometers.
 *
 * The board has two accelerometers with very different strengths:
 *
 *   LSM6DSOX   : 16-bit, ~0.49 mg per count at +/-16 g.  Very fine resolution,
 *                but it clips (saturates) at its full-scale range.
 *   H3LIS100DL : 8-bit, ~780 mg per count, +/-100 g.  Coarse, but it never
 *                clips during a rocket flight.
 *
 * The LSM6DSOX is always preferred. When an axis gets close to its limit,
 * that axis switches to the H3LIS, and switches back once safely in range.
 * Switching is done per axis because usually only the nose axis exceeds the
 * range during boost; the sideways axes can keep the better sensor.
 *
 * All values are in mg and must already be remapped into the body frame
 * (z = nose). The H3LIS will be mounted differently on the PCB, so it needs
 * its own remap in main.c.
 */
#ifndef ACCEL_SELECT_H
#define ACCEL_SELECT_H

#include <stdbool.h>

/**
 * @brief State of the accelerometer selector.
 *
 * Kept between calls so the switching has memory (hysteresis).
 */
typedef struct {
    bool  use_hg[3];     /**< per axis: true = currently using the H3LIS       */
    float hg_offset[3];  /**< mg added to each H3LIS axis so that, on the pad,
                              it reads the same as the LSM6DSOX. Prevents a
                              step in acceleration at the moment of switching. */
} AccelSelect_t;

/**
 * @brief Reset the selector: every axis on the LSM6DSOX, no offsets.
 * @param s  Selector state to reset.
 */
void accel_select_init(AccelSelect_t *s);

/**
 * @brief Line the H3LIS up with the LSM6DSOX using pad averages.
 *
 * On the pad both sensors feel exactly the same thing (1 g of gravity), so
 * any difference between their averages is the H3LIS's offset relative to
 * the more accurate LSM6DSOX. That difference is stored and added to every
 * later H3LIS reading.
 *
 * @param s         Selector state to update.
 * @param lsm_mean  Average LSM6DSOX reading per axis over the pad calibration, mg.
 * @param hg_mean   Average H3LIS reading per axis over the same period, mg.
 */
void accel_select_calibrate(AccelSelect_t *s,
                            const float lsm_mean[3], const float hg_mean[3]);

/**
 * @brief Choose the best reading for each axis.
 *
 * Per axis:
 *   - On the LSM6DSOX and |reading| > 90% of its full scale -> switch to H3LIS.
 *   - On the H3LIS and |corrected reading| < 80% of full scale -> switch back.
 *   - The 10% gap (hysteresis) stops rapid flicking between sensors when the
 *     acceleration hovers near the limit.
 *
 * @param s    Selector state (updated: remembers which sensor each axis uses).
 * @param lsm  LSM6DSOX reading, mg, body frame.
 * @param hg   H3LIS reading, mg, body frame (offset NOT yet applied).
 * @param out  Chosen reading per axis, mg. H3LIS values include the offset.
 */
void accel_select(AccelSelect_t *s,
                  const float lsm[3], const float hg[3], float out[3]);

/**
 * @brief Is any axis currently using the H3LIS?
 * @param s  Selector state.
 * @return   true if at least one axis is on the H3LIS. Worth logging, to see
 *           when the handover happened in flight.
 */
bool accel_select_high_g(const AccelSelect_t *s);

#endif /* ACCEL_SELECT_H */
