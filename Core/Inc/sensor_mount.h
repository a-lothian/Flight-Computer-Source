/**
 * @file    sensor_mount.h
 * @brief   How each sensor is mounted: raw chip axes -> fusion body frame.
 *
 * The fusion works in a body frame with z out of the nose and x, y
 * sideways, right-handed. Each chip has its own axes, printed on the chip,
 * and they point wherever the PCB layout put them. This file is the ONE
 * place that says which raw axis feeds which body axis, and with what sign.
 *
 * *** UNVERIFIED - CHECK ON HARDWARE BEFORE FLIGHT ***
 * The mapping below is a guess: it assumes both chips have their X axis
 * pointing out of the nose, which has never been checked on the real board.
 * Once checked, set SENSOR_MOUNT_VERIFIED to 1 to silence the build warning.
 * The flight firmware's FSM reads this body frame too, so this file is the
 * only axis mapping in the firmware.
 *
 * BENCH CHECK (~5 min, see also README section 7)
 *   1. Stand the board NOSE UP and still. Print the RAW driver values
 *      (imu.acceleration[0..2] in mg, accelData.accel_x/y/z in counts).
 *   2. LSM6DSOX: the axis reading ~+1000 mg is the nose axis. Set
 *      LSM_BODY_Z_AXIS to it with sign +1. If the nose axis reads ~-1000 mg
 *      instead, set sign -1 AND flip the sign of one other axis.
 *   3. H3LIS100DL: same, using the axis reading ~+1 count (780 mg/count is
 *      coarse: flip the board nose-down and look for the axis that changes
 *      by ~2 counts). Set HG_BODY_Z_AXIS / sign the same way.
 *   4. Choose x and y as the other two axes so the frame is right-handed.
 *      The build checks this and fails with "sensor_mount_..._not_right_handed"
 *      if not: try swapping the x and y rows, or flipping one sign.
 *      Which sideways axis is x and which is y does not matter to the
 *      fusion - only that the frame is right-handed and z is the nose.
 *   5. Confirm with the fusion running: nose up -> tilt_deg ~0, accel_vert
 *      ~0; lay it on its side -> tilt_deg ~90; spin it about the nose,
 *      counter-clockwise seen from above -> roll_deg increases.
 *
 * The gyro is inside the LSM6DSOX, so it always uses the LSM6DSOX mapping.
 */
#ifndef SENSOR_MOUNT_H
#define SENSOR_MOUNT_H

#include <stdbool.h>
#include <stdint.h>
#include "sensor_fusion.h"

/** Set to 1 once the mapping below has been checked on the real board. */
#ifndef SENSOR_MOUNT_VERIFIED
#define SENSOR_MOUNT_VERIFIED 0
#endif

/* ---- LSM6DSOX (accelerometer and gyro) ----------------------------
 * For each body axis: which raw axis (0 = X, 1 = Y, 2 = Z) and its sign.
 * Guess: raw X = nose, so body (x, y, z) = raw (Y, Z, X). */
#define LSM_BODY_X_AXIS   1
#define LSM_BODY_X_SIGN   (-1)
#define LSM_BODY_Y_AXIS   2
#define LSM_BODY_Y_SIGN   (+1)
#define LSM_BODY_Z_AXIS   0
#define LSM_BODY_Z_SIGN   (-1)

/* ---- H3LIS100DL (high-g accelerometer) -----------------------------
 * Guess: mounted the same way as the LSM6DSOX. It is a separate chip and
 * may well be rotated differently on the PCB. */
#define HG_BODY_X_AXIS    1
#define HG_BODY_X_SIGN    (+1)
#define HG_BODY_Y_AXIS    2
#define HG_BODY_Y_SIGN    (+1)
#define HG_BODY_Z_AXIS    0
#define HG_BODY_Z_SIGN    (+1)

/** H3LIS100DL sensitivity, mg per count (datasheet, +/-100 g, 8-bit). */
#define HG_MG_PER_COUNT   780.0f

/* ---- Compile-time check: each mapping must be a proper rotation ------
 * A mapping is a rotation (keeps the frame right-handed) only if it uses
 * each raw axis once, and (axis order is a cyclic shift) XOR (an even
 * number of sign flips) - i.e. cyclic with 0 or 2 flips, or a swap with
 * 1 or 3 flips. Anything else mirrors the frame: magnitudes look fine but
 * every angle comes out wrong. A failed check shows up as a "negative
 * array size" error naming the typedef below. */
#define SM_DISTINCT(a, b, c)  ((a) != (b) && (b) != (c) && (a) != (c))
#define SM_CYCLIC(a, b, c)    ((((a) + 1) % 3 == (b)) && (((b) + 1) % 3 == (c)))
#define SM_RIGHT_HANDED(a, sa, b, sb, c, sc) \
    (SM_DISTINCT(a, b, c) && \
     ((SM_CYCLIC(a, b, c) ? 1 : -1) * (sa) * (sb) * (sc) == 1))

typedef char sensor_mount_lsm_not_right_handed[
    SM_RIGHT_HANDED(LSM_BODY_X_AXIS, LSM_BODY_X_SIGN, LSM_BODY_Y_AXIS, LSM_BODY_Y_SIGN,
                    LSM_BODY_Z_AXIS, LSM_BODY_Z_SIGN) ? 1 : -1];
typedef char sensor_mount_hg_not_right_handed[
    SM_RIGHT_HANDED(HG_BODY_X_AXIS, HG_BODY_X_SIGN, HG_BODY_Y_AXIS, HG_BODY_Y_SIGN,
                    HG_BODY_Z_AXIS, HG_BODY_Z_SIGN) ? 1 : -1];

/**
 * @brief Build a FusionInput from raw driver values, applying the mounting.
 *
 * Typical use in main.c, after ReadAllIMU(&imu) and h3lis_read(...):
 * @code
 *   int8_t hg[3] = { accelData.accel_x, accelData.accel_y, accelData.accel_z };
 *   FusionInput in = sensor_mount_input(imu.acceleration, imu.gyro, hg,
 *                                       altitude_m, prs_raw != last_prs_raw);
 * @endcode
 *
 * @param lsm_acc_mg     LSM6DSOX accel, RAW chip axes, mg (imu.acceleration).
 * @param lsm_gyro_mdps  LSM6DSOX gyro, RAW chip axes, mdps (imu.gyro).
 * @param hg_counts      H3LIS100DL raw counts, RAW chip axes (x, y, z).
 * @param baro_alt_m     DPS368 altitude, m (any datum).
 * @param baro_new       true only when baro_alt_m is a new measurement.
 * @return               Input ready for fusion_update().
 */
FusionInput sensor_mount_input(const float lsm_acc_mg[3], const float lsm_gyro_mdps[3],
                               const int8_t hg_counts[3],
                               float baro_alt_m, bool baro_new);

#endif /* SENSOR_MOUNT_H */
