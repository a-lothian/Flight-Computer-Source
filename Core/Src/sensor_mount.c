/**
 * @file    sensor_mount.c
 * @brief   Raw sensor axes -> fusion body frame. See sensor_mount.h.
 */
#include "sensor_mount.h"

#if !SENSOR_MOUNT_VERIFIED
#pragma GCC warning "sensor_mount.h: sensor axis mapping is UNVERIFIED - do the bench check, then set SENSOR_MOUNT_VERIFIED to 1"
#endif

FusionInput sensor_mount_input(const float lsm_acc_mg[3], const float lsm_gyro_mdps[3],
                               const int8_t hg_counts[3],
                               float baro_alt_m, bool baro_new)
{
    FusionInput in;

    /* LSM6DSOX accelerometer */
    in.ax_mg = LSM_BODY_X_SIGN * lsm_acc_mg[LSM_BODY_X_AXIS];
    in.ay_mg = LSM_BODY_Y_SIGN * lsm_acc_mg[LSM_BODY_Y_AXIS];
    in.az_mg = LSM_BODY_Z_SIGN * lsm_acc_mg[LSM_BODY_Z_AXIS];

    /* Gyro: same chip, so always the same mapping as the accelerometer */
    in.gx_mdps = LSM_BODY_X_SIGN * lsm_gyro_mdps[LSM_BODY_X_AXIS];
    in.gy_mdps = LSM_BODY_Y_SIGN * lsm_gyro_mdps[LSM_BODY_Y_AXIS];
    in.gz_mdps = LSM_BODY_Z_SIGN * lsm_gyro_mdps[LSM_BODY_Z_AXIS];

    /* H3LIS100DL: counts -> mg, with its own mapping */
    in.hg_ax_mg = (float)(HG_BODY_X_SIGN * hg_counts[HG_BODY_X_AXIS]) * HG_MG_PER_COUNT;
    in.hg_ay_mg = (float)(HG_BODY_Y_SIGN * hg_counts[HG_BODY_Y_AXIS]) * HG_MG_PER_COUNT;
    in.hg_az_mg = (float)(HG_BODY_Z_SIGN * hg_counts[HG_BODY_Z_AXIS]) * HG_MG_PER_COUNT;

    in.baro_alt_m = baro_alt_m;
    in.baro_new   = baro_new;
    return in;
}
