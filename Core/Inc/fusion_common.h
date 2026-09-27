/**
 * @file    fusion_common.h
 * @brief   Shared constants and unit conversions for the sensor fusion.
 *
 * The drivers report acceleration in milli-g (mg) and rotation rate in
 * milli-degrees per second (mdps). All the fusion maths works in SI units
 * (m/s^2 and rad/s) because sinf/cosf/atan2f use radians and the Kalman
 * filter integrates physical acceleration. These macros do the conversion
 * once, at the top of fusion_update().
 *
 * Only sensor_fusion.c includes this header, so main.c never sees these
 * names and cannot clash with them.
 */
#ifndef FUSION_COMMON_H
#define FUSION_COMMON_H

/** Standard gravity, m/s^2. Also the value of 1 g for unit conversion. */
#define G_MPS2         9.80665f

/** Multiply degrees by this to get radians. */
#define DEG2RAD        0.01745329252f

/** Multiply radians by this to get degrees. */
#define RAD2DEG        57.2957795131f

/** Multiply milli-g by this to get m/s^2 (1 mg = 0.00980665 m/s^2). */
#define MG_TO_MPS2     (G_MPS2 / 1000.0f)

/** Multiply milli-degrees/s by this to get rad/s. */
#define MDPS_TO_RADPS  (DEG2RAD / 1000.0f)

#endif /* FUSION_COMMON_H */
