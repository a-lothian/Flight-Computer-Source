/**
 * @file    sensor_fusion.h
 * @brief   Public interface of the rocket sensor fusion.
 *
 * This is the ONLY header main.c needs. Everything else (attitude filter,
 * Kalman filter, accelerometer selection) is hidden behind these two calls.
 *
 * What it estimates, every loop:
 *   - Attitude: how the rocket is leaning and spinning, relative to the
 *     launch pad (where it is assumed to stand vertical).
 *   - Vertical acceleration in the earth frame, gravity removed.
 *   - Vertical velocity and altitude above the pad.
 *
 * How it gets there (see README for the full description):
 *   0. Pick the LSM6DSOX or H3LIS100DL reading for each accel axis.
 *   1. Convert units (mg -> m/s^2, mdps -> rad/s).
 *   2. First ~2 s: calibrate on the pad (gyro bias, start attitude,
 *      pad altitude, accelerometer offsets).
 *   3. Complementary filter -> attitude (gyro-only while in flight).
 *   4. Rotate acceleration into the earth frame -> vertical acceleration.
 *   5. Kalman filter: vertical acceleration + baro -> velocity, altitude.
 *
 * Typical use in main.c:
 * @code
 *   fusion_init();                            // once, rocket still on pad
 *   while (1) {
 *       FusionInput in = { ... from drivers ... };
 *       FusionOutput out = fusion_update(&in, dt);
 *       if (out.ready) { ... use / log the outputs ... }
 *   }
 * @endcode
 *
 * AXIS CONVENTION (important):
 *   The caller must remap the sensor axes so that body z points out of the
 *   rocket's nose. The remap must keep the frame right-handed: use a cyclic
 *   shift (x,y,z) -> (y,z,x), never a plain swap of two axes, which would
 *   mirror the frame and make every angle wrong. Both accelerometers and the
 *   gyro must be remapped into the SAME body frame.
 */
#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <stdbool.h>

/**
 * @brief Raw sensor values for one loop, exactly as the drivers give them.
 *
 * No unit conversion is needed by the caller - only the axis remap.
 */
typedef struct {
    float ax_mg, ay_mg, az_mg;          /**< LSM6DSOX accel, mg, body frame     */
    float hg_ax_mg, hg_ay_mg, hg_az_mg; /**< H3LIS100DL accel, mg, body frame
                                             (driver counts x 780 mg/count)     */
    float gx_mdps, gy_mdps, gz_mdps;    /**< LSM6DSOX gyro, mdps, body frame    */
    float baro_alt_m;                   /**< DPS368 altitude, m (any datum -
                                             the pad altitude is subtracted)    */
    bool  baro_new;                     /**< true only when baro_alt_m is a NEW
                                             measurement since the last call.
                                             Stale repeats must be false so the
                                             Kalman filter doesn't count them
                                             as fresh information.              */
} FusionInput;

/**
 * @brief Everything the fusion estimates. All zero until ready is true.
 *
 * Angle naming is rocket-style: "roll" is spin about the long axis, "pitch"
 * and "yaw" are the two sideways leans. tilt_deg combines both leans into
 * one number: the angle between the nose and straight up.
 */
typedef struct {
    float accel_vert;    /**< m/s^2, earth-vertical, gravity removed, +up.
                              0 when still, ~-9.8 in free fall.               */
    float velocity;      /**< m/s, vertical, +up. Crosses 0 at apogee.        */
    float altitude;      /**< m above the launch pad.                         */
    float roll_deg;      /**< deg, spin about the long axis. Gyro only, so it
                              drifts slowly (no magnetometer to correct it).  */
    float pitch_deg;     /**< deg, lean about body x, relative to vertical.   */
    float yaw_deg;       /**< deg, lean about body y, relative to vertical.   */
    float tilt_deg;      /**< deg, total angle of the nose from vertical.     */
    bool  ready;         /**< false during the ~2 s pad calibration.          */
    bool  high_g;        /**< true while any accel axis is using the H3LIS.   */
    bool  in_flight;     /**< true from launch detection until the rocket has
                              been still, with |velocity| < 2 m/s, for 3 s.
                              Attitude is gyro-only while this is true.       */
    bool  descent;       /**< true from the filter's own apogee (velocity
                              turning negative in flight) until landing.
                              Altitude and velocity are then baro-led; tilt
                              and accel_vert are no longer meaningful.        */
    bool  baro_ok;       /**< false once no baro reading has been accepted for
                              1 s (sensor failed, or every reading rejected).
                              In descent, altitude then only extrapolates, so
                              the FSM should fall back to its own logic.      */
} FusionOutput;

/**
 * @brief Reset all filter state and restart the pad calibration.
 *
 * Call once before the main loop, with the rocket standing still on the
 * pad or rail. The next FUSION_CAL_SAMPLES (100, ~2 s at 50 Hz) calls to
 * fusion_update() are used for calibration; any movement during that time
 * gets baked into the gyro bias and the attitude will drift.
 *
 * Can be called again at any time (e.g. after the rocket is bumped while
 * waiting on the pad) to redo the calibration.
 */
void fusion_init(void);

/**
 * @brief Run one step of the whole fusion pipeline.
 *
 * Call once per main-loop iteration with the latest sensor readings.
 *
 * @param in  Sensor readings for this loop (see FusionInput).
 * @param dt  Seconds since the previous call, measured from the real clock
 *            (e.g. HAL_GetTick() difference / 1000). Values <= 0 or > 0.5 s
 *            are treated as 0 so that a long stall (debugger pause) cannot
 *            make the integrators jump; shorter overruns are integrated.
 * @return    The current estimates. Check .ready before using them.
 */
FusionOutput fusion_update(const FusionInput *in, float dt);

#endif /* SENSOR_FUSION_H */
