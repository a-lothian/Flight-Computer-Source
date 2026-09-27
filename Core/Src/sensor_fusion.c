/**
 * @file    sensor_fusion.c
 * @brief   Top level of the sensor fusion: runs every stage in order.
 *
 * This file owns all the filter state (as static variables, so main.c
 * can't see or disturb it) and calls the other modules:
 *
 *   accel_select   -> best accelerometer per axis
 *   attitude       -> complementary filter (orientation)
 *   vertical_accel -> earth-frame vertical acceleration
 *   altitude_kf    -> Kalman filter (altitude, velocity)
 *
 * It also handles the things that don't belong to any single stage:
 *   Pad calibration  - averaging the sensors for the first ~2 s.
 *   Pad tracking     - slowly following gyro bias, accel bias and pad
 *                      altitude while the rocket waits, still, on the pad.
 *   Flight detection - deciding when the accelerometer is allowed to
 *                      correct the attitude.
 *   Descent mode     - switching the Kalman filter to baro-led after apogee.
 *   Input checks     - ignoring samples that are not numbers.
 */
#include "sensor_fusion.h"
#include "fusion_common.h"
#include "attitude.h"
#include "vertical_accel.h"
#include "altitude_kf.h"
#include "accel_select.h"
#include <stdint.h>
#include <math.h>

/* ---- Settings ------------------------------------------------------ */

/** Largest time step that is integrated, s. A loop can genuinely overrun
 *  by a few hundred ms (each HAL I2C read in the drivers has a 100 ms
 *  timeout), and dropping that time would lose real motion - 0.3 s of
 *  boost is ~30 m/s. Longer gaps (debugger pause) are treated as dt = 0
 *  so the integrators don't jump. */
#ifndef DT_MAX
#define DT_MAX              0.5f
#endif

/** Number of samples averaged during pad calibration (~2 s at 50 Hz). */
#ifndef FUSION_CAL_SAMPLES
#define FUSION_CAL_SAMPLES  100
#endif

/* Flight detection - decides when the accelerometer may correct attitude */

/** Launch = total acceleration above this many g... */
#ifndef LAUNCH_G
#define LAUNCH_G            2.0f
#endif
/** ...for this many consecutive samples (~100 ms). A knock on the pad is
 *  too short to trigger it; a motor burn is not. */
#ifndef LAUNCH_SAMPLES
#define LAUNCH_SAMPLES      5
#endif
/** "Still" = rotating slower than this (deg/s)... */
#ifndef STILL_GYRO_DPS
#define STILL_GYRO_DPS      3.0f
#endif
/** ...and total acceleration within this fraction of 1 g... */
#ifndef STILL_ACCEL_BAND
#define STILL_ACCEL_BAND    0.05f
#endif
/** ...continuously for this long (s) -> back to ground mode... */
#ifndef STILL_TIME_S
#define STILL_TIME_S        3.0f
#endif
/** ...and only if the filter's vertical velocity is below this (m/s).
 *  A rocket hanging steadily under a parachute can look just as still as
 *  one lying on the ground; only the descent rate tells them apart. */
#ifndef LANDED_MAX_MPS
#define LANDED_MAX_MPS      2.0f
#endif

/** Descent mode arms once the filter has seen at least this upward
 *  velocity in flight (m/s), then switches on when velocity turns
 *  negative - i.e. at the filter's own apogee. The arm speed keeps noise
 *  on the pad or a hop on the rail from ever triggering it. */
#ifndef DESCENT_ARM_MPS
#define DESCENT_ARM_MPS     20.0f
#endif

/**
 * Pad tracking time constant, s. While on the ground and still, gyro bias,
 * vertical-accel bias and pad altitude each move this slowly towards what
 * the sensors currently read. Slow enough that wind rocking the rocket on
 * the rail averages out, fast enough to follow temperature and weather
 * drift over an hour-long wait.
 */
#ifndef PAD_TRACK_TAU_S
#define PAD_TRACK_TAU_S     30.0f
#endif

/** The DPS368 measures 300..1200 hPa (datasheet), which the firmware's
 *  formula turns into about -700 m .. +9200 m. Baro altitudes outside this
 *  band (with margin) cannot come from a working sensor. */
#ifndef BARO_MIN_ASL_M
#define BARO_MIN_ASL_M      -1500.0f
#endif
#ifndef BARO_MAX_ASL_M
#define BARO_MAX_ASL_M      10000.0f
#endif

/** The baro is reported unhealthy (baro_ok = false) once no reading has
 *  been accepted by the Kalman filter for this long, s. */
#ifndef BARO_STALE_S
#define BARO_STALE_S        1.0f
#endif

/* ------------------------------------------------------------------ */
/* Internal state - persists between calls, invisible to main.c       */
/* ------------------------------------------------------------------ */

/* Filter stages */
static Quat_t        attitude;      /**< current best orientation          */
static AltitudeKF_t  kf;            /**< altitude / velocity Kalman filter */
static AccelSelect_t sel;           /**< LSM6DSOX / H3LIS switching        */

/* Flight detection */
static bool     in_flight;          /**< true = gyro-only attitude          */
static uint32_t launch_count;       /**< consecutive samples above LAUNCH_G */
static float    still_time;         /**< seconds continuously still         */
static float    peak_v;             /**< highest velocity seen this flight  */
static bool     launched;           /**< has genuinely flown since fusion_init(): the
                                         filter has seen > DESCENT_ARM_MPS. A knock
                                         on the pad that trips launch detection
                                         never gets anywhere near that speed. */

/* Pad calibration */
static uint32_t cal_count;          /**< samples collected so far           */
static float    acc_sum[3];         /**< sum of chosen accel, m/s^2         */
static float    lsm_sum_mg[3];      /**< sum of LSM6DSOX accel, mg          */
static float    hg_sum_mg[3];       /**< sum of H3LIS accel, mg             */
static float    gyro_sum[3];        /**< sum of gyro, rad/s                 */
static float    baro_sum;           /**< sum of valid baro altitudes, m     */
static uint32_t baro_count;         /**< number of valid baro altitudes     */

/* Calibration results */
static float    gyro_bias[3];       /**< rad/s, subtracted from every gyro sample */
static float    a_vert_bias;        /**< m/s^2, subtracted from vertical accel    */
static float    ground_alt;         /**< baro altitude of the pad, m              */
static bool     ground_valid;       /**< ground_alt has been set from a real reading */

/* Baro health */
static float    baro_age;           /**< s since the Kalman filter last accepted a reading */

/* Last output, returned unchanged if a sample has to be discarded */
static FusionOutput last_out;

/* ------------------------------------------------------------------ */
/** true for an ordinary number (not NaN, not +/-inf). */
static bool finite_f(float x)
{
    return (x == x) && (fabsf(x) <= 3.0e38f);
}

/** A baro altitude worth using: a number, inside what the DPS368 can
 *  physically report. Anything else is a failed sensor or read - e.g. a
 *  zero pressure makes dps368_get_altitude() return exactly 44330 m. */
static bool baro_valid(float alt_m)
{
    return finite_f(alt_m) && alt_m > BARO_MIN_ASL_M && alt_m < BARO_MAX_ASL_M;
}

/* ------------------------------------------------------------------ */
/**
 * Reset everything and restart calibration. See sensor_fusion.h.
 *
 * Note that the attitude is not set here - it is only known after the
 * accelerometer has been averaged during calibration.
 */
void fusion_init(void)
{
    attitude.w = 1.0f;
    attitude.x = attitude.y = attitude.z = 0.0f;
    altitude_kf_init(&kf);
    accel_select_init(&sel);

    in_flight    = false;
    launch_count = 0;
    still_time   = 0.0f;
    peak_v       = 0.0f;
    launched     = false;

    cal_count = 0;
    for (int i = 0; i < 3; i++) {
        acc_sum[i]    = 0.0f;
        lsm_sum_mg[i] = 0.0f;
        hg_sum_mg[i]  = 0.0f;
        gyro_sum[i]   = 0.0f;
        gyro_bias[i]  = 0.0f;
    }
    baro_sum     = 0.0f;
    baro_count   = 0;
    a_vert_bias  = 0.0f;
    ground_alt   = 0.0f;
    ground_valid = false;
    baro_age     = 0.0f;

    FusionOutput zero = {0};
    last_out = zero;
}

/* ------------------------------------------------------------------ */
/**
 * One step of the pipeline. See sensor_fusion.h and the README for the
 * overall flow; each numbered block below is one stage.
 */
FusionOutput fusion_update(const FusionInput *in, float dt)
{
    FusionOutput out = {0};

    /* Protect the integrators from a bad timestamp or a long stall.
     * Written so that a NaN dt also ends up as 0. */
    if (!(dt > 0.0f && dt <= DT_MAX)) dt = 0.0f;

    /* An IMU value that is not a number would poison every integrator
     * for the rest of the flight. Skip the sample and repeat the last
     * output rather than let that happen. */
    const float imu[9] = { in->ax_mg, in->ay_mg, in->az_mg,
                           in->hg_ax_mg, in->hg_ay_mg, in->hg_az_mg,
                           in->gx_mdps, in->gy_mdps, in->gz_mdps };
    for (int i = 0; i < 9; i++) {
        if (!finite_f(imu[i])) return last_out;
    }
    bool baro_ok = in->baro_new && baro_valid(in->baro_alt_m);

    /* 0. Pick the best accelerometer for each axis ------------------ */
    float lsm[3] = { in->ax_mg,    in->ay_mg,    in->az_mg    };
    float hg[3]  = { in->hg_ax_mg, in->hg_ay_mg, in->hg_az_mg };
    float acc_mg[3];
    accel_select(&sel, lsm, hg, acc_mg);

    /* 1. Convert units: everything below works in m/s^2 and rad/s ---- */
    float ax = acc_mg[0] * MG_TO_MPS2;
    float ay = acc_mg[1] * MG_TO_MPS2;
    float az = acc_mg[2] * MG_TO_MPS2;
    float gx = in->gx_mdps * MDPS_TO_RADPS;
    float gy = in->gy_mdps * MDPS_TO_RADPS;
    float gz = in->gz_mdps * MDPS_TO_RADPS;

    /* 2. Calibrate on the pad --------------------------------------- *
     * For the first FUSION_CAL_SAMPLES calls, only accumulate sums and
     * return with ready = false. On the last one, turn the sums into
     * averages and set every calibration value at once.               */
    if (cal_count < FUSION_CAL_SAMPLES) {
        acc_sum[0]  += ax;  acc_sum[1]  += ay;  acc_sum[2]  += az;
        for (int i = 0; i < 3; i++) {
            lsm_sum_mg[i] += lsm[i];
            hg_sum_mg[i]  += hg[i];
        }
        gyro_sum[0] += gx;  gyro_sum[1] += gy;  gyro_sum[2] += gz;
        if (baro_valid(in->baro_alt_m)) {
            baro_sum += in->baro_alt_m;
            baro_count++;
        }
        cal_count++;

        if (cal_count == FUSION_CAL_SAMPLES) {
            float n = (float)FUSION_CAL_SAMPLES;
            float axm = acc_sum[0] / n, aym = acc_sum[1] / n, azm = acc_sum[2] / n;

            /* Not rotating on the pad, so the average gyro reading is
             * pure bias. Subtracted from every later sample. */
            gyro_bias[0] = gyro_sum[0] / n;
            gyro_bias[1] = gyro_sum[1] / n;
            gyro_bias[2] = gyro_sum[2] / n;

            /* Starting lean from the averaged gravity vector (averaging
             * removes most of the accelerometer noise). */
            attitude = attitude_from_accel(axm, aym, azm);

            /* Sitting still, vertical accel should be exactly 0. Whatever
             * it reads instead (accelerometer scale error, slightly wrong
             * g) is a bias. Without removing it, velocity drifts. */
            a_vert_bias = vertical_accel(&attitude, axm, aym, azm);

            /* Line the H3LIS up with the LSM6DSOX */
            float lsm_mean[3], hg_mean[3];
            for (int i = 0; i < 3; i++) {
                lsm_mean[i] = lsm_sum_mg[i] / n;
                hg_mean[i]  = hg_sum_mg[i]  / n;
            }
            accel_select_calibrate(&sel, lsm_mean, hg_mean);

            /* Pad altitude: all altitudes are reported relative to this.
             * If the baro gave nothing usable, take the first good
             * reading later instead of assuming 0. */
            if (baro_count > 0) {
                ground_alt   = baro_sum / (float)baro_count;
                ground_valid = true;
            }
            altitude_kf_init(&kf);
        }

        out.ready = false;           /* everything else stays zero */
        last_out  = out;
        return out;
    }

    if (!ground_valid && baro_ok) {
        ground_alt   = in->baro_alt_m;
        ground_valid = true;
    }

    /* Remove the gyro bias measured on the pad */
    float gxc = gx - gyro_bias[0];
    float gyc = gy - gyro_bias[1];
    float gzc = gz - gyro_bias[2];

    /* Flight detection ---------------------------------------------- *
     * In flight the accelerometer measures thrust and drag as well as
     * gravity, so it can't be used to correct the attitude. This block
     * decides when it's safe to use.                                  */
    float a_mag_g = sqrtf(ax * ax + ay * ay + az * az) / G_MPS2;
    float w_dps   = sqrtf(gxc * gxc + gyc * gyc + gzc * gzc) * RAD2DEG;
    bool  still   = (w_dps < STILL_GYRO_DPS) &&
                    (fabsf(a_mag_g - 1.0f) < STILL_ACCEL_BAND);
    if (!in_flight) {
        /* Sustained high acceleration = launch. A knock on the pad is
         * too short to reach LAUNCH_SAMPLES in a row. */
        if (a_mag_g > LAUNCH_G) {
            if (++launch_count >= LAUNCH_SAMPLES) in_flight = true;
        } else {
            launch_count = 0;
        }
    } else {
        /* Back to ground mode only after being truly still (landed, or
         * hanging very steadily) for STILL_TIME_S. A parachute opening
         * shock re-triggers flight mode through the launch check above. */
        bool landed_now = still && fabsf(kf.v) < LANDED_MAX_MPS;
        still_time = landed_now ? still_time + dt : 0.0f;
        if (still_time >= STILL_TIME_S) {
            in_flight    = false;
            launch_count = 0;
            still_time   = 0.0f;
            peak_v       = 0.0f;
            kf.descent   = false;
        }
    }

    /* Pad tracking --------------------------------------------------- *
     * While waiting on the pad the gyro bias drifts with temperature and
     * the air pressure changes with the weather. Both are followed slowly,
     * but only while the rocket is genuinely still on the ground, so that
     * handling and launch never leak in, and only before the first
     * launch, so a false "landed" in the air can never move the ground
     * reference. Pad tracking never delays "ready" - it only refines
     * values that already exist.                                      */
    bool pad_still = !launched && !in_flight && launch_count == 0 && still;
    float k_pad = dt / (PAD_TRACK_TAU_S + dt);
    if (pad_still) {
        gyro_bias[0] += k_pad * (gx - gyro_bias[0]);
        gyro_bias[1] += k_pad * (gy - gyro_bias[1]);
        gyro_bias[2] += k_pad * (gz - gyro_bias[2]);
    }

    /* 3. Complementary filter -> attitude --------------------------- *
     * Accelerometer correction only when not in flight.               */
    attitude = attitude_update(&attitude, ax, ay, az, gxc, gyc, gzc, dt, !in_flight);

    /* 4. Earth-frame vertical acceleration, pad bias removed --------- */
    float a_vert_raw = vertical_accel(&attitude, ax, ay, az);
    if (pad_still) {
        a_vert_bias += k_pad * (a_vert_raw - a_vert_bias);
        if (baro_ok) {
            ground_alt += k_pad * (in->baro_alt_m - ground_alt);
        }
    }
    float a_vert = a_vert_raw - a_vert_bias;

    /* 5. Kalman filter -> velocity and altitude --------------------- *
     * Predict every loop; correct only when the baro has a NEW reading
     * (a repeated stale value would be counted as extra evidence).
     * After the filter's own apogee it switches to baro-led descent:
     * once the rocket tumbles, the gyro-only attitude - and so a_vert -
     * is meaningless.                                                 */
    if (in_flight && !kf.descent) {
        if (kf.v > peak_v) peak_v = kf.v;
        if (peak_v > DESCENT_ARM_MPS) launched = true;
        if (peak_v > DESCENT_ARM_MPS && kf.v < 0.0f) kf.descent = true;
    }
    altitude_kf_predict(&kf, a_vert, dt);
    baro_age += dt;
    if (baro_ok && ground_valid &&
        altitude_kf_update(&kf, in->baro_alt_m - ground_alt)) {
        baro_age = 0.0f;
    }

    /* 6. Report ------------------------------------------------------ *
     * Euler angles get rocket-style names: phi/theta are the two
     * leans, psi (spin about the nose) is the rocket's roll.          */
    Euler_t e = attitude_euler(&attitude);
    out.accel_vert = a_vert;
    out.velocity   = kf.v;
    out.altitude   = kf.h;
    out.pitch_deg  = e.phi   * RAD2DEG;
    out.yaw_deg    = e.theta * RAD2DEG;
    out.roll_deg   = e.psi   * RAD2DEG;
    out.tilt_deg   = attitude_tilt(&attitude) * RAD2DEG;
    out.ready      = true;
    out.high_g     = accel_select_high_g(&sel);
    out.in_flight  = in_flight;
    out.descent    = kf.descent;
    out.baro_ok    = baro_age < BARO_STALE_S;

    last_out = out;
    return out;
}
