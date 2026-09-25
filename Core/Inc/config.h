/*
 * config.h
 *
 *  Created on: Sep 25, 2026
 *      Author: alexlothian
 *
 * Tunable constants for the flight computer. Units are metric throughout:
 * distances/altitudes in metres, times in ms, acceleration in mg
 * (milli-g, the IMU's native output), angular rate in mdps.
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

/* Main loop ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,, */

#define MIN_CONTROL_LOOP_PERIOD_MS 10 // 100hz

/* Flight state machine ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,, */

/* LAUNCH State Verification
 * Vertical accel must stay above LAUNCH_ACCEL_THRESHOLD_MG for a minimum of
 * LAUNCH_CONFIRM_MS, and baro read LAUNCH_MIN_ALTITUDE_GAIN_M above the pad.
 * The LSM6DSOX saturates at IMU_ACCEL_FS_G, so keep the threshold below that. */
#define LAUNCH_ACCEL_THRESHOLD_MG    3000.0f
#define LAUNCH_MIN_ALTITUDE_GAIN_M   10.0f
#define LAUNCH_CONFIRM_MS            100

/* BURNOUT State Verification
 * Vertical accel (drag deceleration) must stay below BURNOUT_ACCEL_THRESHOLD_MG
 * for a minimum of BURNOUT_CONFIRM_MS. */
#define BURNOUT_ACCEL_THRESHOLD_MG   -1000.0f
#define BURNOUT_CONFIRM_MS           100

/* APOGEE State Verification
 * Needs APOGEE_VOTES_REQUIRED of {tilt, altitude drop, accel}. The altitude
 * vote passes once altitude is APOGEE_ALTITUDE_DROP_M below the max seen; the
 * accel vote once it stays below APOGEE_ACCEL_THRESHOLD_MG for APOGEE_CONFIRM_MS. */
#define APOGEE_ACCEL_THRESHOLD_MG    0.0f
#define APOGEE_ALTITUDE_DROP_M       10.0f
#define APOGEE_CONFIRM_MS            100
#define APOGEE_VOTES_REQUIRED        2

/* SECOND_CHUTE State Verification
 * Deploys once altitude + MAIN_DEPLOY_MARGIN_M drops below MAIN_DEPLOY_ALTITUDE_M.
 * NOTE: compared against the raw baro altitude, which is above standard sea
 * level, NOT above the pad. */
#define MAIN_DEPLOY_ALTITUDE_M       900.0f
#define MAIN_DEPLOY_MARGIN_M         10.0f

/* LANDED State Verification
 * All gyro axes must stay below LANDED_GYRO_TOLERANCE_MDPS for a minimum of
 * LANDED_CONFIRM_MS. */
#define LANDED_GYRO_TOLERANCE_MDPS   10000.0f // 10 dps
#define LANDED_CONFIRM_MS            3000

/* Fallback Timeouts
 * Last resort if sensor detection never fires. Set to ~1.5-2x the real burn
 * time (motor datasheet) / coast time (OpenRocket sim). */
#define MAX_THRUST_DURATION_MS       8000
#define MAX_COAST_DURATION_MS        45000

/* Sensors ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,, */

/* IMU Axis Mapping
 * Which LSM6DSOX axis points up the rocket (reads ~+1000 mg on the pad).
 * The other two are the lateral axes, passed to the FSM as x and z. */
#define IMU_AXIS_VERTICAL            0
#define IMU_AXIS_LATERAL_A           1
#define IMU_AXIS_LATERAL_B           2

/* IMU Range and Data Rate
 * Register bits and LSB sensitivities are derived from these in IMU.c, so
 * they can't get out of sync.
 * Accel FS: 2, 4, 8, 16 g.  Gyro FS: 250, 500, 1000, 2000 dps.
 * ODR: 104, 208, 416, 833 Hz. */
#define IMU_ACCEL_FS_G               4
#define IMU_GYRO_FS_DPS              250
#define IMU_ODR_HZ                   208

/* Barometer Rates
 * DPS368 measurement rate (per second) and oversampling, as enum values
 * from dps368.h. */
#define BARO_PRS_MEAS_RATE           DPS368_MEAS_RATE_1
#define BARO_PRS_OVERSAMPLING        DPS368_SAMP_RATE_8
#define BARO_TMP_MEAS_RATE           DPS368_MEAS_RATE_1
#define BARO_TMP_OVERSAMPLING        DPS368_SAMP_RATE_8

/* Barometer Reference Pressure
 * Sea-level pressure used by the barometric altitude formula. */
#define BARO_SEA_LEVEL_PA            101325.0f

/* H3LIS Data Rate
 * H3LIS331 high-g accelerometer output data rate (bits from h3lis.h). */
#define H3LIS_DATA_RATE              H3LIS_DR_400HZ

/* Logging ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,, */

#define NAND_TIMEOUT_MS              100
#define LOG_DUMP_PAGE_COUNT          100000

/* Buzzer ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,, */

#define STARTUP_BEEP_COUNT           3
#define STARTUP_BEEP_ON_MS           100
#define STARTUP_BEEP_OFF_MS          200
#define READY_BEEP_MS                500

#endif /* INC_CONFIG_H_ */
