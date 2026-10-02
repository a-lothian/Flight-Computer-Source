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

#define MIN_CONTROL_LOOP_PERIOD_MS 20 // 50hz - the sensor fusion is tuned and validated at this rate

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
 * The FSM is fed the fusion's altitude, so all FSM altitudes (this, the
 * launch gain and the apogee drop) are above the pad. */
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
 * Lives in sensor_mount.h, the one place that says which chip axis points
 * where. The fusion and the FSM both read the body frame it produces. */

/* IMU Range and Data Rate
 * Register bits and LSB sensitivities are derived from these in IMU.c, so
 * they can't get out of sync.
 * Accel FS: 2, 4, 8, 16 g.  Gyro FS: 250, 500, 1000, 2000 dps.
 * ODR: 104, 208, 416, 833 Hz.
 * 16 g: boost is ~10 g, so anything lower clips for the whole burn.
 * 2000 dps: attitude is gyro-only in flight; last year's flight rolled at
 * ~940 dps, and any clipping becomes a permanent tilt error. */
#define IMU_ACCEL_FS_G               16
#define IMU_GYRO_FS_DPS              2000
#define IMU_ODR_HZ                   208

/* Barometer Rates
 * DPS368 measurement rate (per second) and oversampling, as enum values
 * from dps368.h. Pressure at 32 Hz so the fusion's Kalman filter gets a
 * fresh reading most loops (at 1 Hz the altitude steps once a second). */
#define BARO_PRS_MEAS_RATE           DPS368_MEAS_RATE_32
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

/* Log read-back. 1 = at boot, wait for a debugger to start SWV, print the
 * whole log as CSV (tools/: uv run capture-log saves it), then stop there:
 * the airbrakes stay closed and nothing new is logged.
 * Debug builds only - Release refuses to build with it set. */
#define LOG_DUMP_AT_BOOT             0
#define LOG_DUMP_START_DELAY_MS      1000 // lets the capture settle before the header

/* Fresh log. Every boot continues the log already on the chip (a reboot
 * mid-flight adds to it). 1 = at boot, erase the log and stop; then set it
 * back to 0 and re-flash. Dump the log first - erased is gone.
 * Debug builds only - Release refuses to build with it set. */
#define LOG_ERASE_AT_BOOT            0

/* Live telemetry ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,, */

/* 0 = no CAN telemetry: CAN2 is never initialised, so PB12/PB13 stay
 * undriven inputs (their reset state), and the loop never sends. Off for
 * the 2026-10 flight: the flight computer's CAN transceiver has no ground
 * connection, so it can't drive the bus, and a send that can never finish
 * would cost the loop 3 ms. 1 only once that hardware is fixed. NAND
 * logging doesn't depend on it. */
#define TELEMETRY_CAN_ENABLED        0

/* Loops between packets to the Telemetry Board over CAN (telemetry_can.h):
 * 50 Hz / 10 = 5 Hz. It keeps only the latest complete packet and sends it
 * over LoRa every few seconds, so faster gains nothing; send at least 1 Hz. */
#define TELEMETRY_CAN_EVERY          10

/* ms between CAN status checks: start CAN if the boot start found the bus
 * busy, and print the error counters over SWO (telemetry_can.h) */
#define TELEMETRY_CAN_STATUS_MS      1000

/* Buzzer ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,, */

#define STARTUP_BEEP_COUNT           3
#define STARTUP_BEEP_ON_MS           50
#define STARTUP_BEEP_OFF_MS          75
#define READY_BEEP_MS                500

/* Motor ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,, */

/* Servo Calibration (DS5160, 270 degree version)
 * MOTOR_PULSE_MIN_US puts the servo at 0 degrees, MOTOR_PULSE_MAX_US at
 * MOTOR_RANGE_DEG. These describe the servo itself, not the airbrakes. */
#define MOTOR_PULSE_MIN_US           500
#define MOTOR_PULSE_MAX_US           2500
#define MOTOR_RANGE_DEG              270.0f

/* Airbrake Angle Limits
 * Every command (degrees or raw pulse) is clamped to MOTOR_ANGLE_MIN_DEG ..
 * MOTOR_ANGLE_MAX_DEG. Fully open = MIN, closed = MAX.
 * MOTOR_STARTUP_ANGLE_DEG is sent as soon as the PWM starts - keep it closed. */
#define MOTOR_ANGLE_MIN_DEG          45.0f
#define MOTOR_ANGLE_MAX_DEG          230.0f
#define MOTOR_STARTUP_ANGLE_DEG      230.0f

/* Airbrakes ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,, */

/* Master switch. 0 flies with the brakes held closed: ab_tick() still runs
 * every loop (inhibited), so logging and the controller's own state are
 * unchanged. The launch-day fallback if anything about the brakes is in doubt. */
#define AIRBRAKE_ENABLED             1

/* Coast (burnout) mass, told to the apogee predictor.
 * UNVERIFIED: 19.69 kg is OpenRocket PostProd-4 Simulation 13. Weigh the
 * rocket as flown and subtract the 3.77 kg of M1845NT propellant. */
#define AB_COAST_MASS_KG             19.69f

#define AB_TARGET_APOGEE_AGL_M       3048.0f  // 10,000 ft

/* Airbrake slew rates, in full travel per second (1 / seconds for full
 * travel). They differ because the brakes are spring loaded, so opening
 * is slower than closing. The controller uses them as its model of where
 * the brakes are, so a rate faster than the real mechanism makes it think
 * the brakes are further out than they are.
 * UNVERIFIED: 1.0 is a placeholder. Time a full closed -> open and a full
 * open -> closed sweep under spring load, and set each to 1 / seconds. */
#define AB_DEPLOY_RATE_PER_S         1.0f
#define AB_RETRACT_RATE_PER_S        1.0f

/* The brakes only move while the nose is within this angle of vertical.
 * Matches the abort tilt the airbrake harness was tuned with
 * (GatingMirror.java); confirm against the AURC airbrake rules. */
#define AB_MAX_TILT_DEG              28.0f

/* Deployment fraction (0 closed .. 1 full, 30 mm) -> servo angle.
 * UNVERIFIED: linear from MOTOR_ANGLE_MAX_DEG (closed) to MOTOR_ANGLE_MIN_DEG
 * (open) is a placeholder. The Cd table is in mm of extension, so measure the
 * servo angle at 0/10/20/30 mm, fix ab_io_servo_angle_deg() if it isn't
 * linear, then set this to 1 to silence the build warning. */
#define AB_SERVO_MAPPING_VERIFIED    1

#endif /* INC_CONFIG_H_ */
