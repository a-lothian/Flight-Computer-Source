/*
 * telemetry.h
 *
 * The telemetry record logged to NAND once per loop: its layout, how it is
 * built from the loop's sensor, fusion, FSM and airbrake values, and how a
 * record read back from NAND is printed.
 *
 * The layout is what's on the flash, so any change to TelemetryRecord_t
 * makes older logs unreadable with the new telemetry_print().
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

#include <stdint.h>
#include "IMU.h"
#include "h3lis.h"
#include "fsm.h"
#include "sensor_fusion.h"
#include "airbrake_api.h"

typedef struct {
    uint32_t timestamp_ms;
    int8_t   h3lis_accel[3];
    float    imu_accel[3];
    float    imu_gyro[3];
    float    temperature_C;
    float    pressure_hPa;
    float    altitude_m;
    uint8_t  fsm_state;
    uint32_t loop_us;      /* how long the PREVIOUS loop body took */
    uint32_t loop_us_max;  /* worst loop body since boot, must stay < 20000 */
    float    fusion_altitude_m;   /* above the pad */
    float    fusion_velocity_mps; /* vertical, +up */
    float    fusion_accel_vert;   /* m/s^2, gravity removed, +up */
    float    fusion_tilt_deg;     /* nose from vertical */
    uint8_t  fusion_flags;        /* FUSION_FLAG_* bits */
    float    ab_cmd;              /* deployment command, 0 closed .. 1 full */
    float    ab_pred_apogee_m;    /* predicted apogee AGL, 0 while inhibited */
    uint16_t ab_faults;           /* sticky AB_FAULT_* bits */
    uint8_t  ab_mode;             /* AB_Mode */
    uint8_t  ab_boot;             /* AB_BOOT_* bits */
} TelemetryRecord_t;

/* Live telemetry sent to the Telemetry Board over CAN (telemetry_can.h),
 * which forwards it over LoRa to the ground station. This is a wire format:
 * packed, little-endian, IEEE-754 floats, so the receiver must define the
 * same struct byte for byte. */
enum FlightState {
	READY,
	THRUST,
	COAST,
	APOGEE,
	SECOND_CHUTE,
	LANDED
};

typedef struct __attribute__((packed)) {
	float imu_accel[3];     /* offset  0: g (1.0 = 9.80665 m/s^2), rocket body frame, z = out of the nose */
	float imu_gyro[3];      /* offset 12: deg/s, rocket body frame */
	float temperature_C;    /* offset 24 */
	float pressure_hPa;     /* offset 28 */
	float fused_altitude;   /* offset 32: m above the pad */
	float fused_velocity;   /* offset 36: m/s, vertical, + up */
	float fusion_tilt_deg;  /* offset 40: nose from vertical */
	uint8_t fsm_state;      /* offset 44: enum FlightState (an enum's size isn't fixed, so 1 byte) */
	float ab_cmd;           /* offset 45: airbrake deployment command, 0 closed .. 1 full */
	float ab_pred_apogee_m; /* offset 49: predicted apogee above the pad, 0 while inhibited */
} TelemetryRecordMini;

_Static_assert(sizeof(TelemetryRecordMini) == 53, "TelemetryRecordMini must be 53 bytes");
/* FlightState is the FSM's state numbering, sent as is */
_Static_assert((int)READY == (int)FSM_STATE_READY && (int)THRUST == (int)FSM_STATE_THRUST &&
               (int)COAST == (int)FSM_STATE_BURNOUT && (int)APOGEE == (int)FSM_STATE_APOGEE &&
               (int)SECOND_CHUTE == (int)FSM_STATE_SECOND_CHUTE && (int)LANDED == (int)FSM_STATE_LANDED,
               "FlightState must match FSM_State_t");

/* fusion_flags bits in the telemetry record */
#define FUSION_FLAG_READY     (1u << 0)
#define FUSION_FLAG_IN_FLIGHT (1u << 1)
#define FUSION_FLAG_DESCENT   (1u << 2)
#define FUSION_FLAG_BARO_OK   (1u << 3)
#define FUSION_FLAG_HIGH_G    (1u << 4)

/* ab_boot bits in the telemetry record */
#define AB_BOOT_CONFIGURED    (1u << 0) /* ab_configure() accepted the config at boot */
#define AB_BOOT_SELF_TEST_OK  (1u << 1) /* ab_self_test() passed at boot */
#define AB_BOOT_PAD_ELEVATION (1u << 2) /* reconfigured with the pad elevation at launch */

/* Fill every field of rec from this loop's values. The IMU values are the
 * raw chip axes, as the drivers read them. */
void telemetry_fill(TelemetryRecord_t *rec, uint32_t timestamp_ms,
                    const AccelData_t *h3lis, const IMU *imu,
                    float temperature_C, float pressure_hPa, float altitude_m,
                    FSM_State_t fsm_state, uint32_t loop_us, uint32_t loop_us_max,
                    const FusionOutput *fusion, const AB_TickOutput *ab,
                    uint8_t ab_boot_flags);

/* Fill the live telemetry packet from this loop's values. body is the
 * FusionInput sensor_mount_input() built (body frame, mg and mdps). */
void telemetry_fill_mini(TelemetryRecordMini *pkt, const FusionInput *body,
                         float temperature_C, float pressure_hPa,
                         const FusionOutput *fusion, FSM_State_t fsm_state,
                         const AB_TickOutput *ab);

/* printf one record. Matches W25N_Log_Dump_Cb, so it can be passed straight
 * to W25N_Log_Dump; page_addr is only used as a label. */
void telemetry_print(uint32_t page_addr, const uint8_t *data, uint32_t length);

/* CSV for reading a log back: print the header line once, then pass
 * telemetry_print_csv to W25N_Log_Dump to print one row per record.
 * Columns are in record order with units in the names; page_addr is the
 * first column. Flags and faults are decimal so they import as numbers. */
void telemetry_print_csv_header(void);
void telemetry_print_csv(uint32_t page_addr, const uint8_t *data, uint32_t length);

/* The fast dump, for capture-log (tools/): pass it to W25N_Log_Dump instead
 * of telemetry_print_csv. One line per record,
 *     @<base64 of page_addr (uint32, little-endian) then the record>*<CRC>\r\n
 * with CRC the CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) of the same
 * bytes, as 4 hex digits. capture-log turns each line back into the row
 * telemetry_print_csv would print. That printf's software float formatting
 * took most of the dump time at 8 MHz; this skips printf altogether. */
void telemetry_print_b64(uint32_t page_addr, const uint8_t *data, uint32_t length);

/* capture-log decodes the dump with its own copy of this layout (RECORD in
 * tools/src/flight_log_tools/capture_log.py): change both together */
_Static_assert(sizeof(TelemetryRecord_t) == 88, "TelemetryRecord_t changed: update RECORD in capture_log.py");

#endif /* INC_TELEMETRY_H_ */
