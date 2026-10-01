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

/* printf one record. Matches W25N_Log_Dump_Cb, so it can be passed straight
 * to W25N_Log_Dump; page_addr is only used as a label. */
void telemetry_print(uint32_t page_addr, const uint8_t *data, uint32_t length);

/* CSV for reading a log back: print the header line once, then pass
 * telemetry_print_csv to W25N_Log_Dump to print one row per record.
 * Columns are in record order with units in the names; page_addr is the
 * first column. Flags and faults are decimal so they import as numbers. */
void telemetry_print_csv_header(void);
void telemetry_print_csv(uint32_t page_addr, const uint8_t *data, uint32_t length);

#endif /* INC_TELEMETRY_H_ */
