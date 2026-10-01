/*
 * telemetry.c
 *
 * See telemetry.h.
 */

#include "telemetry.h"
#include <stdio.h>

static uint8_t FusionFlags(const FusionOutput *f)
{
    return (uint8_t)((f->ready     ? FUSION_FLAG_READY     : 0u) |
                     (f->in_flight ? FUSION_FLAG_IN_FLIGHT : 0u) |
                     (f->descent   ? FUSION_FLAG_DESCENT   : 0u) |
                     (f->baro_ok   ? FUSION_FLAG_BARO_OK   : 0u) |
                     (f->high_g    ? FUSION_FLAG_HIGH_G    : 0u));
}

void telemetry_fill(TelemetryRecord_t *rec, uint32_t timestamp_ms,
                    const AccelData_t *h3lis, const IMU *imu,
                    float temperature_C, float pressure_hPa, float altitude_m,
                    FSM_State_t fsm_state, uint32_t loop_us, uint32_t loop_us_max,
                    const FusionOutput *fusion, const AB_TickOutput *ab,
                    uint8_t ab_boot_flags)
{
    rec->timestamp_ms = timestamp_ms;
    rec->h3lis_accel[0] = h3lis->accel_x;
    rec->h3lis_accel[1] = h3lis->accel_y;
    rec->h3lis_accel[2] = h3lis->accel_z;
    rec->imu_accel[0] = imu->acceleration[0];
    rec->imu_accel[1] = imu->acceleration[1];
    rec->imu_accel[2] = imu->acceleration[2];
    rec->imu_gyro[0] = imu->gyro[0];
    rec->imu_gyro[1] = imu->gyro[1];
    rec->imu_gyro[2] = imu->gyro[2];
    rec->temperature_C = temperature_C;
    rec->pressure_hPa = pressure_hPa;
    rec->altitude_m = altitude_m;
    rec->fsm_state = (uint8_t)fsm_state;
    rec->loop_us = loop_us;
    rec->loop_us_max = loop_us_max;
    rec->fusion_altitude_m = fusion->altitude;
    rec->fusion_velocity_mps = fusion->velocity;
    rec->fusion_accel_vert = fusion->accel_vert;
    rec->fusion_tilt_deg = fusion->tilt_deg;
    rec->fusion_flags = FusionFlags(fusion);
    rec->ab_cmd = ab->deployment_cmd;
    rec->ab_pred_apogee_m = ab->predicted_apogee_agl_m;
    rec->ab_faults = ab->fault_flags;
    rec->ab_mode = ab->mode;
    rec->ab_boot = ab_boot_flags;
}

void telemetry_print(uint32_t page_addr, const uint8_t *data, uint32_t length)
{
    (void)length;
    const TelemetryRecord_t *rec = (const TelemetryRecord_t *)data;

    printf("page %lu | t=%lums | h3lis(%d,%d,%d) | imu_a(%.2f,%.2f,%.2f) mg | imu_g(%.2f,%.2f,%.2f) mdps | %.2fC %.2fhPa %.2fm\r\n",
           (unsigned long)page_addr, (unsigned long)rec->timestamp_ms,
           rec->h3lis_accel[0], rec->h3lis_accel[1], rec->h3lis_accel[2],
           rec->imu_accel[0], rec->imu_accel[1], rec->imu_accel[2],
           rec->imu_gyro[0], rec->imu_gyro[1], rec->imu_gyro[2],
           rec->temperature_C, rec->pressure_hPa, rec->altitude_m);
    printf("  state: %s | loop %luus (max %luus)\r\n", FSM_StateName((FSM_State_t)rec->fsm_state),
           (unsigned long)rec->loop_us, (unsigned long)rec->loop_us_max);
    printf("  fusion: alt %.1fm vel %.1fm/s acc %.2fm/s2 tilt %.1fdeg flags 0x%02x\r\n",
           rec->fusion_altitude_m, rec->fusion_velocity_mps, rec->fusion_accel_vert,
           rec->fusion_tilt_deg, rec->fusion_flags);
    printf("  airbrake: cmd %.3f apogee %.1fm mode %u faults 0x%04x boot 0x%02x\r\n",
           rec->ab_cmd, rec->ab_pred_apogee_m, rec->ab_mode, rec->ab_faults, rec->ab_boot);
}

/* Keep the header and the row format in step: one column per field. */
void telemetry_print_csv_header(void)
{
    printf("page,timestamp_ms,h3lis_x,h3lis_y,h3lis_z,"
           "imu_ax_mg,imu_ay_mg,imu_az_mg,imu_gx_mdps,imu_gy_mdps,imu_gz_mdps,"
           "temperature_C,pressure_hPa,altitude_m,fsm_state,fsm_state_name,"
           "loop_us,loop_us_max,"
           "fusion_altitude_m,fusion_velocity_mps,fusion_accel_vert_mps2,fusion_tilt_deg,fusion_flags,"
           "ab_cmd,ab_pred_apogee_m,ab_faults,ab_mode,ab_boot\r\n");
}

void telemetry_print_csv(uint32_t page_addr, const uint8_t *data, uint32_t length)
{
    (void)length;
    const TelemetryRecord_t *rec = (const TelemetryRecord_t *)data;

    printf("%lu,%lu,%d,%d,%d,"
           "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
           "%.2f,%.2f,%.2f,%u,%s,"
           "%lu,%lu,"
           "%.3f,%.3f,%.3f,%.2f,%u,"
           "%.4f,%.1f,%u,%u,%u\r\n",
           (unsigned long)page_addr, (unsigned long)rec->timestamp_ms,
           rec->h3lis_accel[0], rec->h3lis_accel[1], rec->h3lis_accel[2],
           rec->imu_accel[0], rec->imu_accel[1], rec->imu_accel[2],
           rec->imu_gyro[0], rec->imu_gyro[1], rec->imu_gyro[2],
           rec->temperature_C, rec->pressure_hPa, rec->altitude_m,
           rec->fsm_state, FSM_StateName((FSM_State_t)rec->fsm_state),
           (unsigned long)rec->loop_us, (unsigned long)rec->loop_us_max,
           rec->fusion_altitude_m, rec->fusion_velocity_mps, rec->fusion_accel_vert,
           rec->fusion_tilt_deg, rec->fusion_flags,
           rec->ab_cmd, rec->ab_pred_apogee_m, rec->ab_faults, rec->ab_mode, rec->ab_boot);
}
