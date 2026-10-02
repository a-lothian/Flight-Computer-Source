/*
 * telemetry.c
 *
 * See telemetry.h.
 */

#include "telemetry.h"
#include <stdio.h>
#include <string.h>

extern int __io_putchar(int ch);   /* main.c: one character out over ITM */

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

void telemetry_fill_mini(TelemetryRecordMini *pkt, const FusionInput *body,
                         float temperature_C, float pressure_hPa,
                         const FusionOutput *fusion, FSM_State_t fsm_state,
                         const AB_TickOutput *ab)
{
    pkt->imu_accel[0] = body->ax_mg * 0.001f;     /* mg -> g */
    pkt->imu_accel[1] = body->ay_mg * 0.001f;
    pkt->imu_accel[2] = body->az_mg * 0.001f;
    pkt->imu_gyro[0] = body->gx_mdps * 0.001f;    /* mdps -> deg/s */
    pkt->imu_gyro[1] = body->gy_mdps * 0.001f;
    pkt->imu_gyro[2] = body->gz_mdps * 0.001f;
    pkt->temperature_C = temperature_C;
    pkt->pressure_hPa = pressure_hPa;
    pkt->fused_altitude = fusion->altitude;
    pkt->fused_velocity = fusion->velocity;
    pkt->fusion_tilt_deg = fusion->tilt_deg;
    pkt->fsm_state = (uint8_t)fsm_state;
    pkt->ab_cmd = ab->deployment_cmd;
    pkt->ab_pred_apogee_m = ab->predicted_apogee_agl_m;
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

static const char kBase64[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/* The dump's speed depends on these two, and Debug builds (the only ones
 * that dump) are unoptimised, so optimise them anyway */
#define DUMP_FAST __attribute__((optimize("O2")))

/* CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), half a byte at a time */
DUMP_FAST static uint16_t Crc16(const uint8_t *data, uint32_t length)
{
    static const uint16_t kNibble[16] = {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
        0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF
    };
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < length; i++) {
        crc = (uint16_t)((crc << 4) ^ kNibble[(crc >> 12) ^ (data[i] >> 4)]);
        crc = (uint16_t)((crc << 4) ^ kNibble[(crc >> 12) ^ (data[i] & 0x0Fu)]);
    }
    return crc;
}

DUMP_FAST void telemetry_print_b64(uint32_t page_addr, const uint8_t *data, uint32_t length)
{
    uint8_t buf[4 + sizeof(TelemetryRecord_t)];
    if (length > sizeof(TelemetryRecord_t)) {
        return;
    }
    buf[0] = (uint8_t)page_addr;
    buf[1] = (uint8_t)(page_addr >> 8);
    buf[2] = (uint8_t)(page_addr >> 16);
    buf[3] = (uint8_t)(page_addr >> 24);
    memcpy(&buf[4], data, length);
    uint32_t n = 4 + length;

    /* this bypasses stdio, so anything printf'd before must go out first */
    fflush(stdout);

    __io_putchar('@');
    for (uint32_t i = 0; i < n; i += 3) {
        uint32_t rest = n - i;
        uint32_t v = ((uint32_t)buf[i] << 16) |
                     (rest > 1 ? (uint32_t)buf[i + 1] << 8 : 0u) |
                     (rest > 2 ? (uint32_t)buf[i + 2] : 0u);
        __io_putchar(kBase64[(v >> 18) & 0x3Fu]);
        __io_putchar(kBase64[(v >> 12) & 0x3Fu]);
        __io_putchar(rest > 1 ? kBase64[(v >> 6) & 0x3Fu] : '=');
        __io_putchar(rest > 2 ? kBase64[v & 0x3Fu] : '=');
    }

    uint16_t crc = Crc16(buf, n);
    __io_putchar('*');
    for (int shift = 12; shift >= 0; shift -= 4) {
        __io_putchar("0123456789ABCDEF"[(crc >> shift) & 0x0Fu]);
    }
    __io_putchar('\r');
    __io_putchar('\n');
}
