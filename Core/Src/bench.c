/*
 * bench.c
 *
 * See bench.h. Everything here is compiled only in the Bench build.
 */

#include "bench.h"

#ifdef BENCH_BUILD

#include "airbrake_api.h"
#include "airbrake_io.h"
#include "config.h"
#include <stdio.h>

/* Debug and Bench define DEBUG, Release (flight) does not */
#ifndef DEBUG
#error "BENCH_BUILD moves the airbrakes at boot - it must never be set in a Release (flight) build"
#endif
#pragma GCC warning "BENCH_BUILD: bench tests run at boot and the airbrakes move - never fly this build"

#define BENCH_WARNING_MS         5000   // time to get hands clear before anything moves
#define BENCH_SWEEP_STEPS        6      // deployment 0.0, 0.2 .. 1.0
#define BENCH_SWEEP_HOLD_MS      4000
#define BENCH_SLEW_HOLD_MS       3000
#define BENCH_BEEP_MS            50

/* Simulated coast, starting near the OpenRocket PostProd-4 burnout state.
 * One-dimensional, v' = -g - k(d) v^2, with k chosen so the unbraked coast
 * tops out near OpenRocket's 3310 m. Full brakes roughly double the drag,
 * as the Cd table does at Mach 0.5. Crude on purpose: it only has to give
 * the controller a coast to fly, the real accuracy work is in the harness. */
#define BENCH_COAST_START_ALT_M   880.0f
#define BENCH_COAST_START_VEL_MPS 280.0f
#define BENCH_COAST_K_PER_M       0.00019f
#define BENCH_COAST_BRAKE_GAIN    0.89f
#define BENCH_G_MPS2              9.80665f
#define BENCH_COAST_PRINT_EVERY   25     // ticks, 0.5 s

static void beep(TIM_HandleTypeDef *buzzer, uint32_t on_ms)
{
    HAL_TIM_PWM_Start(buzzer, TIM_CHANNEL_4);
    HAL_Delay(on_ms);
    HAL_TIM_PWM_Stop(buzzer, TIM_CHANNEL_4);
}

static void servo_to(Motor_t *servo, float deployment)
{
    mtr_set_angle_deg(servo, ab_io_servo_angle_deg(deployment));
}

static uint32_t cycles_to_us(uint32_t cycles)
{
    return cycles / (SystemCoreClock / 1000000u);
}

static void bench_servo_sweep(Motor_t *servo)
{
    printf("\r\n[bench 1/3] servo sweep - measure the brake extension at each step\r\n");
    for (int i = 0; i < BENCH_SWEEP_STEPS; i++) {
        float deployment = (float)i / (float)(BENCH_SWEEP_STEPS - 1);
        servo_to(servo, deployment);
        printf("  deployment %.1f -> servo %.1f deg (should be %.0f mm)\r\n",
               deployment, ab_io_servo_angle_deg(deployment), deployment * 30.0f);
        HAL_Delay(BENCH_SWEEP_HOLD_MS);
    }
    servo_to(servo, 0.0f);
    HAL_Delay(BENCH_SLEW_HOLD_MS);
}

static void bench_slew_timing(Motor_t *servo, TIM_HandleTypeDef *buzzer)
{
    printf("\r\n[bench 2/3] slew timing - video it, each beep marks a command\r\n");

    servo_to(servo, 1.0f);
    printf("  t=%lu ms: OPEN commanded\r\n", (unsigned long)HAL_GetTick());
    beep(buzzer, BENCH_BEEP_MS);
    HAL_Delay(BENCH_SLEW_HOLD_MS);

    servo_to(servo, 0.0f);
    printf("  t=%lu ms: CLOSE commanded\r\n", (unsigned long)HAL_GetTick());
    beep(buzzer, BENCH_BEEP_MS);
    HAL_Delay(BENCH_SLEW_HOLD_MS);

    printf("  set AB_DEPLOY_RATE_PER_S = 1 / (seconds to open), "
           "AB_RETRACT_RATE_PER_S = 1 / (seconds to close) in config.h\r\n");
}

static void bench_simulated_coast(Motor_t *servo)
{
    printf("\r\n[bench 3/3] simulated coast - the airbrake controller flies it and drives the servo\r\n");

    const float dt_s = (float)MIN_CONTROL_LOOP_PERIOD_MS * 0.001f;
    float altitude_m = BENCH_COAST_START_ALT_M;
    float velocity_mps = BENCH_COAST_START_VEL_MPS;
    uint32_t tick_us_max = 0;
    uint32_t ticks = 0;
    AB_TickOutput out = {0};

    // real time, so the servo moves at the pace it would in flight
    uint32_t next_ms = HAL_GetTick();
    while (velocity_mps > 0.0f) {
        while ((int32_t)(HAL_GetTick() - next_ms) < 0) {
        }
        next_ms += MIN_CONTROL_LOOP_PERIOD_MS;

        AB_TickInput in = { altitude_m, velocity_mps, velocity_mps, dt_s, 0 };
        uint32_t start = DWT->CYCCNT;
        (void)ab_tick(&in, &out);
        uint32_t tick_us = cycles_to_us(DWT->CYCCNT - start);
        if (tick_us > tick_us_max) {
            tick_us_max = tick_us;
        }
        servo_to(servo, out.deployment_cmd);

        if (ticks % BENCH_COAST_PRINT_EVERY == 0) {
            printf("  t %4.1fs alt %6.1fm vel %5.1fm/s cmd %.3f predicted %6.1fm tick %luus mode %u\r\n",
                   (float)ticks * dt_s, altitude_m, velocity_mps, out.deployment_cmd,
                   out.predicted_apogee_agl_m, (unsigned long)tick_us, out.mode);
        }

        // the simulated brakes sit where the controller believes they are
        float drag_k = BENCH_COAST_K_PER_M * (1.0f + BENCH_COAST_BRAKE_GAIN * out.deployment_cmd);
        velocity_mps += (-BENCH_G_MPS2 - drag_k * velocity_mps * velocity_mps) * dt_s;
        altitude_m += velocity_mps * dt_s;
        ticks++;
    }

    printf("  apogee %.1f m AGL (target %.0f m, unbraked ~3310 m)\r\n",
           altitude_m, AB_TARGET_APOGEE_AGL_M);
    printf("  worst ab_tick %lu us of the %u ms loop, faults 0x%04x\r\n",
           (unsigned long)tick_us_max, (unsigned)MIN_CONTROL_LOOP_PERIOD_MS, out.fault_flags);

    // close through the rate limiter so the core's model of the servo stays
    // true, then clear the run so the real firmware starts fresh
    AB_TickInput closing = { altitude_m, velocity_mps, -velocity_mps, dt_s, 1 };
    while (ab_last_command() > 0.0f) {
        (void)ab_tick(&closing, &out);
        servo_to(servo, out.deployment_cmd);
        HAL_Delay(MIN_CONTROL_LOOP_PERIOD_MS);
    }
    ab_reset();
    beep(buzzer, BENCH_BEEP_MS);
}

void bench_boot_tests(Motor_t *servo, TIM_HandleTypeDef *buzzer)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    printf("\r\n*** BENCH BUILD - the airbrakes will move in %u s, keep clear ***\r\n",
           (unsigned)(BENCH_WARNING_MS / 1000u));

    bench_servo_sweep(servo);
    bench_slew_timing(servo, buzzer);
    bench_simulated_coast(servo);

    printf("\r\n[bench] tests done. Normal firmware from here: keep the board still for 2 s "
           "while the fusion calibrates. Telemetry prints every %u loops.\r\n\r\n",
           (unsigned)BENCH_PRINT_EVERY);
}

#else

/* ISO C forbids an empty translation unit */
typedef int bench_disabled_t;

#endif /* BENCH_BUILD */
