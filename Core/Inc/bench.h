/*
 * bench.h
 *
 * Bench tests, compiled in only when BENCH_BUILD is defined (the "Bench"
 * build configuration). Never fly a bench build: the airbrakes move at boot.
 *
 * At boot, after the airbrake core is configured and before the fusion
 * starts calibrating:
 *   1. servo sweep     deployment 0 .. 1 in steps, held so the brake
 *                      extension can be measured at each step
 *   2. slew timing     full open, then full close, with a beep at each
 *                      command so a video can time both moves
 *   3. simulated coast the real airbrake controller flies a simulated coast
 *                      and drives the servo, and reports its worst tick time
 * Then the normal firmware runs, and main.c prints telemetry every
 * BENCH_PRINT_EVERY loops.
 *
 * Output goes to printf (SWO): enable SWV in the debug configuration to see it.
 */

#ifndef INC_BENCH_H_
#define INC_BENCH_H_

#include "mtr_control.h"

/* print one telemetry record every this many loops (50 Hz / 10 = 5 Hz) */
#define BENCH_PRINT_EVERY 10

/* Runs tests 1-3. The airbrake core must already be configured. Leaves the
 * brakes closed and the core reset (ground use), ready for the normal run. */
void bench_boot_tests(Motor_t *servo, TIM_HandleTypeDef *buzzer);

#endif /* INC_BENCH_H_ */
