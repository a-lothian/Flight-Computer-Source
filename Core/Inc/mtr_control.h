/*
 * mtr_control.h
 *
 *  Created on: Sep 25, 2026
 *      Author: alexlothian
 *
 * PWM motor/servo control. Converts a pulse width in microseconds into a
 * timer compare value, using the timer's actual clock and prescaler, so
 * commands stay correct if the system clock or timer setup in the .ioc
 * changes.
 */

#ifndef INC_MTR_CONTROL_H_
#define INC_MTR_CONTROL_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           channel;   /* TIM_CHANNEL_1 .. TIM_CHANNEL_4 */
    uint32_t           tick_hz;   /* timer counter frequency, set by mtr_init */
    uint16_t           pulse_us;  /* last pulse actually applied (after clamping) */
} Motor_t;

/* Starts PWM on the given timer channel at MOTOR_STARTUP_ANGLE_DEG.
 * Call after the CubeMX MX_TIMx_Init(). */
HAL_StatusTypeDef mtr_init(Motor_t *mtr, TIM_HandleTypeDef *htim, uint32_t channel);

/* Converts a pulse width to a compare value for this motor's timer. */
uint32_t mtr_pulse_us_to_compare(const Motor_t *mtr, uint16_t pulse_us);

/* Sets the pulse width, clamped to the pulses for MOTOR_ANGLE_MIN_DEG ..
 * MOTOR_ANGLE_MAX_DEG. */
void mtr_set_pulse_us(Motor_t *mtr, uint16_t pulse_us);

/* Sets the servo angle in degrees (0 .. MOTOR_RANGE_DEG), clamped to
 * MOTOR_ANGLE_MIN_DEG .. MOTOR_ANGLE_MAX_DEG. */
void mtr_set_angle_deg(Motor_t *mtr, float angle_deg);

#endif /* INC_MTR_CONTROL_H_ */
