/*
 * mtr_control.c
 *
 *  Created on: Sep 25, 2026
 *      Author: alexlothian
 */

#include "mtr_control.h"
#include "config.h"

#if MOTOR_PULSE_MIN_US >= MOTOR_PULSE_MAX_US
#error "MOTOR_PULSE_MIN_US must be less than MOTOR_PULSE_MAX_US"
#endif

/* The angle limits are floats, which #if can't compare, so use static asserts. */
_Static_assert(MOTOR_ANGLE_MIN_DEG >= 0.0f && MOTOR_ANGLE_MAX_DEG <= MOTOR_RANGE_DEG,
               "Motor angle limits must be within 0 .. MOTOR_RANGE_DEG");
_Static_assert(MOTOR_ANGLE_MIN_DEG < MOTOR_ANGLE_MAX_DEG,
               "MOTOR_ANGLE_MIN_DEG must be less than MOTOR_ANGLE_MAX_DEG");
_Static_assert(MOTOR_STARTUP_ANGLE_DEG >= MOTOR_ANGLE_MIN_DEG &&
               MOTOR_STARTUP_ANGLE_DEG <= MOTOR_ANGLE_MAX_DEG,
               "MOTOR_STARTUP_ANGLE_DEG must be within the angle limits");

/* Servo angle -> pulse width, using the servo's full calibrated range. */
static uint16_t mtr_angle_to_pulse_us(float angle_deg)
{
    float span_us = (float)(MOTOR_PULSE_MAX_US - MOTOR_PULSE_MIN_US);
    return (uint16_t)(MOTOR_PULSE_MIN_US + (angle_deg / MOTOR_RANGE_DEG) * span_us + 0.5f);
}

/* Clock feeding the timer's counter, before its own prescaler. On the F4,
 * timers get 2x their APB bus clock whenever that bus is divided down
 * (RM0390 section 6.2, assuming TIMPRE = 0). TIM1/8/9/10/11 sit on APB2,
 * the rest on APB1. */
static uint32_t mtr_timer_clock_hz(const TIM_TypeDef *instance)
{
    uint32_t pclk;
    uint32_t apb_divided;

    if (instance == TIM1 || instance == TIM8 || instance == TIM9 ||
        instance == TIM10 || instance == TIM11) {
        pclk = HAL_RCC_GetPCLK2Freq();
        apb_divided = (RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1;
    } else {
        pclk = HAL_RCC_GetPCLK1Freq();
        apb_divided = (RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1;
    }

    return apb_divided ? pclk * 2 : pclk;
}

HAL_StatusTypeDef mtr_init(Motor_t *mtr, TIM_HandleTypeDef *htim, uint32_t channel)
{
    mtr->htim = htim;
    mtr->channel = channel;
    mtr->tick_hz = mtr_timer_clock_hz(htim->Instance) / (htim->Init.Prescaler + 1);

    /* Load the startup pulse before enabling the output, so the first PWM
     * period is already the safe value rather than whatever CubeMX set. */
    mtr_set_angle_deg(mtr, MOTOR_STARTUP_ANGLE_DEG);
    return HAL_TIM_PWM_Start(htim, channel);
}

uint32_t mtr_pulse_us_to_compare(const Motor_t *mtr, uint16_t pulse_us)
{
    /* 64-bit so fast timer clocks can't overflow the multiply. */
    return (uint32_t)(((uint64_t)pulse_us * mtr->tick_hz) / 1000000u);
}

void mtr_set_pulse_us(Motor_t *mtr, uint16_t pulse_us)
{
    /* Clamp to the angle limits, not the servo's full range, so raw pulse
     * commands can't drive the airbrakes past their end stops either. */
    uint16_t min_us = mtr_angle_to_pulse_us(MOTOR_ANGLE_MIN_DEG);
    uint16_t max_us = mtr_angle_to_pulse_us(MOTOR_ANGLE_MAX_DEG);

    if (pulse_us < min_us) {
        pulse_us = min_us;
    } else if (pulse_us > max_us) {
        pulse_us = max_us;
    }

    mtr->pulse_us = pulse_us;
    __HAL_TIM_SET_COMPARE(mtr->htim, mtr->channel, mtr_pulse_us_to_compare(mtr, pulse_us));
}

void mtr_set_angle_deg(Motor_t *mtr, float angle_deg)
{
    if (angle_deg < MOTOR_ANGLE_MIN_DEG) {
        angle_deg = MOTOR_ANGLE_MIN_DEG;
    } else if (angle_deg > MOTOR_ANGLE_MAX_DEG) {
        angle_deg = MOTOR_ANGLE_MAX_DEG;
    }

    mtr_set_pulse_us(mtr, mtr_angle_to_pulse_us(angle_deg));
}
