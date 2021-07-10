// *************************************************************************
//
// Copyright (c) 2021 Andrei Gramakov. All rights reserved.
//
// This file is licensed under the terms of the MIT license.
// For a copy, see: https://opensource.org/licenses/MIT
//
// site:    https://agramakov.me
// e-mail:  mail@agramakov.me
//
// *************************************************************************

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "esp_log.h"

#include "common.h"
#include "motors_on_esp32.hpp"

static const char* TAG = "motors-ll";

#define MIN_SPEED 20
#define SPEED_CONVERT_COEF 0.78
#define LEFT_SPEED_COEF 1
#define RIGHT_SPEED_COEF 1

MotorsOnEsp32::MotorsOnEsp32(int pin_left_a, int pin_left_b, int pin_right_a, int pin_right_b)
    : pin_l1(pin_left_a)
    , pin_l2(pin_left_b)
    , pin_r1(pin_right_a)
    , pin_r2(pin_right_b)

{
}

void MotorsOnEsp32::Init()
{
    mcpwm_pin_config_t pin_config = {
        .mcpwm0a_out_num = pin_l1,
        .mcpwm0b_out_num = pin_l2,
        .mcpwm1a_out_num = pin_r1,
        .mcpwm1b_out_num = pin_r2,
        .mcpwm2a_out_num = -1, //Not used
        .mcpwm2b_out_num = -1, //Not used
        .mcpwm_sync0_in_num = -1, //Not used
        .mcpwm_sync1_in_num = -1, //Not used
        .mcpwm_sync2_in_num = -1, //Not used
        .mcpwm_fault0_in_num = -1, //Not used
        .mcpwm_fault1_in_num = -1, //Not used
        .mcpwm_fault2_in_num = -1, //Not used
        .mcpwm_cap0_in_num = -1, //Not used
        .mcpwm_cap1_in_num = -1, //Not used
        .mcpwm_cap2_in_num = -1, //Not used
    };
    mcpwm_set_pin(MCPWM_UNIT_0, &pin_config);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 25; //frequency = 25Hz
    pwm_config.cmpr_a = 50; //duty cycle of PWMxA = 50.0%
    pwm_config.cmpr_b = 50; //duty cycle of PWMxb = 50.0%
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_LEFT, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_RIGHT, &pwm_config);

    set_left_speed(0);
    set_right_speed(0);
}

void MotorsOnEsp32::Stop(uint32_t delay_ms)
{
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_LEFT, MCPWM_GEN_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_LEFT, MCPWM_GEN_B, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_RIGHT, MCPWM_GEN_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_RIGHT, MCPWM_GEN_B, 0);
    DELAY_FREERTOS(delay_ms);
}

void MotorsOnEsp32::Forward(uint8_t speed, uint32_t delay_ms)
{
    set_left_speed(speed / 2);
    set_right_speed(speed / 2);
}

void MotorsOnEsp32::Backward(uint8_t speed, uint32_t delay_ms)
{
    set_left_speed(-speed / 2);
    set_right_speed(-speed / 2);
}

void MotorsOnEsp32::Left(uint8_t speed, uint32_t delay_ms)
{
    set_left_speed(-speed / 2);
    set_right_speed(speed / 2);
}

void MotorsOnEsp32::Right(uint8_t speed, uint32_t delay_ms)
{
    set_left_speed(speed / 2);
    set_right_speed(-speed / 2);
}

float MotorsOnEsp32::CalcDuty(float speed)
{
    float duty = abs(speed) * SPEED_CONVERT_COEF;
    if (duty >= 95) {
        duty = 100;
    }
    return duty;
}

void MotorsOnEsp32::SetGens(mcpwm_generator_t& gen_1, mcpwm_generator_t& gen_2, int8_t speed)
{
    if (speed >= 0) {
        gen_1 = MCPWM_GEN_A;
        gen_2 = MCPWM_GEN_B;
        ESP_LOGD(TAG, "Forward\n");
    } else {
        gen_1 = MCPWM_GEN_B;
        gen_2 = MCPWM_GEN_A;
        ESP_LOGD(TAG, "Backward\n");
    }
}

float MotorsOnEsp32::CalcSpeed(float speed, float k)
{
    if (speed == 0) {
        return 0;
    }
    bool sign = (speed >= 0) ? true : false;
    float speed_val = abs(speed) * k;
    bool too_low = (speed_val < MIN_SPEED) ? true : false;
    if (too_low) {
        return MIN_SPEED;
    } else {
        return sign ? speed_val : -speed_val;
    }
}

void MotorsOnEsp32::set_left_speed(int8_t speed)
{
    mcpwm_generator_t gen1, gen2;
    SetGens(gen1, gen2, speed);
    float speed_calc = CalcSpeed(speed, LEFT_SPEED_COEF);
    float duty = CalcDuty(speed_calc);
    ESP_LOGV(TAG, "Motor left. gen_%d to 0, gen_%d to %f\n", gen1, gen2, duty);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_LEFT, gen1, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_LEFT, gen2, duty);
}

void MotorsOnEsp32::set_right_speed(int8_t speed)
{
    mcpwm_generator_t gen1, gen2;
    SetGens(gen1, gen2, speed);
    float speed_calc = CalcSpeed(speed, RIGHT_SPEED_COEF);
    float duty = CalcDuty(speed_calc);
    ESP_LOGV(TAG, "Motor right. gen_%d to 0, gen_%d to %f\n", gen1, gen2, duty);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_RIGHT, gen1, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_RIGHT, gen2, duty);
}

MotorsOnEsp32 motors_esp32(PIN_MOTOR_L1, PIN_MOTOR_L2, PIN_MOTOR_R1, PIN_MOTOR_R2);
