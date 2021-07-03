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

#include <cstdlib>
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "common.h"
#include "motors_on_esp32.hpp"


#define MIN_SPEED 20
#define SPEED_CONVERT_COEF 0.78
#define LEFT_SPEED_COEF 1
#define RIGHT_SPEED_COEF 1

MotorsOnEsp32::MotorsOnEsp32(int pin_left_a, int pin_left_b, int pin_right_a, int pin_right_b)
{
    pin_l1 = pin_left_a;
    pin_l2 = pin_left_b;
    pin_r1 = pin_right_a;
    pin_r2 = pin_right_b;


}

void MotorsOnEsp32::Init()
{
    mcpwm_pin_config_t pin_config = {
        .mcpwm0a_out_num = pin_l1,
        .mcpwm0b_out_num = pin_l2,
        .mcpwm1a_out_num = pin_r1,
        .mcpwm1b_out_num = pin_r2,
        .mcpwm2a_out_num = -1,  //Not used
        .mcpwm2b_out_num = -1,  //Not used
        .mcpwm_sync0_in_num  = -1,  //Not used
        .mcpwm_sync1_in_num  = -1,  //Not used
        .mcpwm_sync2_in_num  = -1,  //Not used
        .mcpwm_fault0_in_num = -1,  //Not used
        .mcpwm_fault1_in_num = -1,  //Not used
        .mcpwm_fault2_in_num = -1,   //Not used
        .mcpwm_cap0_in_num   = -1,  //Not used
        .mcpwm_cap1_in_num   = -1,  //Not used
        .mcpwm_cap2_in_num   = -1,  //Not used
    };
    mcpwm_set_pin(MCPWM_UNIT_0, &pin_config);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 100;    //frequency = 1000Hz
    pwm_config.cmpr_a = 50;    //duty cycle of PWMxA = 50.0%
    pwm_config.cmpr_b = 50;    //duty cycle of PWMxb = 50.0%
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_LEFT, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_RIGHT, &pwm_config);    //Configure PWM1A & PWM1B with above settings
    // mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);    //Configure PWM2A & PWM2B with above settings

    printf(">Left\n");
    set_left_speed(127);
    printf(">Right\n");
    set_right_speed(127);

    // mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_LEFT, MCPWM_GEN_B);
    // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_LEFT, MCPWM_GEN_A, 20); 

    // // mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_RIGHT,50);
    // mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_RIGHT, MCPWM_GEN_B);
    // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_RIGHT, MCPWM_GEN_A, 10.0); 
}

void MotorsOnEsp32::Stop()
{
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_LEFT, MCPWM_GEN_A);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_LEFT, MCPWM_GEN_B);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_RIGHT, MCPWM_GEN_A);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_RIGHT, MCPWM_GEN_B);
}

void MotorsOnEsp32::Forward(float speed_0_100)
{
    // left - Forward
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_LEFT, MCPWM_GEN_B);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_LEFT, MCPWM_GEN_A, speed_0_100*LEFT_SPEED_COEF);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_LEFT, MCPWM_GEN_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    // right - Forward
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_RIGHT, MCPWM_GEN_B);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_RIGHT, MCPWM_GEN_A, speed_0_100*RIGHT_SPEED_COEF);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_RIGHT, MCPWM_GEN_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}


void MotorsOnEsp32::Backward(float speed_0_100)
{
    // left - Backward
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_LEFT, MCPWM_GEN_A);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_LEFT, MCPWM_GEN_B, speed_0_100*LEFT_SPEED_COEF);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_LEFT, MCPWM_GEN_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    // right - Backward
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_RIGHT, MCPWM_GEN_A);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_RIGHT, MCPWM_GEN_B, speed_0_100*RIGHT_SPEED_COEF);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_RIGHT, MCPWM_GEN_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

void MotorsOnEsp32::Left(float speed_0_100)
{
    // left - Backward
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_LEFT, MCPWM_GEN_A);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_LEFT, MCPWM_GEN_B, speed_0_100*LEFT_SPEED_COEF);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_LEFT, MCPWM_GEN_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    // right - Forward
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_RIGHT, MCPWM_GEN_B);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_RIGHT, MCPWM_GEN_A, speed_0_100*RIGHT_SPEED_COEF);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_RIGHT, MCPWM_GEN_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

void MotorsOnEsp32::Right(float speed_0_100)
{
    // left - Forward
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_LEFT, MCPWM_GEN_B);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_LEFT, MCPWM_GEN_A, speed_0_100*LEFT_SPEED_COEF);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_LEFT, MCPWM_GEN_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    // right - Backward
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_RIGHT, MCPWM_GEN_A);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_RIGHT, MCPWM_GEN_B, speed_0_100*RIGHT_SPEED_COEF);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_RIGHT, MCPWM_GEN_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

float MotorsOnEsp32::CalcDuty(float speed){
    float duty = abs(speed) * SPEED_CONVERT_COEF;
    if (duty >= 95){
        duty = 100;
    }
    printf("Duty: %f\n", duty);

    return duty;
}

void MotorsOnEsp32::SetGens(mcpwm_generator_t & gen_1, mcpwm_generator_t &gen_2, int8_t speed){
    if(speed >= 0){
        gen_1 = MCPWM_GEN_A;
        gen_2 = MCPWM_GEN_B;
        printf("Forward\n");

    }
    else{
        gen_1 = MCPWM_GEN_B;
        gen_2 = MCPWM_GEN_A;
        printf("Backward\n");
    }
}

void MotorsOnEsp32::set_left_speed(int8_t speed){
    mcpwm_generator_t gen1, gen2;
    SetGens(gen1, gen2, speed);
    float duty_l = CalcDuty(speed*LEFT_SPEED_COEF);
    printf("Set left. gen_%d to 0, gen_%d to %f\n", gen1, gen2, duty_l);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_LEFT, gen1);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_LEFT, gen2, duty_l);
}

void MotorsOnEsp32::set_right_speed(int8_t speed){
    mcpwm_generator_t gen1, gen2;
    SetGens(gen1, gen2, speed);
    float duty_r = CalcDuty(speed*RIGHT_SPEED_COEF);
    printf("Set right. gen_%d to 0, gen_%d to %f\n", gen1, gen2, duty_r);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_RIGHT, gen1);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_RIGHT, gen2, duty_r);
}


MotorsOnEsp32 motors_esp32(PIN_MOTOR_L1, PIN_MOTOR_L2, PIN_MOTOR_R1, PIN_MOTOR_R2);
