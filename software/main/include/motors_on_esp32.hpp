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

#pragma once

#include <stdint.h>
#include "driver/mcpwm.h"


#define MCPWM_TIMER_LEFT MCPWM_TIMER_0
#define MCPWM_TIMER_RIGHT MCPWM_TIMER_1


class MotorsOnEsp32{
public:
    int pin_l1;
    int pin_l2;
    int pin_r1;
    int pin_r2;

    MotorsOnEsp32(int pin_left_a, int pin_left_b, int pin_right_a, int pin_right_b);
    void Init();
    void Forward(float speed_0_100);
    void Backward(float speed_0_100);
    void Left(float speed_0_100);
    void Right(float speed_0_100);
    void Stop();

protected:

private:
    void set_left_speed(int8_t speed);
    void set_right_speed(int8_t speed);
    static float CalcDuty(float in_val);
    static void SetGens(mcpwm_generator_t & gen_1, mcpwm_generator_t &gen_2, int8_t speed);

};

extern MotorsOnEsp32 motors_esp32;
