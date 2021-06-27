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

#define LEFT_SPEED_COEF 1
#define RIGHT_SPEED_COEF 1


class MotorsOnEsp32{
public:
    int pin_l1;
    int pin_l2;
    int pin_r1;
    int pin_r2;

    MotorsOnEsp32(int pin_left_a, int pin_left_b, int pin_right_a, int pin_right_b);
    void Forward(float speed_0_100);
    void Backward(float speed_0_100);
    void Left(float speed_0_100);
    void Right(float speed_0_100);
    void Stop();

protected:

private:


};

extern MotorsOnEsp32 motors_e32;
