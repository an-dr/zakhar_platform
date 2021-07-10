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

#include "common.h"
#include "motor_controller.hpp"
#include "motors_on_esp32.hpp"



esp_err_t start_motors(void)
{
    motors_esp32.Init();

#if 1

    motors_esp32.Forward(255);
    DELAY_FREERTOS(300);
    motors_esp32.Stop(100);

    motors_esp32.Left(120);
    DELAY_FREERTOS(300);
    motors_esp32.Stop(100);
 
    motors_esp32.Right(120);
    DELAY_FREERTOS(300);
    motors_esp32.Stop();

#else // For demo purposes
    motors_esp32.set_left_speed(-20);
    motors_esp32.set_right_speed(127);
#endif
    return ESP_OK;
}
