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
#include "esp_err.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif


 class MotorController
 {
 private:
    uint64_t millis_stop;
    uint8_t speed;
 public:
     MotorController();
     void Forward(uint8_t speed, uint32_t delay_ms = 0);
     void Backward(uint8_t speed, uint32_t delay_ms = 0);
     void Left(uint8_t speed, uint32_t delay_ms = 0);
     void Right(uint8_t speed, uint32_t delay_ms = 0);
     void Stop(uint32_t delay_ms = 0);    
 };


extern MotorController motors;
esp_err_t start_motors(void);


#ifdef __cplusplus
}
#endif
