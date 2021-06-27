// *************************************************************************
//
// Copyright (c) 2020 Andrei Gramakov. All rights reserved.
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

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

#include "common.h"
#include "hw_motors_impl.hpp"
#include "motors_on_esp32.hpp"
#include "zk_i2c.h"
#if MPU_ENABLED
#include "position_unit.h"
#endif
#include "SharedVirtualRegisters.hpp"
#include "bluetooth_serial.hpp"
#include "controlcallback.h"
#include "indication.hpp"
#include "registers.hpp"
#include "serial.h"
#include "sdkconfig.h"

static const char* TAG = "main";

#define CHECK_LOAD_STAGE(func, unit_name)                    \
    do {                                                     \
        esp_err_t res = (func);                              \
        if (res != ESP_OK) {                                 \
            successfull_boot = false;                        \
            ESP_LOGE(TAG, "Can't initialize %s", unit_name); \
        }                                                    \
    } while (0)


static void logging_loop(){
    #if PRINT_REGS
        while (1) {
            vTaskDelay(100 / portTICK_RATE_MS);
            ESP_LOGI(TAG, "regs: [ 0x%x\t0x%x\t0x%x\t0x%x\t0x%x\t0x%x\t0x%x\t0x%x\t0x%x ]",
                REGR(REG_CMD),
                REGR(REG_ARG),
                REGR(REG_SPEED),
                REGR(REG_ANGLE_X_SIGN),
                REGR(REG_ANGLE_X),
                REGR(REG_ANGLE_Y_SIGN),
                REGR(REG_ANGLE_Y),
                REGR(REG_ANGLE_Z_SIGN),
                REGR(REG_ANGLE_Z));
        }
    #endif
}

extern "C" void app_main()
{
    ESP_LOGI(TAG, "Start!");
    esp_err_t res;
    bool successfull_boot = true;
    CHECK_LOAD_STAGE(init_indication(), "Indication");
    led_red(); // boot is started

#if MPU_ENABLED
    /* Each MPU angle is represented as [SIGN : INT_VALUE]
       Values for Z-rotation (parallel to floor):
       [ 0 : 0...180] rotation clockwise for
       [ 1 : 0...179] rotation counter-clockwise
    */
    CHECK_LOAD_STAGE(start_mpu(), "MPU");
#endif

    CHECK_LOAD_STAGE(start_i2c_slave(), "I2C");
    // CHECK_LOAD_STAGE(start_motors(), "Motors");
    CHECK_LOAD_STAGE(start_serial(), "Serial control");
    CHECK_LOAD_STAGE(start_control(), "Control system");
    CHECK_LOAD_STAGE(start_bt_serial(), "Bluetooth");

#if 1
    motors_e32.Forward(50.0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    motors_e32.Stop();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    motors_e32.Backward(10.0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    motors_e32.Left(25.0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
#endif
    if (successfull_boot) {
        led_green(); // boot is done with no errors
    }
    ESP_LOGI(TAG, "Init done %s", (successfull_boot ? "successfully" : "with errors"));


    logging_loop();

    while (1) {
        vTaskDelay(1);
    }
}
