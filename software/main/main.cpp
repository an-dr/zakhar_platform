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
#include "zk_i2c.h"
#if MPU_ENABLED
#include "position_unit.h"
#endif
#include "SharedVirtualRegisters.h"
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

extern "C" void app_main()
{
    ESP_LOGI(TAG, "Start!");
    bool successfull_boot = true;

    CHECK_LOAD_STAGE(regs_init(), "Registers");
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
    CHECK_LOAD_STAGE(start_serial(), "Serial control");
    CHECK_LOAD_STAGE(start_control(), "Control system");
    CHECK_LOAD_STAGE(start_motors(), "Motors");
    // CHECK_LOAD_STAGE(start_bt_serial(), "Bluetooth");
    if (successfull_boot) {
        led_green(); // boot is done with no errors
    }
    ESP_LOGI(TAG, "Init done %s", (successfull_boot ? "successfully" : "with errors"));

#if PRINT_REGS
    SVR_reg_t buf[REGS_AMOUNT] = {0};
    while (1) {
        vTaskDelay(100 / portTICK_RATE_MS);
        SVR_Dump(&regs, 0, REGS_AMOUNT, buf, false, pdTICKS_TO_MS(1000));
        ESP_LOGI(TAG, "regs: [ 0x%x\t0x%x\t0x%x\t0x%x\t0x%x\t0x%x\t0x%x\t0x%x\t0x%x\t0x%x ]",
            buf[REG_CMD],
            buf[REG_ARG],
            buf[REG_MODE],
            buf[REG_SPEED],
            buf[REG_ANGLE_X_SIGN],
            buf[REG_ANGLE_X],
            buf[REG_ANGLE_Y_SIGN],
            buf[REG_ANGLE_Y],
            buf[REG_ANGLE_Z_SIGN],
            buf[REG_ANGLE_Z]
            );
    }
#endif
    while (1) {
        vTaskDelay(1);
    }
}
