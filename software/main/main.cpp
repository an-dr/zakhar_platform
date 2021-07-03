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

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

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


#if 1

#if MPU_ENABLED
    /* Each MPU angle is represented as [SIGN : INT_VALUE]
       Values for Z-rotation (parallel to floor):
       [ 0 : 0...180] rotation clockwise for
       [ 1 : 0...179] rotation counter-clockwise
    */
    // CHECK_LOAD_STAGE(start_mpu(), "MPU");
#endif

    // CHECK_LOAD_STAGE(start_i2c_slave(), "I2C");
    // CHECK_LOAD_STAGE(start_motors(), "Motors");
    CHECK_LOAD_STAGE(start_serial(), "Serial control");
    // CHECK_LOAD_STAGE(start_control(), "Control system");
    // CHECK_LOAD_STAGE(start_bt_serial(), "Bluetooth");

#endif

#if 1
    motors_esp32.Init();

    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_MOTOR_L1);
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PIN_MOTOR_L2);
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PIN_MOTOR_R1);
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, PIN_MOTOR_R2);



    // mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    // mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    // mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B);
    // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, 100.0);
    // mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    // right - Forward

    // motors_esp32.Forward(100.0);
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    // motors_esp32.Stop();
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    // motors_esp32.Backward(75.0);
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    // motors_esp32.Left(60.0);
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
#endif



    if (successfull_boot) {
        led_green(); // boot is done with no errors
    }
    ESP_LOGI(TAG, "Init done %s", (successfull_boot ? "successfully" : "with errors"));


    // logging_loop();

    while (1) {
        vTaskDelay(1);
    }
}
