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

#include "SharedVirtualRegisters.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "projdefs.h"
#include "registers.hpp"
#include <stdio.h>

static const char* TAG = "serial";

static void serial_task(void*)
{
    // Configure a temporary buffer for the incoming data
    uint8_t ch;
    while (1) {
        ch = fgetc(stdin);
        if (ch != 0xFF) {
            SVR_Set(&regs,REG_CMD,ch,false,pdMS_TO_TICKS(1000));
            ESP_LOGD(TAG, "Got: %c", ch);
        }
        vTaskDelay(1);
    }
}

esp_err_t start_serial(void)
{
    BaseType_t res = xTaskCreatePinnedToCore(&serial_task, "serial_task", 4096, NULL, 5, NULL, 0);
    return (res == pdTRUE ? ESP_OK : ESP_FAIL);
}
