/* Copyright (c) 2020 Andrei Gramakov. All rights reserved.
 *
 * This file is licensed under the terms of the MIT license.
 * For a copy, see: https://opensource.org/licenses/MIT
 *
 * site:    https://agramakov.me
 * e-mail:  mail@agramakov.me
 */

#include "indication.hpp"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

#include "common.h"
#include "sdkconfig.h"

static const char* TAG = "indication";

esp_err_t led_green()
{
    ESP_RETURN_ERROR(gpio_set_level((gpio_num_t)PIN_RED, 0));
    ESP_RETURN_ERROR(gpio_set_level((gpio_num_t)PIN_GREEN, 1));
    return ESP_OK;
}

esp_err_t led_red()
{
    ESP_RETURN_ERROR(gpio_set_level((gpio_num_t)PIN_GREEN, 0));
    ESP_RETURN_ERROR(gpio_set_level((gpio_num_t)PIN_RED, 1));
    return ESP_OK;
}

esp_err_t led_off()
{
    ESP_RETURN_ERROR(gpio_set_level((gpio_num_t)PIN_GREEN, 0));
    ESP_RETURN_ERROR(gpio_set_level((gpio_num_t)PIN_RED, 0));
    return ESP_OK;
}

esp_err_t init_indication()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE; // disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT; // set as output mode
    io_conf.pin_bit_mask = (1ULL << PIN_GREEN) | (1ULL << PIN_RED); // bit mask of the pins that you want to set, e.g.GPIO
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // disable pull-down mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE; // disable pull-up mode
    ESP_RETURN_ERROR(gpio_config(&io_conf)); //configure GPIO with the given settings

    ESP_RETURN_ERROR(gpio_set_level((gpio_num_t)PIN_GREEN, 1));
    vTaskDelay(400 / portTICK_RATE_MS);
    ESP_RETURN_ERROR(gpio_set_level((gpio_num_t)PIN_GREEN, 0));
    ESP_RETURN_ERROR(gpio_set_level((gpio_num_t)PIN_RED, 1));
    vTaskDelay(400 / portTICK_RATE_MS);
    ESP_RETURN_ERROR(gpio_set_level((gpio_num_t)PIN_RED, 0));
    vTaskDelay(400 / portTICK_RATE_MS);
    ESP_LOGI(TAG, "Indication is ready");
    return ESP_OK;
}
