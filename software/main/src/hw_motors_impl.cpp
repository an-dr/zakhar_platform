/* Copyright (c) 2020 Andrei Gramakov. All rights reserved.
 *
 * This file is licensed under the terms of the MIT license.
 * For a copy, see: https://opensource.org/licenses/MIT
 *
 * site:    https://agramakov.me
 * e-mail:  mail@agramakov.me
 */

#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include "sdkconfig.h"
#include "registers.hpp"
#include "hw_motors_impl.hpp"
#include "common.h"

static const char* TAG = "motors";
// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

// TODO: pack
static uint32_t stop_ms = 0;
static uint32_t go_ms = 0;
TimerHandle_t timer;
bool stopped = true;
Motors_dc2platform_pnt_t action = NULL;
Motors_dc2platform motors(PIN_MOTOR_R1, PIN_MOTOR_R2, PIN_MOTOR_L1, PIN_MOTOR_L2);


void SetAction(Motors_dc2platform_pnt_t a){
    if (a == &Motors_dc2platform::Stop){
        timer_stop();
        SetStopped(true);
    } else{
        timer_start();
        SetStopped(false);
    }
    action = a;
}
void SetStopped(bool state){
    stopped = state;
}

bool IsStopped(void){
    return stopped;
}

esp_err_t timer_start_ms(size_t ms)
{
    if (xTimerChangePeriod(timer, (ms / portTICK_PERIOD_MS), (MOTORS_WAIT_TIMER_STOP_MS / portTICK_PERIOD_MS)) != pdPASS) {
        return ESP_FAIL;
    }
    if (xTimerStart(timer, 0) != pdPASS) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t timer_start(void)
{
    return timer_start_ms(MOTORS_TIMER_STEP_MS);
}

esp_err_t timer_stop(void)
{
    if (xTimerStop(timer, MOTORS_WAIT_TIMER_STOP_MS / portTICK_PERIOD_MS) != pdPASS) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

void init_write_pin(int pin)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE; // disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT; // set as output mode
    io_conf.pin_bit_mask = 1ULL << pin; // bit mask of the pins that you want to set, e.g.GPIO
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // disable pull-down mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE; // disable pull-up mode
    gpio_config(&io_conf); //configure GPIO with the given settings
}

void set_pin(int pin, bool value)
{
    gpio_set_level((gpio_num_t)pin, value);
    ESP_LOGV(TAG, "Pin no.%d set to: %d", pin, value);
}

void motors_delay(uint32_t ms)
{
    vTaskDelay(ms / portTICK_RATE_MS);
}

void SetSpeed(MotorsSpeed_t speed)
{
    int d = MOTORS_TIMER_STEP_MS/2;
    switch (speed) {
    case MOTORS_SPEED1:
        go_ms = d*2/4;
        stop_ms = d*2/4;
        break;
    case MOTORS_SPEED2:
        go_ms = d*3/4;
        stop_ms = d*1/4;
        break;
    case MOTORS_SPEEDMAX:
        go_ms = d*4/4;
        stop_ms = d*0/4;
        break;
    case MOTORS_STOP:
        action = NULL;
        go_ms = 0;
        stop_ms = 0;
        break;
    default:
        break;
    }
    REGW(REG_SPEED, (uint8_t)speed);
}



static void motors_task(void*)
{
    ESP_LOGI(TAG, "Task Started!");
    while (1) {
        if (xTimerIsTimerActive(timer)) {
            ESP_LOGD(TAG, "Timer is active");
            if (action) {
                if (go_ms) {
                    (motors.*action)(go_ms);
                    ESP_LOGD(TAG, "Go");
                }
                if (stop_ms) {
                    motors_delay(stop_ms);
                    ESP_LOGD(TAG, "Stop");
                }
            }
            else {
                ESP_LOGD(TAG, "No action");
            }
        } else { // time is up - full stop
            if (!stopped) {
                motors.Stop(0);
                stopped = true;
            }
        }
        vTaskDelay(1);
    }
}

void timer_stop_callback(TimerHandle_t xTimer)
{
    timer_stop();
}


/**
 * @brief motor moves in forward direction, with duty cycle = duty %
 */
static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

/**
 * @brief motor moves in backward direction, with duty cycle = duty %
 */
static void brushed_motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
}

/**
 * @brief motor stop
 */
static void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}

static void init_pwm()
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_MOTOR_R1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PIN_MOTOR_R2);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, PIN_MOTOR_L1);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, PIN_MOTOR_L2);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);
}


esp_err_t start_motors()
{
    motors.MoveForward(100);
    motors.Stop(150);
    motors.MoveLeft(150);
    motors.Stop(150);
    motors.MoveRight(150);
    // init_pwm();

    // brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 50);
    // vTaskDelay(1000 / portTICK_RATE_MS);

    // brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    // vTaskDelay(1000 / portTICK_RATE_MS);

    // brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 100);
    // vTaskDelay(1000 / portTICK_RATE_MS);

    timer = xTimerCreate("Timer", 1, pdTRUE, (void*)0, timer_stop_callback);
    xTaskCreatePinnedToCore(motors_task, "motors_task", 4096, NULL, 5, NULL, 1);
    SetSpeed(MOTORS_SPEEDMAX);
    return ESP_OK;
}
