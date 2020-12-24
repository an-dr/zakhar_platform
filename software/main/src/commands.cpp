/* Copyright (c) 2020 Andrei Gramakov. All rights reserved.
 *
 * This file is licensed under the terms of the MIT license.
 * For a copy, see: https://opensource.org/licenses/MIT
 *
 * site:    https://agramakov.me
 * e-mail:  mail@agramakov.me
 */

#include "commands.hpp"
#include "common.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "hw_motors_impl.hpp"
#include "position_unit.h"
#include "registers.hpp"

static const char *TAG = "Commands";

void S0(void)
{
    ESP_LOGI(TAG, "%s",__func__);
    SetSpeed(MOTORS_STOP);
}

void S1(void)
{
    ESP_LOGI(TAG, "%s",__func__);
    SetSpeed(MOTORS_SPEED1);
}

void S2(void)
{
    ESP_LOGI(TAG, "%s",__func__);
    SetSpeed(MOTORS_SPEED2);
}

void SMAX(void)
{
    ESP_LOGI(TAG, "%s",__func__);
    SetSpeed(MOTORS_SPEEDMAX);
}

void W(void)
{
    ESP_LOGI(TAG, "%s",__func__);
    SetAction(&Motors_dc2platform::MoveForward);
    timer_start();
}

void S(void)
{
    ESP_LOGI(TAG, "%s",__func__);
    SetAction(&Motors_dc2platform::MoveBackward);
    timer_start();
}


#if MPU_ENABLED
static void precise_position(uint8_t current_angle, uint8_t target_angle, bool current_sign)
{
    int delta = abs(current_angle - target_angle);
    ets_printf("Delta: %d\n", delta);
    if (delta > 45) {
        SMAX();
    } else if (delta > 10) {
        S2();
    } else {
        S1();
    }
    if (current_sign) // negative angle
    {
        if (current_angle < target_angle)
            SetAction(&Motors_dc2platform::MoveLeft);
        if (current_angle > target_angle)
            SetAction(&Motors_dc2platform::MoveRight);
    } else { // positive angle
        if (current_angle < target_angle)
            SetAction(&Motors_dc2platform::MoveRight);
        if (current_angle > target_angle)
            SetAction(&Motors_dc2platform::MoveLeft);
    }
}

static void rough_position(Motors_dc2platform_pnt_t rot_direction)
{
    SMAX();
    SetAction(rot_direction);
}

static void set_position_do(Motors_dc2platform_pnt_t rot_direction, bool target_sign, uint8_t target_angle)
{
    // TODO: regs
    // uint8_t current_angle = REGR(REG_ANGLE_Z);
    // uint8_t current_sign = REGR(REG_ANGLE_Z_SIGN);
    // while ((current_sign != target_sign) || (current_angle != target_angle)) {
    //     current_angle = REGR(REG_ANGLE_Z);
    //     current_sign = REGR(REG_ANGLE_Z_SIGN);
    //     if (current_sign == target_sign) { // right sign
    //         precise_position(current_angle, target_angle, current_sign);
    //     } else { // wait the right sign
    //         rough_position(rot_direction);
    //     }
    //     vTaskDelay(1);
    // }
    Stop();
}

static void set_position(Motors_dc2platform_pnt_t rot_direction, int delta_angle)
{
    uint8_t target_angle;
    uint8_t target_sign;
    if (rot_direction == &Motors_dc2platform::MoveLeft) {
        delta_angle = -delta_angle;
    }
    convert_angle_2_sign_and_angle(delta_angle, &target_sign, &target_angle);
    mpu_centralize_angles();
    ets_printf("Target: 0x%x:0x%x\n", target_sign, target_angle);
    set_position_do(rot_direction, (bool)target_sign, target_angle);
}
#endif

void A(void)
{
    ESP_LOGI(TAG, "%s",__func__);
    Motors_dc2platform_pnt_t action = &Motors_dc2platform::MoveLeft;
    SetAction(action);
#if MPU_ENABLED
    // TODO: regs
    // if (regs_get_arg()) {
        // set_position(action, regs_get_arg());
    // }
#endif
}

void D(void)
{
    ESP_LOGI(TAG, "%s",__func__);
    Motors_dc2platform_pnt_t action = &Motors_dc2platform::MoveRight;
    SetAction(action);
#if MPU_ENABLED
    // TODO: regs
    // if (regs_get_arg()) {
        // set_position(action, regs_get_arg());
    // }
#endif
}

void Shiver(void)
{
    ESP_LOGI(TAG, "%s",__func__);
    for (size_t i = 0; i < MOTORS_SHIVERS; i++) {
        motors.MoveRight(MOTORS_SHIVER_PERIOD_MS / 2);
        motors.MoveLeft(MOTORS_SHIVER_PERIOD_MS / 2);
    }
    Stop();
}

void Stop(void)
{
    ESP_LOGI(TAG, "%s",__func__);
    SetAction(&Motors_dc2platform::Stop);
    timer_stop();
}