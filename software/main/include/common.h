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

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Peripherals and sw configs
   ********************************************************************* */

#define DEBUG 1
#define PRINT_REGS 1
#define PRINT_ANGLES 0

// i2c
#define PLATFORM_I2C_ADDRESS 0x2a
#define I2C_COMMAND_DEFINE_PERIOD_MS 1
#define DATA_LENGTH 512 /*!< Data buffer length of test buffer */
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH) /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH) /*!< I2C slave rx buffer size */


#define MPU_ENABLED 1
#define MPU_SAMPLE_RATE_HZ 100
#define MPU_I2C_CLOCK_FREQ_HZ 100000
#define MPU_INIT_CALIBRATION_DELAY_MS 3000

#define MOTORS_SHIVER_PERIOD_MS 100
#define MOTORS_TIMER_STEP_MS 100
#define MOTORS_SHIVERS 10
#define MOTORS_WAIT_TIMER_STOP_MS 1000

/*********************************************************************** Peripherals and sw configs*/
/* Macros
   ********************************************************************* */
#define SIZE_ARR(x) (sizeof(x) / sizeof(x[0]))

#define ESP_RETURN_ERROR(x) \
    do {                           \
        esp_err_t res = (x);       \
        if (res != ESP_OK)         \
            return res;            \
    } while (0)


#define ESTR(x) (esp_err_to_name(x))

#define DELAY_FREERTOS(ms) (vTaskDelay(ms/portTICK_PERIOD_MS))

/*********************************************************************** Macros*/
/* Registers
   ********************************************************************* */

enum reristers {
    REG_CMD = 0x00,
    REG_ARG,
    REG_MODE,
    REG_SPEED,
    REG_ANGLE_X_SIGN,
    REG_ANGLE_X,
    REG_ANGLE_Y_SIGN,
    REG_ANGLE_Y,
    REG_ANGLE_Z_SIGN,
    REG_ANGLE_Z,
    REGS_AMOUNT
};

/*********************************************************************** Registers*/
/* Pinout
   ********************************************************************* */
#define PIN_I2C_S_SDA GPIO_NUM_18
#define PIN_I2C_S_SCL GPIO_NUM_33

#define PIN_MPU_I2C_SDA GPIO_NUM_27
#define PIN_MPU_I2C_SCL GPIO_NUM_25

#define PIN_GREEN GPIO_NUM_21
#define PIN_RED GPIO_NUM_22

#define PIN_MOTOR_R1 GPIO_NUM_26
#define PIN_MOTOR_R2 GPIO_NUM_5
#define PIN_MOTOR_L1 GPIO_NUM_19
#define PIN_MOTOR_L2 GPIO_NUM_23

/*********************************************************************** Pinout*/
/* Commands
   ********************************************************************* */

#define CMD_NONE (-1U)
#define CMD_DONE (0)

#define CMD_FORWARD 119
#define CMD_BACKWARD 115
#define CMD_LEFT 97
#define CMD_RIGHT 100
#define CMD_SHIVER 0x71
#define CMD_STOP_KB 0x20
#define CMD_STOP 0xA0

#define CMD_SPEED0 0x30
#define CMD_SPEED1 0x31
#define CMD_SPEED2 0x32
#define CMD_SPEED3 0x33

#define CMD_MPU_CALIBRATE 99

#define CMD_TEST 0x74

/*********************************************************************** Commands*/
