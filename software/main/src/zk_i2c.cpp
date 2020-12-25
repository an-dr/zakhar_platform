/* Copyright (c) 2020 Andrei Gramakov. All rights reserved.
 *
 * This file is licensed under the terms of the MIT license.
 * For a copy, see: https://opensource.org/licenses/MIT
 *
 * site:    https://agramakov.me
 * e-mail:  mail@agramakov.me
 */

#include "zk_i2c.h"

#include <cstring>

#include "SharedVirtualRegisters.h"
#include "common.h"
// #include "driver/i2c.h"
#include "i2c_mod.h"
#include "esp_log.h"
#include "freertos/portmacro.h"
#include "projdefs.h"
#include "registers.hpp"

static const char* TAG = "zk_i2c";

static intr_handle_t intr_handle;       /*!< I2C interrupt handle*/

// static i2c_context_t i2c_context[I2C_NUM_MAX] = {
//     I2C_CONTEX_INIT_DEF(I2C_NUM_0),
//     I2C_CONTEX_INIT_DEF(I2C_NUM_1),
// };

// static void IRAM_ATTR zk_i2c_isr_handler(void *arg)
// {
//     i2c_obj_t *p_i2c = (i2c_obj_t *) arg;
//     int i2c_num = p_i2c->i2c_num;
//     i2c_intr_event_t evt_type = I2C_INTR_EVENT_ERR;
//     portBASE_TYPE HPTaskAwoken = pdFALSE;
//     if (p_i2c->mode == I2C_MODE_MASTER) {
//         if (p_i2c->status == I2C_STATUS_WRITE) {
//             i2c_hal_master_handle_tx_event(&(i2c_context[i2c_num].hal), &evt_type);
//         } else if (p_i2c->status == I2C_STATUS_READ) {
//             i2c_hal_master_handle_rx_event(&(i2c_context[i2c_num].hal), &evt_type);
//         }
//         if (evt_type == I2C_INTR_EVENT_NACK) {
//             p_i2c_obj[i2c_num]->status = I2C_STATUS_ACK_ERROR;
//             i2c_master_cmd_begin_static(i2c_num);
//         } else if (evt_type == I2C_INTR_EVENT_TOUT) {
//             p_i2c_obj[i2c_num]->status = I2C_STATUS_TIMEOUT;
//             i2c_master_cmd_begin_static(i2c_num);
//         } else if (evt_type == I2C_INTR_EVENT_ARBIT_LOST) {
//             p_i2c_obj[i2c_num]->status = I2C_STATUS_TIMEOUT;
//             i2c_master_cmd_begin_static(i2c_num);
//         } else if (evt_type == I2C_INTR_EVENT_END_DET) {
//             i2c_master_cmd_begin_static(i2c_num);
//         } else if (evt_type == I2C_INTR_EVENT_TRANS_DONE) {
//             if (p_i2c->status != I2C_STATUS_ACK_ERROR && p_i2c->status != I2C_STATUS_IDLE) {
//                 i2c_master_cmd_begin_static(i2c_num);
//             }
//         }
//         i2c_cmd_evt_t evt = {
//             .type = I2C_CMD_EVT_ALIVE
//         };
//         xQueueSendFromISR(p_i2c->cmd_evt_queue, &evt, &HPTaskAwoken);
//     } else {
//         i2c_hal_slave_handle_event(&(i2c_context[i2c_num].hal), &evt_type);
//         if (evt_type == I2C_INTR_EVENT_TRANS_DONE || evt_type == I2C_INTR_EVENT_RXFIFO_FULL) {
//             uint32_t rx_fifo_cnt;
//             i2c_hal_get_rxfifo_cnt(&(i2c_context[i2c_num].hal), &rx_fifo_cnt);
//             i2c_hal_read_rxfifo(&(i2c_context[i2c_num].hal), p_i2c->data_buf, rx_fifo_cnt);
//             xRingbufferSendFromISR(p_i2c->rx_ring_buf, p_i2c->data_buf, rx_fifo_cnt, &HPTaskAwoken);
//             i2c_hal_slave_clr_rx_it(&(i2c_context[i2c_num].hal));
//         } else if (evt_type == I2C_INTR_EVENT_TXFIFO_EMPTY) {
//             uint32_t tx_fifo_rem;
//             i2c_hal_get_txfifo_cnt(&(i2c_context[i2c_num].hal), &tx_fifo_rem);
//             size_t size = 0;
//             uint8_t *data = (uint8_t *) xRingbufferReceiveUpToFromISR(p_i2c->tx_ring_buf, &size, tx_fifo_rem);
//             if (data) {
//                 i2c_hal_write_txfifo(&(i2c_context[i2c_num].hal), data, size);
//                 vRingbufferReturnItemFromISR(p_i2c->tx_ring_buf, data, &HPTaskAwoken);
//             } else {
//                 i2c_hal_disable_slave_tx_it(&(i2c_context[i2c_num].hal));
//             }
//             i2c_hal_slave_clr_tx_it(&(i2c_context[i2c_num].hal));
//         }
//     }
//     //We only need to check here if there is a high-priority task needs to be switched.
//     if (HPTaskAwoken == pdTRUE) {
//         portYIELD_FROM_ISR();
//     }
// }

esp_err_t i2c_slave_init(gpio_num_t sda, gpio_num_t scl, uint8_t address)
{
    ESP_LOGI(TAG, "Slave init");
    i2c_config_t conf_slave;
    conf_slave.mode = I2C_MODE_SLAVE;
    conf_slave.sda_io_num = sda;
    conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.scl_io_num = scl;
    conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.slave.addr_10bit_en = 0;
    conf_slave.slave.slave_addr = address;
    ESP_RETURN_ERROR(i2c_param_config(I2C_NUM_1, &conf_slave));
    ESP_RETURN_ERROR(i2c_driver_install(I2C_NUM_1, conf_slave.mode,
        I2C_SLAVE_RX_BUF_LEN,
        I2C_SLAVE_TX_BUF_LEN, 0));
    // ESP_RETURN_ERROR(i2c_isr_register(
    //     I2C_NUM_1,
    //     zk_i2c_isr_handler,
    //     NULL,
    //     0,
    //     intr_handle )

    // )

    return ESP_OK;
}

esp_err_t i2c_master_init(gpio_num_t sda, gpio_num_t scl, uint32_t clock)
{
    ESP_LOGI(TAG, "Master init");
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda;
    conf.scl_io_num = scl;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = clock;
    ESP_RETURN_ERROR(i2c_param_config(I2C_NUM_0, &conf));
    ESP_RETURN_ERROR(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));
    return ESP_OK;
}

/**
 * @brief Updates tx fifo with a value from selected register
 *
 * @return esp_err_t
 */
static esp_err_t reg_tx_upd()
{
    // TODO: regs
    // uint8_t rd_val = regs.GetReg();
    ESP_RETURN_ERROR(i2c_reset_tx_fifo(I2C_NUM_1));
    // int res = i2c_slave_write_buffer(I2C_NUM_1, &rd_val, 1, 0);
    // if (!res) {
    // return ESP_FAIL;
    // }
    return ESP_OK;
}

static uint8_t reg_num_buf;
static uint8_t reg_data_buf;

static void i2c_reader_task(void*)
{

    int data_count;

    unsigned r_sz;
    const SVR_reg_t* r = SVR_get_regs(&regs,&r_sz);
    while (1) {
        data_count = 0;
        i2c_reset_tx_fifo(I2C_NUM_1);


        /* Reg num */
        if (i2c_slave_read_buffer(I2C_NUM_1, &reg_num_buf, 1, portMAX_DELAY) > 0) {
            i2c_slave_write_buffer(I2C_NUM_1, &r[reg_num_buf], 1, 0);
            ESP_LOGI(TAG, "Got regnum: 0x%x", reg_num_buf);
        }

        /* Data */
        while (1) {
            if (i2c_slave_read_buffer(I2C_NUM_1, &reg_data_buf, 1, 0) <= 0) {
                break;
            } else {
                data_count++;
                ESP_LOGI(TAG, "Got data: 0x%x", reg_data_buf);
            }
        }

        if (!data_count) { /* No data - write to I2C TX FIFO*/
            // i2c_reset_tx_fifo(I2C_NUM_1);
            // SVR_Get(&regs, reg_num_buf, &reg_data_buf, false, pdMS_TO_TICKS(1000));
            // i2c_slave_write_buffer(I2C_NUM_1, &reg_data_buf, 1, 0);
            // ESP_LOGI(TAG, "Set data to send: 0x%x", reg_data_buf);
        } else { /* Save data */
            SVR_Set(&regs, reg_num_buf, reg_data_buf, false, pdMS_TO_TICKS(1000));
        }
        i2c_reset_rx_fifo(I2C_NUM_1);

        vTaskDelay(1);
    }
}

/**
 * @brief Updates tx fifo with a single byte from the selected register to be
 * read by master
 */
static void i2c_writer_task(void*)
{
    unsigned r_sz;
    const SVR_reg_t* r = SVR_get_regs(&regs,&r_sz);
    while (1) {
        i2c_reset_tx_fifo(I2C_NUM_1);
        i2c_slave_write_buffer(I2C_NUM_1, &r[reg_num_buf], 1, 0);

        // res = reg_tx_upd();
        // if (res != ESP_OK) {
        // ESP_LOGE(TAG, "Can't load a value to the TX FIFO!");
        // }
        vTaskDelay(1);
    }
}

esp_err_t start_i2c_slave(void)
{
    ESP_RETURN_ERROR(i2c_slave_init(PIN_I2C_S_SDA, PIN_I2C_S_SCL, PLATFORM_I2C_ADDRESS));
    BaseType_t res = xTaskCreatePinnedToCore(i2c_reader_task, "i2c_reader_task", 1024 * 4, NULL, 5, NULL, 0);
    if (res != pdPASS) {
        return ESP_FAIL;
    }
    // res = xTaskCreatePinnedToCore(i2c_writer_task, "i2c_writer_task", 1024 * 4, NULL, 5, NULL, 0);
    // if (res != pdPASS) {
    //     return ESP_FAIL;
    // }
    ESP_LOGI(TAG, "I2C slave ready! Address: 0x%x", PLATFORM_I2C_ADDRESS);
    return ESP_OK;
}
