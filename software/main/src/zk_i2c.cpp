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

#include "common.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "registers.hpp"

static const char* TAG = "zk_i2c";

// static intr_handle_t intr_handle;       /*!< I2C interrupt handle*/

// static i2c_context_t i2c_context[I2C_NUM_MAX] = {
//     I2C_CONTEX_INIT_DEF(I2C_NUM_0),
//     I2C_CONTEX_INIT_DEF(I2C_NUM_1),
// };

// TODO: rewrite the handler
// void zk_i2c_isr_handler(){
//     i2c_intr_event_t evt_type = I2C_INTR_EVENT_ERR;
//     i2c_hal_slave_handle_event(&(i2c_context[I2C_NUM_1].hal), &evt_type);
//     if (evt_type == I2C_INTR_EVENT_TRANS_DONE || evt_type ==
//     I2C_INTR_EVENT_RXFIFO_FULL) {
//         // new transmission
//         // uint32_t rx_fifo_cnt;
//         // i2c_hal_get_rxfifo_cnt(&(i2c_context[i2c_num].hal),
//         &rx_fifo_cnt);
//         // i2c_hal_read_rxfifo(&(i2c_context[i2c_num].hal), p_i2c->data_buf,
//         rx_fifo_cnt);
//         // xRingbufferSendFromISR(p_i2c->rx_ring_buf, p_i2c->data_buf,
//         rx_fifo_cnt, &HPTaskAwoken);
//         // i2c_hal_slave_clr_rx_it(&(i2c_context[i2c_num].hal));
//     } else if (evt_type == I2C_INTR_EVENT_TXFIFO_EMPTY) {
//         // everything is sent
//         // uint32_t tx_fifo_rem;
//         // i2c_hal_get_txfifo_cnt(&(i2c_context[i2c_num].hal),
//         &tx_fifo_rem);
//         // size_t size = 0;
//         // uint8_t *data = (uint8_t *)
//         xRingbufferReceiveUpToFromISR(p_i2c->tx_ring_buf, &size,
//         tx_fifo_rem);
//         // if (data) {
//             // i2c_hal_write_txfifo(&(i2c_context[i2c_num].hal), data,
//             size);
//             // vRingbufferReturnItemFromISR(p_i2c->tx_ring_buf, data,
//             &HPTaskAwoken);
//         } else {
//             i2c_hal_disable_slave_tx_it(&(i2c_context[i2c_num].hal));
//         }
//         i2c_hal_slave_clr_tx_it(&(i2c_context[i2c_num].hal));
//     }

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

static void i2c_reader_task(void*)
{
    uint8_t data[3] = { 0 };
    int data_count;
    while (1) {
        /* waiting for register to select */
        data_count = 0;
        data_count += i2c_slave_read_buffer(I2C_NUM_1, &data[0], 1, portMAX_DELAY);
        data_count += i2c_slave_read_buffer(I2C_NUM_1, &data[1], 1, 0);
        if (data_count != 2) {
            ESP_LOGE(TAG, "Wrong frame! Ignore");
            while (1) {
                if (!i2c_slave_read_buffer(I2C_NUM_1, &data[2], 1, 0)) {
                    break;
                }
            }
            continue;
        }
        /* flush others - we are not waiting it */
        if (data[1] != 0xFF) {
            // write
            // TODO: regs
            // regs.Write(data[0], data[1]);
            ESP_LOGI(TAG, "Wrote 0x%x : 0x%x", data[0], data[1]);
        } else {
            // select
            // TODO: regs
            // regs.SelectReg(data[0]);
            esp_err_t res = reg_tx_upd();
            if (res != ESP_OK) {
                ESP_LOGE(TAG, "Cannot send a value (%s)", ESTR(res));
            }
            ESP_LOGI(TAG, "Selected register: 0x%x", data[0]);
        }
    }
}



/**
 * @brief Updates tx fifo with a single byte from the selected register to be
 * read by master
 */
static void i2c_writer_task(void*)
{
    esp_err_t res = 0;
    while (1) {
        res = reg_tx_upd();
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Can't load a value to the TX FIFO!");
        }
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
    res = xTaskCreatePinnedToCore(i2c_writer_task, "i2c_writer_task", 1024 * 4, NULL, 5, NULL, 0);
    if (res != pdPASS) {
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "I2C slave ready! Address: 0x%x", PLATFORM_I2C_ADDRESS);
    return ESP_OK;
}
