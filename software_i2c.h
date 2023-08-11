/*

Copyright (c) 2018-2023 Mika Tuupola, Daniel Barth

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#pragma once

#ifdef __cplusplus
extern "C" {
#endif


#include <esp_err.h>
#include <driver/gpio.h>
#include <driver/i2c.h>

#define SW_I2C_FREQENCY     100000 /* 100kHz, should not be bigger as 500000 */
#define SW_I2C_DELAY_US     ((1 / SW_I2C_FREQENCY) * 1000000)

#define SW_I2C_CLOCK_STRETCH_TIMEOUT   (SW_I2C_DELAY_US * 10) /* Allow Clock-Stretching up to 10*DELAY_US */

/**
 * @brief Initialize the software i2c.
 * 
 * @param sda The GPIO pin for the SDA line.
 * @param scl The GPIO pin for the SCL line.
 * 
 * @return ESP_OK if successful, ESP_FAIL otherwise.
 *
*/
esp_err_t sw_i2c_init(gpio_num_t sda, gpio_num_t scl);

/**
 * @brief Check if the bus is busy.
*/
bool sw_i2c_check_arb_lost();

/**
 * @brief Set Start Condition.
 * 
 * @return ESP_OK if successful, ESP_FAIL otherwise.
*/
esp_err_t sw_i2c_master_start();

/**
 * @brief Set Stop Condition.
 * 
 * @return ESP_OK if successful, ESP_FAIL otherwise.
*/
esp_err_t sw_i2c_master_stop();

/**
 * @brief Write a byte to the slave.
 * 
 * @return true if ACK happened.
*/
bool sw_i2c_master_write_byte(uint8_t buffer);

/**
 * @brief Write multiple bytes to the slave.
 * 
 * @return true if everything got ACK.
*/
bool sw_i2c_master_write(uint8_t *buffer, uint8_t length);

/**
 * @brief Read a byte from the slave.
 * @param buffer Pointer to the buffer where the byte should be stored.
 * @param ack If true, an ACK will be sent after the byte is read.
 * 
 * @return ESP_OK if successful, ESP_FAIL otherwise.
*/
esp_err_t sw_i2c_master_read_byte(uint8_t *buffer, bool ack);

/**
 * @brief Read multiple bytes from the slave.
 * @param buffer Pointer to the buffer where the bytes should be stored.
 * @param ack If true, an ACK will be sent after the bytes are read.
 * 
 * @return ESP_OK if successful, ESP_FAIL otherwise.
*/
esp_err_t sw_i2c_master_read(uint8_t *buffer, uint16_t length, i2c_ack_type_t ack);

#ifdef __cplusplus
}
#endif