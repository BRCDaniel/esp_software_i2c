/*

Copyright (c) 2018-2019 Mika Tuupola, Daniel Barth

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

#include "software_i2c.h"
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <rom/ets_sys.h>

static bool g_i2c_started;
static gpio_num_t g_i2c_sda;
static gpio_num_t g_i2c_scl;

#define LOW   0x00
#define HIGH  0x01

#define ACK   0x00
#define NAK   0x01

static const char* TAG = "software_i2c";

/* https://esp-idf.readthedocs.io/en/latest/api-reference/peripherals/i2c.html#_CPPv211i2c_set_pin10i2c_port_tii13gpio_pullup_t13gpio_pullup_t10i2c_mode_t */

/* esp_err_t i2c_set_pin(i2c_port_t i2c_num, int sda_io_num, int scl_io_num, gpio_pullup_t sda_pullup_en, gpio_pullup_t scl_pullup_en, i2c_mode_t mode) */
esp_err_t sw_i2c_init(gpio_num_t sda, gpio_num_t scl)
{
    gpio_config_t io_conf;
    io_conf.pin_bit_mask = 0;
    io_conf.pin_bit_mask |= (1ULL << sda) | (1ULL << scl);
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;    
    gpio_config(&io_conf);
    gpio_set_level(sda, HIGH);
    gpio_set_level(scl, HIGH);

    /* Save the pins in static global variables. */
    g_i2c_sda = sda;
    g_i2c_scl = scl;

    ESP_LOGI(TAG, "Initialised software i2c with pin sda[%d], scl[%d].", sda, scl);
    return ESP_OK;
}

bool sw_i2c_check_arb_lost()
{
    return (gpio_get_level(g_i2c_sda) == LOW);
}

/* esp_err_t i2c_master_start(i2c_cmd_handle_t cmd_handle) */
esp_err_t sw_i2c_master_start()
{
    uint32_t stretch = SW_I2C_CLOCK_STRETCH_TIMEOUT;
    esp_err_t ret = ESP_OK;

     /* If already started, do a restart condition. */
    if (g_i2c_started) {
        gpio_set_level(g_i2c_sda, HIGH);
        ets_delay_us(SW_I2C_DELAY_US);
        gpio_set_level(g_i2c_scl, HIGH);
        while (gpio_get_level(g_i2c_scl) == LOW && stretch--) {
            ets_delay_us(1);
        };
        ets_delay_us(SW_I2C_DELAY_US);
    }

    if (sw_i2c_check_arb_lost()) {
        ESP_LOGD(TAG, "Arbitration lost in sw_i2c_master_start()");
        ret = ESP_ERR_TIMEOUT;
    }

    /* Start bit is indicated by a high-to-low transition of SDA with SCL high. */
    gpio_set_level(g_i2c_sda, LOW);
    ets_delay_us(SW_I2C_DELAY_US);
    gpio_set_level(g_i2c_scl, LOW);

    g_i2c_started = true;

    return ret;
}

/* esp_err_t i2c_master_stop(i2c_cmd_handle_t cmd_handle) */
esp_err_t sw_i2c_master_stop()
{
    uint32_t stretch = SW_I2C_CLOCK_STRETCH_TIMEOUT;
    esp_err_t ret = ESP_OK;

    /* The stop bit is indicated by a low-to-high transition of SDA with SCL high. */
    gpio_set_level(g_i2c_sda, LOW);
    ets_delay_us(SW_I2C_DELAY_US);
    gpio_set_level(g_i2c_scl, HIGH);

    while (gpio_get_level(g_i2c_scl) == LOW && stretch--) {
        ets_delay_us(1);
    };

    ets_delay_us(SW_I2C_DELAY_US);
    gpio_set_level(g_i2c_sda, HIGH);
    ets_delay_us(SW_I2C_DELAY_US);
    if (sw_i2c_check_arb_lost()) {
        ESP_LOGD(TAG, "Arbitration lost in sw_i2c_master_stop()");
        ret = ESP_ERR_TIMEOUT;
    }

    ets_delay_us(SW_I2C_DELAY_US);
    g_i2c_started = false;

    return ret;
}

static esp_err_t sw_i2c_write_bit(bool bit)
{
    uint32_t stretch = SW_I2C_CLOCK_STRETCH_TIMEOUT;
    esp_err_t ret = ESP_OK;

    gpio_set_level(g_i2c_sda, bit);
    ets_delay_us(SW_I2C_DELAY_US); /* SDA change propagation delay */
    gpio_set_level(g_i2c_scl, HIGH); /* New valid SDA value is available. */

    while (gpio_get_level(g_i2c_scl) == LOW && stretch--) {
        ets_delay_us(1);
    };

    ets_delay_us(SW_I2C_DELAY_US); /* Wait for SDA value to be read by slave. */

    if (bit && (sw_i2c_check_arb_lost())) {
        ESP_LOGD(TAG, "Arbitration lost in sw_i2c_write_bit()");
        ret = ESP_ERR_TIMEOUT;
    }

    gpio_set_level(g_i2c_scl, LOW); /* Prepare for next bit. */

    return ret;
}

static bool sw_i2c_read_bit()
{
    uint32_t stretch = SW_I2C_CLOCK_STRETCH_TIMEOUT;
    bool bit;

    gpio_set_level(g_i2c_sda, HIGH); /* Let the slave drive data. */
    ets_delay_us(SW_I2C_DELAY_US); /* Wait for slave to write. */
    gpio_set_level(g_i2c_scl, HIGH); /* New valid SDA value is available. */

    while (gpio_get_level(g_i2c_scl) == LOW && stretch--) {
        ets_delay_us(1);
    };

    ets_delay_us(SW_I2C_DELAY_US); /* Wait for slave to write. */
    bit = gpio_get_level(g_i2c_sda); /* SCL is high, read a bit. */
    gpio_set_level(g_i2c_scl, LOW); /* Prepare for next bit. */

    return bit;
}

static esp_err_t sw_i2c_read_byte(uint8_t *buffer, bool ack)
{
    uint8_t byte = 0;
    uint8_t bit;

    for (bit = 0; bit < 8; ++bit) {
        byte = (byte << 1) | sw_i2c_read_bit();
    }
    
    *buffer = byte;
    return sw_i2c_write_bit(!ack); /* ACK is 0 on I2C bus, so we are flipping it */
}

static bool sw_i2c_write_byte(uint8_t byte)
{
    uint8_t bit;
    bool ack = false;
    bool arb_check = true;
    for (bit = 0; bit < 8; ++bit) {
        if (sw_i2c_write_bit((byte & 0x80) != 0) != ESP_OK) {
            arb_check = false;
        }
        byte <<= 1;
    }
    ack = !sw_i2c_read_bit(); /* ACK is 0 on I2C bus, so we are flipping it */
    return ack && arb_check;
}

/* esp_err_t i2c_master_write_byte(i2c_cmd_handle_t cmd_handle, uint8_t data, bool ack_en) */
bool sw_i2c_master_write_byte(uint8_t buffer)
{
    return sw_i2c_write_byte(buffer);
    //return ESP_OK;
}

/* esp_err_t i2c_master_write(i2c_cmd_handle_t cmd_handle, uint8_t *data, size_t data_len, bool ack_en) */
bool sw_i2c_master_write_handleNACK(uint8_t *buffer, uint8_t length, bool lastByteNACK) // bool ack_enable??
{
    bool rtn = true;
    uint8_t count = 0;
    for (count = 0; (count < length) && (rtn == true); count++) 
    {
        if ((count == length - 1) && (lastByteNACK == true)) 
        {
            rtn = !sw_i2c_write_byte(buffer[count]);
        }
        else
        {
            rtn = sw_i2c_write_byte(buffer[count]);
        }
    }

    return ((rtn) && (length == count));
}

bool sw_i2c_master_write(uint8_t *buffer, uint8_t length)
{
    return sw_i2c_master_write_handleNACK(buffer, length, false);
}

/* esp_err_t i2c_master_read_byte(i2c_cmd_handle_t cmd_handle, uint8_t *data, i2c_ack_type_t ack) */
esp_err_t sw_i2c_master_read_byte(uint8_t *buffer, bool ack)
{
    esp_err_t ret = ESP_OK;
    if (buffer == NULL)
        ret = ESP_ERR_INVALID_ARG;
    else
        ret = sw_i2c_read_byte(buffer, ack);
    return ret;
};

/* esp_err_t i2c_master_read(i2c_cmd_handle_t cmd_handle, uint8_t *data, size_t data_len, i2c_ack_type_t ack) */
esp_err_t sw_i2c_master_read(uint8_t *buffer, uint16_t length, i2c_ack_type_t ack)
{
    esp_err_t ret = ESP_OK;
    while(length) {
        if (length == 1 && ack == I2C_MASTER_LAST_NACK) 
        {
            ret = sw_i2c_master_read_byte(buffer, false);
        } else
        {
            switch (ack)
            {
                case I2C_MASTER_ACK:                
                case I2C_MASTER_LAST_NACK:
                    ret = sw_i2c_master_read_byte(buffer, true);
                    break;
                case I2C_MASTER_NACK:
                    ret = sw_i2c_master_read_byte(buffer, false);
                    break;
                default:
                    ret = ESP_FAIL;
                    break;
            }
            if (ret != ESP_OK)
            {
                break;
            }
        }
        buffer++;
        length--;
    }

    return ret;
}

