/*

Copyright (c) 2018-2019 Mika Tuupola

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

static bool g_i2c_started;
static gpio_num_t g_i2c_sda;
static gpio_num_t g_i2c_scl;

#define LOW   0x00
#define HIGH  0x01

#define ACK   0x00
#define NAK   0x01

static const char* TAG = "software_i2c";

#define NOP() asm volatile ("nop")

unsigned long IRAM_ATTR micros()
{
    return (unsigned long) (esp_timer_get_time());
}

void IRAM_ATTR delayMicroseconds(uint32_t us)
{
    uint32_t m = micros();
    if(us){
        uint32_t e = (m + us);
        if(m > e){ //overflow
            while(micros() > e){
                NOP();
            }
        }
        while(micros() < e){
            NOP();
        }
    }
}

/* https://esp-idf.readthedocs.io/en/latest/api-reference/peripherals/i2c.html#_CPPv211i2c_set_pin10i2c_port_tii13gpio_pullup_t13gpio_pullup_t10i2c_mode_t */

/* esp_err_t i2c_set_pin(i2c_port_t i2c_num, int sda_io_num, int scl_io_num, gpio_pullup_t sda_pullup_en, gpio_pullup_t scl_pullup_en, i2c_mode_t mode) */
esp_err_t sw_i2c_init(gpio_num_t sda, gpio_num_t scl)
{
    gpio_config_t io_conf;
    io_conf.pin_bit_mask = 0;
    io_conf.pin_bit_mask |= (1ULL << sda) | (1ULL << scl);
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;    
    gpio_config(&io_conf);
    gpio_set_level(sda, 1);
    gpio_set_level(scl, 1);

    /* Save the pins in static global variables. */
    g_i2c_sda = sda;
    g_i2c_scl = scl;

    ESP_LOGI(TAG, "Initialised software i2c with pin sda[%d], scl[%d].", sda, scl);
    return ESP_OK;
}

/* esp_err_t i2c_master_start(i2c_cmd_handle_t cmd_handle) */
esp_err_t sw_i2c_master_start()
{
    uint32_t stretch = SW_I2C_CLOCK_STRETCH_TIMEOUT;

     /* If already started, do a restart condition. */
    if (g_i2c_started) {
        gpio_set_level(g_i2c_sda, HIGH);
        delayMicroseconds(SW_I2C_DELAY_US);
        gpio_set_level(g_i2c_scl, HIGH);
        while (gpio_get_level(g_i2c_scl) == LOW && stretch--) {
            delayMicroseconds(1);
        };
        delayMicroseconds(SW_I2C_DELAY_US);
    }

    if (LOW == gpio_get_level(g_i2c_sda)) {
        ESP_LOGE(TAG, "Arbitration lost in sw_i2c_master_start()");
    }

    /* Start bit is indicated by a high-to-low transition of SDA with SCL high. */
    gpio_set_level(g_i2c_sda, LOW);
    delayMicroseconds(SW_I2C_DELAY_US);
    gpio_set_level(g_i2c_scl, LOW);

    g_i2c_started = true;

    return ESP_OK;
}

/* esp_err_t i2c_master_stop(i2c_cmd_handle_t cmd_handle) */
esp_err_t sw_i2c_master_stop()
{
    uint32_t stretch = SW_I2C_CLOCK_STRETCH_TIMEOUT;

    /* The stop bit is indicated by a low-to-high transition of SDA with SCL high. */
    gpio_set_level(g_i2c_sda, LOW);
    delayMicroseconds(SW_I2C_DELAY_US);
    gpio_set_level(g_i2c_scl, HIGH);

    while (gpio_get_level(g_i2c_scl) == LOW && stretch--) {
        delayMicroseconds(1);
    };

    delayMicroseconds(SW_I2C_DELAY_US);
    gpio_set_level(g_i2c_sda, HIGH);
    delayMicroseconds(SW_I2C_DELAY_US);

    if (gpio_get_level(g_i2c_sda) == LOW) {
        ESP_LOGE(TAG, "Arbitration lost in sw_i2c_master_stop()");
    }

    g_i2c_started = false;

    return ESP_OK;
}

static void sw_i2c_write_bit(bool bit)
{
    uint32_t stretch = SW_I2C_CLOCK_STRETCH_TIMEOUT;

    gpio_set_level(g_i2c_sda, bit);
    delayMicroseconds(SW_I2C_DELAY_US); /* SDA change propagation delay */
    gpio_set_level(g_i2c_scl, HIGH); /* New valid SDA value is available. */

    while (gpio_get_level(g_i2c_scl) == LOW && stretch--) {
        delayMicroseconds(1);
    };

    delayMicroseconds(SW_I2C_DELAY_US); /* Wait for SDA value to be read by slave. */

    if (bit && (LOW == gpio_get_level(g_i2c_sda))) {
        ESP_LOGE(TAG, "Arbitration lost in sw_i2c_write_bit()");
    }

    gpio_set_level(g_i2c_scl, LOW); /* Prepare for next bit. */
}

static bool sw_i2c_read_bit()
{
    uint32_t stretch = SW_I2C_CLOCK_STRETCH_TIMEOUT;
    bool bit;

    gpio_set_level(g_i2c_sda, HIGH); /* Let the slave drive data. */
    delayMicroseconds(SW_I2C_DELAY_US); /* Wait for slave to write. */
    gpio_set_level(g_i2c_scl, HIGH); /* New valid SDA value is available. */

    while (gpio_get_level(g_i2c_scl) == LOW && stretch--) {
        delayMicroseconds(1);
    };

    delayMicroseconds(SW_I2C_DELAY_US); /* Wait for slave to write. */
    bit = gpio_get_level(g_i2c_sda); /* SCL is high, read a bit. */
    gpio_set_level(g_i2c_scl, LOW); /* Prepare for next bit. */

    return bit;
}

static uint8_t sw_i2c_read_byte(bool ack)
{
    uint8_t byte = 0;
    uint8_t bit;

    for (bit = 0; bit < 8; ++bit) {
        byte = (byte << 1) | sw_i2c_read_bit();
    }
    sw_i2c_write_bit(!ack); /* ACK is 0 on I2C bus, so we are flipping it */

    return byte;
}

static bool sw_i2c_write_byte(uint8_t byte)
{
    uint8_t bit;
    bool ack;
    for (bit = 0; bit < 8; ++bit) {
        sw_i2c_write_bit((byte & 0x80) != 0);
        byte <<= 1;
    }
    ack = !sw_i2c_read_bit(); /* ACK is 0 on I2C bus, so we are flipping it */
    return ack;
}

/* esp_err_t i2c_master_write_byte(i2c_cmd_handle_t cmd_handle, uint8_t data, bool ack_en) */
bool sw_i2c_master_write_byte(uint8_t buffer)
{
    return sw_i2c_write_byte(buffer);
    //return ESP_OK;
}

/* esp_err_t i2c_master_write(i2c_cmd_handle_t cmd_handle, uint8_t *data, size_t data_len, bool ack_en) */
bool sw_i2c_master_write(uint8_t *buffer, uint8_t length) // bool ack_enable??
{
    bool rtn = true;
    while (length-- && rtn) {
        rtn = sw_i2c_write_byte(*buffer++);
    }

    return rtn && length == 0;
}

/* esp_err_t i2c_master_read_byte(i2c_cmd_handle_t cmd_handle, uint8_t *data, i2c_ack_type_t ack) */
esp_err_t sw_i2c_master_read_byte(uint8_t *buffer, bool ack)
{
    *buffer = sw_i2c_read_byte(ack);
    return ESP_OK;
};

/* esp_err_t i2c_master_read(i2c_cmd_handle_t cmd_handle, uint8_t *data, size_t data_len, i2c_ack_type_t ack) */
esp_err_t sw_i2c_master_read(uint8_t *buffer, uint16_t length, i2c_ack_type_t ack)
{
    while(length) {
        if (length == 1 && ack == I2C_MASTER_LAST_NACK) 
        {
            *buffer = sw_i2c_read_byte(false);
        } else
        {
            switch (ack)
            {
                case I2C_MASTER_ACK:                
                case I2C_MASTER_LAST_NACK:
                    *buffer = sw_i2c_read_byte(true);
                    break;
                case I2C_MASTER_NACK:
                    *buffer = sw_i2c_read_byte(false);
                    break;
                default:
                    return ESP_FAIL;
            }

        }
        buffer++;
        length--;
    }

    return ESP_OK;
}

