/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "I2C_SLAVE";

#define I2C_SLAVE_SCL_IO           2  //2    /*!< GPIO number used for I2C master clock */
#define I2C_SLAVE_SDA_IO           1  //1    /*!< GPIO number used for I2C master data  */
#define I2C_SLAVE_NUM              I2C_NUM_0                  /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_SLAVE_TIMEOUT_MS       1000
#define I2C_SLAVE_ADDR             0x3C
#define I2C_SLAVE_RX_BUF_LEN     128          // 接收缓冲区长度
#define I2C_SLAVE_TX_BUF_LEN     128          // 发送缓冲区长度

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_slave_init(void)
{
    int i2c_slave_port = I2C_SLAVE_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .slave={
            .addr_10bit_en = 0,
            .slave_addr = I2C_SLAVE_ADDR,
        }
    };

    i2c_param_config(i2c_slave_port, &conf);

    return i2c_driver_install(i2c_slave_port, conf.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);
}

void app_main(void){
    ESP_ERROR_CHECK(i2c_slave_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    uint8_t data[128];
    uint8_t sd_data[128];
    // memset(sd_data, 0, 128);
    sd_data[0] = 0x01;
    sd_data[1] = 0x02;
    while (1) {
        int len = i2c_slave_read_buffer(I2C_SLAVE_NUM, data, I2C_SLAVE_RX_BUF_LEN, 10 / portTICK_PERIOD_MS);
        if (len > 0) {
            ESP_LOGI(TAG, "Received data: ");
            for (int i = 0; i < len; i++){
                printf("%02x ", data[i]);
            }
            printf("\n");
        }
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        i2c_slave_write_buffer(I2C_SLAVE_NUM,sd_data,I2C_SLAVE_TX_BUF_LEN, 10 / portTICK_PERIOD_MS);
    }
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_SLAVE_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}
