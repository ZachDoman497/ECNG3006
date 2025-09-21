    
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2c.h"

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"

/*
This code is for testing I2C communication with an external ADC (like ADS1115).
Coding is based on ESP8266 example I2C code for a MPU6050 sensor.
*/
    

#define I2C_MASTER_SCL_IO                   4
#define I2C_MASTER_SDA_IO                   5
#define I2C_MASTER_NUM                      I2C_NUM_0

#define WRITE_BIT                           I2C_MASTER_WRITE
#define READ_BIT                            I2C_MASTER_READ
#define ACK_CHECK_EN                        0x1
#define ACK_CHECK_DIS                       0x0
#define ACK_VAL                             0x0
#define NACK_VAL                            0x1
#define LAST_NACK_VAL                       0x2
#define ADS1115_SENSOR_ADDR                 0x48 //placeholder
#define ADS1115_REG_CONF                    0x01 //placeholder
#define ADS1115_REG_CONVERSION              0x00 //placeholder

static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = 1;
    conf.clk_stretch_tick = 300;
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    return ESP_OK;
}

static esp_err_t i2c_master_ADS1115_write(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1115_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t i2c_master_ADS1115_read(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1115_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1115_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t i2c_master_ADS1115_init(i2c_port_t i2c_num)
{
    uint8_t cmd_data;
    vTaskDelay(pdMS_TO_TICKS(100));
    i2c_master_init();
    cmd_data = 0x01;    // Set the config register to continuous conversion mode, AIN0, FS=+/-4.096V, 128SPS
    ESP_ERROR_CHECK(i2c_master_ADS1115_write(i2c_num, ADS1115_REG_CONF, &cmd_data, 1));
    return ESP_OK;
}

static void i2c_task(void *arg)
{
    uint8_t adc_data[2];
    uint16_t raw_adc;

    while(1){
        i2c_master_ADS1115_init(I2C_MASTER_NUM);
        vTaskDelay(pdMS_TO_TICKS(100));

        esp_err_t ret = i2c_master_ADS1115_read(I2C_MASTER_NUM, ADS1115_REG_CONVERSION, adc_data, 2);
        if (ret == ESP_OK){
            raw_adc = (adc_data[0] << 8) | adc_data[1];
            printf("Raw ADC = %d", raw_adc);
        }
        else{
            printf("error");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(){
xTaskCreate(i2c_task, "i2c_task", 2048, NULL, 10, NULL);
}
