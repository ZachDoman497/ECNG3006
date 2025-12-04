//Kiara Creed
//816036290

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "driver/i2c.h"
#include "driver/i2c.h"

uint8_t bcd2dec(uint8_t val);
uint8_t dec2bcd(uint8_t val);
esp_err_t ds3231_i2c_init(gpio_num_t sda_gpio, gpio_num_t scl_gpio);
esp_err_t i2c_master_ds3231_write(uint8_t reg_address, uint8_t *data, size_t data_size);
esp_err_t i2c_master_ds3231_read(uint8_t reg_address, uint8_t *data, size_t data_size);
esp_err_t ds3231_set_time(struct tm *time);
esp_err_t ds3231_set_flag(uint8_t reg_address, uint8_t mask, int mode);
esp_err_t ds3231_get_flag(uint8_t reg_address, uint8_t mask, uint8_t *flag);
esp_err_t ds3231_enable_squarewave();
esp_err_t ds3231_disable_squarewave();
esp_err_t ds3231_get_time(struct tm *time);
char* dds3231_ftime(struct tm *time);