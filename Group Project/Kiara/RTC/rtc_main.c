#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ds3231.h>
#include <string.h>

#include "esp_log.h"

#define I2C_RTC_MASTER_SCL_IO           5                /*!< gpio number for I2C master clock */
#define I2C_RTC_MASTER_SDA_IO           4  

static const char *TAG = "rtc_main";

//initialize the values to the current time
struct tm rtc_time = {
        .tm_year = 125,     //years since 1900 
        .tm_mon  = 11,      //0-11
        .tm_mday = 5,       //day of the month (1-31)
        .tm_hour = 9,      //in 24hr format (0-23)
        .tm_min  = 50,      //0-59
        .tm_sec  = 10       //0-61;  tm_sec is generally 0-59. The extra range is to accommodate for leap seconds in certain systems
    };

void ds3231_test(void *pvParameters)
{
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_LOGI(TAG, "Initialing DS3231 RTC");
    ESP_ERROR_CHECK(ds3231_init_desc(&dev, 0, I2C_RTC_MASTER_SDA_IO, I2C_RTC_MASTER_SCL_IO));
    
    ESP_LOGI(TAG, "Setting time on RTC");
    ESP_ERROR_CHECK(ds3231_set_time(&dev, &rtc_time));

    ESP_LOGI(TAG, "Begining to test the sqaure wave");
    ESP_ERROR_CHECK(ds3231_set_squarewave_freq(&dev, DS3231_SQWAVE_4096HZ));
    ESP_ERROR_CHECK(ds3231_enable_squarewave(&dev));
    vTaskDelay(5000 / portTICK_PERIOD_MS); //5s
    vTaskDelay(5000 / portTICK_PERIOD_MS); //5s
    ESP_ERROR_CHECK(ds3231_disable_squarewave(&dev));

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(250));

        if (ds3231_get_time(&dev, &rtc_time) != ESP_OK)
        {
            printf("Could not get time\n");
            continue;
        }

        printf("%04d-%02d-%02d %02d:%02d:%02d\n", rtc_time.tm_year + 1900 /*Add 1900 for better readability*/, rtc_time.tm_mon + 1,
            rtc_time.tm_mday, rtc_time.tm_hour, rtc_time.tm_min, rtc_time.tm_sec);
    }
}

void app_main()
{
    ESP_LOGI(TAG, "Initialing DS3231 RTC");
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(ds3231_test, "ds3231_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
}
