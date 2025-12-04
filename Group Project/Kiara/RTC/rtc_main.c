//Kiara Creed
//816036290

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "driver/i2c.h"
#include "rtc_gp.h"

#define I2C_RTC_MASTER_SCL_IO           2                /*!< gpio number for I2C master clock */
#define I2C_RTC_MASTER_SDA_IO           14
#define TIME_LEN                        20                //length of the string contraining the date and time                         

static const char *TAG = "rtc_main";

//initialize the values to the current time
struct tm rtc_time= 
{
    .tm_sec  = 0,       //0-61;  tm_sec is generally 0-59. The extra range is to accommodate for leap seconds in certain systems
    .tm_min  = 0,       //0-59
    .tm_hour = 0,       //in 24hr format (0-23)
    .tm_mday = 0,       //day of the month (1-31)
    .tm_mon  = 11,      //0-11
    .tm_year = 125,     //years since 1900
    .tm_wday = 0,       //day of the week (0-6)
};

void app_main()
{
    ESP_LOGI(TAG, "Initialing DS3231 RTC and I2C\n");
    if (ds3231_i2c_init(I2C_RTC_MASTER_SDA_IO, I2C_RTC_MASTER_SCL_IO)!= ESP_OK) {
		ESP_LOGI(TAG, "Could not init device descriptor.");
		while (1) 
        {
            vTaskDelay(1);
        }
	}

    ESP_LOGI(TAG, "Setting time on RTC\n");
    ds3231_set_time(&rtc_time);

    ESP_LOGI(TAG, "Begining to test the sqaure wave");
    if (ds3231_enable_squarewave() != ESP_OK) {
        ESP_LOGI(TAG, "Could not initialize sqaure wave.");
        while (1) 
        { 
            vTaskDelay(1); 
        }
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS); //5s
    vTaskDelay(5000 / portTICK_PERIOD_MS); //5s
     if (ds3231_disable_squarewave() != ESP_OK) 
     {
        ESP_LOGI(TAG, "Could not disable sqaure wave.");
        while (1) 
        { 
            vTaskDelay(1); 
        }
    }


    ESP_LOGI(TAG, "Begining to get time\n");
    while (1) 
    {
        if (ds3231_get_time(&rtc_time) != ESP_OK) {
            ESP_LOGI(TAG, "Could not get time.");
            while (1) 
            { 
                vTaskDelay(1); 
            }
        }

        ESP_LOGI(TAG, "Output from get_time: %04u-%02u-%02u,%02u:%02u:%02u\n", (unsigned)rtc_time.tm_year, (unsigned)rtc_time.tm_mon + 1, (unsigned)rtc_time.tm_mday, (unsigned)rtc_time.tm_hour, (unsigned)rtc_time.tm_min, (unsigned)rtc_time.tm_sec);
        
        char date[TIME_LEN];
        strcpy(date, dds3231_ftime(&rtc_time));
        ESP_LOGI(TAG, "Time after formating: %s", date);

        vTaskDelay(5000 / portTICK_PERIOD_MS); //5s
	}

}
