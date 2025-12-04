//Kiara Creed
//816036290

#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "driver/i2c.h"
#include "rtc_gp.h"
#include "nvm.h"

#define I2C_RTC_MASTER_SCL_IO           2                //gpio number for I2C master clock
#define I2C_RTC_MASTER_SDA_IO           14

#define MAX_LEN                         37              //max length of each line of the data going to spiffs
#define MAX_LIN                         20              //max number of rows in the data going to spiffs
#define TIME_LEN                        20              //max length of the date and time string including the null terminator

static const char *TAG = "sp_rtc_main";

SemaphoreHandle_t xMutex;

char date [TIME_LEN]; //stores the formated current date and time

//initialize values with the current time
struct tm rtc_time= 
{
    .tm_sec  = 0,       //0-61;  tm_sec is generally 0-59. The extra range is to accommodate for leap seconds in certain systems
    .tm_min  = 0,       //0-59
    .tm_hour = 0,       //in 24hr format (0-23)
    .tm_mday = 1,       //day of the month (1-31)
    .tm_mon  = 11,      //0-11
    .tm_year = 125,     //years since 1900
    .tm_wday = 0,       //day of the week (0-6)
};

void rtc_sp_init()
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

    ESP_LOGI(TAG,"Going into the init_spiffs function");
    //uint64_t start = esp_timer_get_time();  
    bool init= init_spiffs();
    //uint64_t dur = esp_timer_get_time()-start;  //calculating the duration of the routine is
    //ESP_LOGI(TAG,"Duration of init_spiffs function: %lu", (unsigned long int)dur);
    if (init==true)
    {
        ESP_LOGI(TAG,"SPIFFS sucessfully initialized");
    }
    else
    {
        ESP_LOGI(TAG,"Failure occurred trying to intilize SPIFFS");   
        return;
    }

}

void sqw_test()
{
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
     if (ds3231_disable_squarewave() != ESP_OK) {
        ESP_LOGI(TAG, "Could not disable sqaure wave.");
        while (1) 
        { 
            vTaskDelay(1); 
        }
    }
}

char* get_time()
{
    ESP_LOGI(TAG,"Getting time");
    //copying the current time to the rtc_time struct
    if (ds3231_get_time(&rtc_time) != ESP_OK) 
    {
        ESP_LOGI(TAG, "Could not get time.");
        while (1) 
        { 
            vTaskDelay(1); 
        }
    }
    xSemaphoreTake(xMutex, portMAX_DELAY);
    strcpy(date, dds3231_ftime(&rtc_time));
    xSemaphoreGive(xMutex);

    return date;
}

void add_date(char data[MAX_LEN])
{
    //adding date to the existing dummy data to send to spiffs
    xSemaphoreTake(xMutex, portMAX_DELAY);
    for (int i = 0; i < 19; i++) 
    {
        data[i] = date[i];
    }
    xSemaphoreGive(xMutex);
}

void app_main()
{
    xMutex = xSemaphoreCreateMutex();

    rtc_sp_init();  //initializing the rtc and spiffs
    sqw_test();     //testing the sqaure wave output from the rtc

    //format: Date(10), Time(8), Patient ID(6), HR(3), SpO2(3)
    //the brackets say the number of characters expected for each
    //Overall structure: yyyy-mm-dd,hh:mm:ss,PPPPPP,HHH,SSS
    //typical patient ID is 6 numbers 
    char ddata[MAX_LIN][MAX_LEN] = {"2025-01-12,08:14:22,102394,118,89\n",
                                    "2025-01-12,08:16:10,657201,132,100\n",
                                    "2025-01-12,08:19:45,849302,110,92\n",
                                    "2025-01-12,08:21:33,903114,145,81\n",
                                    "2025-01-12,08:24:58,204839,124,88\n",
                                    "2025-01-12,08:27:11,553920,138,85\n",
                                    "2025-01-12,08:29:05,113849,115,90\n",
                                    "2025-01-12,08:31:52,774120,129,86\n",
                                    "2025-01-12,08:34:47,448291,121,89\n",
                                    "2025-01-12,08:37:29,995103,150,79\n",
                                    "2025-01-12,08:40:10,320488,112,93\n",
                                    "2025-01-12,08:42:56,774553,140,83\n",
                                    "2025-01-12,08:45:34,682901,119,91\n",
                                    "2025-01-12,08:48:20,550391,134,87\n",
                                    "2025-01-12,08:50:42,112398,125,90\n",
                                    "2025-01-12,08:53:17,991204,142,82\n",
                                    "2025-01-12,08:56:01,884220,117,94\n",
                                    "2025-01-12,08:58:49,430119,130,88\n",
                                    "2025-01-12,09:01:22,310552,136,85\n",
                                    "2025-01-12,09:03:57,770431,95,92\n" };
    
    ESP_LOGI(TAG, "Begining to get time and interact with SPIFFS\n");
    bool write; //return from spiffswrite function
    while (1) 
    {
        for(int n=0; n<MAX_LIN; n++)
        {
            ESP_LOGI(TAG,"Getting time");
            //getting and formating the current time
            strcpy(date, get_time());
            ESP_LOGI(TAG, "Time from RTC: %s", date);
            //adding the date and time to the existing data
            add_date(ddata[n]);

            ESP_LOGI(TAG, "Message going to SPIFFS: %s", ddata[n]);
            ESP_LOGI(TAG,"Writing to spiffs");
            write= singlewrite_spiffs(&ddata[n]);
            if (write==true)
            {
                ESP_LOGI(TAG,"Successfully attempted to write to file");
            }
            else
            {
                ESP_LOGI(TAG,"Failure occured trying to write to file");   
                return;
            }

        }

        vTaskDelay(5000 / portTICK_PERIOD_MS); //sending message every 5 seconds roughly

        ESP_LOGI(TAG,"Going into the singleread_spiffs function");
        //start = esp_timer_get_time();
        char data[MAX_LIN][MAX_LEN];
        bool x= burstread_spiffs(data);
        //dur = esp_timer_get_time()-start;  //calculating the duration of the routine is
        //ESP_LOGI(TAG,"Duration of singleread_spiffs function when HR=1: %lu", (unsigned long int)dur);
        if (x==true)
        {
            ESP_LOGI(TAG,"SPIFFS sucessfully read file");
        }
        else
        {
            ESP_LOGI(TAG,"Failure occurred trying to read file");   
            return;
        }

        ESP_LOGI(TAG, "Read from file: ");
        for (int n=0;n<MAX_LIN;n++)
        {
            ESP_LOGI(TAG, "%s", data[n]);
        }


        unmount_spiffs();

	}


}
