//Kiara Creed
//816036290

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "nvm.h"

#define MAX_LEN 37
#define MAX_LIN 20
static const char *TAG = "lab3_q3";

void app_main()
{
    while (1)
    {
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

        printf("Flash size: %d bytes\n", spi_flash_get_chip_size());

        
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
        
        ESP_LOGI(TAG,"Going into the singlewrite_spiffs function");
        char message [MAX_LEN];
        bool write;
        for(int n=0; n<20; n++)
        {
            strcpy(message, ddata[n]);
            write= singlewrite_spiffs(&message);
        }
        
        //dur = esp_timer_get_time()-start;  //calculating the duration of the routine is
        //ESP_LOGI(TAG,"Duration of singlewrite_spiffs function when HR=1: %lu", (unsigned long int)dur);
        if (write==true)
        {
            ESP_LOGI(TAG,"Successfully attempted to write to file");
        }
        else
        {
            ESP_LOGI(TAG,"Failure occured trying to write to file");   
            return;
        }

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
