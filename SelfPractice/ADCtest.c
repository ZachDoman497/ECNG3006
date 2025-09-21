    
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
    

void app_main()
{
    adc_config_t adc = {
    .mode       = ADC_READ_TOUT_MODE,
    .clk_div    = 16
   };
   if(adc_init (&adc) == ESP_OK){
    while(1){
        uint16_t val;
        if(adc_read(&val) == ESP_OK){             //ESP_OK = ADC has been successful
            uint16_t percentVal = (val*100/1023);
            printf("ADC value is %d\n%d%% of the total value\n\n", val, percentVal);
        }
        else{
            printf("ADC failed");
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
    else{
        ESP_LOGE("ADC", "ADC initialization has failed");
    }   
}
