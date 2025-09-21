#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2c.h"

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "driver/ledc.h"

#define     LED_PIN 14

    
      gpio_config_t ioconf =
    {
        .pin_bit_mask   = (1ULL << LED_PIN),
        .mode           = GPIO_MODE_OUTPUT,
        .pull_down_en   = GPIO_PULLDOWN_DISABLE,
        .pull_up_en     = GPIO_PULLUP_DISABLE,
        .intr_type      = GPIO_INTR_DISABLE
    };

    void configureGPIO(void)
    {
        if(gpio_config(&ioconf) == ESP_OK){
            printf("GPIO successfully configured");
        }
        else{
            printf("GPIO failed to configure");
        }
    }

   adc_config_t adcconf = 
    {
        .mode           = ADC_READ_TOUT_MODE,
        .clk_div        = 16

    };

        void configureADC(void)
    {
        if(adc_config(&adcconf) == ESP_OK){
            printf("ADC successfully configured");
        }
        else{
            printf("ADC failed to configure");
        }
    }


   uint16_t readADC(void)
    {
        uint16_t val = 0;
        if(adc_read(&val) == ESP_OK){
            return val;
        }
        else{
            printf("ADC failed to configure");
            return val;
        }
    };

    void displayADC(uint16_t x)
    {
         printf("The bit value is: %u\n",x);
    }

void app_main()
{
    configureGPIO();
    configureADC();
    gpio_set_level(LED_PIN, 0);
    uint16_t lastADC;
    uint16_t digitalADC = readADC();
    displayADC(digitalADC);
 
while(1){
    lastADC = digitalADC;
    digitalADC = readADC();
    if (digitalADC != lastADC){
        gpio_set_level(LED_PIN, 1);
        displayADC(digitalADC);
        vTaskDelay(pdMS_TO_TICKS(25));
        gpio_set_level(LED_PIN, 0);
       }
    vTaskDelay(pdMS_TO_TICKS(25));
    }
}