#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <lab3header.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "lab3header.h"

#define LED_PIN 14

void gpio_init_led(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask     = (1ULL << LED_PIN),
        .mode             = GPIO_MODE_OUTPUT,
        .pull_up_en       = GPIO_PULLUP_DISABLE,
        .pull_down_en     = GPIO_PULLDOWN_DISABLE,
        .intr_type        = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_PIN, 0);  // Start with LED off
}

void app_main(void)
{
      // Init library
    ESP_ERROR_CHECK(i2cdev_init());

    printf("Initializing ADS1115...\n");

    if (!init_ads1115())
    {
        printf("Init failed!\n");
        return;
    }

    gpio_init_led();


    printf("Init OK.\n");
    bool ledstate = false;
    
    while (1)
    {
        int16_t val = singleread_ads1115();
        printf("ADC Reading: %d\n", val);
        
        ledstate = !ledstate;
        gpio_set_level(LED_PIN, ledstate);
        // Test burst read
        // uint8_t buf[2];
        // int n = burstread_ads1115(2, buf);
        // printf("Burst Read: %02X %02X (%d bytes)\n", buf[0], buf[1], n);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}