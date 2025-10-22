//Kiara Creed
//816036290

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/hw_timer.h"
#include "driver/uart.h"

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "esp_timer.h"

#define     LED_PIN         2
#define     UART_NUM        UART_NUM_0

#define     TASK1PRIO       3
#define     TASK2PRIO       2
#define     TASK3PRIO       1

SemaphoreHandle_t xMutex;
volatile bool   ledOn;

static const char *TAG="APP";
esp_log_level_set(TAG,ESP_LOG_INFO);

// Create an example C application for your platform with three FreeRTOS tasks, and a FreeRTOS mutex and adjust the configuration file appropriately:

// Task1 and task2 share a single GPIO pin connected to an LED. 
// The GPIO pin should be managed using a mutex. 
// Each task should have it's own priority level determined by a const int.

void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    
    uart_param_config(UART_NUM, &uart_config);
    uart_driver_install(UART_NUM, 256, 0, 0, NULL, 0);
}

static void uart_send_string(const char* str)
{
    uart_write_bytes(UART_NUM, str, strlen(str));
}

//Shares a GPIO pin with task2
//Will turn the GPIO pin on, and actively wait for 0.5 seconds, before yielding. 
void vTask1 ( void *arg )
{
    ESP_LOGI(TAG, "starting task1");
    for(;;){
    ESP_LOGI(TAG, "Entered Task1");
    xSemaphoreTake(xMutex, portMAX_DELAY);
    //works for active high aka when the led is tied to the esp and resitor to gnd
    gpio_set_level(GPIO_NUM_2, 1);// Setting GPIO pin 15 high
    ledOn = true;
    
    //active wait for 0.5s
    int64_t xStart = esp_timer_get_time();
    ESP_LOGI(TAG, "starting busywait, start time = %u\n", (uint32_t)xStart);
    while(esp_timer_get_time()<=(xStart+500000))
    {

    }
    ESP_LOGI(TAG, "finished busywait, exit time = %u\n", (uint32_t) esp_timer_get_time());

    //yield
    xSemaphoreGive(xMutex);
    vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete( NULL ); //allows tasks to exit cleanly
}

// Shares a single GPIO pin with task1
// Will turn the GPIO pin off, and task-delay for 1 second. 
void vTask2 ( void *arg )
{
    for(;;){
    ESP_LOGI(TAG, "Entered Task2");
    xSemaphoreTake(xMutex, portMAX_DELAY);
        //works for active high
    gpio_set_level(GPIO_NUM_2, 0);// Setting GPIO pin 15 low
    ledOn = false;

    vTaskDelay(pdMS_TO_TICKS(1000)); //1s ; places task2 in the blocked state

    xSemaphoreGive(xMutex);
    vTaskDelay(pdMS_TO_TICKS(10));

    }
    vTaskDelete( NULL );
}

//Will print a status message via the serial UART, and task-delay for 1 second. 
void vTask3 ( void *arg )
{
    for(;;){
    ESP_LOGI(TAG, "Entered Task3");
    if(ledOn == true){
        uart_send_string("LED is on\n"); //status message
    }
    else{
        uart_send_string("LED is off\n"); //status message
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    }   
    vTaskDelete( NULL );
}


void gpio_init_led(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask     = (1ULL << LED_PIN),      //bit masking to set GPIO pin 15 as an output
        .mode             = GPIO_MODE_OUTPUT,       //setting as output mode
        .pull_up_en       = GPIO_PULLUP_DISABLE,    //disabling pull-up mode
        .pull_down_en     = GPIO_PULLDOWN_DISABLE,  //disabling pull-down mode
        .intr_type        = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);      //to apply the configuration
    gpio_set_level(LED_PIN, 0);  // Start with LED off
}


void app_main()
{
    gpio_init_led();
    uart_init();

    xMutex = xSemaphoreCreateMutex();
    if (xMutex != NULL)
    {
        xTaskCreate(vTask1, "task1", 1000, NULL, TASK1PRIO, NULL );
        xTaskCreate(vTask2, "task2", 1000, NULL, TASK2PRIO, NULL );
    }

    xTaskCreate(vTask3, "task3", 1000, NULL, TASK3PRIO, NULL ); 
    //xTaskCreate(vTask4, "task4", 1000, NULL, 3, NULL);
}

