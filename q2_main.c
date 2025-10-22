//Kiara Creed
//816036290

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_timer.h"

#include "driver/gpio.h"

SemaphoreHandle_t xMutex;

// Create an example C application for your platform with three FreeRTOS tasks, and a FreeRTOS mutex and adjust the configuration file appropriately:

// Task1 and task2 share a single GPIO pin connected to an LED. 
// The GPIO pin should be managed using a mutex. 
// Each task should have it's own priority level determined by a const int.

//Shares a GPIO pin with task2
//Will turn the GPIO pin on, and actively wait for 0.5 seconds, before yielding. 
void vTask1 ( void *arg )
{
    xSemaphoreTake(xMutex, portMAX_DELAY);

    //works for active high aka when the led is tied to the esp and resitor to gnd
    gpio_set_level(GPIO_NUM_2, 1);// Setting GPIO pin 15 high
    //active wait for 0.5s
    int64_t xStart= esp_timer_get_time();
    while(esp_timer_get_time()<=(xStart+500000))
    {

    }

    //yield
    xSemaphoreGive(xMutex);

    vTaskDelete( NULL ); //allows tasks to exit cleanly
}

// Shares a single GPIO pin with task1
// Will turn the GPIO pin off, and task-delay for 1 second. 
void vTask2 ( void *arg )
{
    xSemaphoreTake(xMutex, portMAX_DELAY);

    //works for active high
    gpio_set_level(GPIO_NUM_2, 0);// Setting GPIO pin 15 low
    vTaskDelay(1000 / portTICK_RATE_MS); //1s ; places task2 in the blocked state

    xSemaphoreGive(xMutex);

    vTaskDelete( NULL );
}

//Will print a status message via the serial UART, and task-delay for 1 second. 
void vTask3 ( void *arg )
{
    printf("task3 is running"); //status message??
    vTaskDelay(1000 / portTICK_RATE_MS); 

    vTaskDelete( NULL );
}


void app_main()
{

    xMutex= xSemaphoreCreateMutex();
    if (xMutex!=NULL)
    {
        xTaskCreate(vTask1, "task1", 1000, NULL, 1, NULL );
        xTaskCreate(vTask2, "task2", 1000, NULL, 1, NULL );
        xTaskCreate(vTask3, "task3", 1000, NULL, 1, NULL ); 
        //Start the scheduler so the tasks start executing.
        vTaskStartScheduler();
    }

    gpio_config_t io_conf; //to start configuring the pins
    io_conf.mode = GPIO_MODE_OUTPUT; //setting as output mode
    io_conf.pin_bit_mask = (1ULL<<2); //bit masking to set GPIO pin 15 as an output
    io_conf.pull_down_en = 0; //disabling pull-down mode
    io_conf.pull_up_en = 0; //disable pull-up mode
    gpio_config(&io_conf); //to apply the configuration

   return;

}
