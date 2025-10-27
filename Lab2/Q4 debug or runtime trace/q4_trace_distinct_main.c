// Kiara Creed
// 816036290

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"     

/*traceBLOCKING_ON_QUEUE_RECEIVE(pxQueue)
Called immediately before the currently executing task enters the Blocked state following an attempt to
read from an empty queue, or an attempt to 'take' an empty semaphore or mutex. The pxQueue
parameter passes the handle of the target queue or semaphore into the macro.*/

/*traceTASK_DELAY()
Called from within vTaskDelay() immediately before the calling task enters the Blocked state*/


#define LED_PIN     2
#define UART_NUM    UART_NUM_0

#define TASK1PRIO   3
#define TASK2PRIO   2
#define TASK3PRIO   1
#define MONPRIO     0

SemaphoreHandle_t xMutex;
volatile bool ledOn;

static TaskHandle_t hTask1, hTask2, hTask3;

static const char *TAG = "Q4_logging";

void gpio_init_led(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_PIN, 0);
}

//task gets ready every 1s and samples the states of each task
void vTaskMonitor(void *arg)
{
    char *pcWriteBuffer = malloc(240); //needs aprox 40 bytes per task
    if (pcWriteBuffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer for task monitor");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        vTaskList(pcWriteBuffer);  // Shows task state, priority, stack, and runtime
        ESP_LOGI(TAG, "\nTask Stats:\n%s", pcWriteBuffer);

        vTaskDelay(pdMS_TO_TICKS(1000));  // Print every 1 second
    }

    free(pcWriteBuffer);
    vTaskDelete(NULL);
    
}



// Task 1: Turns LED ON and busy-waits for 0.5s
void vTask1(void *arg)
{
    ESP_LOGI(TAG, "[%p] Task1 STARTED (Priority %d)", hTask1, uxTaskPriorityGet(NULL));

        ESP_LOGI(TAG, "[%p] Entered Task1 loop", hTask1);

        ESP_LOGI(TAG, "[%p] Task1 trying to take mutex...", hTask1);
        xSemaphoreTake(xMutex, portMAX_DELAY);
        ESP_LOGI(TAG, "[%p] Task1 ACQUIRED mutex", hTask1);

        gpio_set_level(GPIO_NUM_2, 1);
        ledOn = true;
        ESP_LOGI(TAG, "[%p] LED turned ON", hTask1);

        int64_t xStart = esp_timer_get_time();
        ESP_LOGI(TAG, "[%p] Busy-wait start @ %u µs", hTask1, (uint32_t)xStart);

        while (esp_timer_get_time() <= (xStart + 500000)) {
            // simulate CPU load
        }

        int64_t xEnd = esp_timer_get_time();
        ESP_LOGI(TAG, "[%p] Busy-wait done @ %u µs (duration: %u µs)", hTask1, (uint32_t)xEnd, (uint32_t)(xEnd - xStart));

        xSemaphoreGive(xMutex);
        ESP_LOGI(TAG, "[%p] Task1 RELEASED mutex", hTask1);

        ESP_LOGI(TAG, "[%p] Task1 is blocked for 10ms", hTask1);
        vTaskDelay(pdMS_TO_TICKS(10));

    vTaskDelete(NULL);
}

// Task 2: Turns LED OFF and delays 1s
void vTask2(void *arg)
{
    ESP_LOGI(TAG, "\n\n[%p] Task2 STARTED (Priority %d)", hTask2, uxTaskPriorityGet(NULL));

        ESP_LOGI(TAG, "[%p] Entered Task2 loop", hTask2);

        ESP_LOGI(TAG, "[%p] Task2 trying to take mutex...", hTask2);
        xSemaphoreTake(xMutex, portMAX_DELAY);
        ESP_LOGI(TAG, "[%p] Task2 ACQUIRED mutex", hTask2);

        gpio_set_level(GPIO_NUM_2, 0);
        ledOn = false;
        ESP_LOGI(TAG, "[%p] LED turned OFF", hTask2);

        ESP_LOGI(TAG, "[%p] Task2 is blocked for 1s while holding mutex", hTask2);
        vTaskDelay(pdMS_TO_TICKS(1000));

        xSemaphoreGive(xMutex);
        ESP_LOGI(TAG, "[%p] Task2 RELEASED mutex", hTask2);

        ESP_LOGI(TAG, "[%p] Task2 is blocked for 10ms (post-release)", hTask2);
        vTaskDelay(pdMS_TO_TICKS(10));

    vTaskDelete(NULL);
}

// Task 3: Reports LED state
void vTask3(void *arg)
{
    ESP_LOGI(TAG, "\n\n[%p] Task3 STARTED (Priority %d)", hTask3, uxTaskPriorityGet(NULL));

        ESP_LOGI(TAG, "[%p] Entered Task3 loop", hTask3);

        if (ledOn) {
            ESP_LOGI(TAG, "[%p] UART: LED is ON", hTask3);
        } else {
            ESP_LOGI(TAG, "[%p] UART: LED is OFF", hTask3);
        }

        ESP_LOGI(TAG, "[%p] Task3 delaying 1s", hTask3);
        vTaskDelay(pdMS_TO_TICKS(1000));

    vTaskDelete(NULL);
}

void app_main()
{
    gpio_init_led();

    xMutex = xSemaphoreCreateMutex();
    if (xMutex != NULL)
    {
        ESP_LOGI(TAG, "Mutex created successfully");
        xTaskCreate(vTask1, "task1", 2048, NULL, TASK1PRIO, &hTask1);
        xTaskCreate(vTask2, "task2", 2048, NULL, TASK2PRIO, &hTask2);
    }
    else
    {
        ESP_LOGE(TAG, "Mutex creation FAILED");
    }
    
    xTaskCreate(vTask3, "task3", 2048, NULL, TASK3PRIO, &hTask3);
    
    xTaskCreate(vTaskMonitor, "taskMonitor", 2048, NULL, MONPRIO, NULL);
    ESP_LOGI(TAG, "All tasks created successfully");
}
