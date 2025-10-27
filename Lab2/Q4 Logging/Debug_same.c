//Ameera Chaitram
//816024370

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

#define TRACE_TAG "TRACE"

// Called when a task is about to block on taking a mutex
#define traceBLOCKING_ON_SEMAPHORE_TAKE(xMutex) \
    ESP_LOGW(TRACE_TAG, "Task '%s' is BLOCKING on mutex", pcTaskGetName(pxCurrentTCB))

// Called when a task successfully takes a mutex
#define traceSEMAPHORE_TAKE(xMutex, xTicksToWait) \
    ESP_LOGI(TRACE_TAG, "Task '%s' ACQUIRED mutex", pcTaskGetName(pxCurrentTCB))

// Called when a task gives a mutex
#define traceSEMAPHORE_GIVE(xMutex) \
    ESP_LOGI(TRACE_TAG, "Task '%s' RELEASED mutex", pcTaskGetName(pxCurrentTCB))

// End Trace Macro Definitions

#define LED_PIN     2
#define UART_NUM    UART_NUM_0

// All Tasks Have the Same Priority
#define TASK1PRIO   2
#define TASK2PRIO   2
#define TASK3PRIO   2
#define MONPRIO     0   // Lowest (monitor task)

SemaphoreHandle_t xMutex;
volatile bool ledOn;
volatile uint64_t ulIdleCycleCount = 0; // Counter for IDLE hook

static const char *TAG = "APP";

// IDLE Hook Implementation
void vApplicationIdleHook(void)
{
    ulIdleCycleCount++;
}

// Monitor Task
void vMonitorTask(void *arg)
{
    ESP_LOGI(TAG, "Monitor Task STARTED (Priority %d)", uxTaskPriorityGet(NULL));
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI("MONITOR", "Idle Hook Cycles (past 1s): %llu", ulIdleCycleCount);
        ulIdleCycleCount = 0;
    }
}

// GPIO LED Setup
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

// Task 1: LED ON and Busy-wait for 0.5s
void vTask1(void *arg)
{
    TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
    ESP_LOGI(TAG, "[%p] Task1 STARTED (Priority %d)", taskHandle, uxTaskPriorityGet(NULL));

    for (;;)
    {
        ESP_LOGI(TAG, "[%p] Task1 trying to take mutex...", taskHandle);
        xSemaphoreTake(xMutex, portMAX_DELAY);

        gpio_set_level(GPIO_NUM_2, 1);
        ledOn = true;
        ESP_LOGI(TAG, "[%p] LED turned ON", taskHandle);

        int64_t xStart = esp_timer_get_time();
        while (esp_timer_get_time() <= (xStart + 500000))
        {
            // Busy-wait for ~0.5s
        }

        xSemaphoreGive(xMutex);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Task 2: LED OFF and Delay 1s
void vTask2(void *arg)
{
    TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
    ESP_LOGI(TAG, "[%p] Task2 STARTED (Priority %d)", taskHandle, uxTaskPriorityGet(NULL));

    for (;;)
    {
        ESP_LOGI(TAG, "[%p] Task2 trying to take mutex...", taskHandle);
        xSemaphoreTake(xMutex, portMAX_DELAY);

        gpio_set_level(GPIO_NUM_2, 0);
        ledOn = false;
        ESP_LOGI(TAG, "[%p] LED turned OFF", taskHandle);

        vTaskDelay(pdMS_TO_TICKS(1000));

        xSemaphoreGive(xMutex);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Task 3: UART Status Report 
void vTask3(void *arg)
{
    TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
    ESP_LOGI(TAG, "[%p] Task3 STARTED (Priority %d)", taskHandle, uxTaskPriorityGet(NULL));

    for (;;)
    {
        if (ledOn)
            ESP_LOGI(TAG, "[%p] UART: LED is ON", taskHandle);
        else
            ESP_LOGI(TAG, "[%p] UART: LED is OFF", taskHandle);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    gpio_init_led();

    xMutex = xSemaphoreCreateMutex();
    if (xMutex != NULL)
    {
        ESP_LOGI(TAG, "Mutex created successfully");
        xTaskCreate(vTask1, "task1", 2048, NULL, TASK1PRIO, NULL);
        xTaskCreate(vTask2, "task2", 2048, NULL, TASK2PRIO, NULL);
    }
    else
    {
        ESP_LOGE(TAG, "Mutex creation FAILED");
    }

    xTaskCreate(vTask3, "task3", 2048, NULL, TASK3PRIO, NULL);
    xTaskCreate(vMonitorTask, "monitor", 2048, NULL, MONPRIO, NULL);

    ESP_LOGI(TAG, "All tasks created successfully");
}

