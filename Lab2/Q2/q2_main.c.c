// Kiara Creed & Zachary Doman
// 816036290   & 816041157

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

#define LED_PIN     2
#define UART_NUM    UART_NUM_0

#define TASK1PRIO   3
#define TASK2PRIO   2
#define TASK3PRIO   1

SemaphoreHandle_t xMutex;
volatile bool ledOn;

static const char *TAG = "APP";

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


// Task 1: Turns LED ON and busy-waits for 0.5s
void vTask1(void *arg)
{
    TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
    ESP_LOGI(TAG, "[%p] Task1 STARTED (Priority %d)", taskHandle, uxTaskPriorityGet(NULL));

        ESP_LOGI(TAG, "[%p] Entered Task1 loop", taskHandle);

        ESP_LOGI(TAG, "[%p] Task1 trying to take mutex...", taskHandle);
        xSemaphoreTake(xMutex, portMAX_DELAY);
        ESP_LOGI(TAG, "[%p] Task1 ACQUIRED mutex", taskHandle);

        gpio_set_level(GPIO_NUM_2, 1);
        ledOn = true;
        ESP_LOGI(TAG, "[%p] LED turned ON", taskHandle);

        int64_t xStart = esp_timer_get_time();
        ESP_LOGI(TAG, "[%p] Busy-wait start @ %u µs", taskHandle, (uint32_t)xStart);

        while (esp_timer_get_time() <= (xStart + 500000)) {
            // simulate CPU load
        }

        int64_t xEnd = esp_timer_get_time();
        ESP_LOGI(TAG, "[%p] Busy-wait done @ %u µs (duration: %u µs)", taskHandle, (uint32_t)xEnd, (uint32_t)(xEnd - xStart));

        xSemaphoreGive(xMutex);
        ESP_LOGI(TAG, "[%p] Task1 RELEASED mutex", taskHandle);

        ESP_LOGI(TAG, "[%p] Task1 delaying 10ms", taskHandle);
        vTaskDelay(pdMS_TO_TICKS(10));

    vTaskDelete(NULL);
}

// Task 2: Turns LED OFF and delays 1s
void vTask2(void *arg)
{
    TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
    ESP_LOGI(TAG, "\n\n[%p] Task2 STARTED (Priority %d)", taskHandle, uxTaskPriorityGet(NULL));

        ESP_LOGI(TAG, "[%p] Entered Task2 loop", taskHandle);

        ESP_LOGI(TAG, "[%p] Task2 trying to take mutex...", taskHandle);
        xSemaphoreTake(xMutex, portMAX_DELAY);
        ESP_LOGI(TAG, "[%p] Task2 ACQUIRED mutex", taskHandle);

        gpio_set_level(GPIO_NUM_2, 0);
        ledOn = false;
        ESP_LOGI(TAG, "[%p] LED turned OFF", taskHandle);

        ESP_LOGI(TAG, "[%p] Task2 delaying 1s while holding mutex", taskHandle);
        vTaskDelay(pdMS_TO_TICKS(1000));

        xSemaphoreGive(xMutex);
        ESP_LOGI(TAG, "[%p] Task2 RELEASED mutex", taskHandle);

        ESP_LOGI(TAG, "[%p] Task2 delaying 10ms (post-release)", taskHandle);
        vTaskDelay(pdMS_TO_TICKS(10));

    vTaskDelete(NULL);
}

// Task 3: Reports LED state
void vTask3(void *arg)
{
    TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
    ESP_LOGI(TAG, "\n\n[%p] Task3 STARTED (Priority %d)", taskHandle, uxTaskPriorityGet(NULL));

        ESP_LOGI(TAG, "[%p] Entered Task3 loop", taskHandle);

        if (ledOn) {
            ESP_LOGI(TAG, "[%p] UART: LED is ON", taskHandle);
        } else {
            ESP_LOGI(TAG, "[%p] UART: LED is OFF", taskHandle);
        }

        ESP_LOGI(TAG, "[%p] Task3 delaying 1s", taskHandle);
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
        xTaskCreate(vTask1, "task1", 2048, NULL, TASK1PRIO, NULL);
        xTaskCreate(vTask2, "task2", 2048, NULL, TASK2PRIO, NULL);
        xTaskCreate(vTask3, "task3", 2048, NULL, TASK3PRIO, NULL);
    }
    else
    {
        ESP_LOGE(TAG, "Mutex creation FAILED");
    }

    ESP_LOGI(TAG, "All tasks created successfully");
}
