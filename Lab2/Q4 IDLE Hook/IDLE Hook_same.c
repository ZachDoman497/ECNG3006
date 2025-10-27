//Ameera Chaitram
//816024370

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_log.h"

#define LED_PIN 2

#define TASK1PRIO 2
#define TASK2PRIO 2
#define TASK3PRIO 2

SemaphoreHandle_t xMutex;
volatile bool ledOn = false;
volatile uint64_t idleCount = 0;

static const char *TAG = "IDLE_SAME";

void vApplicationIdleHook(void)
{
    idleCount++;
}

void vMonitorTask(void *arg)
{
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI("MONITOR", "Idle cycles in last second: %llu", idleCount);
        idleCount = 0;
    }
}

void gpio_init_led(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
    gpio_set_level(LED_PIN, 0);
}

void vTask1(void *arg)
{
    for (;;)
    {
        xSemaphoreTake(xMutex, portMAX_DELAY);
        gpio_set_level(LED_PIN, 1);
        ledOn = true;
        ESP_LOGI(TAG, "Task1: LED ON");
        vTaskDelay(pdMS_TO_TICKS(500));
        xSemaphoreGive(xMutex);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void vTask2(void *arg)
{
    for (;;)
    {
        xSemaphoreTake(xMutex, portMAX_DELAY);
        gpio_set_level(LED_PIN, 0);
        ledOn = false;
        ESP_LOGI(TAG, "Task2: LED OFF");
        vTaskDelay(pdMS_TO_TICKS(1000));
        xSemaphoreGive(xMutex);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void vTask3(void *arg)
{
    for (;;)
    {
        ESP_LOGI(TAG, "Task3: LED state = %s", ledOn ? "ON" : "OFF");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    gpio_init_led();

    xMutex = xSemaphoreCreateMutex();

    xTaskCreate(vTask1, "Task1", 2048, NULL, TASK1PRIO, NULL);
    xTaskCreate(vTask2, "Task2", 2048, NULL, TASK2PRIO, NULL);
    xTaskCreate(vTask3, "Task3", 2048, NULL, TASK3PRIO, NULL);
    xTaskCreate(vMonitorTask, "Monitor", 2048, NULL, 0, NULL);

    ESP_LOGI(TAG, "Tasks created with same priorities.");
}
