/*
ID:816041157, Zachary Doman
Chosen FSM implementation: Finite State Table

I have two states, (an on state and off state) and one event (char received), I handle the debouncing within the fsm task,

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TICKPERIOD_MS));
        tickCount++;
This is how I count my ticks, I use vtaskdelayuntil to ensure that I accept reads every 10ms (1 tick) 
When I read a character, I do checks to ignore certain characters and also check for the debounce period, once all checks are verified it sets events = 1 
ecause there is only one event, the bitmasking in get_event_mask() and get_next_event() always returns this event.
fsm_dispatch() then executes the corresponding action from the state table (turn LED ON or OFF) and updates the current state.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "driver/uart.h"

#include "esp_timer.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"

#define     LED_PIN                 2
#define     UART_NUM                UART_NUM_0
#define     BUF_SIZE                128

#define     STATESX                 2
#define     STATE_OFF               0
#define     STATE_ON                1

#define     EVENTSX                 1
#define     CHAR_REC                0x01
#define     TICKPERIOD_MS           10
#define     DEBOUNCEPERIOD_TICKS    50            

static const char *TAG = "FSM_DEBOUNCE";

unsigned int state, events, state_events, tickCount;
//Set Null
char nextChar = '\0';

void led_STATE_ON()
{
    gpio_set_level(LED_PIN, 1);
}

void led_STATE_OFF()
{
    gpio_set_level(LED_PIN,0);
}

void resetCount()
{
    tickCount = 0;
    ESP_LOGI(TAG, "Timer reset");
}

void null(){}
typedef void (*TPT) ();

struct state_table
{
    unsigned int active;
    unsigned int next_state;
    TPT action;
}state_table [EVENTSX][STATESX] =  {

{
    {1, STATE_ON,   led_STATE_ON},
    {1, STATE_OFF,  led_STATE_OFF}
}
};

/**
 * Get event mask for current state
 * Returns a bitmask of valid events for the current state
 */
unsigned int get_event_mask(unsigned int state)
{
    unsigned int mask = 0;
    for (int i = EVENTSX - 1; i >= 0; i--) {
        mask <<= 1;
        mask |= state_table[i][state].active;
    }
    return mask;
}

unsigned int get_next_event(void)
{
    unsigned int state_events = events & get_event_mask(state);
    unsigned int bitmask = 1;
    
    for (unsigned int i = 0; i < EVENTSX; i++) {
        if (state_events & bitmask) {
            events &= ~bitmask;  // Clear this event
            return i;
        }
        bitmask <<= 1;
    }
    return ~0;  // No valid event
}
/**
 * FSM Dispatcher - executes state transitions
 */
void fsm_dispatch(void)
{
    unsigned int event = get_next_event();
    
    if (event != ~0) {
        unsigned int last_state = state;
        
        // Execute action
        (state_table[event][state].action)();
        
        // Transition to next state
        state = state_table[event][state].next_state;
        
        ESP_LOGI(TAG, "Event: %d, State: %d->%d", event, last_state, state);
    }
}

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
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

}

/**
 * Non-blocking serial read
 * Returns true if a character was read, false otherwise
 */
bool serial_read_char(char *c)
{
    int len = uart_read_bytes(UART_NUM, (uint8_t*)c, 1, 0);
    return (len > 0);
}
void gpio_init_led(void) {
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

void fsm_task(void *pvParameters) 
{
    TickType_t last_wake_time = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "FSM Task started - Send characters via serial!");
    
    while (1) {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TICKPERIOD_MS));
        tickCount++;
        
        char received_char;
        if (serial_read_char(&received_char)) {                 
            if (received_char == '\r' || received_char == '\n')
            continue;
            if (tickCount < DEBOUNCEPERIOD_TICKS) {
            continue;
            } else {
                nextChar = received_char;
                events |= CHAR_REC;
            }
        }
        // FSM
        if (events & CHAR_REC) {
            fsm_dispatch();
            events &= ~CHAR_REC;  // Clear the event
            tickCount = 0;         // Reset timer for next debounce period
            ESP_LOGI(TAG, "Timer reset, State now: %d", state);
        }
        
    }
}

void app_main(){
    
    ESP_LOGI(TAG, "Debounce time: 500ms");
    ESP_LOGI(TAG, "Send characters to toggle LED");

    state = STATE_OFF;
    events = 0;
    tickCount = 0;

    gpio_init_led();
    uart_init();
    
    xTaskCreate(fsm_task, "fsm_task", 4096, NULL, 5, NULL);

}
