#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "driver/pwm.h"
#include "driver/hw_timer.h"
#include "driver/uart.h"

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "esp_timer.h"

#define __ESP_FILE__                        __FILE__

#define UART_NUM                            UART_NUM_0
#define UART_BAUD                           115200
#define BUF_SIZE                            1024
#define RX_PIN                              3
#define TX_PIN                              1

#define I2C_MASTER_SCL_IO                   5
#define I2C_MASTER_SDA_IO                   4
#define I2C_MASTER_NUM                      I2C_NUM_0
#define ADC_ALERT_PIN                       14
#define WRITE_BIT                           I2C_MASTER_WRITE
#define READ_BIT                            I2C_MASTER_READ
#define ACK_CHECK_EN                        0x1
#define ACK_CHECK_DIS                       0x0
#define ACK_VAL                             0x0
#define NACK_VAL                            0x1
#define LAST_NACK_VAL                       0x2
#define ADS1115_SENSOR_ADDR                 0x48
#define ADS1115_REG_CONF                    0x01
#define ADS1115_REG_CONVERSION              0x00

#define N                                   1000
// Config for single-shot  mode, AIN0-GND, ±4.096V, 860SPS
// High Byte: OS=1 (start conversion) MUX=100, PGA=001, MODE=1 (single-shot)
// Low Byte: DR=111 (860SPS), COMP_QUE=11 (comparator disabled)
// Conversion time with 860 SPS: ~1.16ms

typedef struct {
    uint8_t avg_samples;  // Number of samples to average (1, 4, 8, 16)
    const char* name;
} avg_config_t;

// Test configurations
avg_config_t avg_configs[] = {
    {1, "No Averaging"},
    {4, "4x Averaging"},
};
#define NUM_CONFIGS (sizeof(avg_configs) / sizeof(avg_config_t))

int16_t                 DataBuf[1000];
uint64_t                TimeBuf[1000];
uint64_t                initialTick;
uint64_t                finalTick;
static TaskHandle_t     xI2CTaskHandle = NULL;
static volatile         bool task_ready = false;
uint8_t                 current_avg = 1;

static void uart_init_custom()
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
        
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
}

static void uart_send_string(const char* str)
{
    uart_write_bytes(UART_NUM, str, strlen(str));
}

static void uart_send_formatted(char* buffer, const char* format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, 256, format, args);
    va_end(args);
    uart_send_string(buffer);
}

static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;  
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = 1;
    conf.clk_stretch_tick = 150;
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    return ESP_OK;
}

static esp_err_t i2c_master_ADS1115_write(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1115_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_ADS1115_read(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1115_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1115_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t i2c_master_ADS1115_init(i2c_port_t i2c_num)
{
    uint8_t init_cfg[2] = {0x43, 0xE3};
    ESP_ERROR_CHECK(i2c_master_ADS1115_write(i2c_num, ADS1115_REG_CONF, init_cfg, 2));
    ets_delay_us(10);
    return ESP_OK;
}

static esp_err_t i2c_master_ADS1115_startconversion(i2c_port_t i2c_num)
{
    uint8_t cmd_data[2] = {0xC3, 0xE3};
    ESP_ERROR_CHECK(i2c_master_ADS1115_write(i2c_num, ADS1115_REG_CONF, cmd_data, 2));
    ets_delay_us(20);
    return ESP_OK;
}

static esp_err_t i2c_master_ADS1115_busywait (i2c_port_t i2c_num, uint64_t time_to_wait)

{
    uint8_t read_data[2];
    uint32_t starttime = esp_timer_get_time();

    while((esp_timer_get_time() - starttime) < time_to_wait){
        esp_err_t ret = i2c_master_ADS1115_read(I2C_MASTER_NUM, ADS1115_REG_CONF, read_data, 2);
        if(ret != ESP_OK){
            return ret;
        }

        if(read_data[0] & 0x80){
            return ESP_OK;
        }

        ets_delay_us(100);
    }
return ESP_ERR_TIMEOUT;
}

static int16_t read_averaged_sample(i2c_port_t i2c_num, uint8_t num_avg)
{
    int32_t sum = 0;
    uint8_t adc_data[2];
    
    for(uint8_t j = 0; j < num_avg; j++){
        i2c_master_ADS1115_startconversion(i2c_num);
        
        if(i2c_master_ADS1115_busywait(i2c_num, 20000) == ESP_OK){
            esp_err_t ret = i2c_master_ADS1115_read(i2c_num, ADS1115_REG_CONVERSION, adc_data, 2);
            if (ret == ESP_OK){
                int16_t sample = (adc_data[0] << 8) | adc_data[1];
                sum += sample;
            }
        }
    }
    
    return (int16_t)(sum / num_avg);
}

void timer_callback(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (task_ready) {
        vTaskNotifyGiveFromISR(xI2CTaskHandle, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR();

}

static void i2c_task(void *arg)
{   
    // Initialize ADS1115
    i2c_master_ADS1115_init(I2C_MASTER_NUM);

    // Signal that task is ready
    task_ready = true;
    
    initialTick = esp_timer_get_time();
    for(int i = 0; i < 1000; i++){
        // Wait for timer notification
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        // Read with averaging
        DataBuf[i] = read_averaged_sample(I2C_MASTER_NUM, current_avg);
        TimeBuf[i] = esp_timer_get_time();
    }
    finalTick = esp_timer_get_time();
    
    // Signal task completion
    task_ready = false;
    vTaskDelete(NULL);
}

static int int_log2_u32(uint32_t x) 
{
    if (x == 0) return -1; // undefined
    int r = 0;
    while (x >>= 1) r++;
    return r;
}

static uint32_t int_sqrt_u64(uint64_t x)
{
    uint64_t r = 0;
    uint64_t bit = (uint64_t)1 << 62; // top even bit
    while (bit > x) bit >>= 2;
    while (bit) {
        uint64_t tmp = r + bit;
        if (x >= tmp) {
            x -= tmp;
            r = (r >> 1) + bit;
        } else {
            r = r >> 1;
        }
        bit >>= 2;
    }
    return (uint32_t)r;
}

float calcJitter(uint64_t arr[], char* buffer)
{
    uint64_t max = 0;
    uint64_t min = 999999;
    for(int i = 1; i < 1000; i++){
        uint64_t diff = arr[i] - arr[i-1];   
        if(diff > max){
            max = diff;
        }
        if(diff < min){
            min = diff;
        }
    }
    uart_send_formatted(buffer, "Max: %u, Min: %u\r\n", (uint32_t)max, (uint32_t)min);
    return (max-min);
}

int32_t calcSNR_int(const int16_t arr[], char* buffer)
{
    // Calculate mean
    int64_t sum = 0;
    for (int i = 0; i < N; ++i) {
        sum += (int32_t)arr[i];
    }
    int32_t mean = (int32_t)(sum / N);

    // Calculate variance
    uint64_t sumsq = 0;
    for (int i = 0; i < N; ++i) {
        int64_t diff = (int64_t)arr[i] - (int64_t)mean;
        uint64_t sq = (uint64_t)(diff * diff);
        sumsq += sq;
    }

    uint64_t variance = sumsq / (uint64_t)N;
    uint32_t stddev = int_sqrt_u64(variance);

    uart_send_formatted(buffer, "Mean: %ld, StdDev: %u\r\n", (long)mean, stddev);

    if (stddev == 0) {
        return 0;
    }

    int32_t snr = mean / (int32_t)stddev;
    return snr;
}

void calcENOB_int(int32_t snr, char* buffer)
{
    if (snr <= 0) {
        uart_send_string("SNR <= 0, ENOB undefined\r\n");
        return;
    }
    int log2snr = int_log2_u32((uint32_t)snr);
    if (log2snr < 0) {
        uart_send_string("ENOB undefined\r\n");
        return;
    }
    uart_send_formatted(buffer, "SNR: %d, ENOB: %d\r\n", snr, log2snr);
}

// Calculate and print PMF via UART
void print_PMF(const int16_t arr[], const char* label, char* buffer)
{
    uart_send_formatted(buffer, "\r\n=== PMF Data: %s ===\r\n", label);
    uart_send_string("Value,Count\r\n");
    
    // Find min and max values
    int16_t min_val = arr[0];
    int16_t max_val = arr[0];
    for(int i = 1; i < N; i++){
        if(arr[i] < min_val) min_val = arr[i];
        if(arr[i] > max_val) max_val = arr[i];
    }
    
    // Count occurrences
    for(int16_t val = min_val; val <= max_val; val++){
        int count = 0;
        for(int i = 0; i < N; i++){
            if(arr[i] == val) count++;
        }
        if(count > 0){
            uart_send_formatted(buffer, "%d,%d\r\n", val, count);
        }
    }
}

void app_main(){
    char tx_buffer[256];
    
    // Initialize UART 
    uart_init_custom();
    vTaskDelay(pdMS_TO_TICKS(100)); // Let UART stabilize
    
    uart_send_string("\r\n\r\n=================================\r\n");
    uart_send_string("ESP8266 ADC System Starting...\r\n");
    uart_send_string("UART Initialized on TX=GPIO1, RX=GPIO3\r\n");
    uart_send_string("=================================\r\n\r\n");
    
    // Initialize I2C
    i2c_master_init();
    
    uart_send_string("\r\n=== SNR and ENOB vs Hardware Averaging ===\r\n");
    uart_send_string("Config\t\t\tSNR\tENOB\r\n");
    uart_send_string("--------------------------------------------\r\n");
    
    // Test each averaging configuration
    for(int cfg = 0; cfg < NUM_CONFIGS; cfg++){
        current_avg = avg_configs[cfg].avg_samples;
        
        uart_send_formatted(tx_buffer, "\r\nTesting: %s\r\n", avg_configs[cfg].name);
        
        // Reset task_ready flag
        task_ready = false;
        
        // Create I2C task
        xTaskCreate(i2c_task, "i2c_task", 2048, NULL, 10, &xI2CTaskHandle);
        
        // Wait for task to be ready
        vTaskDelay(pdMS_TO_TICKS(200));
        
        // Start hardware timer
        hw_timer_init(timer_callback, NULL);
        hw_timer_alarm_us(2650, true);
        
        // Wait for completion
        vTaskDelay(pdMS_TO_TICKS(5000));
        hw_timer_deinit();
        
        // Calculate SNR and ENOB
        int32_t snr = calcSNR_int(DataBuf, tx_buffer);
        int enob = (snr > 0) ? int_log2_u32((uint32_t)snr) : 0;
        
        uart_send_formatted(tx_buffer, "%s\t\tSNR=%d\tENOB=%d\r\n", avg_configs[cfg].name, snr, enob);
        
        // Print PMF for first two configurations
        if(cfg < 2){
            print_PMF(DataBuf, avg_configs[cfg].name, tx_buffer);
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
