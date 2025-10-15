#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "driver/pwm.h"
#include "driver/hw_timer.h"

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "esp_timer.h"

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

int16_t                 DataBuf[1000];
uint64_t                TimeBuf[1000];
uint64_t                initialTick;
uint64_t                finalTick;
static TaskHandle_t     xI2CTaskHandle = NULL;
static volatile         bool task_ready = false;

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
    uint8_t adc_data[2];
    // Initialize ADS1115
    i2c_master_ADS1115_init(I2C_MASTER_NUM);

    // Signal that task is ready
    task_ready = true;
    
    initialTick = esp_timer_get_time();
    for(int i = 0; i < 1000; i++){
        // Wait for timer notification (every 1ms = 1000Hz)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        // Start conversion
        i2c_master_ADS1115_startconversion(I2C_MASTER_NUM);

        // Busy wait
        if(i2c_master_ADS1115_busywait(I2C_MASTER_NUM, 20000) == ESP_OK){
            // If busy wait completes read from conversion register
            esp_err_t ret = i2c_master_ADS1115_read(I2C_MASTER_NUM, ADS1115_REG_CONVERSION, adc_data, 2);
            if (ret == ESP_OK){
                DataBuf[i] = (adc_data[0] << 8) | adc_data[1];
                TimeBuf[i] = esp_timer_get_time();
            } else{
                printf("Read error at sample %d\n", i);
                DataBuf[i] = 0;
                TimeBuf[i] = esp_timer_get_time();
            }
        }
        else{
            printf("Busy wait timeout at sample %d\n", i);
            DataBuf[i] = 0;
            TimeBuf[i] = esp_timer_get_time();
        }
    }
    finalTick = esp_timer_get_time();
    
    // Signal task completion
    task_ready = false;
    vTaskDelete(NULL);
}

static int int_log2_u32(uint32_t x) {
    if (x == 0) return -1; // undefined
    int r = 0;
    while (x >>= 1) r++;
    return r;
}

static uint32_t int_sqrt_u64(uint64_t x) {
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

float calcJitter(uint64_t arr[]){
    printf("Entered Calc jitter\n");
    uint64_t max =  0;
    uint64_t min =  999999;
    for(int i = 1; i < 1000; i++){
        uint64_t diff = arr[i] - arr[i-1];   
        if(diff > max){
            max = diff;
        }
        if(diff < min){
            min = diff;
        }
    }
    printf("Max value is %u and min value is %u\n", (uint32_t)max, (uint32_t)min);
return (max-min);
}

int32_t calcSNR_int(const int16_t arr[]) {
    // 1) sum + mean
    int64_t sum = 0;
    for (int i = 0; i < N; ++i) {
        sum += (int32_t)arr[i];
    }
    int32_t mean = (int32_t)(sum / N);

    // 2) sum squared differences in 64-bit
    uint64_t sumsq = 0;
    for (int i = 0; i < N; ++i) {
        int64_t diff = (int64_t)arr[i] - (int64_t)mean; // signed
        uint64_t sq = (uint64_t)(diff * diff); // non-negative
        sumsq += sq;
    }

    // variance (floor)
    uint64_t variance = sumsq / (uint64_t)N;

    // integer standard deviation
    uint32_t stddev = int_sqrt_u64(variance);

    printf("The mean of the data is %ld\n", (long)mean);
    printf("The standard deviation of the data is %u\n", stddev);

    if (stddev == 0) {
        printf("Std dev is 0 → SNR undefined (returning 0)\n");
        return 0;
    }

    // compute integer SNR = mean / stddev
    // mean is signed but should be positive for your DC measurement
    int32_t snr = mean / (int32_t)stddev;
    return snr;
}


void calcENOB_int(int32_t snr) {
    if (snr <= 0) {
        printf("SNR <= 0, ENOB undefined\n");
        return;
    }
    int log2snr = int_log2_u32((uint32_t)snr);
    if (log2snr < 0) {
        printf("ENOB undefined\n");
        return;
    }
    printf("The signal to noise ratio (SNR) is %d\n", snr);
    printf("ENOB (floor(log2(SNR))) = %d\n", log2snr);
}


void app_main(){
    i2c_master_init();
    // Create I2C task
    xTaskCreate(i2c_task, "i2c_task", 2048, NULL, 10, &xI2CTaskHandle);
    
    // Wait for task to be ready before starting timer
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Initialize and start hardware timer at 350Hz (2650 microseconds)
    hw_timer_init(timer_callback, NULL);
    hw_timer_alarm_us(2650, true);
    
    // Wait for data collection to complete (1000 samples at 500Hz = 1 seconds)
    vTaskDelay(pdMS_TO_TICKS(5000));
    hw_timer_deinit();
    printf("\n=== 500Hz Sampling Results ===\n");
    printf("Data\t\tTimestamp (us)\n");
    printf("-----------------------------\n");
    
    for(int i = 0; i < 1000; i++){
        printf("%d\t\t%u\n", DataBuf[i], (uint32_t)TimeBuf[i]);
    }

    fflush(stdout);
    
    // Calculate actual sample rate
    if (finalTick > initialTick) {
        printf("\nInitial Tick = %u\nFinal Tick = %u\n", (uint32_t)initialTick, (uint32_t)finalTick);
        int actual_rate = (1000*1000000) / ((finalTick - initialTick));
        printf("\nActual sample rate: %d Hz\n", actual_rate);
    }
    // Calculate jitter
    uint32_t jitter = calcJitter(TimeBuf);
    printf("Jitter of sampling system = %u us\n", jitter);

    int16_t SNR = calcSNR(DataBuf);
    calcENOB(SNR);
}

