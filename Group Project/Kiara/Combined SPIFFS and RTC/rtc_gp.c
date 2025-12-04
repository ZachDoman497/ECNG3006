//Kiara Creed
//816036290

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "driver/i2c.h"
#include "driver/i2c.h"
#include "rtc_gp.h"

//DS3231 register addresses 
#define DS3231_ADDR                         0x68
#define DS3231_ADDR_TIME                    0x00            //address of the seconds register
#define DS3231_ADDR_HOUR                    0x02            //address of the hours register
//#define DS3231_ADDR_ALARM1                  0x07            
//#define DS3231_ADDR_ALARM2                  0x0b        
#define DS3231_ADDR_CONTROL                 0x0e            //address of the control register (set the SQW freq using bit 4 and 3)
#define DS3231_ADDR_STATUS                  0x0f            //address of the status register (has all the flags)
//#define DS3231_ADDR_AGING                   0x10
//#define DS3231_ADDR_TEMP                    0x11

//timekeeping registers
#define DS3231_12HOUR_FLAG                  0x40            
#define DS3231_12HOUR_MASK                  0x1f            //used to extract the lower 5 bits when reading in 12-hour mode
#define DS3231_PM_FLAG                      0x20
#define DS3231_MONTH_MASK                   0x1f

//bit masks for status register
#define DS3231_STAT_OSCILLATOR              0x80
#define DS3231_STAT_32KHZ                   0x08
#define DS3231_STAT_ALARM_2                 0x02
#define DS3231_STAT_ALARM_1                 0x01
//bit masks for control register. Can enable the different functions from here
#define DS3231_CTRL_OSCILLATOR              0x80
#define DS3231_CTRL_TEMPCONV                0x20
#define DS3231_CTRL_INTCN                   0x04
#define DS3231_CTRL_ALARM2_INT              0x02
#define DS3231_CTRL_ALARM1_INT              0x01
#define DS3231_SQW_RS                       0x18

//definitions for I2C 
#define I2C_MASTER_NUM                      I2C_NUM_0        //I2C port number for master dev
#define SDA_PULLUP_ENABLE                   1
#define SCL_PULLUP_ENABLE                   1
#define ACK_CHECK_EN                        0x1              //I2C master will check ack from slave

//modes for affecting the bits in the register
#define DS3231_SET                          1
#define DS3231_REPLACE                      2
#define DS3231_CLEAR                        3

//square wave frequencies
#define DS3231_SQWAVE_1HZ                   0x00              // RS2=0, RS1=0
#define DS3231_SQWAVE_1024HZ                0x08              // RS2=0, RS1=1
#define DS3231_SQWAVE_4096HZ                0x10              // RS2=1, RS1=0
#define DS3231_SQWAVE_8192HZ                0x18              // RS2=1, RS1=1

#define REG_SIZE                            1  
#define TIME_LEN                            20                //length of the string contraining the date and time including null terminator                         

static const char *TAG = "rtc_gp";

uint8_t bcd2dec(uint8_t val)
{
    return (val >> 4) * 10 + (val & 0x0f);
}

uint8_t dec2bcd(uint8_t val)
{
    return ((val / 10) << 4) + (val % 10);
}

esp_err_t ds3231_i2c_init(gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    //initializing i2c to communicate with the rtc
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER; 
    conf.sda_io_num = sda_gpio;
    conf.sda_pullup_en = SDA_PULLUP_ENABLE; 
    conf.scl_io_num = scl_gpio;
    conf.scl_pullup_en = SCL_PULLUP_ENABLE;
    conf.clk_stretch_tick = 300; // 300 ticks, Clock stretch is about 210us.
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));  //initializing config
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));    //installing the driver

    //initalizing the rtc    
    //disabling alarms
    ds3231_set_flag(DS3231_ADDR_CONTROL, DS3231_CTRL_ALARM2_INT, DS3231_CLEAR);
    ds3231_set_flag(DS3231_ADDR_CONTROL, DS3231_CTRL_ALARM1_INT, DS3231_CLEAR);
    //setting frequency of the square wave to X
    ds3231_set_flag(DS3231_ADDR_CONTROL, DS3231_CTRL_INTCN, DS3231_CLEAR);
    //disabling the 32k output
    ds3231_set_flag(DS3231_ADDR_STATUS, DS3231_STAT_32KHZ, DS3231_CLEAR);
    ds3231_disable_squarewave();
    //setting frequency of the square wave to 4096Hz
    uint8_t flag = 0;
    ds3231_get_flag(DS3231_ADDR_CONTROL, 0xff, &flag);
    flag &= ~DS3231_SQW_RS; //clear bits RS1(bit 3) and RS2(bit 4)
    flag |= ((uint8_t)DS3231_SQWAVE_4096HZ & 0x18);  //ORing RS1 and RS2
    ds3231_set_flag(DS3231_ADDR_CONTROL, flag, DS3231_REPLACE);

    return ESP_OK;
}

/**
 * structure to write data to reg address on ds3231
 * ________________________________________________________________________________
 * | start | slave_addr + write_bit + ack | reg_address + ack | data + ack  | stop |
 * --------|-----------------------------|--------------------|-------------|------|
 *
 * @param reg_address slave register address
 * @param data data to send
 * @param data_size size of data to send
 * @param REG_SIZE size of register in DS3231
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
esp_err_t i2c_master_ds3231_write(uint8_t reg_address, uint8_t *data, size_t data_size)
{
    //error checking
    if (!data || !data_size) return ESP_ERR_INVALID_ARG;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();   //creating a command link
    //populating the command link with data to send to the slave
    i2c_master_start(cmd);  //sending start bit
    i2c_master_write_byte(cmd, (DS3231_ADDR  << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN); //sending ds3231 address and the write bit then waiting for ack from slave
    i2c_master_write(cmd, &reg_address, REG_SIZE, ACK_CHECK_EN); //sending ds3231 register address, size and waiting for ack
    i2c_master_write(cmd, data, data_size, ACK_CHECK_EN);  //sending data and data size
    i2c_master_stop(cmd);   //sending stop bit

    esp_err_t res = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);    //executing the I2C 
    if (res != ESP_OK)
    {
		ESP_LOGE(TAG, "Could not write to DS3231 register at 0x%02x: %d", reg_address, res);
    }
    i2c_cmd_link_delete(cmd);

    return res;
}

/**
 * structure to write/read data from specific reg address on ds3231
 * * _________________________________________________________
 * | start | slave_addr + write_bit + ack | reg_address + ack |
 *  --------|-----------------------------|-------------------|
 *  _________________________________________________________________________________________
 * | repeated start | slave_addr + read_bit + ack | read data + ack(after each byte)  | stop |
 *  ----------------|-----------------------------|-----------------------------------|------|
 *
 * @param reg_address slave reg address
 * @param REG_SIZE size of slave register
 * @param data data to read
 * @param data_size data size
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */

esp_err_t i2c_master_ds3231_read(uint8_t reg_address, uint8_t *data, size_t data_size)
{
    //error checking
    if (!data || !data_size) return ESP_ERR_INVALID_ARG;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    //sending the address for the rtc register we want to read from
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, DS3231_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write(cmd, &reg_address, REG_SIZE, ACK_CHECK_EN);

    //reading the data from that register
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
	i2c_master_read(cmd, data, data_size, I2C_MASTER_LAST_NACK);
	i2c_master_stop(cmd);

    esp_err_t res = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Could not read from DS3231 register at 0x%02x: %d", reg_address, res);
    }
    i2c_cmd_link_delete(cmd);

    return res;
}

//can use mutex for this
esp_err_t ds3231_set_time(struct tm *time)
{
    uint8_t data[7];

    // time/date data
    data[0] = dec2bcd((uint8_t)time->tm_sec);
    data[1] = dec2bcd((uint8_t)time->tm_min);
    data[2] = dec2bcd((uint8_t)time->tm_hour);
    data[2] &= ~(DS3231_12HOUR_FLAG);                   // Clear bit 6 so its in 24-hour mode
    data[3] = dec2bcd((uint8_t)time->tm_wday + 1);      //this is the day; tm_wday is 0–6 (Sun–Sat) in struct tm; DS3231 expects 1–7, so add 1; we dont need this
    data[4] = dec2bcd((uint8_t)time->tm_mday);          //day of mouth month (1-31)
    data[5] = dec2bcd((uint8_t)time->tm_mon + 1);       //tm_mon is 0–11; DS3231 expects 1–12, so add 1
    data[6] = dec2bcd((uint8_t)time->tm_year - 100);    //tm_year is 125 but DS3231 needs 0-99

    //The DS3231 allows sequential writes so if you start at the first register and send multiple bytes, they automatically go into the next registers.
    //sending the current time to the rtc
    i2c_master_ds3231_write(DS3231_ADDR_TIME, data, 7);
    
    return ESP_OK;
}

//allows you change one bit in a register
esp_err_t ds3231_set_flag(uint8_t reg_address, uint8_t mask, int mode)
{
    uint8_t data;

    //get status register
    esp_err_t res = i2c_master_ds3231_read(reg_address, &data, 1);
    if (res != ESP_OK)
        return res;
    //clear the flag
    if (mode == DS3231_REPLACE)
        data = mask;
    else if (mode == DS3231_SET)
        data |= mask;
    else //clear
        data &= ~mask;

    return i2c_master_ds3231_write(reg_address, &data, 1);

}

esp_err_t ds3231_get_flag(uint8_t reg_address, uint8_t mask, uint8_t *flag)
{
    uint8_t data;

    //get register
    esp_err_t res = i2c_master_ds3231_read(reg_address, &data, 1);
    if (res != ESP_OK)
        return res;

    // return only requested flag
    *flag = (data & mask);
    return ESP_OK;
}

esp_err_t ds3231_enable_squarewave()
{
    esp_err_t res= (ds3231_set_flag(DS3231_ADDR_CONTROL, DS3231_CTRL_INTCN, DS3231_CLEAR));

    return res;
}

esp_err_t ds3231_disable_squarewave()
{
    esp_err_t res = ds3231_set_flag(DS3231_ADDR_CONTROL, DS3231_CTRL_INTCN, DS3231_SET);

    return res;
}

esp_err_t ds3231_get_time(struct tm *time)
{
    uint8_t data[7];

    i2c_master_ds3231_read(DS3231_ADDR_TIME, data, 7);

    //convert to unix time structure
    time->tm_sec = bcd2dec(data[0]);
    time->tm_min = bcd2dec(data[1]);
    if (data[2] & DS3231_12HOUR_FLAG) //checking if the time is in 12hr mode
    {
        time->tm_hour = bcd2dec(data[2] & DS3231_12HOUR_MASK) - 1;
        //checking if its AM or PM
        if (data[2] & DS3231_PM_FLAG) time->tm_hour += 12;
    }
    else time->tm_hour = bcd2dec(data[2]); //24h
    //time->tm_wday = bcd2dec(data[3]) - 1;
    time->tm_mday = bcd2dec(data[4]);
    time->tm_mon  = bcd2dec(data[5] & DS3231_MONTH_MASK) - 1;
    time->tm_year = bcd2dec(data[6]) + 100;
    //time->tm_isdst = 0;
    //time->tm_yday = days_since_january_1st(time->tm_year, time->tm_mon, time->tm_mday);

    return ESP_OK;
}

char* dds3231_ftime(struct tm *time)
{
    static char date [TIME_LEN]; //entire date and time to combine with other data including the null terminator
    //formating the time and date and storing it in data
    snprintf(date, sizeof(date), "%04u-%02u-%02u,%02u:%02u:%02u",
            (unsigned)time->tm_year + 1900,
            (unsigned)time->tm_mon + 1,
            (unsigned)time->tm_mday,
            (unsigned)time->tm_hour,
            (unsigned)time->tm_min,
            (unsigned)time->tm_sec);
    
    return date;
}
