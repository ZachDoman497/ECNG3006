//Kiara Creed
//816036290

//Lab 1 Question 5:
// Amend the C code in William Ch 4, Section 4.7 "Demonstration Cyclic Executive Code" as needed for your platform.
// Key items to look out for:
// you will need a serial print function - locate an appropriate library for your compiler (4 marks)
// the "sleep" and "time" function(s) will need to be written - based on a hardware timer that uses an isr to increment a time structure (8 marks)

//From William section 4.7:
//each task prints a message then sleeps for 1 sec
//The slots are set for a 5sec period with the burn task telling us how long it has been waiting for the next 5sec tick to arrive

/* Demo table-based cyclic task dispatcher for Linux with
multiple task slots */
#include <stdio.h>
#include <ctype.h>
//#include <unistd.h> //Linux; Used for sleep(), sysconf(), and _SC_CLK_TCK
//#include <sys/times.h> // gcc cyclicx.c -o cyclicx. Linux; Used for times() and struct tms for CPU/process time accounting.

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/hw_timer.h"

#define SLOTX 4 //number of frames in the bigger cycle
#define CYCLEX 5 //number of slots in each frame
//#define SLOT_T 5000 // 5 sec slot time

#define TEST_ONE_SHOT    false        
#define TEST_RELOAD      true         

static const char *TAG = "Lab1 q5";

int cycle=0, slot=0, count=0; 
//int tps; //tps is ticks per second
//clock_t now, then; //now is the curent process time in ticks
//then is the last time recorded at the end of the previous slot

bool slot_done = false; 

// the 5 tasks
void one() { 
    printf("%lu: task 1 running\n", (unsigned long)(esp_timer_get_time()/1000));    //used for question 6
    //printf("task 1 running\n");      //used for question 5
    vTaskDelay(1000 / portTICK_RATE_MS); //delay for 1 second
}
void two() {
    printf("%lu: task 2 running\n",(unsigned long)(esp_timer_get_time()/1000));
    //printf("task 2 running\n");
    vTaskDelay(2000 / portTICK_RATE_MS); //delay for 2 seconds
}
void three() {
    printf("%lu: task 3 running\n", (unsigned long)(esp_timer_get_time()/1000));
    //printf("task 3 running\n");
    vTaskDelay(3000 / portTICK_RATE_MS); //delay for 3 seconds
}
void four() {
    printf("%lu: task 4 running\n", (unsigned long)(esp_timer_get_time()/1000));
    //printf("task 4 running\n");
    vTaskDelay(4000 / portTICK_RATE_MS); //delay for 4 seconds
}
void five() {
    printf("%lu: task 5 running\n", (unsigned long)(esp_timer_get_time()/1000));
    //printf("task 5 running\n");
    vTaskDelay(5000 / portTICK_RATE_MS); //delay for 5 seconds
}

void burn() {
/* 
Williams Code
    clock_t bstart = times(&n); //recording the CPU time when the function starts; times(&n) records time in ticks
    while ( ((now=times(&n))-then) < SLOT_T*tps/1000 ) { //so elapsed time>= slot_t(5s) 
        //the slot_t*tps/1000 = 5000*tps/1000 so it converts the slot length to ticks
        // burn time here
    }
    printf("burn time = %2.2dms\n\n", (times(&n)-bstart)*(1000/tps)); //prints how long the burn function lasted
    //(times(&n) - bstart) = number of ticks spent burning.
    //1000/tps to convert ticks into milliseconds.
    then = now;
    cycle = CYCLEX; 
*/

    //tick frequency is 10ms
    TickType_t xStart,xDiff;   
    slot_done=false;
    xStart= xTaskGetTickCount();
    printf("Starting tick count= %us \n", xStart);
    ESP_LOGI(TAG, "Starting burn. \n");
    while(slot_done!=true)
    {
        vTaskDelay(1); 
    }
    xDiff= xTaskGetTickCount() - xStart;
    printf("Difference in tick count= %us \n", xDiff);
    printf("Burn time = %us \n", xDiff/100);
    cycle = CYCLEX;
}

//sets up the actual CEC
void (*ttable[SLOTX][CYCLEX])() = //a [4][5] array so we have 4 frames and 5 slots per frame
{
    {one, two, burn, burn, burn},
    {one, three, burn, burn, burn},
    {one, four, burn, burn, burn},
    {burn, burn, burn, burn, burn}
};

void hw_timer_callback(void *arg)
{
    count++;
    if (count>=5) //waiting until 5 seconds
    {
        slot_done=true;
        count=0;
        hw_timer_alarm_us(1000000, TEST_ONE_SHOT);
    }
    else
    {
        hw_timer_alarm_us(1000000, TEST_ONE_SHOT);   //timer with reload for 1s
    }
}

void app_main () {
    //tps = sysconf(_SC_CLK_TCK); 
    //printf("clock ticks/sec = %d\n\n", tps);
    ESP_LOGI(TAG, "Initializing timer \n");
    hw_timer_init(hw_timer_callback,NULL);
    hw_timer_alarm_us(1000000, TEST_ONE_SHOT);   //timer with reload for 1s

    ESP_LOGI(TAG, "Starting the tasks");
    while (1) 
    {
        for(slot=0; slot<SLOTX; slot++) //row
        {
            for(cycle=0; cycle<CYCLEX; cycle++) //column
            {
                (*ttable[slot][cycle])(); // dispatch next task from table
            }
        }
    }
    
    return;
    //vTaskDelay(1000 / portTICK_RATE_MS);
    //hw_timer_deinit();
}
