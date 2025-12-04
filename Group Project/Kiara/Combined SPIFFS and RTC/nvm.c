//Kiara Creed
//816036290

#include <stdio.h>
#include <string.h>
#include "esp_spiffs.h"
#include "esp_log.h"

#include "nvm.h"

#define MAX_LEN 37
#define MAX_LIN 20

static const char *TAG = "spiffs_src";

//The spiffs file is alloated 448K according the partition table eg where K is the size multiplier 1024

bool init_spiffs()
{
    ESP_LOGI(TAG, "Initializing SPIFFS");
    
    //configuration structure for the esp_vfs_spiffs_register
    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",   //choosing a mount point. This is the file path prefix associated with the filesystem
      .partition_label = NULL,  //using default SPIFFS partition. If set to NULL, first partition with subtype=spiffs will be used.
      .max_files = 5,           //max number of open files at once
      .format_if_mount_failed = true //format the partition if mounting fails
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf); //registers and mounts the filesystem to VFS with the configurations

    //error handling 
     if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return false;
    }

    //creating an empty file
    ESP_LOGI(TAG, "Creating empty file");
    FILE* f = fopen("/spiffs/nvm.txt", "w"); //opening file in write mode and clearing contents
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file");
        return false;
    }
    fclose(f);

    return true;
}

//especting message in the form of "HR= xxx BPM\n" inclusive of the spaces
bool singlewrite_spiffs(char (*message) [MAX_LEN])
{
    ESP_LOGI(TAG, "Opening file");
    FILE* f = fopen("/spiffs/nvm.txt", "a"); //opening file in append mode
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return false;
    }
    ESP_LOGI(TAG, "Writing to file");
    fprintf(f, "%s", *message);
    fclose(f);
    ESP_LOGI(TAG, "File written and closed");
    return true;
}


//to retrieve a single word from the device
char* singleread_spiffs()
{
    ESP_LOGI(TAG, "Opening file");
    FILE* f = fopen("/spiffs/nvm.txt", "r"); //opening file in read mode
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return NULL;
    }

    ESP_LOGI(TAG, "Reading file");
    static char line[MAX_LEN];
    fgets(line, sizeof(line), f);
    fclose(f);
    // strip newline
    char* pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }

    return line;
}

bool burstread_spiffs(char data[MAX_LIN][MAX_LEN])
{
    ESP_LOGI(TAG, "Opening file");
    FILE* f = fopen("/spiffs/nvm.txt", "r"); //opening file in read mode
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return false;
    }

    ESP_LOGI(TAG, "Reading file");
    //static char data[MAX_LIN][MAX_LEN];
    int line=0;
    while (line<MAX_LIN && !feof(f)&&!ferror(f))
    {
        fgets(data[line], MAX_LEN, f);
        // strip newline
        char* pos = strchr(data[line], '\n');
        if (pos) 
        {
            *pos = '\0';
        }
        line++;
    }
    
    fclose(f);
    return true;
}

void unmount_spiffs()
{
    esp_vfs_spiffs_unregister(NULL);
    ESP_LOGI(TAG, "SPIFFS unmounted");
}

