//Kiara Creed
//816036290

#include <stdbool.h>
#include <stddef.h>
#define MAX_LEN 37
#define MAX_LIN 20

bool init_spiffs();
bool singlewrite_spiffs(char (*message) [MAX_LEN]);
char* singleread_spiffs();
bool burstread_spiffs(char data[MAX_LIN][MAX_LEN]);
void unmount_spiffs();
