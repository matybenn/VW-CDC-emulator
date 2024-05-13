#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "vagComm.hpp"
//#include "esp_a2dp_api.h"

#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE
#define LOG_TAG "MAIN"

void identify(uint8_t stav);

extern "C" void app_main(void)
{
    VagComm radio(GPIO_NUM_18,GPIO_NUM_19,GPIO_NUM_23, identify);
    ESP_LOGI(LOG_TAG,"hi");
    vTaskDelay(500);
    radio.update(0,10,9,0);
    radio.set_status(1,1);
    
    vTaskDelay(10000);
}

void identify(uint8_t stav){
    switch (stav)
    {
    case PLAY:
        ESP_LOGI("callback", "PLAY");
        break;
    case PAUSE:
        ESP_LOGI("callback", "PAUSE");
        break;
    case FWD:
        ESP_LOGI("callback", "FWD");
        break;
    case BWD:
        ESP_LOGI("callback", "BWD");
        break;
    case SEEK_FWD_PUSH:
        ESP_LOGI("callback", "S_FWD");
        break;
    case SEEK_BWD_PUSH:
        ESP_LOGI("callback", "S_BWD");
        break;
    case CD1:
        ESP_LOGI("callback", "CD1");
        break;
    case CD2:
        ESP_LOGI("callback", "CD2");
        break;
    } 
}