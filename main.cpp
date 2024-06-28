#include "vagComm.hpp"

#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"



#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE
#define LOG_TAG "MAIN"

/*prototype*/
void identify(uint8_t stav, VagComm* radio);

extern "C" void app_main(void)
{
    ESP_LOGI(LOG_TAG,"hi"); 
    VagComm radio(GPIO_NUM_18,GPIO_NUM_19,GPIO_NUM_23, identify); //set up our radio      
    vTaskDelay(10000);

}

void identify(uint8_t stav, VagComm* radio){
    switch (stav)
    {
    case PLAY:
        ESP_LOGI("callback", "PLAY");
        radio->set_status(1,1);
        break;
    case PAUSE:
        ESP_LOGI("callback", "STOP");
        radio->set_status(0,0); //set idle mode
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
    case CD1: /*BLT*/
        ESP_LOGI("callback", "CD1");
        break;
    case CD2: /*AUX*/
        ESP_LOGI("callback", "CD2");
        //todo - pause bluetooth, set gpio high to switch audio MUX
        radio->update(1,0,0,0);
        radio->set_status(1,0); //non-idle, but not increment digits
        break;
    } 
}
