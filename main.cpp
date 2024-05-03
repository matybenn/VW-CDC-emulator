#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "vagComm.hpp"

#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE
#define LOG_TAG "MAIN"


extern "C" void app_main(void)
{
    VagComm radio(GPIO_NUM_18,GPIO_NUM_19,GPIO_NUM_23);
    ESP_LOGI(LOG_TAG,"hi");
    vTaskDelay(8000);
}