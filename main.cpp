#include "vagComm.hpp"
#include "esp_a2dp_api.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_avrc_api.h"
#include "esp_gap_bt_api.h"
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"

/*definitions*/
#define WAKE_UP GPIO_NUM_26 //ovládací pin

/*global var*/
i2s_chan_handle_t tx_handle = NULL;
VagComm *radio;
bool avrc_set = false;
uint16_t track = 0;

/*prototype*/
void identify(uint8_t stav);
void a2dp_event(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
void pcm_data_rx(const uint8_t *buf, uint32_t len);
void avrc_callback(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);
void blt_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);

extern "C" void app_main(void)
{
    //rtc gpio init (wake up pin)
    rtc_gpio_set_direction(WAKE_UP,RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_dis(WAKE_UP);
    rtc_gpio_pulldown_en(WAKE_UP);
    rtc_gpio_init(WAKE_UP);
    //bt intit
    nvs_flash_init();
    esp_bt_controller_config_t blt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&blt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    esp_bluedroid_init();
    esp_bluedroid_enable();
    
    esp_avrc_ct_register_callback(avrc_callback);
    esp_avrc_ct_init();

    esp_a2d_register_callback(a2dp_event);
    esp_a2d_sink_register_data_callback(pcm_data_rx);
    esp_a2d_sink_init();

    esp_bt_gap_register_callback(blt_gap_callback);
    esp_bt_dev_set_device_name("Škoda Octavia II");
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    //i2s init
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_NUM_32,
            .ws = GPIO_NUM_27,
            .dout = GPIO_NUM_25,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    i2s_new_channel(&chan_cfg, &tx_handle, NULL);
    i2s_channel_init_std_mode(tx_handle, &std_cfg);
    
    radio = new VagComm(GPIO_NUM_18,GPIO_NUM_19,GPIO_NUM_23, identify); //set up our radio
    
    //po 1000t kdy je na WAKE_UP pinu 0 přejde do režimu deep sleep, vybudí se pomocí HIGH na WAKE UP 
    //(Zapnutí rádia)
    
    while (true)
    {
        if(rtc_gpio_get_level(WAKE_UP) == 0){
            vTaskDelay(1000);
            //proti uspání/odpojení při startu motoru
            if(rtc_gpio_get_level(WAKE_UP) == 0){
                break;
            }
        }
        vTaskDelay(500);
    }
    
    //deinitalize and sleep
    if(avrc_set){
        esp_avrc_ct_send_passthrough_cmd(5, ESP_AVRC_PT_CMD_PAUSE, ESP_AVRC_PT_CMD_STATE_PRESSED);
    }
    esp_a2d_sink_deinit();
    esp_avrc_ct_deinit();
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    delete radio;
    radio = nullptr;
    esp_sleep_enable_ext0_wakeup(WAKE_UP, 1);
    //ESP_LOGI("deinit", "deep sleep");
    esp_deep_sleep_start();
}

void identify(uint8_t stav){
    switch (stav)
    {
    case PLAY:
        //ESP_LOGI("callback", "PLAY");
        if(avrc_set){
            esp_avrc_ct_send_passthrough_cmd(5, ESP_AVRC_PT_CMD_PLAY, ESP_AVRC_PT_CMD_STATE_PRESSED);
            radio->set_status(1,1);
        }
        else{
            radio->set_status(1,0);
            radio->update(0,0,0);
        }
        
        break;
    case PAUSE:
        //ESP_LOGI("callback", "STOP");
        radio->set_status(0,0); //set idle mode
        if(avrc_set){
            esp_avrc_ct_send_passthrough_cmd(5, ESP_AVRC_PT_CMD_PAUSE, ESP_AVRC_PT_CMD_STATE_PRESSED);
        }
        break;
    case FWD:
        //ESP_LOGI("callback", "FWD");
        if(avrc_set){
            esp_avrc_ct_send_passthrough_cmd(5, ESP_AVRC_PT_CMD_FORWARD, ESP_AVRC_PT_CMD_STATE_PRESSED);
        }
        break;
    case BWD:
        //ESP_LOGI("callback", "BWD");
        if(avrc_set){
            esp_avrc_ct_send_passthrough_cmd(5, ESP_AVRC_PT_CMD_BACKWARD, ESP_AVRC_PT_CMD_STATE_PRESSED);
        }
        break;
    case SEEK_FWD_PUSH:
        //ESP_LOGI("callback", "S_FWD");
        
        if(avrc_set){
            esp_avrc_ct_send_passthrough_cmd(5, ESP_AVRC_PT_CMD_FAST_FORWARD, ESP_AVRC_PT_CMD_STATE_PRESSED);
            vTaskDelay(100);
            esp_avrc_ct_send_passthrough_cmd(5, ESP_AVRC_PT_CMD_FAST_FORWARD, ESP_AVRC_PT_CMD_STATE_RELEASED);
        }
        
        break;
    case SEEK_BWD_PUSH:
        //ESP_LOGI("callback", "S_BWD");
        
        if(avrc_set){
            esp_avrc_ct_send_passthrough_cmd(5, ESP_AVRC_PT_CMD_REWIND, ESP_AVRC_PT_CMD_STATE_PRESSED);
            vTaskDelay(100);
            esp_avrc_ct_send_passthrough_cmd(5, ESP_AVRC_PT_CMD_REWIND, ESP_AVRC_PT_CMD_STATE_RELEASED);
        }
        
        break;
    case CD1: //BLT
        //ESP_LOGI("callback", "CD1");
        if(radio->get_cd() != 0){
            radio->update(0,track,0);
            if(avrc_set){esp_avrc_ct_send_passthrough_cmd(5, ESP_AVRC_PT_CMD_PLAY, ESP_AVRC_PT_CMD_STATE_PRESSED);}
        }
        break;
    case CD2: //AUX
        //ESP_LOGI("callback", "CD2");
        if(radio->get_cd() != 1){
            //todo - pause bluetooth, set gpio high to switch audio MUX
            if(avrc_set){esp_avrc_ct_send_passthrough_cmd(5, ESP_AVRC_PT_CMD_PAUSE, ESP_AVRC_PT_CMD_STATE_PRESSED);}
            vTaskDelay(100);
            radio->set_status(1,0); //non-idle, but not increment digits
            radio->update(1,0,0);
        }
        break;
    default:
        //ESP_LOGI("PACKET","ID:%X", stav);
        break;
    }  
}

void a2dp_event(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param){
    switch (event)
    {
    case ESP_A2D_CONNECTION_STATE_EVT:
        if(param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED){
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            //ESP_LOGI("BT", "conn");
            i2s_channel_enable(tx_handle);
        }
        else if(param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED){
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            //ESP_LOGI("BT", "disconn");
            i2s_channel_disable(tx_handle);

        }
        break;
    default:
        break;
    }
}

void pcm_data_rx(const uint8_t *buf, uint32_t len){
    //ESP_LOGI("BT","data_rx");
    size_t wb;
    i2s_channel_write(tx_handle, buf, len, &wb, portMAX_DELAY);
}

void avrc_callback(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param){
    uint8_t a = 0;
    switch(event){
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
        if (param->conn_stat.connected){
            esp_avrc_ct_send_get_rn_capabilities_cmd(1);
            esp_avrc_ct_send_register_notification_cmd(2, ESP_AVRC_RN_PLAY_STATUS_CHANGE, 0);
            esp_avrc_ct_send_register_notification_cmd(3, ESP_AVRC_RN_PLAY_POS_CHANGED, 0);
            esp_avrc_ct_send_register_notification_cmd(4, ESP_AVRC_RN_TRACK_CHANGE, 0);
            esp_avrc_ct_send_metadata_cmd(1, ESP_AVRC_MD_ATTR_TRACK_NUM);
            avrc_set = true;
            
        }
        else{avrc_set = false;}
        break;
    
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
        //ESP_LOGI("BT","NOT_EV");
        switch(param->change_ntf.event_id)
        {
        case ESP_AVRC_RN_PLAY_STATUS_CHANGE:
            //ESP_LOGI("BT", "st_chng %d",(int)param->change_ntf.event_parameter.playback);
            esp_avrc_ct_send_register_notification_cmd(2, ESP_AVRC_RN_PLAY_STATUS_CHANGE, 0);
            if(param->change_ntf.event_parameter.playback == ESP_AVRC_PLAYBACK_PLAYING){
                radio->set_status(1,1);
                esp_avrc_ct_send_metadata_cmd(1, ESP_AVRC_MD_ATTR_TRACK_NUM);
                }
            else if (param->change_ntf.event_parameter.play_pos == ESP_AVRC_PLAYBACK_PAUSED){radio->set_status(1,0);}
            else{
                track = 0;
                radio->set_status(1,0);
                radio->update(0,track,0);
            }
            break;

        case ESP_AVRC_RN_TRACK_CHANGE:
            //ESP_LOGI("BT", "track_chng");
            esp_avrc_ct_send_register_notification_cmd(3, ESP_AVRC_RN_TRACK_CHANGE, 0);
            radio->update(0,track,uint8_t(param->change_ntf.event_parameter.play_pos/1000));
            esp_avrc_ct_send_metadata_cmd(1, ESP_AVRC_MD_ATTR_TRACK_NUM);
            break;

        case ESP_AVRC_RN_PLAY_POS_CHANGED:
            //ESP_LOGI("BT", "new_tr_pos %d",int(param->change_ntf.event_parameter.play_pos));
            esp_avrc_ct_send_register_notification_cmd(4, ESP_AVRC_RN_PLAY_POS_CHANGED, 0);
            radio->update(0,track,param->change_ntf.event_parameter.play_pos/1000);
            break;

        default:
            break;
        }
        break;

    case ESP_AVRC_CT_METADATA_RSP_EVT:
        track = 0;
        a = param->meta_rsp.attr_length-1;
        for(size_t i = 0; i < param->meta_rsp.attr_length; ++i){
            uint8_t val = *(param->meta_rsp.attr_text + i);
            track += (val-'0')*std::pow(10,a);
            --a;
        }
        track %=100; //na výpis cisla skladby máme max 2 digity => tato uprava
        radio->update(track);
        //ESP_LOGI("METADATA", "track: %d",track);
        break;

    default:
        break;
    }
        
}

void blt_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param){
    /*todo*/
}
