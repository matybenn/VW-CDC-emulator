/*include*/
#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <mutex>
#include <bits/stdc++.h>
#include <array>

#define CD_EN 2 // enabled no. of cds 1-6 /cd1 - blt, cd2 - aux

/*PARAMETRY SPI --> RADIO*/
#define CLK_SPEED 62500
#define MODE 2
#define CMD_DELAY 10  // delay mezi byty

/*HODNOTY BYTŮ KTERÉ VYSÍLÁ RÁDIO - NEC protocol*/
#define PREFIX_1 0x53
#define PREFIX_2 0x2C

#define PLAY 0xE4
#define PAUSE 0x10
#define FWD 0xF8
#define BWD 0x78
#define SEEK_FWD_PUSH 0xD8
#define SEEK_BWD_PUSH 0x58
#define CD1 0x0C
#define CD2 0x8C

/*PARAMETRY SBĚRNICE - čas[us]*/
#define START_BIT_high 9000
#define START_BIT_low 4500
#define ONE_BIT_high 550
#define ONE_BIT_low 1700
#define ZERO_BIT_high 550
#define ZERO_BIT_low 550
#define TOLERANCE 150

extern "C" class VagComm
{
private:
    typedef bool (*rmt_rx_done_callback_t)(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_ctx);
    typedef void (*call_t)(uint8_t);
    //general
    gpio_num_t DATA_I;
    call_t callback_function;
    bool mode,playing;
    uint8_t cd,track;
    uint32_t total_time;
    std::mutex mut;
    QueueHandle_t fronta1 = NULL;
    TaskHandle_t data_checker = NULL;
    //spi
    spi_device_handle_t spiHandle = NULL;
    spi_device_interface_config_t dev_config;
    spi_bus_config_t bus_config;
    //rmt
    rmt_channel_handle_t data_in_rmt = NULL;
    rmt_rx_channel_config_t data_in_config;
    rmt_receive_config_t receive_config;
    rmt_rx_event_callbacks_t rx_call;
    rmt_rx_done_event_data_t rxdata;
    //timers
    esp_timer_handle_t time = NULL;
    esp_timer_create_args_t time_param;
    esp_timer_handle_t sender = NULL;
    esp_timer_create_args_t sender_param;
    //data packets
    std::array<uint8_t, 8> idle_data_packet = {0x74, 0xBE, 0xFE, 0xFF, 0xFF, 0xFF, 0x8F, 0x7C};
    std::array<uint8_t, 8> init_data_packet = {0x74, 0xBE, 0xFE, 0xFF, 0xFF, 0xFF, 0x8F, 0x7C};
    std::array<uint8_t, 8> data_save = {0x34, 0xBE, 0xFE, 0xFF, 0xFF, 0xFF, 0xEF, 0x3C};
    std::array<uint8_t, 4> nec_received;
    rmt_symbol_word_t received_data[64];
    //static f
    static void sender_function(void* arg);
    static void count_up(void* arg);
    static bool data_rx(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_ctx);
    static void data_in_checker(void* arg);
    //void
    void send_data(std::array<uint8_t, 8> data);
    void send_data(uint8_t data);
    void send_info_bytes();
    void send_init_info_bytes();
    uint8_t format_num(uint8_t number);
    void init();
    void cd_init(uint8_t cd);
    void send_packet();
    void countup_time();
    void decode();
public:
    VagComm(gpio_num_t CLOCK, gpio_num_t DATA_IN, gpio_num_t DATA_OUT, call_t ID_callback);
    ~VagComm();
    void update(uint8_t cd_num, uint8_t track_num, uint32_t time_tot);
    void update(uint8_t tr);
    void set_status(bool MODE_idle, bool PLAYING);
    uint8_t get_cd();
};