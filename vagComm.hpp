/*include*/
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include <thread>
#include <mutex>
#include <bits/stdc++.h>
#include <array>

#define CD_EN 2
/*PARAMETRY SPI --> RADIO*/
#define CLK_SPEED 62500
#define MODE 2
#define CMD_DELAY 10
/*HODNOTY BYTŮ KTERÉ VYSÍLÁ RÁDIO*/
#define PREFIX_1 0x53
#define PREFIX_2 0x2C

#define PLAY 0xE4
#define PAUSE 0x10
#define FWD 0xF8
#define BWD 0x78
#define SEEK_FWD_PUSH 0xD8
#define SEEK_BWD_PUSH 0x58

#define CHECKSUM 0xFF

/*PARAMETRY SBĚRNICE - čas[us]*/
#define START_BIT_high 9000
#define START_BIT_low 4500
#define ONE_BIT_high 550
#define ONE_BIT_low 1700
#define ZERO_BIT_high 550
#define ZERO_BIT_low 550

#define TOLERANCE 50

extern "C" class VagComm
{
private:
    gpio_num_t DATA_I;
    bool enable;
    uint8_t cd,track,min,sec,mode;
    std::thread t1;
    std::mutex mut;
    spi_device_handle_t spiHandle;
    spi_device_interface_config_t dev_config;
    spi_bus_config_t bus_config;
    gpio_config_t input_bus;
    gpio_isr_handle_t data_in_itr;


    std::array<uint8_t, 8> idle_data_packet = {0x74, 0xBE, 0xFE, 0xFF, 0xFF, 0xFF, 0x8F, 0x7C};
    std::array<uint8_t, 8> init_data_packet = {0x74, 0xBE, 0xFE, 0xFF, 0xFF, 0xFF, 0x8F, 0x7C};
    std::array<uint8_t, 8> data_save = {0x34, 0xBE, 0xFE, 0xFF, 0xFF, 0xFF, 0xEF, 0x3C};

    void send_data(std::array<uint8_t, 8> data);
    void send_data(uint8_t data);
    void thread_function();
    void init();
    void decode();
    void send_info_bytes();
    void send_init_info_bytes();
    void send_int_bytes();
    void cd_init(uint8_t cd);

public:
    VagComm(gpio_num_t CLOCK, gpio_num_t DATA_IN, gpio_num_t DATA_OUT);
    ~VagComm();
    void update(uint8_t cd_num, uint8_t track_num, uint8_t time_min, uint8_t time_sec);
    void set_status(uint8_t PLAYING);
};