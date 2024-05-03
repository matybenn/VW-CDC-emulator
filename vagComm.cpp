#include "vagComm.hpp"
#define THR_TAG "THREAD"

#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE

extern "C" void VagComm::thread_function(){
    mut.lock();
    while (enable)
    {
        if (play){
            mut.unlock();
            ESP_LOGI(THR_TAG, "play");
        }
        else {
            mut.unlock();
            send_data(data_default);
        }
        
        vTaskDelay(100);
        mut.lock();
    }
}

extern "C" VagComm::VagComm(gpio_num_t CLOCK, gpio_num_t DATA_IN, gpio_num_t DATA_OUT){
    DATA_I = DATA_IN;
    std::memset(&bus_config, 0, sizeof(spi_bus_config_t));//make default
    std::memset(&dev_config,0,sizeof(spi_device_interface_config_t));//make default
    bus_config.mosi_io_num = DATA_OUT;
    bus_config.miso_io_num = -1;
    bus_config.sclk_io_num = CLOCK;
	dev_config.clock_speed_hz = CLK_SPEED;
    dev_config.mode = MODE;
	dev_config.spics_io_num = -1;
	dev_config.queue_size = 1;
	dev_config.pre_cb = NULL;
	dev_config.post_cb = NULL;
    

    gpio_set_direction(DATA_I,GPIO_MODE_INPUT);
    spi_bus_initialize(SPI3_HOST, &bus_config, 0);
    spi_bus_add_device(SPI3_HOST, &dev_config, &spiHandle);

    init();
    play = false;
    enable = true;
    t1 = std::thread(&VagComm::thread_function,this);
}

VagComm::~VagComm(){
    mut.lock();
    enable = false;
    mut.unlock();
    if(t1.joinable()) {t1.join();}
    spi_bus_remove_device(spiHandle);
}

void VagComm::update(uint8_t cd_num, uint8_t track_num, uint8_t time_min, uint8_t time_sec){
    mut.lock();

    mut.unlock();
}

void VagComm::set_status(bool PLAYING){
    mut.lock();
    play = PLAYING;
    mut.unlock();
}

void VagComm::send_data(std::array<uint8_t, 8> data){    
    spi_transaction_t trans;
    std::memset(&trans,0,sizeof(trans));
    trans.length=8;
    for (uint8_t i : data){
        trans.tx_buffer = &i;
        spi_device_transmit(spiHandle, &trans);
    }
}

void VagComm::init(){
    std::array<uint8_t, 8> data_init = {0x74,0xBE,0xFE,0xFF,0xFF,0xFF,0x8F,0x7C};
    std::array<uint8_t, 8> data_init_wait = {0x34,0xFF,0xFE,0xFE,0xFE,0xFF,0xFA,0x3C};
    send_data(data_init);
    vTaskDelay(10);
    send_data(data_init_wait);
    vTaskDelay(50);
    send_data(data_init);
}