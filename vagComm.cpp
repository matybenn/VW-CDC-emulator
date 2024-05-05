#include "vagComm.hpp"
#define THR_TAG "THREAD"

#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE

extern "C" void VagComm::thread_function(){
    mut.lock();
    while (enable)
    {
        switch (mode)
        {
        case 0: /*idle*/
            ESP_LOGI(THR_TAG, "idle");
            send_data(idle_data_packet);
            break;

        case 1: /*pause*/
            ESP_LOGI(THR_TAG, "pause");
            send_data(data_save[0]^0xFF);
            send_data(0x41+cd);
            send_info_bytes();
            send_data(0x30);
            send_data(data_save[7]^0xFF);
            break;
        
        case 2: /*play*/
            ESP_LOGI(THR_TAG, "play");
            break;
        }
        mut.unlock();
        vTaskDelay(CMD_DELAY);
        mut.lock();
    }
    mut.unlock();
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
    //DATA_IN init
    gpio_set_direction(DATA_I,GPIO_MODE_INPUT);
    gpio_set_pull_mode(DATA_I,GPIO_PULLDOWN_ONLY);
    gpio_set_intr_type(DATA_I,GPIO_INTR_POSEDGE);
    gpio_intr_enable(DATA_I);
    //gpio_isr_register(VagComm::decode,&data_in_itr);
    //spi init
    spi_bus_initialize(SPI3_HOST, &bus_config, 0);
    spi_bus_add_device(SPI3_HOST, &dev_config, &spiHandle);
    //cdc init
    enable = true;
    cd=track=min=sec=mode = 0;
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
    track = track_num;
    min = time_min;
    sec = time_sec;
    if(cd_num != cd && cd_num < CD_EN){
        cd_init(cd_num);
        cd = cd_num;
    }
    mut.unlock();
}

void VagComm::set_status(uint8_t PLAYING){
    mut.lock();
    if (mode == 0){
        init();
    }
    mode = PLAYING;
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

void VagComm::send_data(uint8_t data){    
    spi_transaction_t trans;
    std::memset(&trans,0,sizeof(trans));
    data = 0xFF^data;
    trans.length=8;
    trans.tx_buffer = &data;
    spi_device_transmit(spiHandle, &trans);
}

void VagComm::init(){
    
    for (uint8_t i = 0; i<CD_EN; ++i){
        cd_init(i); 
    }
}

void VagComm::cd_init(uint8_t cd){
    send_data(data_save[0]^0xFF);
    send_data(0xD1+cd);
    send_init_info_bytes();
    send_data(0x00);
    send_data(data_save[7]^0xFF);
    vTaskDelay(CMD_DELAY);
    send_data(data_save);
    vTaskDelay(CMD_DELAY);
}

void VagComm::decode(){
    
}

void VagComm::send_info_bytes(){
    send_data(track);
    send_data(min);
    send_data(sec);
    send_data(0x00); //scan and mix off
}

void VagComm::send_init_info_bytes(){
    send_data(0x99); // max tr
    send_data(0x99); // max min
    send_data(0x59); // max sec
    send_data(0x49); // modes
}
