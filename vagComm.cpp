#include "vagComm.hpp"
#define THR_TAG "THREAD"

#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE

/*static voids*/
extern "C" bool VagComm::data_rx(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_ctx){
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_ctx;
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

void VagComm::sender_function(void* arg){
    VagComm *trida = (VagComm*)arg;
    trida->send_packet();
    
}

void VagComm::count_up(void* arg){
    VagComm *trida = (VagComm*)arg;
    trida->countup_time();
}

void VagComm::data_in_checker(void* arg){
    VagComm *trida = (VagComm*)arg;
    while (true)
    {
        if(xQueueReceive(trida->fronta1, &trida->rxdata, portMAX_DELAY) == 1){
            trida->decode();            
            rmt_receive(trida->data_in_rmt, trida->received_data, sizeof(received_data), &trida->receive_config);
        }
    }
}

/*classic functions*/
extern "C" VagComm::VagComm(gpio_num_t CLOCK, gpio_num_t DATA_IN, gpio_num_t DATA_OUT, call_t ID_callback){
    DATA_I = DATA_IN;
    callback_function = ID_callback;

    //spi init
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
    spi_bus_initialize(SPI3_HOST, &bus_config, 0);
    spi_bus_add_device(SPI3_HOST, &dev_config, &spiHandle);
    
    //TIMER INIT
    time_param.callback = this->count_up;
    time_param.arg = this;
    time_param.dispatch_method = ESP_TIMER_TASK;
    sender_param.callback = this->sender_function;
    sender_param.arg = this;
    sender_param.dispatch_method = ESP_TIMER_TASK;
    esp_timer_create(&time_param, &time);
    esp_timer_create(&sender_param, &sender);
    esp_timer_start_periodic(sender,10000);
    
    //DATA_IN GPIO RMT INIT
    data_in_config.gpio_num = DATA_I;
    data_in_config.clk_src = RMT_CLK_SRC_DEFAULT;
    data_in_config.intr_priority = 0;
    data_in_config.mem_block_symbols = 64;
    data_in_config.resolution_hz = 1000000; //1tick = 1us
    data_in_config.flags.with_dma = false;
    data_in_config.flags.invert_in = false;
    data_in_config.flags.io_loop_back = false;
    receive_config.signal_range_min_ns = 1250;     // the shortest duration for NEC signal is 560 µs, 1250 ns < 560 µs, valid signal is not treated as noise
    receive_config.signal_range_max_ns = 12000000; // the longest duration for NEC signal is 9000 µs, 12000000 ns > 9000 µs, the receive does not stop early

    //queve + task
    fronta1 = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    xTaskCreate(this->data_in_checker, "checker_task", 4096, this, 0, &data_checker);

    //rx_en
    rmt_new_rx_channel(&data_in_config, &data_in_rmt);
    rx_call.on_recv_done = this->data_rx;
    rmt_rx_register_event_callbacks(data_in_rmt, &rx_call, fronta1);
    gpio_set_pull_mode(DATA_I,GPIO_PULLDOWN_ONLY);
    rmt_enable(data_in_rmt);
    rmt_receive(data_in_rmt, received_data, sizeof(received_data), &receive_config);
    
    //cdc init
    cd=track=total_time=mode=playing=0;
}

VagComm::~VagComm(){
    esp_timer_stop(time);
    esp_timer_stop(sender);
    esp_timer_delete(time);
    esp_timer_delete(sender);
    esp_timer_deinit();
    spi_bus_remove_device(spiHandle);
    rmt_disable(data_in_rmt);
    rmt_del_channel(data_in_rmt);
    vTaskDelete(data_checker);
}

void VagComm::update(uint8_t cd_num, uint8_t track_num, uint8_t time_min, uint8_t time_sec){
    mut.lock();
    track = track_num;
    total_time = (time_min*60) + time_sec;
    if(cd_num != cd && cd_num < CD_EN){
        cd_init(cd_num);
        cd = cd_num;
    }
    mut.unlock();
}

void VagComm::set_status(bool MODE_idle, bool PLAYING){
    mut.lock();
    if(MODE_idle == true && mode == false){
        init();
    }
    mode = MODE_idle;
    if (playing != PLAYING){
        switch (playing)
        {
        case false:
            esp_timer_start_periodic(time,1000000);
            break;
        
        case true:
            esp_timer_stop(time);
            break;
        }
        playing = PLAYING;
    }
    mut.unlock();
}

void VagComm::send_data(std::array<uint8_t, 8> data){    
    spi_transaction_t trans;
    std::memset(&trans,0,sizeof(trans));
    trans.length=8;
    for (const uint8_t &i : data){
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
    mut.lock();
    for (uint8_t i = 0; i<CD_EN; ++i){
        cd_init(i); 
    }
    mut.unlock();
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

void VagComm::send_info_bytes(){
    send_data(format_num(track));
    send_data(format_num(total_time/60)); //min
    send_data(format_num(total_time%60)); //sec
    send_data(0x00); //scan and mix off
    //ESP_LOGI("send_f","min: %X, sec: %X",format_num(total_time/60),format_num(total_time%60));
}

uint8_t VagComm::format_num(uint8_t number){
    uint8_t to_send = 0;
    to_send += ((number/10)*16);
    to_send += (number%10);
    return to_send;
}

void VagComm::send_init_info_bytes(){
    send_data(0x99); // max tr
    send_data(0x99); // max min
    send_data(0x59); // max sec
    send_data(0x49); // modes
}

void VagComm::send_packet(){
    mut.lock();
    switch (mode)
    {
    case false: /*idle*/
        //ESP_LOGI(THR_TAG, "idle, total time: %d", total_time);
        send_data(idle_data_packet);
        break;

    case true: /*non-idle*/
        //ESP_LOGI(THR_TAG, "non idle, total time: %d", total_time);
        send_data(data_save[0]^0xFF);
        send_data(0x41+cd);
        send_info_bytes();
        send_data(0x30);
        send_data(data_save[7]^0xFF);
        break;
    }
    mut.unlock();
}

void VagComm::countup_time(){
    mut.lock();
    ++total_time;
    mut.unlock(); 
}

void VagComm::decode(){
    
    /*
    for (size_t i = 0; i < rxdata.num_symbols; i++) {
        ESP_LOGI("dec", "{%d:%d},{%d:%d}", rxdata.received_symbols[i].level0, rxdata.received_symbols[i].duration0,
               rxdata.received_symbols[i].level1, rxdata.received_symbols[i].duration1);
    }*/

    if(rxdata.num_symbols != 34){return;}
    //ESP_LOGI("dec", "frame");
    for (uint8_t a = 0; a<4; ++a){
        uint8_t data = 0;
        for(uint8_t b = 1; b<=8; ++b){
            uint8_t poradi = b + (8*a);
            if((rxdata.received_symbols[poradi].duration0 <= ONE_BIT_high+TOLERANCE && rxdata.received_symbols[poradi].duration0 >= ONE_BIT_high-TOLERANCE) && (rxdata.received_symbols[poradi].duration1 <= ONE_BIT_low+TOLERANCE && rxdata.received_symbols[poradi].duration1 >= ONE_BIT_low-TOLERANCE)){
                data ^= (0b10000000 >> (b-1));
            }
        }
        nec_received[a] = data;
        //ESP_LOGI("dec", "frame, %X", data);
    }
    //is valid?
    if((nec_received[0] == PREFIX_1) && (nec_received[1] == PREFIX_2)){
        callback_function(nec_received[2], this); //send third byte to callback
    }
}
