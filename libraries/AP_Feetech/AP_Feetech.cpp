#include "AP_Feetech.h"

#include <GCS_MAVLink/GCS.h>

const extern AP_HAL::HAL& hal;

Feetech *Feetech::_singleton;

Feetech::Feetech()
{
    // 1. sending a message in console from here breaks USB com for some reason
    // 2. also initializing the uart variable here results in issues (nullptr)

    // The above occur most probably due to the serial drivers not having 
    // loaded up to the moment this constructor is called.
    _singleton = this;
}

void Feetech::init() 
{
    // hal.gpio->pinMode(59, HAL_GPIO_OUTPUT); // S10

    // if (_uart != nullptr) {          // probably useless
    //     return;
    // }
    
    _uart = hal.serial(SERIAL_PORT);

    if (_uart == nullptr) {        
        gcs().send_text(MAV_SEVERITY_INFO, "Feetech: Initialization of Serial %d failed!", SERIAL_PORT);
        return;
    }

    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);  // probably useless
    // _uart->set_unbuffered_writes(true);                              // if set here --> internal error 0x4000020
    _uart->begin(BAUD_RATE);
    _uart->set_unbuffered_writes(true);                                 // if set here no internal error!
    // _uart->set_options(AP_HAL::UARTDriver::OPTION_HDPLEX);              // enable if DMA support for Half duplex 
    
    gcs().send_text(MAV_SEVERITY_INFO, "Feetech: Initialized Serial %d", SERIAL_PORT);
    _init_done = true;
}

void Feetech::send_pos_cmd(uint8_t id, uint16_t pos)
{
    Tx_packet<POS_CMD> msg = Tx_packet<POS_CMD>{id, POS_CMD{pos}};
    uint8_t *bytes = (uint8_t *)&msg;
    _uart->write(bytes, 13);
    _uart->flush();     // not sure if needed
}

void Feetech::send_status_query(uint8_t id)
{
    Tx_packet<STATUS_QUERY> msg = Tx_packet<STATUS_QUERY>{id, STATUS_QUERY{}};
    uint8_t *bytes = (uint8_t *)&msg;
    _uart->write(bytes, 8);
    _uart->flush();     // not sure if needed
}

bool Feetech::sanity_check()
{
    if (_pos[0] < 2000 || _pos[0] > 2100 || _pos[1] < 2000 || _pos[1] > 2100) {
        gcs().send_text(MAV_SEVERITY_EMERGENCY, "ERROR! pos1 = %d, pos2 = %d", _pos[0], _pos[1]);
        gcs().send_text(MAV_SEVERITY_INFO, 
                        "%d %d %d %d %d %d %d %d %d %d %d %d %d %d",
                        _rx_buf[0], _rx_buf[1], _rx_buf[2], _rx_buf[3], _rx_buf[4], 
                        _rx_buf[5], _rx_buf[6], _rx_buf[7], _rx_buf[8], _rx_buf[9],
                        _rx_buf[10], _rx_buf[11], _rx_buf[12], _rx_buf[13]);
        gcs().send_text(MAV_SEVERITY_INFO,
                        "%d %d %d %d %d %d %d %d %d %d %d %d %d %d",
                        _rx_buf[14], _rx_buf[15], _rx_buf[16], _rx_buf[17], _rx_buf[18], 
                        _rx_buf[19], _rx_buf[20], _rx_buf[21], _rx_buf[22], _rx_buf[23],
                        _rx_buf[24], _rx_buf[25], _rx_buf[26], _rx_buf[27]);
        return false;
    }

    return true;
}

bool Feetech::response_valid()
{
    uint8_t sum;
    uint8_t checksum;
    
    // crc check message 2
    sum = 0;
    for (u_int8_t i = 8; i < 13; i++) {
        sum = sum + _rx_buf[i];
    }
    checksum = ~sum;
    if (checksum != _rx_buf[13]) {
        return false;
    }
    
    // crc check message 4
    sum = 0;
    for (u_int8_t i = 8+14; i < 13+14; i++) {
        sum = sum + _rx_buf[i];
    }
    checksum = ~sum;
    if (checksum != _rx_buf[13+14]) {
        return false;
    }

    // check the rest of the bytes that are known a-priori
    if (std::memcmp(_rx_buf, _chck_buf1, 11) != 0) {
        return false;
    }
    if (std::memcmp((_rx_buf+14), _chck_buf2, 11) != 0) {
        return false;
    }
    
    return true;
}

uint16_t Feetech::rc2srv_defl(uint8_t ch)
{
    SRV_Channel *c = SRV_Channels::srv_channel(ch);
    uint16_t pwm = c->get_output_pwm();
    uint16_t min = c->get_output_min();
    uint16_t max = c->get_output_max();
    float v = float(pwm - min) / (max - min);
    
    return v * 4096;
}

void Feetech::update()
{
    if (!_init_done) {
        init();
        return;
    }

    uint16_t left_servo_defl = rc2srv_defl(2); // Channel 3
    gcs().send_named_float("SERVO_L", left_servo_defl);

    uint16_t right_servo_defl = rc2srv_defl(3);
    gcs().send_named_float("SERVO_R", right_servo_defl);

    // SEND MESSAGES
    // hal.gpio->toggle(59);
    send_pos_cmd(SERVO_ID_1, left_servo_defl);
    send_status_query(SERVO_ID_1);
    hal.scheduler->delay_microseconds(500); 
    
    // hal.gpio->toggle(59);
    send_pos_cmd(SERVO_ID_2, right_servo_defl);
    send_status_query(SERVO_ID_2);
    hal.scheduler->delay_microseconds(500);

    // GET REPLIES
    _uart->read(_rx_buf, sizeof(_rx_buf));

    if (response_valid() == true) {
        _pos[0] = (_rx_buf[12] << 8) + _rx_buf[11];
        _pos[1] = (_rx_buf[26] << 8) + _rx_buf[25];
        
        // improve sanity check!!
        if (sanity_check() == true) {
            // convert raw position to deflection angle delta
            Feetech::delta[0] = _pos[0];
            Feetech::delta[1] = _pos[1];
        } else {
            _pos_err_cnt = _pos_err_cnt + 1;
        }
    }
    else {
        _uart->discard_input();
        _err_cnt = _err_cnt +1;
    }
 
    gcs().send_named_float("ERR_CNT", _err_cnt);
    gcs().send_named_float("POS_ERR_CNT", _pos_err_cnt);
}