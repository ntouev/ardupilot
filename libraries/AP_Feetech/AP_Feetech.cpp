#include "AP_Feetech.h"

#include <GCS_MAVLink/GCS.h>

const extern AP_HAL::HAL& hal;

Feetech *Feetech::_singleton;

Feetech::Feetech()
{
    // sending a message in console from here breaks USB com for some reason
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "feetech constructor!");

    // also initializing the uart variable here results in issues (nullptr)
    // probably the uart is not ready yet

    // The above occur most probably due to the serial drivers not having 
    // loaded up to the moment this constructor is called.
    _singleton = this;
}

void Feetech::init() 
{
    // if (_uart != nullptr) {
    //     return;
    // }
    
    _uart = hal.serial(SERIAL_PORT);

    if (_uart == nullptr) {
        gcs().send_text(MAV_SEVERITY_INFO, "Feetech: Initialization of Serial %d failed!", SERIAL_PORT);
        return;
    }

    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_ENABLE);
    // _uart->set_unbuffered_writes(true);
    // _uart->set_options(AP_HAL::UARTDriver::OPTION_HDPLEX);
    _uart->begin(BAUD_RATE);

    gcs().send_text(MAV_SEVERITY_INFO, "Feetech: Initialized Serial %d", SERIAL_PORT);
    _init_done = true;
}

void Feetech::send_pos_cmd(uint8_t id, uint16_t pos)
{
    Tx_packet<POS_CMD> msg = Tx_packet<POS_CMD>{id, POS_CMD{pos}};
    uint8_t *bytes = (uint8_t *)&msg;
    _uart->write(bytes, 13);
    _uart->flush();
}

void Feetech::send_status_query(uint8_t id)
{
    Tx_packet<STATUS_QUERY> msg = Tx_packet<STATUS_QUERY>{id, STATUS_QUERY{}};
    uint8_t *bytes = (uint8_t *)&msg;
    _uart->write(bytes, 8);
    _uart->flush();
}

bool Feetech::response_valid()
{
    if (std::memcmp(_rx_buf, _chck_buf1, 11) != 0) {
        return false;
    }
    if (std::memcmp((_rx_buf+14), _chck_buf2, 11) != 0) {
        return false;
    }

    return true;
}

void Feetech::update()
{
    if (!_init_done) {
        init();
        return;
    }

    send_pos_cmd(SERVO_ID_1, 2048);
    send_status_query(SERVO_ID_1);
    hal.scheduler->delay_microseconds(1245); //1200 ok --> Hz, 1500 even better! --> ~330 Hz
    send_pos_cmd(SERVO_ID_2, 2048);
    send_status_query(SERVO_ID_2);
    hal.scheduler->delay_microseconds(1245); //1200 ok --> Hz, 1500 even better! --> ~330 Hz

    // GET REPLIES
    _n_bytes = _uart->available();
    _uart->read(_rx_buf, _n_bytes);

    if (response_valid() == false) {
        _uart->discard_input();
        _err_cnt = _err_cnt +1;
        // gcs().send_text(MAV_SEVERITY_INFO, "%d", _n_bytes);
    }
    else {
        _pos[0] = (_rx_buf[12] << 8) + _rx_buf[11];
        _pos[1] = (_rx_buf[26] << 8) + _rx_buf[25];
        
        // include sanity check here as soon as I have the first flight data
        // this is just for testing
        if (_pos[0] < 2000 || _pos[0] > 2100 || _pos[1] < 2000 || _pos[1] > 2100) {
            _pos_err_cnt = _pos_err_cnt + 1;
            gcs().send_text(MAV_SEVERITY_EMERGENCY, "ERROR! pos1 = %d, pos2 = %d", _pos[0], _pos[1]);
        }

        // gcs().send_text(MAV_SEVERITY_INFO, "pos1 = %d, pos2 = %d", _pos[0], _pos[1]);
        gcs().send_named_float("POS1", _pos[0]); // ~ 15 usec each
        gcs().send_named_float("POS2", _pos[1]); // ~ 15 usec each
        
        // gcs().send_text(MAV_SEVERITY_INFO, 
        //                 "%d %d %d %d %d %d %d %d %d %d %d %d %d %d",
        //                 _rx_buf[0], _rx_buf[1], _rx_buf[2], _rx_buf[3], _rx_buf[4], 
        //                 _rx_buf[5], _rx_buf[6], _rx_buf[7], _rx_buf[8], _rx_buf[9],
        //                 _rx_buf[10], _rx_buf[11], _rx_buf[12], _rx_buf[13]);
        // gcs().send_text(MAV_SEVERITY_INFO,
        //                 "%d %d %d %d %d %d %d %d %d %d %d %d %d %d",
        //                 _rx_buf[14], _rx_buf[15], _rx_buf[16], _rx_buf[17], _rx_buf[18], 
        //                 _rx_buf[19], _rx_buf[20], _rx_buf[21], _rx_buf[22], _rx_buf[23],
        //                 _rx_buf[24], _rx_buf[25], _rx_buf[26], _rx_buf[27]);
    }
 
    gcs().send_named_float("ERR_CNT", _err_cnt);
    gcs().send_named_float("POS_ERR_CNT", _pos_err_cnt);
}