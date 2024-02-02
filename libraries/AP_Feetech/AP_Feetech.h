#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <cstring>

static constexpr uint32_t BAUD_RATE = 500000;
static constexpr uint8_t  SERIAL_PORT = 4; // use a port with DMA change it also in SerialManager!!
static constexpr uint8_t  SERVO_ID_1 = 1;
static constexpr uint8_t  SERVO_ID_2 = 2;

class Feetech
{

public:
    Feetech();

    CLASS_NO_COPY(Feetech);

    static Feetech *get_singleton() {
        return _singleton;
    }

    void update();
    void pre_arm_check();

private:    
    uint8_t _err_cnt = 0;
    uint16_t _pos_err_cnt = 0;
    // uint8_t _n_bytes;        // probably useless
    static Feetech *_singleton;
    AP_HAL::UARTDriver *_uart;
    bool _init_done = false;

    uint8_t _rx_buf[2*14];
    uint16_t _pos[2];
    uint8_t _chck_buf1[11] = {0xff, 0xff, 0x01, 0x02, 0x00, 0xfc, 0xff, 0xff, 0x01, 0x04 ,0x00};
    uint8_t _chck_buf2[11] = {0xff, 0xff, 0x02, 0x02, 0x00, 0xfb, 0xff, 0xff, 0x02, 0x04 ,0x00};

    template <typename T>
    class PACKED Tx_packet {
    public:
        Tx_packet(uint8_t _servo_id, T _msg) : 
            servo_id(_servo_id),
            msg(_msg)
        {
            update_checksum();
        }

        uint8_t header[2] {0xFF, 0xFF};
        uint8_t servo_id;
        uint8_t frame_length {sizeof(T) + sizeof(checksum)};
        T msg;
        uint8_t checksum;

        void update_checksum() {
            uint8_t sum = 0;
            for (u_int8_t i = 2; i < ((2 + 1 + 1 + frame_length) - 1); i++) {
                sum += *((uint8_t *)this + i);
            }
            checksum = ~sum;
        }
    };

    class PACKED POS_CMD {
    public:
        POS_CMD(uint16_t _pos)
        { 
            pos[0] = _pos & 0x00ff; 
            pos[1] = _pos >> 8;
        }
            
        uint8_t instruction {0x03};
        uint8_t address {0x2A};
        uint8_t pos[2];
        uint8_t time[2] {0x00, 0x00};
        uint8_t speed[2] {0xE8, 0x03};
    };

    class PACKED STATUS_QUERY {
    public:
        STATUS_QUERY()
        { }
        uint8_t instruction {0x02};
        uint8_t address {0x38};
        uint8_t number_of_bytes_to_read {0x02};
    };

    void init();
    void send_pos_cmd(uint8_t id, uint16_t pos);
    void send_status_query(uint8_t id);
    bool response_valid();
    bool sanity_check();

protected:

};