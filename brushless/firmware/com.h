#ifndef _COM_H
#define _COM_H

#include <stdint.h>

#define DRIVER_PACKET_SET   0x00
struct driver_packet_set {
    bool enable;
    float targetSpeed;
    int16_t pwm;
    uint32_t padding1;
    uint8_t padding2;
} __attribute__((packed));

struct driver_packet_ans {
    uint8_t status;
    float speed;
    int16_t pwm;
    uint32_t enc_cnt;
} __attribute__((packed));

#define DRIVER_PACKET_PARAMS   0x01
struct driver_packet_params {
    float kp;
    float ki;
    float kd;
} __attribute__((packed));

void com_init();

void com_tick();

#endif
