#ifndef _COM_H
#define _COM_H

#include <stdint.h>
#include "hybrid.h"
#include "servo.h"
#include "ssl.h"

#define DRIVER_PACKET_SET   0x00

struct driver_packet_set {
    bool enable;
    float targetSpeed;
    int16_t pwm;
    uint32_t padding1; // See the next static_assert
    uint8_t padding2; // See the next static_assert
} __attribute__((packed));

struct driver_packet_ans {
    uint8_t status;
    float speed;
    int16_t pwm;
    uint32_t enc_cnt;
} __attribute__((packed));

// TODO : DEPRECATED: TO REMOVE
#define DRIVER_PACKET_PARAMS   0x01
struct driver_packet_params {
    float kp;
    float ki;
    float kd;
} __attribute__((packed));

static_assert(
    sizeof(driver_packet_set) > sizeof(driver_packet_ans),
    "In SPI, packet answer have to be strictly smaller than the resquest packet"
);

void com_init();

void com_tick();

#endif
