#ifndef _COM_H
#define _COM_H

#define DRIVER_PACKET_SET   0x00
struct driver_packet_set {
    bool enable;
    float targetSpeed;
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
