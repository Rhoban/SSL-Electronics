#ifndef _COM_H
#define _COM_H

struct driver_packet {
    bool enable;
    float targetSpeed;
    float kp;
    float ki;
    float kd;
} __attribute__((packed));

void com_init();

void com_tick();

#endif
