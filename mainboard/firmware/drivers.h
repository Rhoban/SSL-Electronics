#ifndef _DRIVERS_H
#define _DRIVERS_H

#include <stdint.h>

struct driver_packet {
    bool enable;
    float targetSpeed;
    float kp;
    float ki;
    float kd;
} __attribute__((packed));

void drivers_init();

/**
 * Set the speed of the nth wheel [turn/s]
 */
uint8_t drivers_set(int index, bool enable, float target);
void drivers_set_safe(int index, bool enable, float target);
void drivers_set_pid(float kp, float ki, float kd);

/**
 * Tick the drivers
 */
void drivers_tick();

/**
 * Is the driver alive ?
 */
int drivers_ping(int index);
bool drivers_is_all_ok();
void drivers_diagnostic();

#endif
