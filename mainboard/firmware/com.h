#ifndef _COM_H
#define _COM_H

#include <stdint.h>

struct packet_master {
    #define ACTION_ON      (1<<0)   // The robot should be on (else everything is stopped)
    #define ACTION_KICK1   (1<<1)   // Kick on kicker 1
    #define ACTION_KICK2   (1<<2)   // Kick on kicker 2
    #define ACTION_DRIBBLE (1<<3)   // Enable/disable the dribbler
    uint8_t actions;
    float wheel1;
    float wheel2;
    float wheel3;
    float wheel4;
} __attribute__((packed));

struct packet_robot {
    uint8_t id;
    #define STATUS_OK           (1<<0)  // The robot is alive and ok
    #define STATUS_DRIVER_ERR   (1<<1)  // Error with drivers
    #define STATUS_IR           (1<<2)  // The infrared barrier detects something
    uint8_t status;
    int cap_volt;
    int bat1_volt;
    int bat2_volt;
} __attribute__((packed));

void com_init();

void com_tick();

bool com_is_all_ok();
void com_diagnostic();

#endif
