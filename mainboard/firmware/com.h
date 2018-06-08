#ifndef _COM_H
#define _COM_H

#include <stdint.h>

#define PACKET_SIZE                 16
#define PACKET_INSTRUCTIONS         2
#define MAX_ROBOTS                  8

#define INSTRUCTION_MASTER          0x00
struct packet_master {
    #define ACTION_ON      (1<<0)   // The robot should be on (else everything is stopped)
                                    // Kicking, transition from 0 to 1 trigger kick if IR is present
    #define ACTION_KICK1   (1<<1)   // Kick on kicker 1 - chip
    #define ACTION_KICK2   (1<<2)   // Kick on kicker 2 - normal
    #define ACTION_DRIBBLE (1<<3)   // Enable/disable the dribbler
    #define ACTION_CHARGE  (1<<5)   // Enable/disable the capacitor charge
    uint8_t actions;

    int16_t x_speed;                // Kinematic orders [mm/s]
    int16_t y_speed;
    int16_t t_speed;                // Rotation in [mrad/s]

    uint8_t kickPower;             // Kick power (this is a duration in [x25 uS])
} __attribute__((packed));

#define INSTRUCTION_PARAMS          0x01
struct packet_params {
    float kp, ki, kd;               // Servo parameter
} __attribute__((packed));

// Robot status packet
struct packet_robot {
    uint8_t id;

    #define STATUS_OK           (1<<0)  // The robot is alive and ok
    #define STATUS_DRIVER_ERR   (1<<1)  // Error with drivers
    #define STATUS_IR           (1<<2)  // The infrared barrier detects the ball
    uint8_t status;

    uint8_t cap_volt;                  // Kick capcaitor voltage [V]

    uint8_t voltage;                  // Battery voltage [8th of V]
} __attribute__((packed));

void com_init();

void com_tick();

bool com_is_all_ok();
void com_diagnostic();

#endif
