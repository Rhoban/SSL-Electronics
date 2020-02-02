#ifndef _COM_H
#define _COM_H

#include "odometry.h"
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
    #define ACTION_TARE_ODOM (1<<7)   // Tare Odometry
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

    uint8_t cap_volt;                  // Kick capacitor voltage [V]

    uint8_t voltage;                  // Battery voltage [8th of V]

    int16_t xpos;                     // Data planned by odometry
    int16_t ypos;                     // In mm
    int16_t ang;                      // In rad/10000

} __attribute__((packed));


#define MUSIC_PARAMS       0x02
struct packet_music {

    #define SOUND_ON  (1<<0)   
    #define BEEPER    (1<<1)   
    #define KICK      (1<<2)  
    #define CHIP_KICK (1<<3)   
    #define DRIBBLER  (1<<5)   

    uint8_t instrument;
    uint16_t note;
    uint16_t duration;

} __attribute__((packed));

struct buzzer_note {
    unsigned int freq;
    unsigned int duration;
};

// void buzzer_play(unsigned int melody, bool repeat=false);


void com_init();

void com_tick();


int has_data(int card);
void receive(int card, uint8_t *payload, int size);
void send(int card, uint8_t *payload, int size);



bool com_is_all_ok();
void com_diagnostic();

int get_channel(int card);
void set_channel(int index, int chan);

int get_lost_count(int card);
int get_retransmitted_count(int card);

void clear_status(int card);

int get_pipe_payload(int card,int pipe);
void set_pipe_payload(int card,int pipe,uint8_t pl);

int get_ack(int card);
void set_ack(int card, bool v);

void set_crc(int card,int crc);

void com_set_reg(int index, uint8_t reg, uint8_t value);
void com_set_reg5(int index, uint8_t reg, uint8_t value[5]);
void com_flush_rx(int index);
int get_retransmission_delay(int card);
int get_retransmission_count(int card);
void set_retransmission(int card, int delay, int count);

void power(int card,bool up);
#define SPEED_250k 0x20
#define SPEED_1M   0x00
#define SPEED_2M   0x08
#define POW_18db 0x00
#define POW_12db 0x01
#define POW_6db  0x02
#define POW_0db  0x03
void set_rf(int card, uint8_t speed, uint8_t pow);
uint8_t get_config(int card);
void set_config(int card,uint8_t);
uint8_t get_status(int card);
void reset_status(int card);
uint8_t get_rf_setup(int card);
uint8_t get_fifo_status(int card);
struct addr5{
uint8_t addr[5];
};
struct addr5 com_get_tx_addr(int index);
void com_set_tx_addr(int index, struct addr5 add);
void com_set_rx_addr(int index,int pipe, struct addr5 add);
struct addr5 com_get_rx_addr(int index,int pipe);


void com_ce_enable(int index);
void com_ce_disable(int index);

#endif
