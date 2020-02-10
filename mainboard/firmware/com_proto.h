#pragma once

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
    uint8_t rid;
    uint8_t actions;

    int16_t x_speed;                // Kinematic orders [mm/s]
    int16_t y_speed;
    int16_t t_speed;                // Rotation in [mrad/s]

    uint8_t kickPower;             // Kick power (this is a duration in [x25 uS])
} __attribute__((packed));

//#define INSTRUCTION_PARAMS          0x01
//struct packet_params {
//    float kp, ki, kd;               // Servo parameter
//} __attribute__((packed));

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

#define ICMP_ECHO         0x01
#define ICMP_DHCP_REQUEST 0x02
#define ICMP_DHCP_REPLY   0x03
#define ICMP_NOREPLY      0x04

#define ICMP_FULL 0x01
#define ICMP_OK   0x02

#define CARD_STATUS 0
#define CARD_ORDER  1
#define CARD_ICMP   2
bool card_status_ok=false;
bool card_order_ok=false;
bool card_icmp_ok=false;


struct icmp_order{
    uint8_t icmp_type;
    uint8_t arg;
    uint8_t icmp_addr[5];
    struct icmp_order &operator=(const struct icmp_order &o){
        icmp_type = o.icmp_type;
        arg=o.arg;
        for(int i=0;i<5;++i)
            icmp_addr[i]=o.icmp_addr[i];
        return *this;
    }
};


// last byte will be from FF for tx and C0 to C5 for pipes 1 to 6
#define addr_for_status {0xC0, 0xB4, 0xC5, 0xD6, 0XA3};
#define status_chan 110
#define status_payload_size sizeof(packet_robot)
#define STATUS_TIMEOUT_MS 500

// last byte will be from A0 to A5 for master to robot com
#define addr_for_orders {0x8A,0x7B,0x6C,0x5D,0xA0};
#define orders_chan 116
#define order_payload_size sizeof(packet_master)
#define ORDER_TIMEOUT_MS 5000

#define icmp_chan 120
#define addr_for_icmp {0xA1,0xA3,0xB6,0xE4,0xB6};
#define icmp_payload_size sizeof(icmp_order)
#define ICMP_TIMEOUT_MS 1000
//#define icmp_payload_size 32





