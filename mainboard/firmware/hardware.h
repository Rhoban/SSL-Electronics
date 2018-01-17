#ifndef _HARDWARE_H
#define _HARDWARE_H

// Brushless drivers
#define DRIVERS_CS1 7
#define DRIVERS_CS2 26
#define DRIVERS_CS3 19
#define DRIVERS_CS4 18
#define DRIVERS_CS5 17

#define DRIVERS_SPI 1

// Communication pins
#define COM_CE  13

#define COM_CS1 20
#define COM_CS2 21
#define COM_CS3 22

#define COM_IRQ1 23
#define COM_IRQ2 24
#define COM_IRQ3 25

#define COM_SPI 2

#define COM_ADDR   {0x22, 0x87, 0xe8, 0xf9, 0x00}
#define COM_ID     1
#define COM_MASTER 0xff

// Buzzer
#define BUZZER_PIN      11
#define BUZZER_TIMER    2

// Booster
#define BOOSTER_PIN     27
#define BOOSTER_TIMER   1
#define CAP_PIN         3
#define CAP_R1          2000000.0
#define CAP_R2          10000.0

// Kicker
#define KICKER1_PIN     0
#define KICKER_TIMER    4

// Voltage
#define BAT1_PIN        8
#define BAT2_PIN        9
#define BAT1_R1         10000
#define BAT1_R2         1000
#define BAT2_R1         10000
#define BAT2_R2         1000

#endif
