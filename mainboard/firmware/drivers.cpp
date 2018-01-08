#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include <watchdog.h>
#include "hardware.h"
#include "drivers.h"

HardwareSPI drivers(DRIVERS_SPI);

static int drivers_pin[5] = {
    DRIVERS_CS1, DRIVERS_CS2, DRIVERS_CS3,
    DRIVERS_CS4, DRIVERS_CS5
};

static int drivers_ping(int index)
{
    digitalWrite(drivers_pin[index], LOW);
    delay_us(20);
    int answer = drivers.send(0);
    delay_us(1);
    digitalWrite(drivers_pin[index], HIGH);

    return (answer == 0xaa);
}

void drivers_set(int index, bool enable, float target)
{
    uint8_t frame[5];
    frame[0] = enable;
    *(float*)(&frame[1]) = target;

    digitalWrite(drivers_pin[index], LOW);
    delay_us(25);
    for (int k=0; k<5; k++) {
        drivers.send(frame[k]);
    }
    delay_us(1);
    digitalWrite(drivers_pin[index], HIGH);
}

void drivers_init()
{
    // Initializing SPI
    drivers.begin(SPI_9MHZ, MSBFIRST, 0);

    // Initializing CS pins
    for (int k=0; k<5; k++) {
        digitalWrite(drivers_pin[k], HIGH);
        pinMode(drivers_pin[k], OUTPUT);
        digitalWrite(drivers_pin[k], HIGH);
    }
}

TERMINAL_COMMAND(scan, "Scan for drivers")
{
    for (int k=0; k<5; k++) {
        terminal_io()->print("Driver #");
        terminal_io()->print(k);
        terminal_io()->print(" ");
        terminal_io()->print(drivers_pin[k]);
        terminal_io()->print(" ");
        if (drivers_ping(k)) {
            terminal_io()->println("Present!");
        } else {
            terminal_io()->println("-");
        }
    }
}

TERMINAL_COMMAND(set, "Set speed for one driver")
{
    if (argc != 2) {
        terminal_io()->println("Usage: set [driver] [speed]");
    } else {
        while (!SerialUSB.available()) {
            drivers_set(atoi(argv[0]), true, atof(argv[1]));
            watchdog_feed();
            delay(10);
        }
    }
}

TERMINAL_COMMAND(em, "Emergency")
{
    for (int k=0; k<5; k++) {
        drivers_set(k, 0, 0.0);
    }
}
