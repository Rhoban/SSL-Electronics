#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include <watchdog.h>
#include "hardware.h"
#include "drivers.h"
#include "buzzer.h"

HardwareSPI drivers(DRIVERS_SPI);
static bool drivers_is_error = false;

static int drivers_pins[5] = {
    DRIVERS_CS1, DRIVERS_CS2, DRIVERS_CS3,
    DRIVERS_CS4, DRIVERS_CS5
};


int drivers_ping(int index)
{
    digitalWrite(drivers_pins[index], LOW);
    delay_us(25);
    int answer = drivers.send(0);
    delay_us(5);
    digitalWrite(drivers_pins[index], HIGH);

    return (answer == 0xaa);
}

uint8_t drivers_set(int index, bool enable, float target)
{
    uint8_t frame[5];
    frame[0] = enable;
    *(float*)(&frame[1]) = target;
    uint8_t answer;

    digitalWrite(drivers_pins[index], LOW);
    delay_us(25);
    for (int k=0; k<5; k++) {
        uint8_t reply = drivers.send(frame[k]);
        if (k == 0) {
            answer = reply;
        }
    }
    delay_us(5);
    digitalWrite(drivers_pins[index], HIGH);

    return answer;
}

void drivers_set_safe(int index, bool enable, float target)
{
    if (!drivers_is_error) {
        uint8_t answer = drivers_set(index, enable, target);

        if ((answer&0xf0) == 0x50) {
            for (int k=0; k<5; k++) {
                drivers_set(k, false, 0.0);
            }

            drivers_is_error = true;
            buzzer_play(MELODY_WARNING);
        }
    }
}

void drivers_tick()
{
    if (drivers_is_error) {
        if (buzzer_is_playing()) {
            for (int k=0; k<5; k++) {
                drivers_set(k, false, 0.0);
            }
        } else {
            drivers_is_error = false;
        }
    }
}

void drivers_init()
{
    // Initializing SPI
    drivers.begin(SPI_9MHZ, MSBFIRST, 0);

    // Initializing CS pins
    for (int k=0; k<5; k++) {
        digitalWrite(drivers_pins[k], HIGH);
        pinMode(drivers_pins[k], OUTPUT);
        digitalWrite(drivers_pins[k], HIGH);
    }
}

TERMINAL_COMMAND(scan, "Scan for drivers")
{
    for (int k=0; k<5; k++) {
        terminal_io()->print("Driver #");
        terminal_io()->print(k);
        terminal_io()->print(" ");
        terminal_io()->print(drivers_pins[k]);
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
            drivers_set_safe(atoi(argv[0]), true, atof(argv[1]));
            drivers_tick();
            buzzer_tick();
            watchdog_feed();
            delay(5);
        }
    }
}

TERMINAL_COMMAND(blink, "Blink the drivers")
{
    for (int k=0; k<5; k++) {
        terminal_io()->print("Blinking ");
        terminal_io()->println(k);

        for (int n=0; n<50; n++) {
            delay(10);
            drivers_set(k, true, 0);
            watchdog_feed();
        }
    }
}

bool drivers_is_all_ok()
{
    for (int k=0; k<5; k++) {
        if (!drivers_ping(k)) {
            return false;
        }
    }

    return true;
}

void drivers_diagnostic()
{
    for (int k=0; k<5; k++) {
        terminal_io()->print("* Driver #");
        terminal_io()->print(k);
        if (!drivers_ping(k)) {
            terminal_io()->println(" MISSING");
        } else {
            terminal_io()->println(" OK");
        }
    }
}

TERMINAL_COMMAND(err, "Error")
{
    if (drivers_is_error) {
        terminal_io()->println("Drivers are in error mode");
    } else {
        terminal_io()->println("Drivers are OK");
    }
}
