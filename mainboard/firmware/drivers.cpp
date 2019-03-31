#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include <watchdog.h>
#include "hardware.h"
#include "drivers.h"
#include "buzzer.h"
#include "../../brushless/firmware/errors.h"

HardwareSPI drivers(DRIVERS_SPI);
static bool drivers_is_error = false;
static bool drivers_present[5] = {false};
struct driver_packet_ans driver_answers[5];

static int drivers_pins[5] = {
    DRIVERS_CS1, DRIVERS_CS2, DRIVERS_CS3,
    DRIVERS_CS4, DRIVERS_CS5
};

uint8_t drivers_status(int index)
{
    digitalWrite(drivers_pins[index], LOW);
    delay_us(35);
    drivers.send(0);
    uint8_t answer = drivers.send(0);
    delay_us(5);
    digitalWrite(drivers_pins[index], HIGH);

    return answer;
}

int drivers_ping(int index)
{
    uint8_t status = drivers_status(index);
    return (status == 0x55 || ((status&0xf0) == 0x80));
}

static void drivers_send(int index, uint8_t instruction, uint8_t *data, size_t len, uint8_t *answer)
{
    digitalWrite(drivers_pins[index], LOW);
    delay_us(35);

    drivers.send(instruction);

    for (int k=0; k < len; k++) {
        if (answer == NULL) {
            drivers.send(data[k]);
        } else {
            *(answer++) = drivers.send(data[k]);
        }
    }
    delay_us(5);
    digitalWrite(drivers_pins[index], HIGH);
}

struct driver_packet_ans drivers_set(int index, bool enable, float target, int16_t pwm)
{
    struct driver_packet_set packet;
    packet.enable = enable;
    packet.targetSpeed = target;
    packet.pwm = pwm;
    packet.padding1 = 0;
    packet.padding2 = 0;

    struct driver_packet_ans answer;
    drivers_send(index, DRIVER_PACKET_SET, (uint8_t*)&packet, sizeof(struct driver_packet_set), (uint8_t*)&answer);
    return answer;
}

void drivers_set_params(float kp, float ki, float kd)
{
    for (int index = 0; index < 5; index++) {
        struct driver_packet_params packet;
        packet.kp = kp;
        packet.ki = ki;
        packet.kd = kd;

        drivers_send(index, DRIVER_PACKET_PARAMS, (uint8_t*)&packet, sizeof(struct driver_packet_params), NULL);
    }
}

void drivers_set_safe(int index, bool enable, float target, int16_t pwm)
{
    if (!drivers_is_error && drivers_present[index]) {
        struct driver_packet_ans tmp = drivers_set(index, enable, target, pwm);

        if ((tmp.status & 0xf0) == 0x80) {
            driver_answers[index] = tmp;
            for (int k=0; k<5; k++) {
                drivers_set(k, false, 0.0);
            }

            drivers_is_error = true;
            buzzer_play(MELODY_WARNING);
            // terminal_io()->println("Error on driver:");
            // terminal_io()->println(index);
            // terminal_io()->println(driver_answers[index].status&0xf);
        } else if (tmp.status == 0x55) {
            driver_answers[index] = tmp;
        }
    }
}

TERMINAL_COMMAND(pid, "PID")
{
    if (argc != 3) {
        terminal_io()->println("Usage: pid [p] [i] [d]");
    } else {
        drivers_set_params(atof(argv[0]), atof(argv[1]), atof(argv[2]));
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
    drivers.begin(SPI_281_250KHZ, MSBFIRST, 0);

    // Initializing CS pins
    for (int k=0; k<5; k++) {
        pinMode(drivers_pins[k], OUTPUT);
        digitalWrite(drivers_pins[k], HIGH);
    }

    for (int k=0; k<5; k++) {
        drivers_present[k] = drivers_ping(k);
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

//GROS DEBUG
            digitalWrite(IR_EMIT, HIGH);
            int value = analogRead(IR_RECEIVE);
            digitalWrite(IR_EMIT, LOW);
            terminal_io()->println(value);
            delay(5);
            /////////


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
    // XXX: We are only looking for the 4th firsts
    for (int k=0; k<4; k++) {
        if (!drivers_present[k]) {
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
        if (!drivers_present[k]) {
            terminal_io()->println(" MISSING");
        } else {
            uint8_t status = drivers_status(k);
            if (status == 0x55) {
                terminal_io()->println(" OK");
            } else {
                terminal_io()->print(" ERR: ");
                terminal_io()->println(driver_error(status&0xf));
            }
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

TERMINAL_COMMAND(ddb, "")
{
    struct driver_packet_ans ans = drivers_set(2, true, 0, 1);

    terminal_io()->print(ans.speed);
    terminal_io()->print(" ");
    terminal_io()->println(ans.pwm);

    uint8_t *ptr = (uint8_t*)&ans;
    for (size_t k=0; k<sizeof(ans); k++) {
        terminal_io()->println((int)ptr[k]);
    }
}
