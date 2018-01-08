#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include "hardware.h"
#include "current.h"

// Current sensing
static float current = 0.0;
static float current_ref = 0.0;

void current_init()
{
    pinMode(CURRENT_PIN, INPUT);
}

void current_tick()
{
    static int samples = 0;
    static int last_update = millis();

    if ((millis() - last_update) > 10) {
        last_update += 10;
        samples++;

        // Measured voltage [V] on the current sensor output
        float voltage = (5.0/3.0)*analogRead(CURRENT_PIN)*3300.0/4096.0;

        if (samples == 1) {
            current = voltage;
        } else {
            current = current*0.95 + voltage*0.05;
        }

        if (samples == 100) {
            // XXX: We should re-estimate the reference sometime, when we know
            // that the motor is off for instance
            current_ref = current;
        }
    }
}

float current_amps()
{
    return -20.0*((current - current_ref)/current_ref);
}

TERMINAL_COMMAND(amps, "Current")
{
    terminal_io()->print(current_amps());
    terminal_io()->println(" A");
}
