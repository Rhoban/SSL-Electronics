#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include "hardware.h"
#include "current.h"
#include "motor.h"
#include "security.h"

// Current sensing
static float current = 0.0;
static float current_ref = 0.0;

void current_init()
{
    pinMode(CURRENT_PIN, INPUT);
}

static int samples = 0;

void current_resample()
{
    samples = 0;
    current = 0.0;
    current_ref = 0.0;
}

void current_tick()
{
    return;
    static int last_update = millis();
    static int last_limit = 0;

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
            last_limit = millis();
        }

        // Security
        if (samples > 100) {
            float amps = fabs(current_amps());

            if (amps > CURRENT_LIMIT) {
                // We are over CURRENT_LIMIT for more than CURRENT_DURATION ms
                if (millis() - last_limit > CURRENT_DURATION) {
                    security_set_error(SECURITY_CURRENT_MAX);
                }
            } else {
                last_limit = millis();
            }

            // We are over CURRENT_MAX
            if (amps > CURRENT_MAX) {
                security_set_error(SECURITY_CURRENT_LIMIT);
            }
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
