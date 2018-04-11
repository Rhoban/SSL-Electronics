#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include "hardware.h"
#include "voltage.h"
#include "mux.h"

TERMINAL_PARAMETER_FLOAT(bat, "Battery 1", 0.0);

void voltage_init()
{
    bat = 0;
}

void voltage_tick()
{
    static int lastSample = millis();

    if ((millis() - lastSample) > 5) {
        lastSample = millis();

        float v1 = mux_sample(BAT_ADDR)*3.3/4096;
        v1 = v1*(BAT_R1 + BAT_R2)/BAT_R2;
        bat = bat*0.99 + v1*0.01;
    }
}

float voltage_value()
{
    return bat;
}
