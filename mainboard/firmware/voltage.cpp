#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include "hardware.h"
#include "voltage.h"

TERMINAL_PARAMETER_FLOAT(bat1, "Battery 1", 0.0);
TERMINAL_PARAMETER_FLOAT(bat2, "Battery 2", 0.0);

void voltage_init()
{
    pinMode(BAT1_PIN, INPUT);
    pinMode(BAT2_PIN, INPUT);
}

void voltage_tick()
{
    static int lastSample = millis();

    if ((millis() - lastSample) > 5) {
        lastSample = millis();

        float v1 = analogRead(BAT1_PIN)*3.3/4096;
        v1 = v1*(BAT1_R1 + BAT1_R2)/BAT1_R2;
        bat1 = bat1*0.99 + v1*0.01;

        float v2 = analogRead(BAT1_PIN)*3.3/4096;
        v2 = (v2*(BAT1_R1 + BAT1_R2)/BAT1_R2) - v1;
        if (v2 < 0) v2 = 0;
        bat2 = bat2*0.99 + v2*0.01;
    }
}

float voltage_bat1()
{
    return bat1;
}

float voltage_bat2()
{
    return bat2;
}
