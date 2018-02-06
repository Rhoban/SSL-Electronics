#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include "ir.h"
#include "hardware.h"
#include <watchdog.h>

void ir_init()
{
    digitalWrite(IR_EMIT, LOW);
    pinMode(IR_EMIT, OUTPUT);
    pinMode(IR_RECEIVE, INPUT_ANALOG);
}

TERMINAL_COMMAND(ir, "Test IR")
{
    while (!SerialUSB.available()) {
        digitalWrite(IR_EMIT, HIGH);
        int value = analogRead(IR_RECEIVE);
        digitalWrite(IR_EMIT, LOW);

        terminal_io()->println(value);
        delay(5);
        watchdog_feed();
    }
}
