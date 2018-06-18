#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include "ir.h"
#include "hardware.h"
#include <watchdog.h>

bool ir_detected = false;
int ir_value = 0;

void ir_init()
{
    digitalWrite(IR_EMIT, LOW);
    pinMode(IR_EMIT, OUTPUT);
    pinMode(IR_RECEIVE, INPUT_ANALOG);

    ir_value = 0;
}

int presentSince = 0;
bool ir_present()
{
    return (millis() - presentSince) > 10;
}

bool ir_present_now()
{
    return ir_detected;
}

void ir_tick()
{
    static int lastSample = 0;

    if (millis() - lastSample > 1) {
        lastSample = millis();
        digitalWrite(IR_EMIT, HIGH);
        ir_value = analogRead(IR_RECEIVE);

        if (ir_value < IR_THRESHOLD) {
            ir_detected = false;
            presentSince = millis();
        } else {
            ir_detected = true;
        }
        digitalWrite(IR_EMIT, LOW);
    }
}

void ir_diagnostic()
{
    if (ir_present()) {
        terminal_io()->println("* IR: ERROR OR SOMETHING PRESENT");
    } else {
        terminal_io()->println("* IR: OK");
    }
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

TERMINAL_COMMAND(irp, "Ir present?")
{
    terminal_io()->println(ir_present() ? 1 : 0);
    terminal_io()->println(ir_present_now() ? 1 : 0);
}
