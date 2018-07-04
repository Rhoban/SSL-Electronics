#include "hardware.h"
#ifdef ENCODER_MAGNETIC
#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include <watchdog.h>
#include "motor.h"
#include "encoder.h"


HardwareSPI encoder(ENCODER_SPI);

// Counter value
static uint32_t encoder_cnt = 0;
static uint16_t encoder_magnitude = 0;
static bool encoder_present = true;

bool encoder_is_ok()
{
    return encoder_magnitude > 500;
}

bool encoder_is_present()
{
    return encoder_present;
}

// Instruction
uint16_t encoder_read_value()
{
    uint16_t result;
    digitalWrite(ENCODER_SELECT_PIN, LOW);

    result = (encoder.send(0x7f) << 8);
    result |= encoder.send(0xfe);

    digitalWrite(ENCODER_SELECT_PIN, HIGH);
    digitalWrite(ENCODER_SELECT_PIN, LOW);

    encoder_magnitude = (encoder.send(0xff) << 8);
    encoder_magnitude |= encoder.send(0xff);
    encoder_present = (encoder_magnitude != 0 && encoder_magnitude != 0xffff);
    encoder_magnitude &= 0x3fff;

    digitalWrite(ENCODER_SELECT_PIN, HIGH);

    return result & 0x3fff;
}

TERMINAL_COMMAND(erv, "Encoder Read Value")
{
    int start = millis();

    while (!SerialUSB.available()) {
        uint16_t value = encoder_read_value();

        SerialUSB.print(value);
        SerialUSB.print(" (");
        SerialUSB.print(encoder_magnitude);
        SerialUSB.print(")");
        SerialUSB.println();

        delay(5);
        watchdog_feed();
    }
}

void encoder_init()
{
    // Initializing pins
    digitalWrite(ENCODER_SELECT_PIN, HIGH);
    encoder.begin(SPI_9MHZ, MSBFIRST, SPI_MODE_LOW_FALLING);
    pinMode(ENCODER_SELECT_PIN, OUTPUT);

    encoder_read();
}

uint16_t magnetic_value = 0;
uint8_t sample = 0;
uint16_t average_sample = 0;

static int32_t encoder_deltas = 0;
static uint8_t encoder_delta_pos = 0;

int32_t encoder_compute_delta(uint16_t a, uint16_t b)
{
    int32_t delta = b - a;

    if (delta > 0x1fff) {
        delta -= 0x4000;
    }
    if (delta < -0x1fff) {
        delta += 0x4000;
    }

    return delta;
}

bool encoder_read()
{
    uint16_t fresh_value = encoder_read_value();
    encoder_deltas += encoder_compute_delta(magnetic_value, fresh_value);
    encoder_delta_pos++;

    if (encoder_delta_pos >= 8) {
        encoder_deltas /= 8;
        magnetic_value = (magnetic_value + encoder_deltas + 0x4000)%(0x4000);

        encoder_cnt -= encoder_deltas;

        encoder_deltas = 0;
        encoder_delta_pos = 0;
        return true;
    }
    return false;
}

uint32_t encoder_value()
{
    return encoder_cnt;
}

TERMINAL_COMMAND(eb, "Encoder benchmark")
{
    int start = micros();
    for (int k=0; k<10000; k++) {
        watchdog_feed();
        encoder_read();
    }
    terminal_io()->println((micros()-start)/10000.0);
}

TERMINAL_COMMAND(cnt, "Cnt debug")
{
    terminal_io()->println((int32_t)encoder_cnt);
    terminal_io()->println(encoder_read_value());
}
#endif
