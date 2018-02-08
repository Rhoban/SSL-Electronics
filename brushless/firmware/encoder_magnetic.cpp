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

// Instruction
static uint16_t encoder_read_value()
{
    uint16_t result;
    digitalWrite(ENCODER_SELECT_PIN, LOW);
    //delay_us(1);
    result = (encoder.send(0xff) << 8);
    result |= encoder.send(0xff);
    //delay_us(1);
    digitalWrite(ENCODER_SELECT_PIN, HIGH);

    return result & 0x3fff;
}

TERMINAL_COMMAND(erv, "Encoder Read Value")
{
    int start = millis();

    while (!SerialUSB.available()) {
        uint16_t value = encoder_read_value();

        SerialUSB.println(value);

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

int32_t encoder_read_delta()
{
    uint16_t fresh_value = encoder_read_value();
    uint16_t new_magnetic_value = fresh_value;
    int32_t delta = (new_magnetic_value - magnetic_value);

    if (delta > 0x1fff) {
        delta -= 0x4000;
    }
    if (delta < -0x1fff) {
        delta += 0x4000;
    }

    magnetic_value = new_magnetic_value;
    return delta;
}

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

bool errored = false;
int last_error = 0;

TERMINAL_COMMAND(tst, "Test")
{
    if (errored) {
        terminal_io()->println(last_error);
    }
}

bool encoder_read()
{
    uint16_t fresh_value = encoder_read_value();
    encoder_deltas += encoder_compute_delta(magnetic_value, fresh_value);
    encoder_delta_pos++;

    if (encoder_delta_pos >= 32) {
        uint16_t old_magnetic_value = magnetic_value;
        encoder_deltas /= 32;
        magnetic_value = (magnetic_value + encoder_deltas + 0x4000)%(0x4000);

        if (encoder_deltas < -0x4000) {
            errored = true;
            last_error = encoder_deltas;
        }

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
    terminal_io()->println(encoder_cnt);
}
#endif
