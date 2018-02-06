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

void encoder_read()
{
    uint16_t fresh_value = encoder_read_value();
    uint16_t new_magnetic_value = fresh_value>>4;
    int16_t delta = (new_magnetic_value - magnetic_value);

    if (delta > 0x1ff) {
        delta -= 0x400;
    }
    if (delta < -0x1ff) {
        delta += 0x400;
    }

    encoder_cnt -= delta;
    magnetic_value = new_magnetic_value;
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
    int k = 0;
    while (!SerialUSB.available()) {
        delay(1);
        watchdog_feed();
        k++;
        if (k > 10) {
            k = 0;
            encoder_read();
            // terminal_io()->println(encoder_value());
            terminal_io()->println(magnetic_value);
        }
    }
}
#endif
