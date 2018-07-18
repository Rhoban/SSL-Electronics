/**
 *  WARNING:
 * 
 * The encoder API changed since this code was written and it should be updated now
 * if we want to use it (using asynchronous reading of the encoder with SPI)
 */
#include "hardware.h"
#ifdef ENCODER_QUADRATURE
#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include "encoder.h"

HardwareSPI encoder(ENCODER_SPI);

// Counter value
static uint32_t encoder_cnt = 0;

// Instruction
#define ENCODER_CLR     (0<<6)
#define ENCODER_RD      (1<<6)
#define ENCODER_WR      (2<<6)
#define ENCODER_LOAD    (3<<6)

// Register
#define ENCODER_NONE    (0<<3)
#define ENCODER_MDR0    (1<<3)
#define ENCODER_MDR1    (2<<3)
#define ENCODER_DTR     (3<<3)
#define ENCODER_CNTR    (4<<3)
#define ENCODER_OTR     (5<<3)
#define ENCODER_STR     (6<<3)

static void encoder_write(uint8_t reg, uint8_t value)
{
    digitalWrite(ENCODER_SELECT_PIN, LOW);
    delay_us(1);
    encoder.send(ENCODER_WR|reg);
    encoder.wait();
    encoder.send(value);
    encoder.wait();
    delay_us(10);
    digitalWrite(ENCODER_SELECT_PIN, HIGH);
}

static uint8_t encoder_read(uint8_t reg)
{
    uint8_t result;
    digitalWrite(ENCODER_SELECT_PIN, LOW);
    delay_us(1);
    encoder.send(ENCODER_RD|reg);
    encoder.wait();
    result = encoder.send(0x00);
    encoder.wait();
    delay_us(1);
    digitalWrite(ENCODER_SELECT_PIN, HIGH);

    return result;
}

static uint32_t encoder_read4(uint8_t reg)
{
    uint32_t result;
    digitalWrite(ENCODER_SELECT_PIN, LOW);
    delay_us(1);
    encoder.send(ENCODER_RD|reg);
    encoder.wait();
    result = encoder.send(0x00)<<24;
    encoder.wait();
    result |= encoder.send(0x00)<<16;
    encoder.wait();
    result |= encoder.send(0x00)<<8;
    encoder.wait();
    result |= encoder.send(0x00)<<0;
    encoder.wait();
    delay_us(1);
    digitalWrite(ENCODER_SELECT_PIN, HIGH);

    return result;
}

void encoder_init()
{
    // Initializing pins
    digitalWrite(ENCODER_SELECT_PIN, HIGH);
    encoder.begin(SPI_9MHZ, MSBFIRST, 0);
    pinMode(ENCODER_SELECT_PIN, OUTPUT);
    encoder_read(ENCODER_STR);

    // Entering x4 quadrature mode
    encoder_write(ENCODER_MDR0, 0b11);
}

bool encoder_read()
{
    encoder_cnt = encoder_read4(ENCODER_CNTR);
    return true;
}

uint32_t encoder_value()
{
    return encoder_cnt;
}

TERMINAL_COMMAND(cnt, "Cnt debug")
{
    encoder_read();
    terminal_io()->println(encoder_value());
}
#endif
