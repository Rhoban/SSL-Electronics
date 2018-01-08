#include <stdlib.h>
#include <wirish/wirish.h>
#include <libmaple/iwdg.h>
#include <terminal.h>
#include <main.h>
#include <series/gpio.h>
#include <watchdog.h>
#include "encoder.h"
#include "current.h"
#include "motor.h"

HardwareSPI slave(1);
#define SLAVE_PIN           20

#define LED_PIN     22

TERMINAL_PARAMETER_INT(rcv, "Received byte", 0);
TERMINAL_PARAMETER_INT(irqed, "IRQed", 0);
TERMINAL_PARAMETER_INT(ssed, "Slave selected", 0);

extern "C"
{
void __irq_spi1()
{
    static uint8_t n = 0;
    irqed += 1;
    rcv = SPI1->regs->SR;

    if (spi_is_rx_nonempty(SPI1)) {
        rcv = spi_rx_reg(SPI1);
        if (rcv == 0xaa) {
            n += 1;
            spi_tx_reg(SPI1, n);
        }
        // Data received
    }
}
}

void slave_irq()
{
    digitalWrite(LED_PIN, digitalRead(SLAVE_PIN));
    int is_slave = digitalRead(SLAVE_PIN) == LOW;

    if (is_slave) {
        ssed = 1;
        slave.beginSlave(MSBFIRST, 0);
        spi_irq_enable(slave.c_dev(), SPI_RXNE_INTERRUPT);
    } else {
        slave.end();
        pinMode(slave.misoPin(), INPUT_FLOATING);
        spi_irq_disable(slave.c_dev(), SPI_INTERRUPTS_ALL);
    }
}

/**
 * Setup function
 */
void setup()
{
    // Enabling remap on SPI1
    afio_remap(AFIO_REMAP_SPI1);
    pinMode(SLAVE_PIN, INPUT);
    attachInterrupt(SLAVE_PIN, slave_irq, CHANGE);

    // Turning led off
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Initializng current  sensor
    current_init();

    // Initalizing encoder
    encoder_init();

    // Initalizing motor
    motor_init();

    // Starting the watchdog
    watchdog_start(WATCHDOG_14MS);

    terminal_init(&SerialUSB);
}

/**
 * Loop function
 */
void loop()
{
    // Feeding watchdog
    if (feed) watchdog_feed();

    // Updating motor phases, this is also done in the hall pin interrupt but
    // it seems safe to do it often anyway
    motor_tick();

    // Updating current sensor value
    current_tick();

    // Ticking the terminal
    terminal_tick();
}
