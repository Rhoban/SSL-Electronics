#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include "hardware.h"
#include "com.h"
#include "servo.h"
#include "security.h"

HardwareSPI slave(SLAVE_SPI);

TERMINAL_PARAMETER_INT(rcv, "Received byte", 0);
TERMINAL_PARAMETER_INT(irqed, "IRQed", 0);
TERMINAL_PARAMETER_INT(ssed, "Slave selected", 0);

static uint8_t frame[sizeof(struct driver_packet)];
static int frame_pos = 0;
static int last_receive = 0;
static bool controlling = false;

extern "C"
{
void __irq_spi1()
{
    static uint8_t n = 0;
    irqed += 1;
    rcv = SPI1->regs->SR;

    if (spi_is_rx_nonempty(SPI1)) {
        rcv = spi_rx_reg(SPI1);

        if (frame_pos < sizeof(struct driver_packet)) {
            frame[frame_pos++] = rcv;
            if (frame_pos == sizeof(struct driver_packet)) {
                struct driver_packet *packet;
                packet = (struct driver_packet *)frame;

                last_receive = millis();
                controlling = true;
                digitalWrite(LED_PIN, HIGH);
                servo_set(packet->enable, packet->targetSpeed);
                servo_set_pid(packet->kp, packet->ki, packet->kd);
            }
        }
    }
}
}

static void slave_irq()
{
    int is_slave = digitalRead(SLAVE_PIN) == LOW;

    if (is_slave) {
        ssed = 1;
        frame_pos = 0;
        slave.beginSlave(MSBFIRST, 0);
        spi_irq_enable(slave.c_dev(), SPI_RXNE_INTERRUPT);
        if (security_get_error() == SECURITY_NO_ERROR) {
            spi_tx_reg(SPI1, 0xaa);
        } else {
            spi_tx_reg(SPI1, 0x50|security_get_error());
        }
    } else {
        slave.end();
        pinMode(slave.misoPin(), INPUT_FLOATING);
        spi_irq_disable(slave.c_dev(), SPI_INTERRUPTS_ALL);
    }
}


void com_init()
{
    // Enabling remap on SPI1
    afio_remap(AFIO_REMAP_SPI1);
    pinMode(SLAVE_PIN, INPUT);
    attachInterrupt(SLAVE_PIN, slave_irq, CHANGE);

    // Turning led off
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
}

void com_tick()
{
    if (controlling) {
        if (millis() - last_receive > 100) {
            controlling = false;
            digitalWrite(LED_PIN, LOW);
            servo_set(false, 0);
        }
    }
}
