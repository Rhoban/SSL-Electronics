#include <stdlib.h>
#include <wirish/wirish.h>
#include <libmaple/iwdg.h>
#include <terminal.h>
#include <main.h>
#include <libmaple/rcc.h>
#include <series/rcc.h>
#include <series/gpio.h>
#include "encoder.h"
#include "current.h"

HardwareSPI slave(1);
#define SLAVE_PIN           20

static void _init_timer(int number)
{
    HardwareTimer timer(number);

    // Configuring timer
    timer.pause();
    timer.setPrescaleFactor(1);
    timer.setOverflow(3000); // 24Khz
    timer.refresh();
    timer.resume();
}

// Motors pins
#define U_LOW_PIN        10
#define U_HIGH_PIN       11
#define V_LOW_PIN        8
#define V_HIGH_PIN       9
#define W_LOW_PIN        33
#define W_HIGH_PIN       3

#define LED_PIN     22

#define HALLU_PIN   7
#define HALLV_PIN   6
#define HALLW_PIN   5

TERMINAL_COMMAND(hall, "Test the hall sensors")
{
    while (!SerialUSB.available()) {
        terminal_io()->print(digitalRead(HALLU_PIN));
        terminal_io()->print(" ");
        terminal_io()->print(digitalRead(HALLV_PIN));
        terminal_io()->print(" ");
        terminal_io()->print(digitalRead(HALLW_PIN));
        terminal_io()->println();
        delay(10);
    }
}

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

    // Initializing hall sensors input
    pinMode(HALLU_PIN, INPUT);
    pinMode(HALLV_PIN, INPUT);
    pinMode(HALLW_PIN, INPUT);

    // Initializng current  sensor
    current_init();

    // Initalizing encoder
    encoder_init();

    digitalWrite(U_LOW_PIN, LOW);
    digitalWrite(U_HIGH_PIN, LOW);
    digitalWrite(V_LOW_PIN, LOW);
    digitalWrite(V_HIGH_PIN, LOW);
    digitalWrite(W_LOW_PIN, LOW);
    digitalWrite(W_HIGH_PIN, LOW);

    _init_timer(2);
    _init_timer(3);

    pwmWrite(U_LOW_PIN, 0);
    pwmWrite(U_HIGH_PIN, 0);
    pwmWrite(V_LOW_PIN, 0);
    pwmWrite(V_HIGH_PIN, 0);
    pwmWrite(W_LOW_PIN, 0);
    pwmWrite(W_HIGH_PIN, 0);
    pinMode(U_LOW_PIN, PWM);
    pinMode(U_HIGH_PIN, PWM);
    pinMode(V_LOW_PIN, PWM);
    pinMode(V_HIGH_PIN, PWM);
    pinMode(W_LOW_PIN, PWM);
    pinMode(W_HIGH_PIN, PWM);
    pwmWrite(U_LOW_PIN, 0);
    pwmWrite(U_HIGH_PIN, 0);
    pwmWrite(V_LOW_PIN, 0);
    pwmWrite(V_HIGH_PIN, 0);
    pwmWrite(W_LOW_PIN, 0);
    pwmWrite(W_HIGH_PIN, 0);

    terminal_init(&SerialUSB);
}

int hall_value()
{
    uint8_t hall = 0;

    hall  = ((digitalRead(HALLU_PIN)&1)<<2);
    hall |= ((digitalRead(HALLV_PIN)&1)<<1);
    hall |= ((digitalRead(HALLW_PIN)&1)<<0);

    return hall;
}

void set_phases(int u, int v, int w)
{
    if (u >= 0) {
        pwmWrite(U_HIGH_PIN, u);
        pwmWrite(U_LOW_PIN, 0);
    } else {
        pwmWrite(U_HIGH_PIN, 0);
        pwmWrite(U_LOW_PIN, -u);
    }

    if (v >= 0) {
        pwmWrite(V_HIGH_PIN, v);
        pwmWrite(V_LOW_PIN, 0);
    } else {
        pwmWrite(V_HIGH_PIN, 0);
        pwmWrite(V_LOW_PIN, -v);
    }

    if (w >= 0) {
        pwmWrite(W_HIGH_PIN, w);
        pwmWrite(W_LOW_PIN, 0);
    } else {
        pwmWrite(W_HIGH_PIN, 0);
        pwmWrite(W_LOW_PIN, -w);
    }
}

TERMINAL_COMMAND(wd, "Watchdog test")
{
    iwdg_init(IWDG_PRE_32, 350);

    while (!SerialUSB.available()) {
        iwdg_feed();
        delay_us(1250);
    }
}

TERMINAL_COMMAND(mos, "Mos test")
{
    if (argc > 0) {
        int speed = atoi(argv[0]);
        if (speed < -3000) speed = -3000;
        if (speed > 3000) speed = 3000;

        int phases[6][3] = {
            { 0,  1, -1},
            { 1,  0, -1},
            { 1, -1,  0},
            { 0, -1,  1},
            {-1,  0,  1},
            {-1,  1,  0},
        };

        int hall[8] = {
            -1,                     // 0b000 (impossible)
            5,                      // 0b001
            1,                      // 0b010
            0,                      // 0b011
            3,                      // 0b100
            4,                      // 0b101
            2,                      // 0b110
            -1,                     // 0b111 (impossible)
        };

        int s = millis();

        while (!SerialUSB.available()) {
            current_tick();

            int phase = hall[hall_value()];
            // terminal_io()->println(phase);
            // delay(500);

            if (phase >= 0) {
                phase = (phase + 1) % 6;

                set_phases(
                    phases[phase][0]*speed,
                    phases[phase][1]*speed,
                    phases[phase][2]*speed
                );
            }
            if (millis() - s > 40) {
                terminal_io()->println(current_amps());
                s = millis();
            }
        }

        set_phases(0, 0, 0);
    } else {
        terminal_io()->println("Usage: mos [speed]");
    }
}

/**
 * Loop function
 */
void loop()
{
    terminal_tick();
}
