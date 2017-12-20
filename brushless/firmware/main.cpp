#include <stdlib.h>
#include <wirish/wirish.h>
#include <libmaple/iwdg.h>
#include <terminal.h>
#include <main.h>
#include <libmaple/rcc.h>
#include <series/rcc.h>
#include <series/gpio.h>

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

// Current sensing
#define CURRENT_PIN     4
float current = 0.0;
float current_ref = 0.0;

// Motors pins
#define U_LOW_PIN        10
#define U_HIGH_PIN       11
#define V_LOW_PIN        8
#define V_HIGH_PIN       9
#define W_LOW_PIN        33
#define W_HIGH_PIN       3


HardwareSPI decoder(2);
#define DECODER_SELECT_PIN  31
#define DECODER_INDEX_PIN   12

// Instruction
#define DECODER_CLR     (0<<6)
#define DECODER_RD      (1<<6)
#define DECODER_WR      (2<<6)
#define DECODER_LOAD    (3<<6)

// Register
#define DECODER_NONE    (0<<3)
#define DECODER_MDR0    (1<<3)
#define DECODER_MDR1    (2<<3)
#define DECODER_DTR     (3<<3)
#define DECODER_CNTR    (4<<3)
#define DECODER_OTR     (5<<3)
#define DECODER_STR     (6<<3)

#define LED_PIN     22

#define HALLU_PIN   7
#define HALLV_PIN   6
#define HALLW_PIN   5

void decoder_write(uint8_t reg, uint8_t value)
{
    digitalWrite(DECODER_SELECT_PIN, LOW);
    delay_us(1);
    decoder.send(DECODER_WR|reg);
    decoder.wait();
    decoder.send(value);
    decoder.wait();
    delay_us(10);
    digitalWrite(DECODER_SELECT_PIN, HIGH);
}

uint8_t decoder_read(uint8_t reg)
{
    uint8_t result;
    digitalWrite(DECODER_SELECT_PIN, LOW);
    delay_us(1);
    decoder.send(DECODER_RD|reg);
    decoder.wait();
    result = decoder.send(0x00);
    decoder.wait();
    delay_us(1);
    digitalWrite(DECODER_SELECT_PIN, HIGH);

    return result;
}

uint32_t decoder_read4(uint8_t reg)
{
    uint32_t result;
    digitalWrite(DECODER_SELECT_PIN, LOW);
    delay_us(1);
    decoder.send(DECODER_RD|reg);
    decoder.wait();
    result = decoder.send(0x00)<<24;
    decoder.wait();
    result |= decoder.send(0x00)<<16;
    decoder.wait();
    result |= decoder.send(0x00)<<8;
    decoder.wait();
    result |= decoder.send(0x00)<<0;
    decoder.wait();
    delay_us(1);
    digitalWrite(DECODER_SELECT_PIN, HIGH);

    return result;
}

TERMINAL_COMMAND(dec, "Tests decoder")
{
    uint8_t r;

    while (!SerialUSB.available()) {
        terminal_io()->println("Reading MDR0");
        r = decoder_read(DECODER_MDR0);
        terminal_io()->println(r);
        delay(1000);

        terminal_io()->println("Writing 1 to MDR0");
        decoder_write(DECODER_MDR0, 0b00000011);
        delay(10);
    }
}

TERMINAL_COMMAND(cnt, "Cnt debug")
{
    while (!SerialUSB.available()) {
        uint32_t c = decoder_read4(DECODER_CNTR);
        terminal_io()->println(c);
        delay(10);
    }
}

TERMINAL_COMMAND(rd, "Read")
{
    uint8_t r;

    terminal_io()->println("Reading STR");
    r = decoder_read(DECODER_STR);
    terminal_io()->println(r);

    terminal_io()->println("Reading MDR0");
    r = decoder_read(DECODER_MDR0);
    terminal_io()->println(r);

    terminal_io()->println("Reading MDR1");
    r = decoder_read(DECODER_MDR1);
    terminal_io()->println(r);

    terminal_io()->println("Reading CNTR");
    uint32_t c = decoder_read4(DECODER_CNTR);
    terminal_io()->println(c);
}

TERMINAL_COMMAND(rd2, "Read")
{
    uint8_t r;

    terminal_io()->println("Reading STR");
    r = decoder_read(DECODER_STR);
    terminal_io()->println(r);
}

TERMINAL_COMMAND(rd3, "Read")
{
    uint8_t r;

    terminal_io()->println("Reading MDR0");
    r = decoder_read(DECODER_MDR0);
    terminal_io()->println(r);
}

TERMINAL_COMMAND(wr, "Write")
{
    terminal_io()->println("Writing 0b11 to MDR0");
    decoder_write(DECODER_MDR0, 0b11);
}

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

    // Current sensor
    pinMode(CURRENT_PIN, INPUT);

    // Initalizing SPI
    digitalWrite(DECODER_SELECT_PIN, HIGH);
    decoder.begin(SPI_9MHZ, MSBFIRST, 0);
    pinMode(DECODER_SELECT_PIN, OUTPUT);
    decoder_read(DECODER_STR);
    // digitalWrite(DECODER_INDEX_PIN, LOW);
    // pinMode(DECODER_INDEX_PIN, OUTPUT);

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

void current_tick()
{
    static int samples = 0;
    static int last_update = millis();

    if ((millis() - last_update) > 10) {
        last_update += 10;
        samples++;

        float voltage = (5.0/3.0)*analogRead(CURRENT_PIN)*3300.0/4096.0;

        if (samples == 1) current = voltage;
        else current = current*0.95 + voltage*0.05;

        if (samples == 200) {
            current_ref = current;
        }
    }
}

float current_amps()
{
    return -20.0*((current - current_ref)/current_ref);
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
