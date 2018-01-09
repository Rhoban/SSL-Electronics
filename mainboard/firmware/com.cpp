#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include <watchdog.h>
#include "hardware.h"
#include "com.h"

// Channels
static int com_channels[3] = {0, 50, 100};

HardwareSPI com(COM_SPI);

// Operations
#define OP_READ         0x00
#define OP_WRITE        0x20
#define OP_RX           0x61
#define OP_TX           0xa0
#define OP_NOP          0xff

// Registers
#define REG_CONFIG      0x00
#define REG_EN_AA       0x01
#define REG_EN_RXADDR   0x02
#define REG_SETUP_AW    0x03
#define REG_SETUP_RETR  0x04
#define REG_RF_CH       0x05
#define REG_RF_SETUP    0x06
#define REG_STATUS      0x07
#define REG_OBSERVE_TX  0x08
#define REG_RPD         0x09
#define REG_RX_ADDR_P0  0x0a  // 5 bytes
#define REG_RX_ADDR_P1  0x0b  // 5 bytes
#define REG_RX_ADDR_P2  0x0c
#define REG_RX_ADDR_P3  0x0d
#define REG_RX_ADDR_P4  0x0e
#define REG_RX_ADDR_P5  0x0f
#define REG_TX_ADDR     0x10 // 5 bytes
#define REG_RX_PW_P0    0x11
#define REG_RX_PW_P1    0x12
#define REG_RX_PW_P2    0x13
#define REG_RX_PW_P3    0x14
#define REG_RX_PW_P4    0x15
#define REG_RX_PW_P5    0x16
#define REG_FIFO_STATUS 0x17
#define REG_DYNPD       0x1c
#define REG_FEATURE     0x1d

#define PAYLOAD_SIZE    5

uint8_t com_data[3][PAYLOAD_SIZE];
bool com_available[3] = {false};

int com_pins[3] = {
    COM_CS1, COM_CS2, COM_CS3
};

TERMINAL_PARAMETER_INT(irqed, "", 0);

void com_send(int index, uint8_t *packet, size_t n)
{
    digitalWrite(com_pins[index], LOW);
    delay_us(10);
    for (size_t k=0; k<n; k++) {
        uint8_t reply = com.send(packet[k]);
        packet[k] = reply;
    }
    delay_us(10);
    digitalWrite(com_pins[index], HIGH);
}

void com_set_reg(int index, uint8_t reg, uint8_t value)
{
    reg |= OP_WRITE;

    uint8_t packet[2] = {
        reg,
        value
    };

    com_send(index, packet, 2);
}

void com_set_reg5(int index, uint8_t reg, uint8_t value[5])
{
    reg |= OP_WRITE;

    uint8_t packet[6] = {
        reg,
        value[0],
        value[1],
        value[2],
        value[3],
        value[4],
    };

    com_send(index, packet, 6);
}

uint8_t com_read_reg(int index, uint8_t reg)
{
    reg |= OP_READ;

    uint8_t packet[2] = {
        reg,
        0x00
    };

    com_send(index, packet, 2);

    return packet[1];
}

void com_read_reg5(int index, uint8_t reg, uint8_t result[5])
{
    reg |= OP_READ;

    uint8_t packet[6] = {
        reg,
        0x00, 0x00, 0x00, 0x00, 0x00
    };

    com_send(index, packet, 6);

    for (int k=1; k<6; k++) {
        result[k-1] = packet[k];
    }
}

static void com_tx(int index, uint8_t *payload, size_t n)
{
    uint8_t packet[n+1];
    packet[0] = OP_TX;
    for (int k=0; k<n; k++) {
        packet[k+1] = payload[k];
    }

    com_send(index, packet, n+1);
}

static void com_rx(int index, uint8_t *payload, size_t n)
{
    uint8_t packet[n+1] = {0};

    packet[0] = OP_RX;

    com_send(index, packet, n+1);

    for (int k=1; k<n+1; k++) {
        payload[k-1] = packet[k];
    }
}

void com_irq(int index)
{
    int status = com_read_reg(index, REG_STATUS);

    if (status & 0x40) { // RX
        com_available[index] = true;
        com_rx(index, com_data[index], PAYLOAD_SIZE);
    }

    // Resetting flags
    com_set_reg(index, REG_STATUS, 0x70);
}

void com_irq1()
{
    //com_irq(0);
}

void com_irq2()
{
    //com_irq(1);
}

void com_irq3()
{
    com_irq(2);
}

static void com_set_tx_addr(int index, uint8_t target)
{
    uint8_t addr[5] = COM_ADDR;
    addr[4] = target;

    com_set_reg5(index, REG_TX_ADDR, addr);
}

static void com_ce_enable()
{
    digitalWrite(COM_CE, HIGH);
}

static void com_ce_disable()
{
    digitalWrite(COM_CE, LOW);
}

static void com_ce_pulse()
{
    com_ce_enable();
    delay_us(10);
    com_ce_disable();
}

static void com_mode(int index, bool power, bool rx)
{
    uint8_t value = (1<<3); // Enable CRC

    if (power) {
        value |= (1<<1);
    }
    if (rx) {
        value |= (1);
    }

    com_set_reg(index, REG_CONFIG, value);
}

void com_init()
{
    // Initializing SPI
    com.begin(SPI_9MHZ, MSBFIRST, 0);

    // Initializing CS pins
    for (int k=0; k<5; k++) {
        digitalWrite(com_pins[k], HIGH);
        pinMode(com_pins[k], OUTPUT);
        digitalWrite(com_pins[k], HIGH);
    }

    // Initializing COM_CE
    digitalWrite(COM_CE, LOW);
    pinMode(COM_CE, OUTPUT);
    digitalWrite(COM_CE, LOW);

    for (int k=0; k<3; k++) {
        // Disabling auto acknowledgement
        com_set_reg(k, REG_EN_AA, 0x00);

        // Setting the appropriate challen for this module
        com_set_reg(k, REG_RF_CH, com_channels[k]);

        // Setting the address
        uint8_t addr[5] = COM_ADDR;
        com_set_reg5(k, REG_RX_ADDR_P0, addr);

        // Enabling only the pipe 1
        com_set_reg(k, REG_EN_RXADDR, 1);

        // Setting payload in rx p0 to 1
        com_set_reg(k, REG_RX_PW_P0, 5);

        // Power up
        com_mode(k, true, true);
    }

    // Listening
    com_ce_enable();

    // Initializing IRQ pins
    pinMode(COM_IRQ1, INPUT);
    pinMode(COM_IRQ2, INPUT);
    pinMode(COM_IRQ3, INPUT);
    attachInterrupt(COM_IRQ1, com_irq1, FALLING);
    attachInterrupt(COM_IRQ2, com_irq2, FALLING);
    attachInterrupt(COM_IRQ3, com_irq3, FALLING);
}

void com_tick()
{
    static int last = millis();

#if 0
    if (millis() - last > 1000) {
        last = millis();

        // Disabling CE, going to TX mode
        com_ce_disable();
        com_mode(2, true, false);

        // Setting TX address
        com_set_tx_addr(2, 0);
        com_set_reg(2, REG_STATUS, 0x70);

        // Sending payload
        uint8_t packet[] = "hello";
        com_tx(2, packet, 5);

        // Sending
        com_ce_pulse();
    }
#endif
}

TERMINAL_COMMAND(ct, "Com tx")
{
    if (com_available[2]) {

        for (int k=0; k<PAYLOAD_SIZE; k++) {
            terminal_io()->println(com_data[2][k]);
        }

        com_available[2] = false;
    } else {
        terminal_io()->println("Nothing to read");
    }
}

TERMINAL_COMMAND(st, "St")
{
    terminal_io()->println("STATUS");
    terminal_io()->println(com_read_reg(2, REG_STATUS));
    terminal_io()->println("CONFIG");
    terminal_io()->println(com_read_reg(2, REG_CONFIG));
    terminal_io()->println("EN_AA");
    terminal_io()->println(com_read_reg(2, REG_EN_AA));
    terminal_io()->println("EN_RXADDR");
    terminal_io()->println(com_read_reg(2, REG_EN_RXADDR));
    terminal_io()->println("RF_CH");
    terminal_io()->println(com_read_reg(2, REG_RF_CH));
    terminal_io()->println("RX_PW_P0");
    terminal_io()->println(com_read_reg(2, REG_RX_PW_P0));

    terminal_io()->println("RX_ADDR_P0");
    uint8_t value[5];
    com_read_reg5(2, REG_RX_ADDR_P0, value);
    for (int k=0; k<5; k++)
    terminal_io()->println((int)value[k]);

    terminal_io()->println("TX_ADDR");
    com_read_reg5(2, REG_TX_ADDR, value);
    for (int k=0; k<5; k++)
    terminal_io()->println((int)value[k]);
}
