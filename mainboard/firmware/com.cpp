#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include <watchdog.h>
#include "kicker.h"
#include "hardware.h"
#include "drivers.h"
#include "voltage.h"
#include "kicker.h"
#include "com.h"

// Channels
static int com_channels[3] = {0, 50, 100};

// Only for master board
static bool com_master = false;
static struct packet_master com_robots[6];
static struct packet_robot com_statuses[6];
static int com_robot_reception[6];
static int com_master_pos = 0;
static int com_usb_reception = 0;

// Only for robot
static int com_master_reception = 0;
static int com_master_packets = 0;
static bool com_master_new = false;
static bool com_master_controlling = false;
static struct packet_master com_master_packet;

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
    for (size_t k=0; k<n; k++) {
        uint8_t reply = com.send(packet[k]);
        packet[k] = reply;
    }
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

        if (com_master) {
            struct packet_robot packet;
            com_rx(index, (uint8_t*)&packet, sizeof(struct packet_robot));

            if (packet.id < 6) {
                com_statuses[packet.id] = packet;
                com_robot_reception[packet.id] = millis();
            }
        } else {
            // Reading master packet!
            com_master_reception = millis();
            com_master_new = true;
            com_master_packets++;
            com_rx(index, (uint8_t*)&com_master_packet, sizeof(struct packet_master));
        }
    }
    if (status & 0x20) { // TX
        com_mode(index, true, true);
    }

    // Resetting flags
    com_set_reg(index, REG_STATUS, 0x70);
}

void com_irq1()
{
    com_irq(0);
}

void com_irq2()
{
    com_irq(1);
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

bool com_is_ok(int index)
{
    uint8_t myAddr[5] = COM_ADDR;
    uint8_t addr[5];
    com_read_reg5(index, REG_RX_ADDR_P0, addr);

    // Checking only address prefix
    for (int k=0; k<4; k++) {
        if (addr[k] != myAddr[k]) {
            return false;
        }
    }

    return true;
}

void com_init()
{
    // Initializing SPI
    com.begin(SPI_18MHZ, MSBFIRST, 0);

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
        if (com_master) {
            addr[4] = COM_MASTER;
        }
        com_set_reg5(k, REG_RX_ADDR_P0, addr);

        // Enabling only the pipe 1
        com_set_reg(k, REG_EN_RXADDR, 1);

        // Setting payload in rx p0 to 1
        if (com_master) {
            com_set_reg(k, REG_RX_PW_P0, sizeof(struct packet_robot));
        } else {
            com_set_reg(k, REG_RX_PW_P0, sizeof(struct packet_master));
        }

        // Power up
        com_mode(k, true, true);

        // If I am not the master, I will talk only to the master
        if (!com_master) {
            com_set_tx_addr(2, COM_MASTER);
        }
    }

    // Listening
    com_ce_enable();

    // Initializing IRQ pins
    detachInterrupt(COM_IRQ1);
    if (com_is_ok(0)) {
        if (com_read_reg(0, REG_STATUS) & 0xf0) {
            com_irq(0);
        }
        pinMode(COM_IRQ1, INPUT);
        attachInterrupt(COM_IRQ1, com_irq1, FALLING);
        int status = com_read_reg(0, REG_STATUS);
    }
    detachInterrupt(COM_IRQ2);
    if (com_is_ok(1)) {
        if (com_read_reg(1, REG_STATUS) & 0xf0) {
            com_irq(1);
        }
        pinMode(COM_IRQ2, INPUT);
        attachInterrupt(COM_IRQ2, com_irq2, FALLING);
    }
    detachInterrupt(COM_IRQ3);
    if (com_is_ok(2)) {
        if (com_read_reg(2, REG_STATUS) & 0xf0) {
            com_irq(2);
        }
        pinMode(COM_IRQ3, INPUT);
        attachInterrupt(COM_IRQ3, com_irq3, FALLING);
    }

    // Disable everything in robot commands
    if (com_master) {
        for (int k=0; k<6; k++) {
            com_robots[k].actions = 0;
            com_robots[k].wheel1 = 0;
            com_robots[k].wheel2 = 0;
            com_robots[k].wheel3 = 0;
            com_robots[k].wheel4 = 0;
            com_robot_reception[k] = 0;
        }
    }
    com_master_packets = 0;
    com_master_pos = 0;
}

static void com_usb_tick()
{
    static int state = 0;
    static uint8_t temp[sizeof(com_robots)];
    static int pos = 0;

    while (SerialUSB.available()) {
        watchdog_feed();
        uint8_t c = SerialUSB.read();
        if (state == 0) {
            if (c == 0xaa) {
                state++;
            }
        } else if (state == 1) {
            if (c == 0x55) {
                state++;
                pos = 0;
            } else {
                state = 0;
            }
        } else if (pos < sizeof(com_robots)) {
            temp[pos++] = c;
        } else {
            digitalWrite(BOARD_LED_PIN, HIGH);
            if (c == 0xff) {
                // Received message from USB
                uint8_t *target = (uint8_t *)com_robots;
                for (int k = 0; k < sizeof(com_robots); k++) {
                    target[k] = temp[k];
                }
                com_usb_reception = millis();

                // Updating robot statuses
                for (int k = 0; k < 6; k++) {
                    if ((millis() - com_robot_reception[k]) >= 100) {
                        com_statuses[k].status = 0;
                    }
                }

                // Sending back robot status
                uint8_t statuses[sizeof(com_statuses)+3];
                statuses[0] = 0xaa;
                statuses[1] = 0x55;
                statuses[sizeof(com_statuses)+2] = 0xff;
                uint8_t *source = (uint8_t *)com_statuses;
                for (int k = 0; k < sizeof(com_statuses); k++) {
                    statuses[k+2] = source[k];
                }
                SerialUSB.write(statuses, sizeof(statuses));
            }
            state = 0;
        }
    }
}

void com_tick()
{
    static int actions = 0;
    static int last = micros();

// #define BINARY

    // Entering master infinite loop
    while (com_master) {
        // Feed the watchdog
        watchdog_feed();

        // Tick the communication with USB master
        #ifdef BINARY
        com_usb_tick();
        #else
        terminal_tick();
        #endif

        // Feed the watchdog
        watchdog_feed();

        // Sending a packet to a robot
        // XXX: Using micros() in unsafe because it sometime overflow, to fix!
        // XXX: If the robot answered, we can ask the next quicker?
        #ifdef BINARY
        if ((millis() - com_usb_reception) < 100 && com_master && (micros() - last) > 1650) {
        #else
        if (com_master && (micros() - last) > 1650) {
        #endif
            last = micros();

            com_ce_disable();
            for (int k=0; k<3; k++) {
                com_mode(k, true, false);
                com_set_reg(k, REG_STATUS, 0x70);
                com_set_tx_addr(k, com_master_pos);
                com_tx(k, (uint8_t*)&com_robots[com_master_pos], sizeof(struct packet_master));
            }
            com_ce_enable();

            com_master_pos++;
            if (com_master_pos >= 6) {
                com_master_pos = 0;
            }
        }

        if ((millis() - com_usb_reception) < 100) {
            digitalWrite(BOARD_LED_PIN, HIGH);
        } else {
            digitalWrite(BOARD_LED_PIN, LOW);
        }
    }

    if (!com_master && com_master_new) {
        com_master_controlling = true;
        com_master_new = false;

        struct packet_robot packet;
        // XXX: Configure id per robot
        packet.id = 0;
        // XXX: Complete the status
        packet.status = STATUS_OK;
        packet.cap_volt = kicker_cap_voltage()*10.0;
        packet.bat1_volt = voltage_bat1()*10.0;
        packet.bat2_volt = voltage_bat2()*10.0;

        com_ce_disable();
        for (int k=0; k<3; k++) {
            com_mode(k, true, false);
            com_set_reg(k, REG_STATUS, 0x70);
            com_tx(k, (uint8_t *)&packet, sizeof(struct packet_robot));
        }
        com_ce_enable();

        // Driving wheels
        if (com_master_packet.actions & ACTION_ON) {
            drivers_set_safe(0, true, com_master_packet.wheel1);
            drivers_set_safe(1, true, com_master_packet.wheel2);
            drivers_set_safe(2, true, com_master_packet.wheel3);
            drivers_set_safe(3, true, com_master_packet.wheel4);
            drivers_set_safe(4, false, 0);

            // Charging
            if (com_master_packet.actions & ACTION_CHARGE) {
                kicker_boost_enable(true);
            } else {
                kicker_boost_enable(false);
            }

            // Kicking
            if ((com_master_packet.actions & ACTION_KICK1) &&
                !(actions & ACTION_KICK1)) {
                kicker_kick(com_master_packet.kickPower);
            }
            // XXX: Handle KICK2

            actions = com_master_packet.actions;
        } else {
            drivers_set(0, false, 0);
            drivers_set(1, false, 0);
            drivers_set(2, false, 0);
            drivers_set(3, false, 0);
            drivers_set(4, false, 0);
            actions = 0;
            kicker_boost_enable(false);
        }
    }

    if ((millis() - com_master_reception) < 100) {
        digitalWrite(BOARD_LED_PIN, HIGH);
    } else {
        actions = 0;
        digitalWrite(BOARD_LED_PIN, LOW);
    }
}

TERMINAL_COMMAND(mp, "Master packets")
{
    terminal_io()->println(com_master_packets);
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

bool com_is_all_ok()
{
    for (int k=0; k<3; k++) {
        if (!com_is_ok(k)) {
            return false;
        }
    }

    return true;
}

void com_diagnostic()
{
    com_init();

    for (int k=0; k<3; k++) {
        terminal_io()->print("* Com module #");
        terminal_io()->print(k);
        if (com_is_ok(k)) {
            terminal_io()->println(" OK");
        } else {
            terminal_io()->println(" MISSING");
        }
    }
}

TERMINAL_COMMAND(master, "Enable master mode")
{
    com_master = true;
    com_init();
}

TERMINAL_COMMAND(ms, "Master status")
{
    for (int k=0; k<6; k++) {
        if (millis() - com_robot_reception[k] < 100) {
            terminal_io()->print("Robot #");
            terminal_io()->print(k);
            terminal_io()->println(" is alive.");
        }
    }
}

TERMINAL_COMMAND(m, "Master set")
{
    if (argc == 4) {
        com_robots[0].actions = ACTION_ON;
        com_robots[0].wheel1 = atof(argv[0]);
        com_robots[0].wheel2 = atof(argv[1]);
        com_robots[0].wheel3 = atof(argv[2]);
        com_robots[0].wheel4 = atof(argv[3]);
    }
}

TERMINAL_COMMAND(rc, "Remote charge test")
{
    if (atoi(argv[0])) {
        com_robots[0].actions |= ACTION_ON;
        com_robots[0].actions |= ACTION_CHARGE;
    } else {
        com_robots[0].actions &= ~ACTION_CHARGE;
    }
}

TERMINAL_COMMAND(rk, "Remote kick test")
{
    if (atoi(argv[0])) {
        com_robots[0].kickPower = atoi(argv[0]);
        com_robots[0].actions |= ACTION_KICK1;
    } else {
        com_robots[0].kickPower = 0;
        com_robots[0].actions &= ~ACTION_KICK1;
    }
}

TERMINAL_COMMAND(em, "Emergency")
{
    if (com_master) {
        for (int k=0; k<6; k++) {
            com_robots[k].actions = 0;
            com_robots[k].wheel1 = 0;
            com_robots[k].wheel2 = 0;
            com_robots[k].wheel3 = 0;
            com_robots[k].wheel4 = 0;
        }
    } else {
        for (int k=0; k<5; k++) {
            drivers_set(k, false, 0.0);
        }
    }
}
