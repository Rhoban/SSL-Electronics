#include <stdlib.h>
#include <stdint.h>
#include <terminal.h>
#include "buzzer.h"
#include "com.h"
#include "mux.h"
#include <wirish/wirish.h>
#include <watchdog.h>
#include "kicker.h"
#include "kinematic.h"
#include "hardware.h"
#include "drivers.h"
#include "voltage.h"
#include "kicker.h"
#include "infos.h"
#include "ir.h"

// Channels
static int com_channels[3] = {0, 10, 60};

// Only for master board
static bool com_master = false;

// Master control packet to send to each robot
static uint8_t com_robots[MAX_ROBOTS][PACKET_SIZE + 1];
static bool com_should_transmit[MAX_ROBOTS] = {false};

// Status replies from robots
static struct packet_robot com_statuses[MAX_ROBOTS];
static bool com_has_status[MAX_ROBOTS] = {false};

// Parmeters to send to robot
static struct packet_params com_master_params;
volatile static bool com_has_params[MAX_ROBOTS] = {false};

// Timestamp of reception for each packets
static int com_robot_reception[MAX_ROBOTS];

// Current robot we are communicating with
static int com_master_pos = 0;

// Reception of order via USB
static int com_usb_reception = 0;
static size_t com_usb_nb_robots = 0;

// Only for robot
static int com_master_reception = 0;
static bool com_has_master = false;
static uint8_t com_master_frame[PACKET_SIZE];
static volatile int com_master_packets = 0;
static int com_master_channel_packets[3] = {0};
static int com_last_init = 0;
static bool com_master_new = false;
static bool com_master_controlling = false;
static int my_actions = 0;

static int com_txing[3] = {0};

static bool com_module_present[3] = {true};
static int com_module_last_missing[3] = {0};

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
#define REG_STATUS_ZERO     0x80
#define REG_STATUS_RX_DR    0x40
#define REG_STATUS_TX_DS    0x20
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

#define TX_FULL         (1<<5)
#define TX_EMPTY        (1<<4)
#define RX_FULL         (1<<1)
#define RX_EMPTY        (1<<0)

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

uint8_t com_read_status(int index)
{
    uint8_t packet[1] = {OP_NOP};
    com_send(index, packet, 1);

    return packet[0];
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
    com_txing[index] = micros();

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

static bool com_rxes_empty()
{
    for (int index=0; index<3; index++) {
        int fifo = com_read_reg(index, REG_FIFO_STATUS);

        if ((fifo & RX_EMPTY) == 0) {
            return false;
        }
    }

    return true;
}

bool com_irq(int index)
{
    if ((micros()-com_txing[index]) < 300) {
        return true;
    }

    int fifo = com_read_reg(index, REG_FIFO_STATUS);

    // Checking that the module is present (bit 7 should be always 0)
    if (fifo != 0xff) {
        if ((fifo & RX_EMPTY) == 0) { // RX
            com_available[index] = true;

            if (com_master) {
                // Receiving a status packet from a robot
                struct packet_robot packet;
                com_rx(index, (uint8_t*)&packet, sizeof(struct packet_robot));

                if (packet.id < MAX_ROBOTS) {
                    com_has_status[packet.id] = true;
                    com_statuses[packet.id] = packet;
                    com_robot_reception[packet.id] = millis();
                }
            } else {
                // Receiving an instruction packet from the master
                com_master_reception = millis();
                com_master_new = true;
                com_master_packets++;
                com_master_channel_packets[index]++;
                com_rx(index, com_master_frame, PACKET_SIZE);
            }
        }
        if (com_txing[index] && ((fifo & TX_EMPTY) != 0)) { // TX is over
            com_txing[index] = 0;
            com_mode(index, true, true);
        }

        return true;
    }

    return false;
}

/*
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
*/


static void com_poll()
{
    bool reinit = false;
    for (int k=0; k<3; k++) {
        bool present = com_irq(k);

        if (!present) {
            com_module_last_missing[k] = millis();
            com_module_present[k] = false;
        } else {
            if (!com_module_present[k] && (millis() - com_module_last_missing[k] > 150)) {
                reinit = true;
                com_module_present[k] = true;
            }
        }
    }

    if (reinit) {
        com_init();
    }
}

static void com_set_tx_addr(int index, uint8_t target)
{
    uint8_t addr[5] = COM_ADDR;
    addr[4] = target;

    com_set_reg5(index, REG_TX_ADDR, addr);
}

static void com_ce_enable(int index)
{
    if (index == 0) digitalWrite(COM_CE1, HIGH);
    else if (index == 1) digitalWrite(COM_CE2, HIGH);
    else if (index == 2) digitalWrite(COM_CE3, HIGH);
}

static void com_ce_disable(int index)
{
    if (index == 0) digitalWrite(COM_CE1, LOW);
    else if (index == 1) digitalWrite(COM_CE2, LOW);
    else if (index == 2) digitalWrite(COM_CE3, LOW);
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
    // Not initializing if we don't have an id and are not master
    if (infos_get_id() == 0xff && !com_master) {
        return;
    }

    // Initializing SPI
    // com.begin(SPI_4_5MHZ, MSBFIRST, 0);
    com.begin(SPI_2_25MHZ, MSBFIRST, 0);
    // com.begin(SPI_1_125MHZ, MSBFIRST, 0);

    // Initializing CS pins
    for (int k=0; k<5; k++) {
        pinMode(com_pins[k], OUTPUT);
        digitalWrite(com_pins[k], HIGH);
    }

    // Initializing COM_CE
    pinMode(COM_CE1, OUTPUT);
    digitalWrite(COM_CE1, LOW);
    pinMode(COM_CE2, OUTPUT);
    digitalWrite(COM_CE2, LOW);
    pinMode(COM_CE3, OUTPUT);
    digitalWrite(COM_CE3, LOW);

    for (int k=0; k<3; k++) {
        // Disabling auto acknowledgement
        com_set_reg(k, REG_EN_AA, 0x00);

        // Setting the appropriate channel for this module
        com_set_reg(k, REG_RF_CH, com_channels[k]);

        // Setting the address
        uint8_t addr[5] = COM_ADDR;
        if (com_master) {
            addr[4] = COM_MASTER;
        } else {
            addr[4] = infos_get_id();
        }
        com_set_reg5(k, REG_RX_ADDR_P0, addr);

        // Enabling only the pipe 1
        com_set_reg(k, REG_EN_RXADDR, 1);

        // Setting payload in rx p0 to 1
        if (com_master) {
            com_set_reg(k, REG_RX_PW_P0, sizeof(struct packet_robot));
        } else {
            com_set_reg(k, REG_RX_PW_P0, PACKET_SIZE);
        }

        // Power up
        com_mode(k, true, true);

        // If I am not the master, I will talk only to the master
        if (!com_master) {
            for (int k=0; k<3; k++) {
                com_set_tx_addr(k, COM_MASTER);
            }
        }
    }

    // Listening
    for (int k=0; k<3; k++) {
        com_txing[k] = 0;
        com_set_reg(k, REG_STATUS, 0x70);
        com_ce_enable(k);
    }

    // Initializing IRQ pins
    // XXX: We don't listen to IRQs anymore because there was a routing issue
    // in rev1, IRQ2 & IRQ3 was routed to USB D+/D- pins which make it not
    // suitable to use, se we now poll the communication module.
    // This may be fixed in the future.
    /*
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
    */

    // Disable everything in robot commands
    com_master_packets = 0;
    com_master_pos = 0;
    com_last_init = millis();

    mux_init();
}

TERMINAL_COMMAND(ci, "CI")
{
    com_init();
}

TERMINAL_COMMAND(comi, "Com stats")
{
    terminal_io()->println(millis()-com_last_init);
}

static void com_usb_tick()
{
    static int state = 0;
    static uint8_t temp[1024];
    static size_t pos = 0;

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
            } else {
                state = 0;
            }
        } else if (state == 2) {
            com_usb_nb_robots = c;
            pos = 0;
            if (com_usb_nb_robots <= MAX_ROBOTS) {
                state++;
            } else {
                state = 0;
            }
        } else if (pos < com_usb_nb_robots*(1 + PACKET_SIZE) && pos < sizeof(temp)) {
            temp[pos++] = c;
        } else {
            if (c == 0xff) {
                // Fetch all the data that should be transmitted to the robots
                for (size_t k=0; k<com_usb_nb_robots; k++) {
                    uint8_t robot_id = temp[k*(PACKET_SIZE + 1)];
                    if (robot_id < MAX_ROBOTS) {
                        for (size_t n=0; n<PACKET_SIZE; n++) {
                            com_robots[robot_id][n] = temp[k*(1 + PACKET_SIZE)+1+n];
                        }
                        com_should_transmit[robot_id] = true;
                    }
                }

                com_poll();
                for (size_t k=0; k<MAX_ROBOTS; k++) {
                    com_has_status[k] = false;
                }
                com_master_pos = -1;
                com_usb_reception = millis();
            }

            state = 0;
        }
    }
}

void com_send_status_to_master()
{
    struct packet_robot packet;
    packet.id = infos_get_id();
    packet.status = STATUS_OK;

    if (!drivers_is_all_ok()) {
        packet.status |= STATUS_DRIVER_ERR;
    }

    if (ir_present()) {
        packet.status |= STATUS_IR;
    }

    packet.cap_volt = kicker_cap_voltage();
    packet.voltage = voltage_value()*8.0;

    for (size_t k=0; k<3; k++) {
        com_ce_disable(k);
        com_mode(k, true, false);
        com_set_reg(k, REG_STATUS, 0x70);
        com_tx(k, (uint8_t *)&packet, sizeof(struct packet_robot));
        com_ce_enable(k);
    }
}

TERMINAL_PARAMETER_INT(actions, "actions", 0);

void com_process_master()
{
    if (com_master_frame[0] == INSTRUCTION_MASTER) {
        // Answering with status packet
        com_send_status_to_master();

        // Decoding instruction packet
        struct packet_master *master_packet;
        master_packet = (struct packet_master*)(com_master_frame + 1);

        // Driving wheels
        if (master_packet->actions & ACTION_ON) {
            kinematic_set(master_packet->x_speed/1000.0, master_packet->y_speed/1000.0,
                 master_packet->t_speed/1000.0);
            actions = master_packet->actions;

            //if ((master_packet->actions & ACTION_DRIBBLE) && (ir_present()) ) {
            if ((master_packet->actions & ACTION_DRIBBLE)) {
                drivers_set_safe(4, true, 0.5);
            } else {
                drivers_set(4, false, 0);
            }

            // Charging
            if (master_packet->actions & ACTION_CHARGE) {
                kicker_boost_enable(true);
            } else {
                kicker_boost_enable(false);
            }


            // Kicking
            if (ir_present()) {
                bool inverted = infos_kicker_inverted();
                if ((master_packet->actions & ACTION_KICK1) &&
                    !(my_actions & ACTION_KICK1)) {
                    kicker_kick(inverted ? 0 : 1, master_packet->kickPower*25);
                }

                if ((master_packet->actions & ACTION_KICK2) &&
                    !(my_actions & ACTION_KICK2)) {
                    kicker_kick(inverted ? 1 : 0, master_packet->kickPower*25);
                }

                my_actions = master_packet->actions;
            } else {
                my_actions = master_packet->actions;
                my_actions &= ~(ACTION_KICK1 | ACTION_KICK2);
            }
        } else {
            drivers_set(0, false, 0);
            drivers_set(1, false, 0);
            drivers_set(2, false, 0);
            drivers_set(3, false, 0);
            drivers_set(4, false, 0);
            my_actions = 0;
            kicker_boost_enable(false);
        }
    } else if (com_master_frame[0] == INSTRUCTION_PARAMS) {
        struct packet_params *params;
        params = (struct packet_params*)(com_master_frame+1);

        // Setting PID parmeters
        drivers_set_params(params->kp, params->ki, params->kd);
    }
}

void com_tick()
{
    static int last = micros();

// Comment to debug on shell
#define BINARY

       // Entering master infinite loop
      while (com_master) {
        // Feed the watchdog
        watchdog_feed();

        // Polling com IRQs
        com_poll();

        // Tick the communication with USB master
#ifdef BINARY
        com_usb_tick();
#else
        terminal_tick();
#endif

        // Feed the watchdog
        watchdog_feed();

        bool transmitting = false;
        while (com_master_pos < MAX_ROBOTS && !transmitting) {
          // Sending a packet to a robot
          // XXX: Using micros() in unsafe because it sometime overflow, to fix!
          // We either received a status from the previous robot or the timeout expired,
          // we should ask the next one
          if (com_master_pos < 0 || com_has_status[com_master_pos] || (micros() - last) > 2000) {
            // Asking the next
            com_master_pos++;
            last = 0;

            // Should we send a packet to this robot ?
            if (com_master_pos < MAX_ROBOTS && com_should_transmit[com_master_pos]) {
              last = micros();
              transmitting = true;

              // Sending the packet to the 3 modules
              for (size_t k=0; k<3; k++) {
                com_ce_disable(k);
                // Preparing to transmit
                com_mode(k, true, false);
                com_set_reg(k, REG_STATUS, 0x70);
                com_set_tx_addr(k, com_master_pos);

                // Transmitting the payload
                com_tx(k, com_robots[com_master_pos], PACKET_SIZE);
                com_ce_enable(k);
              }
              com_should_transmit[com_master_pos] = false;
            }

            if (com_master_pos >= MAX_ROBOTS) {
              // Our cycle is over, sending back the robot statuses
              size_t statuses = 0;
              for (size_t k=0; k<MAX_ROBOTS; k++) {
                if (com_has_status[k] ) {
                  statuses++;
                }
              }
#ifdef BINARY
              uint8_t answer[1+1+1+statuses*(1+sizeof(struct packet_robot))+1];
              // Answer header
              answer[0] = 0xaa;
              answer[1] = 0x55;
              // Number of robots in the packet
              answer[2] = statuses;
              size_t pos = 3;
              for (size_t k=0; k<MAX_ROBOTS; k++) {
                if (com_has_status[k]) {
                  // Inserting the robot #
                  answer[pos++] = k;
                  // Copying structure data
                  uint8_t *ptr = (uint8_t *)&com_statuses[k];
                  for (size_t n=0; n<sizeof(struct packet_robot); n++) {
                    answer[pos++] = ptr[n];
                  }
                }
              }
              // Ending with 0xff
              answer[pos] = 0xff;
              SerialUSB.write(answer, sizeof(answer));
#endif
            }
          } else {
            transmitting = true;
          }
        }

        // XXX: Led for master
        if ((millis() - com_usb_reception) < 100) {
          digitalWrite(BOARD_LED_PIN, HIGH);
        } else {
          digitalWrite(BOARD_LED_PIN, LOW);
        }
      }

      // Polling IRQs
      com_poll();

      // Processing a packet from the master
      if (!com_master && com_master_new) {
        last = micros();
        com_master_controlling = true;
        com_master_new = false;
        com_process_master();
      }

      // Checking the last master reception, playing melody when the reception is changing
      if ((millis() - com_master_reception) < 100 && com_master_reception != 0) {
        if (!com_has_master) {
          com_has_master = true;
          // buzzer_play(MELODY_BEGIN);
        }
        digitalWrite(BOARD_LED_PIN, HIGH);
      } else {
        if (com_has_master) {
          com_has_master = false;
          kicker_boost_enable(false); // Stopping the charge
          // buzzer_play(MELODY_END);
        }
        my_actions = 0;
        digitalWrite(BOARD_LED_PIN, LOW);
      }
}

TERMINAL_COMMAND(mp, "Master packets")
{
    terminal_io()->println(com_master_packets);
    terminal_io()->println(com_master_channel_packets[0]);
    terminal_io()->println(com_master_channel_packets[1]);
    terminal_io()->println(com_master_channel_packets[2]);
}

TERMINAL_COMMAND(ms, "Master send")
{
    for (int k=0; k<PACKET_SIZE; k++) {
        com_robots[3][k] = 0;
    }
    com_has_status[3] = false;
    com_should_transmit[3] = true;
    com_master_pos = -1;
}

TERMINAL_COMMAND(mr, "Master receive")
{
    if (com_has_status[3]) {
        terminal_io()->println("Got answer");
    } else {
        terminal_io()->println("Got nothing");
    }
}

TERMINAL_COMMAND(st, "St")
{
    int index = atoi(argv[0]);

    terminal_io()->println("STATUS");
    terminal_io()->println(com_read_status(index));
    terminal_io()->println("CONFIG");
    terminal_io()->println(com_read_reg(index, REG_CONFIG));
    terminal_io()->println("EN_AA");
    terminal_io()->println(com_read_reg(index, REG_EN_AA));
    terminal_io()->println("EN_RXADDR");
    terminal_io()->println(com_read_reg(index, REG_EN_RXADDR));
    terminal_io()->println("RF_CH");
    terminal_io()->println(com_read_reg(index, REG_RF_CH));
    terminal_io()->println("FIFO_STATUS");
    terminal_io()->println(com_read_reg(index, REG_FIFO_STATUS));
    terminal_io()->println("RX_PW_P0");
    terminal_io()->println(com_read_reg(index, REG_RX_PW_P0));

    terminal_io()->println("RX_ADDR_P0");
    uint8_t value[5];
    com_read_reg5(index, REG_RX_ADDR_P0, value);
    for (size_t k=0; k<5; k++)
    terminal_io()->println((int)value[k]);

    terminal_io()->println("TX_ADDR");
    com_read_reg5(index, REG_TX_ADDR, value);
    for (size_t k=0; k<5; k++)
    terminal_io()->println((int)value[k]);
}

bool com_is_all_ok()
{
    for (size_t k=0; k<3; k++) {
        if (!com_is_ok(k)) {
            return false;
        }
    }

    return true;
}

void com_diagnostic()
{
    com_init();

    for (size_t k=0; k<3; k++) {
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

TERMINAL_COMMAND(em, "Emergency")
{
    if (!com_master) {
        kinematic_stop();
        for (int k=0; k<5; k++) {
            drivers_set(k, false, 0.0);
        }
    }

    kicker_boost_enable(false);
}
