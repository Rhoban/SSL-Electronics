#include <stdlib.h>
#include <stdint.h>
#include <terminal.h>
#include "com.h"
#include "mux.h"
#include <wirish/wirish.h>
#include "buzzer.h"
#include <watchdog.h>
#include "kicker.h"
#include "kinematic.h"
#include "hardware.h"
#include "drivers.h"
#include "voltage.h"
#include "kicker.h"
#include "infos.h"
#include "ir.h"
#include "nrf24l01.h"

// Channels
static int com_channels[3] = {110, 119, 125};
static int com_channels_developers[3] = {111, 118, 124};

extern int presentSince;

// Only for master board
static bool com_master = false;

// Master control packet to send to each robot
static uint8_t com_robots[MAX_ROBOTS][PACKET_SIZE + 1];
static bool com_should_transmit[MAX_ROBOTS] = {false};

// Status replies from robots
static struct packet_robot com_statuses[MAX_ROBOTS];
static bool com_has_status[MAX_ROBOTS] = {false};

static struct buzzer_note note_now = {0,0};

// Parmeters to send to robot
// static struct packet_params com_master_params; //NOT USED

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

#define VALUE_TO_STRING(x) #x
#define VALUE(x) VALUE_TO_STRING(x)
#define VAR_NAME_VALUE(var) #var "="  VALUE(var)

#pragma message(VAR_NAME_VALUE(FIRMWARE_VERSION))

#ifdef FIRMWARE_VERSION
static char firmware_version[]=VALUE(FIRMWARE_VERSION);
#else
static char firmware_version[]="TEST";
#endif

HardwareSPI com(COM_SPI);


#define PAYLOAD_SIZE    5

uint8_t com_data[3][PAYLOAD_SIZE];
bool com_available[3] = {false};

int com_pins[3] = {
   COM_CS1, COM_CS2, COM_CS3 // dont't touch this !
};

TERMINAL_PARAMETER_INT(irqed, "", 0);

void com_send(int index, uint8_t *packet, size_t n)
{
    //pause_boost();
    digitalWrite(com_pins[index], LOW);
    for (size_t k=0; k<n; k++) {
        uint8_t reply = com.send(packet[k]);
        packet[k] = reply;
    }
    digitalWrite(com_pins[index], HIGH);
    //resume_boost();
}

void com_set_reg(int index, uint8_t reg, uint8_t value)
{
    SerialUSB.print("set reg: ");
    SerialUSB.print(reg,HEX);
    SerialUSB.print(" ");
    SerialUSB.println(value,HEX);

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
    SerialUSB.print("set reg5:");
    SerialUSB.print(reg,HEX);
    SerialUSB.print(" ");
    SerialUSB.print(value[0],HEX);
    SerialUSB.print("-");
    SerialUSB.print(value[1],HEX);
    SerialUSB.print("-");
    SerialUSB.print(value[2],HEX);
    SerialUSB.print("-");
    SerialUSB.print(value[3],HEX);
    SerialUSB.print("-");
    SerialUSB.println(value[4],HEX);

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

    SerialUSB.print("read reg:");
    SerialUSB.print(reg,HEX);
    SerialUSB.print(" ");
    SerialUSB.println(packet[1],HEX);
    return packet[1];
}

int get_channel(int index){
    return com_read_reg(index,REG_RF_CH);
}

void set_channel(int index, int chan){
    com_set_reg(index,REG_RF_CH, chan);
}

int get_ack(int index){
    return com_read_reg(index,REG_EN_AA);
}
void set_ack(int index,bool v){
    if (v)
        com_set_reg(index,REG_EN_AA,REG_EN_AA_ALL);
    else
        com_set_reg(index,REG_EN_AA,0x00);
}


int get_retransmission_delay(int card){
    return (com_read_reg(card,REG_SETUP_RETR)>>4);
}
int get_retransmission_count(int card){
    return (com_read_reg(card,REG_SETUP_RETR)&0x0F);
}
void set_retransmission(int card, int delay, int count){
    uint8_t v=(delay<<0x04) | (count);
    com_set_reg(card,REG_SETUP_RETR,v);
}


uint8_t get_config(int index){
    return com_read_reg(index,REG_CONFIG);
}

uint8_t get_fifo_status(int index){
    return com_read_reg(index,REG_FIFO_STATUS);
}

uint8_t get_status(int card){
    return com_read_reg(card,REG_STATUS);
}
uint8_t get_rf_setup(int card){
    return com_read_reg(card,REG_RF_SETUP);
}

void reset_status(int card){
    com_set_reg(card,REG_STATUS,REG_STATUS_RX_DR | REG_STATUS_TX_DS | REG_STATUS_MAX_RT);
}
int get_pipe_payload(int card,int pipe){
    switch(pipe){
    case 0:
        return com_read_reg(card,REG_RX_PW_P0);
        break;
    case 1:
        return com_read_reg(card,REG_RX_PW_P1);
        break;
    case 2:
        return com_read_reg(card,REG_RX_PW_P2);
        break;
    case 3:
        return com_read_reg(card,REG_RX_PW_P3);
        break;
    case 4:
        return com_read_reg(card,REG_RX_PW_P4);
        break;
    case 5:
        return com_read_reg(card,REG_RX_PW_P5);
        break;
    }
    return 0;
}
void set_pipe_payload(int card,int pipe,uint8_t pl){
    if (pl>32) return;
    uint8_t en= com_read_reg(card,REG_EN_RXADDR);
    switch(pipe){
    case 0:
        if (pl==0) en&=~REG_EN_RXADDR_P0;
        else en|=REG_EN_RXADDR_P0;
        com_set_reg(card,REG_RX_PW_P0,pl);
        break;
    case 1:
        if (pl==0) en&=~REG_EN_RXADDR_P1;
        else en|=REG_EN_RXADDR_P1;
        com_set_reg(card,REG_RX_PW_P1,pl);
        break;
    case 2:
        if (pl==0) en&=~REG_EN_RXADDR_P2;
        else en|=REG_EN_RXADDR_P2;
        com_set_reg(card,REG_RX_PW_P2,pl);
        break;
    case 3:
        if (pl==0) en&=~REG_EN_RXADDR_P3;
        else en|=REG_EN_RXADDR_P3;
        com_set_reg(card,REG_RX_PW_P3,pl);
        break;
    case 4:
        if (pl==0) en&=~REG_EN_RXADDR_P4;
        else en|=REG_EN_RXADDR_P4;
        com_set_reg(card,REG_RX_PW_P4,pl);
        break;
    case 5:
        if (pl==0) en&=~REG_EN_RXADDR_P5;
        else en|=REG_EN_RXADDR_P5;
        com_set_reg(card,REG_RX_PW_P5,pl);
        break;
    }
    com_set_reg(card,REG_EN_RXADDR,en);
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
    for (uint8_t k=0; k<n; k++) {
      packet[k+1] = payload[k];
    }

    com_send(index, packet, n+1);
}

static void com_rx(int index, uint8_t *payload, size_t n)
{
    uint8_t packet[n+1] = {0};

    packet[0] = OP_RX;

    com_send(index, packet, n+1);

    for (uint8_t k=1; k<n+1; k++) {
      payload[k-1] = packet[k];
    }
}

void com_flush_rx(int index){
    uint8_t packet[1] = {OP_FLUSH_RX};
    com_send(index, packet, 1);

    SerialUSB.print("flush rx\n");
//    return packet[0];
}

void com_flush_tx(int index){
    uint8_t packet[1] = {OP_FLUSH_TX};
    com_send(index, packet, 1);
    SerialUSB.print("flush tx\n");
//    return packet[0];
}

void clear_status(int card){
    com_set_reg(card,REG_STATUS,REG_STATUS_MAX_RT|REG_STATUS_RX_DR|REG_STATUS_TX_DS);
}
int get_lost_count(int card){
    return com_read_reg(card,REG_OBSERVE_TX) >> 4;
}
int get_retransmitted_count(int card){
    return com_read_reg(card,REG_OBSERVE_TX) & 0x0F;
}


int has_data(int card)
{
    uint8_t s=com_read_reg(card,REG_STATUS);
    //if (s&REG_STATUS_RX_DR)
    uint8_t v=(s&REG_STATUS_RX_P_NO)>>1;
    if (v==7)return -1;
    return v;
//    return -1;
}

void receive(int card, uint8_t *payload, int size){

    com_ce_disable(card);
    com_mode(card, true, false);
    uint8_t conf=com_read_reg(card,REG_CONFIG);
    com_set_reg(card, REG_CONFIG,conf | CONFIG_PWR_UP | CONFIG_PRIM_RX ); // dont touch other config flags...

    uint8_t packet[size+1] = {0};

    packet[0] = OP_RX;

    com_send(card, packet, size+1);

    for (uint8_t k=1; k<size+1; k++) {
      payload[k-1] = packet[k];
    }
    //com_ce_enable(card);
}

void send(int card,  uint8_t *payload, int size){
//    com_ce_disable(card);

    // set config PRIM_RX to low:

    //com_set_reg(k, REG_STATUS, 0x70);
    com_tx(card,payload,size);
    com_ce_enable(card);
    delay_us(20);
    com_ce_disable(card);
    clear_status(card);

    //uint8_t conf=com_read_reg(card,REG_CONFIG);
    //com_set_reg(card, REG_CONFIG,(conf | CONFIG_PWR_UP) & ~CONFIG_PRIM_RX ); // dont touch other config flags...

      //  com_flush_tx(card);

//    com_ce_enable(card);
//    uint8_t s;
//    do {
//        s=com_read_reg(card,REG_STATUS);
//        terminal_io()->print("status is: ");
//        terminal_io()->println(s);
//    } while (((s&REG_STATUS_TX_DS)==0) || ((s&REG_STATUS_MAX_RT)==0));
//    com_ce_disable(card);
}

//NOT USED??
/*
  static bool com_rxes_empty()
  {
  for (uint8_t index=0; index<3; index++) {
  int fifo = com_read_reg(index, REG_FIFO_STATUS);

  if ((fifo & RX_EMPTY) == 0) {
  return false;
  }
  }

  return true;
  }
*/


/*
  * Actually com_irq is used in com_poll() and not call in an irq !
  *
  * BE CAREFULL ! If you activate the IRQ MODE, you have to manage concurrency
  * with resume_boost() and pause_boost(). See kicker.cpp for more details.
  *
  */
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

      // int safe_com_module = 1; //IT SHOULD BE CS2
      // if(!com_master and is_charging() and k!=safe_com_module ) continue;

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

struct addr5 com_get_tx_addr(int index){
    uint8_t addr[5];
    com_read_reg5(index,REG_TX_ADDR,addr);
    struct addr5 a;
    for(int i=0;i<5;++i)
        a.addr[i] = addr[i];
    return a;
}

void com_set_tx_addr(int index, uint8_t target)
{
    uint8_t addr[5] = COM_ADDR;
    addr[4] = target;

    com_set_reg5(index, REG_TX_ADDR, addr);
}

void com_set_tx_addr(int index, struct addr5 add){
    com_set_reg5(index, REG_TX_ADDR, add.addr);
}


void com_set_rx_addr(int index,int pipe, struct addr5 add){
    switch(pipe){
    case 0:
        com_set_reg5(index,REG_RX_ADDR_P0,add.addr);
        break;
    case 1:
        com_set_reg5(index,REG_RX_ADDR_P1,add.addr);
        break;
    case 2:
        com_set_reg(index,REG_RX_ADDR_P2,add.addr[4]);
        break;
    case 3:
        com_set_reg(index,REG_RX_ADDR_P3,add.addr[4]);
        break;
    case 4:
        com_set_reg(index,REG_RX_ADDR_P4,add.addr[4]);
        break;
    case 5:
        com_set_reg(index,REG_RX_ADDR_P5,add.addr[4]);
        break;
    }
}

struct addr5 com_get_rx_addr(int index,int pipe){

    uint8_t addr[5];
    if (pipe==0)
        com_read_reg5(index,REG_RX_ADDR_P0,addr);
else if (pipe>=1){
        com_read_reg5(index,REG_RX_ADDR_P1,addr);
        uint8_t last=addr[4];
        if (pipe==2)
            last=com_read_reg(index,REG_RX_ADDR_P2);
        if (pipe==3)
            last=com_read_reg(index,REG_RX_ADDR_P3);
        if (pipe==4)
            last=com_read_reg(index,REG_RX_ADDR_P4);
        if (pipe==5)
            last=com_read_reg(index,REG_RX_ADDR_P5);
        addr[4]=last;
    }
    struct addr5 a;
    for(int i=0;i<5;++i)
        a.addr[i] = addr[i];
    return a;
}
void com_ce_enable(int index)
{
    SerialUSB.println("enable ce");
//    terminal_io()->println("set ce");
    if (index == 0) digitalWrite(COM_CE1, HIGH);
    else if (index == 1) digitalWrite(COM_CE2, HIGH);
    else if (index == 2) digitalWrite(COM_CE3, HIGH);
}

void com_ce_disable(int index)
{
SerialUSB.println("disable ce");
//    terminal_io()->println("unset ce");
    if (index == 0) digitalWrite(COM_CE1, LOW);
    else if (index == 1) digitalWrite(COM_CE2, LOW);
    else if (index == 2) digitalWrite(COM_CE3, LOW);

    // We are not suppose to disable CE, this is only for power reduction
    // keep com cards in standby II mode
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
    com.begin(SPI_2_25MHZ, MSBFIRST, 0);
    for (uint8_t k=0; k<3; k++) { // Set CSN to 1 as it is default state
      pinMode(com_pins[k], OUTPUT);
      digitalWrite(com_pins[k], HIGH);
    }
    pinMode(COM_CE1, OUTPUT);
    digitalWrite(COM_CE1, LOW);
    pinMode(COM_CE2, OUTPUT);
    digitalWrite(COM_CE2, LOW);
    pinMode(COM_CE3, OUTPUT);
    digitalWrite(COM_CE3, LOW);

    delay_us(1000); // wait few

    // set default state for all cards:
    // this setup is a copy from arduino rf24l01+ library
    for (uint8_t k=0; k<3; k++) {
        com_ce_disable(k);
        set_config(k,CONFIG_EN_CRC|CONFIG_CRCO);
        set_retransmission(k,5,15);
        com_set_reg(k,REG_RF_SETUP,0x21);
        com_set_reg(k,REG_RF_SETUP,0x01);
        com_set_reg(k,REG_FEATURE,0x00);
        com_set_reg(k,REG_DYNPD,0x00);
        clear_status(k);
        set_channel(k,76);
        com_flush_rx(k);
        com_flush_tx(k);
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


void power(int card,bool up){
    if (up)
        set_config(card,get_config(card) | CONFIG_PWR_UP);
    else
        set_config(card,get_config(card) & ~CONFIG_PWR_UP);
}

void set_rf(int card, uint8_t speed, uint8_t pow){
    com_set_reg(card,REG_RF_SETUP,speed|pow);
}


void set_config(int card,uint8_t v){
    com_set_reg(card,REG_CONFIG,v);
}
void set_crc(int card,int crc){
    if (crc==0)
        set_config(card,get_config(card) &~CONFIG_EN_CRC );
    else if (crc==1)
        set_config(card,get_config(card)|CONFIG_EN_CRC & ~CONFIG_CRCO);
    else
        set_config(card,get_config(card)|CONFIG_EN_CRC | CONFIG_CRCO);
}

TERMINAL_COMMAND(ci, "CI")
{
    com_init();
}

TERMINAL_COMMAND(comi, "Com stats")
{
    terminal_io()->println(millis()-com_last_init);
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
    packet.voltage  = voltage_value()*8.0;
    packet.xpos     = getOdometry().xpos*1000;
    packet.ypos     = getOdometry().ypos*1000;
    packet.ang      = getOdometry().ang*1000;

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
        if ((master_packet->actions & ACTION_ON)) {
            if(developer_mode == false){
                kicker_boost_enable(true);
            }
            if(master_packet->actions & ACTION_TARE_ODOM) {
                odometry_tare((master_packet->x_speed)/1000.0, (master_packet->y_speed)/1000.0, (master_packet->t_speed)/10000.0);
                //odometry_tare(0.0, 0.0, 0.0);

            }else{
            kinematic_set(master_packet->x_speed/1000.0, master_packet->y_speed/1000.0,
                 master_packet->t_speed/1000.0);
            }
            actions = master_packet->actions;

            // TODO !
            // If we want to drible only when IR is activated, we need
            // to uncomment
            // the following line
            //if ((master_packet->actions & ACTION_DRIBBLE) && (ir_present()) ) {
            if ((master_packet->actions & ACTION_DRIBBLE)) {
              drivers_set_safe(4, true, 1000);
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
            if ((my_actions & ACTION_KICK1) || (my_actions & ACTION_KICK2) || ir_present_now()) {
                if(kicker_cap_voltage() > 80.0){
                    bool inverted = infos_kicker_inverted();
                    if ((master_packet->actions & ACTION_KICK1) &&
                        !(my_actions & ACTION_KICK1)) {
                        kicker_kick(inverted ? 0 : 1, master_packet->kickPower*30);
                    }

                    if ((master_packet->actions & ACTION_KICK2) &&
                        !(my_actions & ACTION_KICK2)) {
                        kicker_kick(inverted ? 1 : 0, master_packet->kickPower*30);
                    }

                    my_actions = master_packet->actions;
                } else {

                    my_actions = master_packet->actions;
                    my_actions &= ~(ACTION_KICK1 | ACTION_KICK2);
                }
            }
        } else {
            buzzer_play(MELODY_ALERT_FAST, false);

            drivers_set(0, false, 0);
            drivers_set(1, false, 0);
            drivers_set(2, false, 0);
            drivers_set(3, false, 0);
            drivers_set(4, false, 0);
            my_actions = 0;
            kicker_boost_enable(false);
        }
    }
    else if (com_master_frame[0] == MUSIC_PARAMS){
         struct packet_music *music_params;
         music_params = (struct packet_music*)(com_master_frame + 1);

        if(music_params->instrument & SOUND_ON){
            if(music_params->instrument & BEEPER){
                buzzer_beep(music_params->note, music_params->duration);
                buzzer_wait_play();
            }
            if(music_params->instrument & KICK){
                kicker_kick(1,1000);
            }
            if(music_params->instrument & CHIP_KICK){
                kicker_kick(0,1000);
            }
            if(music_params->instrument & DRIBBLER){
                drivers_set(4, true, 1000);
            }
            else
            {
                drivers_set(4, false, 0);
            }
            
        }

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
        //com_usb_tick();
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
                com_set_reg(k, REG_STATUS, 0x70); // 0111 0000
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

//TERMINAL_COMMAND(em, "Emergency")
//{
//    if (!com_master) {
//        kinematic_stop();
//        for (int k=0; k<5; k++) {
//            drivers_set(k, false, 0.0);
//        }
//    }

//    kicker_boost_enable(false);
//}

TERMINAL_COMMAND(version, "")
{
  terminal_io()->println(firmware_version);
}
