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

//#define DEBUG_COM


//extern int presentSince;

// Only for master board
//static bool com_master = false;

// Master control packet to send to each robot
//static uint8_t com_robots[MAX_ROBOTS][PACKET_SIZE + 1];
//static bool com_should_transmit[MAX_ROBOTS] = {false};

// Status replies from robots
//static struct packet_robot com_statuses[MAX_ROBOTS];
//static bool com_has_status[MAX_ROBOTS] = {false};

//static struct buzzer_note note_now = {0,0};

// Parmeters to send to robot
// static struct packet_params com_master_params; //NOT USED

//volatile static bool com_has_params[MAX_ROBOTS] = {false};

// Timestamp of reception for each packets
//static int com_robot_reception[MAX_ROBOTS];

// Current robot we are communicating with
//static int com_master_pos = 0;

// Reception of order via USB
//static int com_usb_reception = 0;
//static size_t com_usb_nb_robots = 0;

// Only for robot
//static int com_master_reception = 0;
//static bool com_has_master = false;
//static uint8_t com_master_frame[PACKET_SIZE];
//static volatile int com_master_packets = 0;
//static int com_master_channel_packets[3] = {0};
//static int com_last_init = 0;
//static bool com_master_new = false;
//static bool com_master_controlling = false;
//static int my_actions = 0;

//static int com_txing[3] = {0};

//static bool com_module_present[3] = {true};
//static int com_module_last_missing[3] = {0};

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

//uint8_t com_data[3][PAYLOAD_SIZE];
//bool com_available[3] = {false};

int com_csn_pins[3] = {
   COM_CS1, COM_CS2, COM_CS3 // dont't touch this !
};
int com_ce_pins[3] = {
   COM_CE1, COM_CE2, COM_CE3 // dont't touch this !
};

ComState com_current_state[3] = {
    OFF,OFF,OFF
};

ComState com_get_state(int card){
    return com_current_state[card];
}
bool com_check_state(int card){
    uint8_t c=com_read_reg(card,REG_CONFIG);
    if (com_current_state[card]==OFF)
        return (c & CONFIG_PWR_UP) == 0;
    if (com_current_state[card]==RX){
        return ((c&CONFIG_PRIM_RX)!=0) && ((c&CONFIG_PWR_UP)!=0) && (digitalRead(com_ce_pins[card])==LOW);
    }
    // TX:
    return (digitalRead(com_ce_pins[card])==HIGH) && ((c&CONFIG_PRIM_RX)==0) && ((c&CONFIG_PWR_UP)!=0);

}

void com_set_state(int card, ComState state){
    if (state==com_current_state[card]) return;
    if (state==OFF){ // turn off the card
        com_ce_disable(card);
        com_power(card,false);
        com_current_state[card]=OFF;
        return;
    }
    SerialUSB.print("state for card ");
    SerialUSB.print(card);
    SerialUSB.print(" change from ");
    SerialUSB.print(com_current_state[card]);

    SerialUSB.print(" to ");
    SerialUSB.println(state);
    com_ce_disable(card);
    if (com_current_state[card]==OFF){
        com_power(card,true);
        delay_us(1500); // wait 1.5ms
    }
    // we are in standby I mode
    if (state==TX){
        com_stop_listening(card);
        com_ce_enable(card);
        delay_us(10+130); // we should be in standby II mode now
        com_current_state[card] = TX;
    } else { // RX
        com_start_listening(card);
        com_ce_enable(card);
        delay_us(130); // in rx until ce move down
        com_current_state[card] = RX;
    }
}

TERMINAL_PARAMETER_INT(irqed, "", 0);



void print_byte_as_hex(uint8_t v){
    uint8 a=(v>>4);
    if (a<10)
        SerialUSB.print(a);
    else
        SerialUSB.print((char )('A'+a-10));
    a=v&0x0F;
    if (a<10)
        SerialUSB.print(a);
    else
        SerialUSB.print((char )('A'+a-10));
}

void print_addr(uint8_t a[5]){
    for(int i=0;i<5;++i){
        print_byte_as_hex(a[i]);
        if (i<4)
            SerialUSB.print("-");
    }
    terminal_io()->print("  ");
    for(int i=0;i<5;++i){
        terminal_io()->print(a[i]);
        if (i<4)
            SerialUSB.print(".");
    }
}


const char *reg_to_str(uint8_t reg){
  static char buffer[5];
  switch(reg){
  case 0x00:
    return "CONFIG";
    break;
  case 0x01:
    return "EN_AA";
    break;
  case 0x02:
    return "EN_RXADDR";
    break;
  case 0X03:
    return "SETUP_AW";
    break;
  case 0x04:
    return "SETUP_RETR";
    break;
  case 0x05:
    return "RF_CH";
    break;
  case 0x06:
    return "RF_SETUP";
    break;
  case 0x07:
    return "STATUS";
    break;
  case 0x08:
    return "OBSERVE_TX";
    break;
  case 0x09:
    return "RPD";
    break;
  case 0x0A:
    return "RX_ADDR_P0";
    break;
  case 0x0B:
    return "RX_ADDR_P1";
    break;
  case 0x0C:
    return "RX_ADDR_P2";
    break;
  case 0x0D:
    return "RX_ADDR_P3";
    break;
  case 0x0E:
    return "RX_ADDR_P4";
    break;
  case 0x0F:
    return "RX_ADDR_P5";
    break;
  case 0x10:
    return "TX_ADDR";
    break;
  case 0x11:
    return "RX_PW_P0";
    break;
  case 0x12:
    return "RX_PW_P1";
    break;
  case 0x13:
    return "RX_PW_P2";
    break;
  case 0x14:
    return "RX_PW_P3";
    break;
  case 0x15:
    return "RX_PW_P4";
    break;
  case 0x16:
    return "RX_PW_P5";
    break;
  case 0x17:
    return "FIFO_STATUS";
    break;
  case 0x1C:
    return "DYN_PL";
    break;
  case 0x1D:
    return "FEATURE";
    break;
  default:
      buffer[0] = (reg>>4)+'0';
      buffer[1] = (reg&0x0f)+'0';
      buffer[2] = 0;
    //    snprintf(buffer,5,"%02x",reg);
    break;
  }
  return buffer;
}



static void com_spi_send(int index, uint8_t *packet, size_t n)
{
    //pause_boost();
  digitalWrite(com_csn_pins[index], LOW);
  delay_us(10);
    for (int k=0; k<n; k++) {
      uint8_t reply = com.send(packet[k]);
        packet[k] = reply;
    }
    delay_us(10);
    digitalWrite(com_csn_pins[index], HIGH);
    //resume_boost();
}

void com_set_reg(int index, uint8_t reg, uint8_t value)
{
#ifdef DEBUG_COM
    SerialUSB.print("set reg: ");
    SerialUSB.print(reg_to_str(reg));
    SerialUSB.print(" ");
    SerialUSB.println(value,HEX);
#endif
    reg |= OP_WRITE;

    uint8_t packet[2] = {
        reg,
        value
    };

    com_spi_send(index, packet, 2);
}

void com_set_reg5(int index, uint8_t reg, uint8_t value[5])
{

#ifdef DEBUG_COM
    SerialUSB.print("set reg5:");
    SerialUSB.print(reg_to_str(reg));
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
#endif

    reg |= OP_WRITE;

    uint8_t packet[6] = {
        reg,
        value[0],
        value[1],
        value[2],
        value[3],
        value[4],
    };

    com_spi_send(index, packet, 6);
}

uint8_t com_read_reg(int index, uint8_t reg)
{
#ifdef DEBUG_COM
    SerialUSB.print("read reg:");
    SerialUSB.print(reg_to_str(reg));
#endif

    reg |= OP_READ;

    uint8_t packet[2] = {
        reg,
        0x00
    };

    com_spi_send(index, packet, 2);

#ifdef DEBUG_COM
    SerialUSB.print(" ");
    SerialUSB.println(packet[1],HEX);
#endif

    return packet[1];
}

int com_get_channel(int index){
    return com_read_reg(index,REG_RF_CH);
}

void com_set_channel(int index, int chan){
    com_set_reg(index,REG_RF_CH, chan);
}

int com_get_ack(int index){
    return com_read_reg(index,REG_EN_AA);
}
void com_set_ack(int index,bool v){
    if (v)
        com_set_reg(index,REG_EN_AA,REG_EN_AA_ALL);
    else
        com_set_reg(index,REG_EN_AA,0x00);
}


int com_get_retransmission_delay(int card){
    return (com_read_reg(card,REG_SETUP_RETR)>>4);
}
int com_get_retransmission_count(int card){
    return (com_read_reg(card,REG_SETUP_RETR)&0x0F);
}
void com_set_retransmission(int card, int delay, int count){
    uint8_t v=(delay<<0x04) | (count);
    com_set_reg(card,REG_SETUP_RETR,v);
}


uint8_t com_get_config(int index){
    return com_read_reg(index,REG_CONFIG);
}

uint8_t com_get_fifo_status(int index){
    return com_read_reg(index,REG_FIFO_STATUS);
}

uint8_t com_get_status(int card){
    return com_read_reg(card,REG_STATUS);
}
uint8_t com_get_rf_setup(int card){
    return com_read_reg(card,REG_RF_SETUP);
}

void com_reset_status(int card){
    com_set_reg(card,REG_STATUS,REG_STATUS_RX_DR | REG_STATUS_TX_DS | REG_STATUS_MAX_RT);
}
int com_get_pipe_payload(int card,int pipe){
    uint8_t en= com_read_reg(card,REG_EN_RXADDR);
    switch(pipe){
    case 0:
        return en & REG_EN_RXADDR_P0?com_read_reg(card,REG_RX_PW_P0):-1;
        break;
    case 1:
        return en & REG_EN_RXADDR_P1?com_read_reg(card,REG_RX_PW_P1):-1;
        break;
    case 2:
        return en & REG_EN_RXADDR_P2?com_read_reg(card,REG_RX_PW_P2):-1;
        break;
    case 3:
        return en & REG_EN_RXADDR_P3?com_read_reg(card,REG_RX_PW_P3):-1;
        break;
    case 4:
        return en & REG_EN_RXADDR_P4?com_read_reg(card,REG_RX_PW_P4):-1;
        break;
    case 5:
        return en & REG_EN_RXADDR_P5?com_read_reg(card,REG_RX_PW_P5):-1;
        break;
    }
    return 0;
}
void com_set_pipe_payload(int card,int pipe,uint8_t pl){
    if (pl>32) return;
    uint8_t en= com_read_reg(card,REG_EN_RXADDR);


    switch(pipe){
    case 0:
        if (pl==0) en=en & ~REG_EN_RXADDR_P0;
        else en|=REG_EN_RXADDR_P0;
        com_set_reg(card,REG_RX_PW_P0,pl);
        break;
    case 1:
        if (pl==0) en=en & ~REG_EN_RXADDR_P1;
        else en|=REG_EN_RXADDR_P1;
        com_set_reg(card,REG_RX_PW_P1,pl);
        break;
    case 2:
        if (pl==0) en=en & ~REG_EN_RXADDR_P2;
        else en|=REG_EN_RXADDR_P2;
        com_set_reg(card,REG_RX_PW_P2,pl);
        break;
    case 3:
        if (pl==0) en=en & ~REG_EN_RXADDR_P3;
        else en|=REG_EN_RXADDR_P3;
        com_set_reg(card,REG_RX_PW_P3,pl);
        break;
    case 4:
        if (pl==0) en=en & ~REG_EN_RXADDR_P4;
        else en|=REG_EN_RXADDR_P4;
        com_set_reg(card,REG_RX_PW_P4,pl);
        break;
    case 5:
        if (pl==0) en=en & ~REG_EN_RXADDR_P5;
        else en|=REG_EN_RXADDR_P5;
        com_set_reg(card,REG_RX_PW_P5,pl);
        break;
    }
    com_set_reg(card,REG_EN_RXADDR,en);
}

uint8_t com_read_status(int index)
{
    uint8_t packet[1] = {OP_NOP};
    com_spi_send(index, packet, 1);

    return packet[0];
}

void com_read_reg5(int index, uint8_t reg, uint8_t result[5])
{
    reg |= OP_READ;

    uint8_t packet[6] = {
        reg,
        0x00, 0x00, 0x00, 0x00, 0x00
    };

    com_spi_send(index, packet, 6);

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
    //com_txing[index] = micros();

    uint8_t packet[n+1];
    packet[0] = OP_TX;
    for (uint8_t k=0; k<n; k++) {
      packet[k+1] = payload[k];
    }

    com_spi_send(index, packet, n+1);
}

static void com_rx(int index, uint8_t *payload, size_t n)
{
    uint8_t packet[n+1] = {0};

    packet[0] = OP_RX;

    com_spi_send(index, packet, n+1);

    for (uint8_t k=1; k<n+1; k++) {
      payload[k-1] = packet[k];
    }
}

void com_flush_rx(int index){
    uint8_t packet[1] = {OP_FLUSH_RX};
    com_spi_send(index, packet, 1);}

void com_flush_tx(int index){
    uint8_t packet[1] = {OP_FLUSH_TX};
    com_spi_send(index, packet, 1);
}

void com_clear_status(int card){
    com_set_reg(card,REG_STATUS,REG_STATUS_MAX_RT|REG_STATUS_RX_DR|REG_STATUS_TX_DS);
}
int com_get_lost_count(int card){
    return com_read_reg(card,REG_OBSERVE_TX) >> 4;
}
int com_get_retransmitted_count(int card){
    return com_read_reg(card,REG_OBSERVE_TX) & 0x0F;
}

void com_start_listening(int card){
    com_set_reg(card, REG_CONFIG, com_read_reg(card,REG_CONFIG)| CONFIG_PWR_UP | CONFIG_PRIM_RX );
}

void com_stop_listening(int card){
    com_set_reg(card, REG_CONFIG, (com_read_reg(card,REG_CONFIG)| CONFIG_PWR_UP ) & ~CONFIG_PRIM_RX );
}

int com_has_data(int card)
{
    com_set_state(card,ComState::RX);
    uint8_t s=com_read_reg(card,REG_STATUS);
    uint8_t v=(s&REG_STATUS_RX_P_NO)>>1;
    if (v==7)return -1;
    return v;
}

void com_receive(int card, uint8_t *payload, int size){
    com_set_state(card,ComState::RX);

    if (com_has_data(card)==-1) return;

    uint8_t packet[size+1] = {0};

    packet[0] = OP_RX;

    com_spi_send(card, packet, size+1);

    for (uint8_t k=1; k<size+1; k++) {
      payload[k-1] = packet[k];
    }

    com_clear_status(card);
}

bool com_send(int card,  uint8_t *payload, int size){
    com_set_state(card,ComState::TX); // we are suppose to be in standy II mode
    com_clear_status(card);
    com_tx(card,payload,size);

    uint8_t s,f;
    uint32 d=micros();
    do {
        s=com_read_reg(card,REG_STATUS);
        watchdog_feed();
        f=com_read_reg(card,REG_FIFO_STATUS);
//        terminal_io()->print("status is: ");
//        terminal_io()->println(s);
    } while (((micros()-d)<150000) && (
                 (f&REG_FSTAT_TX_EMPTY!=0) || (((s&REG_STATUS_TX_DS)==0) && ((s&REG_STATUS_MAX_RT)==0) )));
    if (f&REG_FSTAT_TX_EMPTY!=0) com_flush_tx(card);
    terminal_io()->print("status is: ");
    print_byte_as_hex(s);
    terminal_io()->println();
    com_clear_status(card);
    if ((s&REG_STATUS_TX_DS)!=0) return true;
    return false;
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
//bool com_irq(int index)
//{
//    if ((micros()-com_txing[index]) < 300) {
//        return true;
//    }

//    int fifo = com_read_reg(index, REG_FIFO_STATUS);

//    // Checking that the module is present (bit 7 should be always 0)
//    if (fifo != 0xff) {
//        if ((fifo & RX_EMPTY) == 0) { // RX
//            com_available[index] = true;

//            if (com_master) {
//                // Receiving a status packet from a robot
//                struct packet_robot packet;
//                com_rx(index, (uint8_t*)&packet, sizeof(struct packet_robot));

//                if (packet.id < MAX_ROBOTS) {
//                    com_has_status[packet.id] = true;
//                    com_statuses[packet.id] = packet;
//                    com_robot_reception[packet.id] = millis();
//                }
//            } else {
//                // Receiving an instruction packet from the master
//                com_master_reception = millis();
//                com_master_new = true;
//                com_master_packets++;
//                com_master_channel_packets[index]++;
//                com_rx(index, com_master_frame, PACKET_SIZE);
//            }
//        }
//        if (com_txing[index] && ((fifo & TX_EMPTY) != 0)) { // TX is over
//            com_txing[index] = 0;
//            com_mode(index, true, true);
//        }

//        return true;
//    }

//    return false;
//}

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


//static void com_poll()
//{
//    bool reinit = false;
//    for (int k=0; k<3; k++) {

//      // int safe_com_module = 1; //IT SHOULD BE CS2
//      // if(!com_master and is_charging() and k!=safe_com_module ) continue;

//      bool present = com_irq(k);

//      if (!present) {
//        com_module_last_missing[k] = millis();
//        com_module_present[k] = false;
//      } else {
//        if (!com_module_present[k] && (millis() - com_module_last_missing[k] > 150)) {
//          reinit = true;
//          com_module_present[k] = true;
//        }
//      }
//    }

//    if (reinit) {
//        com_init();
//    }
//}

void com_get_tx_addr(int index,uint8_t addr[5]){
    com_read_reg5(index,REG_TX_ADDR,addr);
}



void com_set_tx_addr(int index, uint8_t add[5]){
    com_set_reg5(index, REG_TX_ADDR, add);
}


void com_set_rx_addr(int index,int pipe, uint8_t addr[5]){
    switch(pipe){
    case 0:
        com_set_reg5(index,REG_RX_ADDR_P0,addr);
        break;
    case 1:
        com_set_reg5(index,REG_RX_ADDR_P1,addr);
        break;
    case 2:
        com_set_reg(index,REG_RX_ADDR_P2,addr[0]);
        break;
    case 3:
        com_set_reg(index,REG_RX_ADDR_P3,addr[0]);
        break;
    case 4:
        com_set_reg(index,REG_RX_ADDR_P4,addr[0]);
        break;
    case 5:
        com_set_reg(index,REG_RX_ADDR_P5,addr[0]);
        break;
    }
}

void com_get_rx_addr(int index,int pipe,uint8_t addr[5] ){

    if (pipe==0)
        com_read_reg5(index,REG_RX_ADDR_P0,addr);
    else if (pipe>=1){
        com_read_reg5(index,REG_RX_ADDR_P1,addr);
        uint8_t last=addr[0];
        if (pipe==2)
            last=com_read_reg(index,REG_RX_ADDR_P2);
        if (pipe==3)
            last=com_read_reg(index,REG_RX_ADDR_P3);
        if (pipe==4)
            last=com_read_reg(index,REG_RX_ADDR_P4);
        if (pipe==5)
            last=com_read_reg(index,REG_RX_ADDR_P5);
        addr[0]=last;
    }
}
void com_ce_enable(int index)
{
//    SerialUSB.println("enable ce");
//    terminal_io()->println("set ce");
    if (index == 0) digitalWrite(COM_CE1, HIGH);
    else if (index == 1) digitalWrite(COM_CE2, HIGH);
    else if (index == 2) digitalWrite(COM_CE3, HIGH);
}

void com_ce_disable(int index)
{
//SerialUSB.println("disable ce");
//    terminal_io()->println("unset ce");
    if (index == 0) digitalWrite(COM_CE1, LOW);
    else if (index == 1) digitalWrite(COM_CE2, LOW);
    else if (index == 2) digitalWrite(COM_CE3, LOW);

    // We are not suppose to disable CE, this is only for power reduction
    // keep com cards in standby II mode
}

bool com_is_ok(int card)
{
    // we store actual address
    uint8_t old[5];
    com_get_tx_addr(card,old);
    uint8_t test[5]={0x01,0xBB,0x4C,0x5D,0x33};
    com_set_tx_addr(card,test);
    uint8_t res[5];
    com_get_tx_addr(card,res);
    // Checking only address prefix
    for (int k=0; k<4; k++) {
        if (test[k] != res[k]) {
            return false;
        }
    }
    // restore tx addr
    com_set_tx_addr(card,old);

    return true;
}


bool com_is_all_ok(){
    return com_is_ok(0) && com_is_ok(1) && com_is_ok(2);
}

void com_init()
{
    com.begin(SPI_2_25MHZ, MSBFIRST, 0);

    //mux_init();

//        for (uint8_t k=0; k<3; k++) { // Set CSN to 1 as it is default state
//          pinMode(com_csn_pins[k], OUTPUT);
//          digitalWrite(com_csn_pins[k], HIGH);
//          delay_us(2000); // wait few
//        }


    pinMode(COM_CS1,OUTPUT);
    digitalWrite(COM_CS1,HIGH);
    pinMode(COM_CE1, OUTPUT);
    digitalWrite(COM_CE1, LOW);
    delay_us(2000); // wait few
    pinMode(COM_CS2,OUTPUT);
    digitalWrite(COM_CS2,HIGH);
    pinMode(COM_CE2, OUTPUT);
    digitalWrite(COM_CE2, LOW);
    delay_us(2000); // wait few
    pinMode(COM_CS3,OUTPUT);
    digitalWrite(COM_CS3,HIGH);
    pinMode(COM_CE3, OUTPUT);
    digitalWrite(COM_CE3, LOW);
    delay_us(2000); // wait few


    // set default state for all cards:
    // this setup is a copy from arduino rf24l01+ library
    for (uint8_t k=0; k<3; k++) {
        com_ce_enable(k);
         // 2 bytes CRC and disable all IRQ
        com_set_config(k,CONFIG_MASK_MAX_RT|CONFIG_MASK_RX_DR|CONFIG_MASK_TX_DS|CONFIG_EN_CRC|CONFIG_CRCO);
        com_set_retransmission(k,5,15);
        com_set_reg(k,REG_RF_SETUP,0x21);
        com_set_reg(k,REG_RF_SETUP,0x01);
        com_set_reg(k,REG_FEATURE,0x00);
        com_set_reg(k,REG_SETUP_AW,0x03); // set address on 5 bytes
        com_set_reg(k,REG_EN_RXADDR,0x01); // disable all pipe except 0
        com_set_reg(k,REG_DYNPD,0x00);
        com_clear_status(k);
        com_set_channel(k,76);
        com_flush_rx(k);
        com_flush_tx(k);
        com_ce_disable(k); // leave down for standby II mode
        com_power(k,false); // turn off the card
        com_current_state[k]=ComState::OFF;
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


}


void com_power(int card,bool up){
    if (up)
        com_set_config(card,com_get_config(card) | CONFIG_PWR_UP);
    else
        com_set_config(card,com_get_config(card) & ~CONFIG_PWR_UP);
}

void com_set_rf(int card, uint8_t speed, uint8_t pow){
    com_set_reg(card,REG_RF_SETUP,speed|pow);
}


void com_set_config(int card,uint8_t v){
    com_set_reg(card,REG_CONFIG,v);
}
void com_set_crc(int card,int crc){
    if (crc==0)
        com_set_config(card,com_get_config(card) &~CONFIG_EN_CRC );
    else if (crc==1)
        com_set_config(card,(com_get_config(card)|CONFIG_EN_CRC) & ~CONFIG_CRCO);
    else
        com_set_config(card,com_get_config(card)|CONFIG_EN_CRC | CONFIG_CRCO);
}

TERMINAL_COMMAND(ci, "CI")
{
    com_init();
}

//TERMINAL_COMMAND(comi, "Com stats")
//{
//    terminal_io()->println(millis()-com_last_init);
//}


//void com_send_status_to_master()
//{
//    struct packet_robot packet;
//    packet.id = infos_get_id();
//    packet.status = STATUS_OK;

//    if (!drivers_is_all_ok()) {
//        packet.status |= STATUS_DRIVER_ERR;
//    }
    
//    if (ir_present()) {
//        packet.status |= STATUS_IR;
//    }

//    packet.cap_volt = kicker_cap_voltage();
//    packet.voltage  = voltage_value()*8.0;
//    packet.xpos     = getOdometry().xpos*1000;
//    packet.ypos     = getOdometry().ypos*1000;
//    packet.ang      = getOdometry().ang*1000;

//    for (size_t k=0; k<3; k++) {
//        com_ce_disable(k);
//        com_mode(k, true, false);
//        com_set_reg(k, REG_STATUS, 0x70);
//        com_tx(k, (uint8_t *)&packet, sizeof(struct packet_robot));
//        com_ce_enable(k);
//    }
//}

TERMINAL_PARAMETER_INT(actions, "actions", 0);


//void com_process_master()
//{
//    if (com_master_frame[0] == INSTRUCTION_MASTER) {
//        // Answering with status packet
//        com_send_status_to_master();

//        // Decoding instruction packet
//        struct packet_master *master_packet;
//        master_packet = (struct packet_master*)(com_master_frame + 1);

//        // Driving wheels
//        if ((master_packet->actions & ACTION_ON)) {
//            if(developer_mode == false){
//                kicker_boost_enable(true);
//            }
//            if(master_packet->actions & ACTION_TARE_ODOM) {
//                odometry_tare((master_packet->x_speed)/1000.0, (master_packet->y_speed)/1000.0, (master_packet->t_speed)/10000.0);
//                //odometry_tare(0.0, 0.0, 0.0);

//            }else{
//            kinematic_set(master_packet->x_speed/1000.0, master_packet->y_speed/1000.0,
//                 master_packet->t_speed/1000.0);
//            }
//            actions = master_packet->actions;

//            // TODO !
//            // If we want to drible only when IR is activated, we need
//            // to uncomment
//            // the following line
//            //if ((master_packet->actions & ACTION_DRIBBLE) && (ir_present()) ) {
//            if ((master_packet->actions & ACTION_DRIBBLE)) {
//              drivers_set_safe(4, true, 1000);
//            } else {
//                drivers_set(4, false, 0);
//            }

//            // Charging
//            if (master_packet->actions & ACTION_CHARGE) {
//                kicker_boost_enable(true);
//            } else {
//                kicker_boost_enable(false);
//            }


            

//            // Kicking
//            if ((my_actions & ACTION_KICK1) || (my_actions & ACTION_KICK2) || ir_present_now()) {
//                if(kicker_cap_voltage() > 80.0){
//                    bool inverted = infos_kicker_inverted();
//                    if ((master_packet->actions & ACTION_KICK1) &&
//                        !(my_actions & ACTION_KICK1)) {
//                        kicker_kick(inverted ? 0 : 1, master_packet->kickPower*30);
//                    }

//                    if ((master_packet->actions & ACTION_KICK2) &&
//                        !(my_actions & ACTION_KICK2)) {
//                        kicker_kick(inverted ? 1 : 0, master_packet->kickPower*30);
//                    }

//                    my_actions = master_packet->actions;
//                } else {

//                    my_actions = master_packet->actions;
//                    my_actions &= ~(ACTION_KICK1 | ACTION_KICK2);
//                }
//            }
//        } else {
//            buzzer_play(MELODY_ALERT_FAST, false);

//            drivers_set(0, false, 0);
//            drivers_set(1, false, 0);
//            drivers_set(2, false, 0);
//            drivers_set(3, false, 0);
//            drivers_set(4, false, 0);
//            my_actions = 0;
//            kicker_boost_enable(false);
//        }
//    }
//    else if (com_master_frame[0] == MUSIC_PARAMS){
//         struct packet_music *music_params;
//         music_params = (struct packet_music*)(com_master_frame + 1);

//        if(music_params->instrument & SOUND_ON){
//            if(music_params->instrument & BEEPER){
//                buzzer_beep(music_params->note, music_params->duration);
//                buzzer_wait_play();
//            }
//            if(music_params->instrument & KICK){
//                kicker_kick(1,1000);
//            }
//            if(music_params->instrument & CHIP_KICK){
//                kicker_kick(0,1000);
//            }
//            if(music_params->instrument & DRIBBLER){
//                drivers_set(4, true, 1000);
//            }
//            else
//            {
//                drivers_set(4, false, 0);
//            }
            
//        }

//     }
//}

//void com_tick()
//{
//    static int last = micros();

//// Comment to debug on shell
//#define BINARY

//       // Entering master infinite loop
//      while (com_master) {
//        // Feed the watchdog
//        watchdog_feed();

//        // Polling com IRQs
//        com_poll();

//        // Tick the communication with USB master
//#ifdef BINARY
//        //com_usb_tick();
//#else
//        terminal_tick();
//#endif

//        // Feed the watchdog
//        watchdog_feed();

//        bool transmitting = false;
//        while (com_master_pos < MAX_ROBOTS && !transmitting) {
//          // Sending a packet to a robot
//          // XXX: Using micros() in unsafe because it sometime overflow, to fix!
//          // We either received a status from the previous robot or the timeout expired,
//          // we should ask the next one
//          if (com_master_pos < 0 || com_has_status[com_master_pos] || (micros() - last) > 2000) {
//            // Asking the next
//            com_master_pos++;
//            last = 0;

//            // Should we send a packet to this robot ?
//            if (com_master_pos < MAX_ROBOTS && com_should_transmit[com_master_pos]) {
//              last = micros();
//              transmitting = true;

//              // Sending the packet to the 3 modules
//              for (size_t k=0; k<3; k++) {
//                com_ce_disable(k);
//                // Preparing to transmit
//                com_mode(k, true, false);
//                com_set_reg(k, REG_STATUS, 0x70); // 0111 0000
//               // com_set_tx_addr(k, com_master_pos);

//                // Transmitting the payload
//                com_tx(k, com_robots[com_master_pos], PACKET_SIZE);
//                com_ce_enable(k);
//              }
//              com_should_transmit[com_master_pos] = false;
//            }

//            if (com_master_pos >= MAX_ROBOTS) {
//              // Our cycle is over, sending back the robot statuses
//              size_t statuses = 0;
//              for (size_t k=0; k<MAX_ROBOTS; k++) {
//                if (com_has_status[k] ) {
//                  statuses++;
//                }
//              }
//#ifdef BINARY
//              uint8_t answer[1+1+1+statuses*(1+sizeof(struct packet_robot))+1];
//              // Answer header
//              answer[0] = 0xaa;
//              answer[1] = 0x55;
//              // Number of robots in the packet
//              answer[2] = statuses;
//              size_t pos = 3;
//              for (size_t k=0; k<MAX_ROBOTS; k++) {
//                if (com_has_status[k]) {
//                  // Inserting the robot #
//                  answer[pos++] = k;
//                  // Copying structure data
//                  uint8_t *ptr = (uint8_t *)&com_statuses[k];
//                  for (size_t n=0; n<sizeof(struct packet_robot); n++) {
//                    answer[pos++] = ptr[n];
//                  }
//                }
//              }
//              // Ending with 0xff
//              answer[pos] = 0xff;
//              SerialUSB.write(answer, sizeof(answer));
//#endif
//            }
//          } else {
//            transmitting = true;
//          }
//        }

//        // XXX: Led for master
//        if ((millis() - com_usb_reception) < 100) {
//          digitalWrite(BOARD_LED_PIN, HIGH);
//        } else {
//          digitalWrite(BOARD_LED_PIN, LOW);
//        }
//      }

//      // Polling IRQs
//      com_poll();

//      // Processing a packet from the master
//      if (!com_master && com_master_new) {
//        last = micros();
//        com_master_controlling = true;
//        com_master_new = false;
//        com_process_master();
//      }

//      // Checking the last master reception, playing melody when the reception is changing
//      if ((millis() - com_master_reception) < 100 && com_master_reception != 0) {
//        if (!com_has_master) {
//          com_has_master = true;
//          // buzzer_play(MELODY_BEGIN);
//        }
//        digitalWrite(BOARD_LED_PIN, HIGH);
//      } else {
//        if (com_has_master) {
//          com_has_master = false;
//          kicker_boost_enable(false); // Stopping the charge
//          // buzzer_play(MELODY_END);
//        }
//        my_actions = 0;
//        digitalWrite(BOARD_LED_PIN, LOW);
//      }
//}

//TERMINAL_COMMAND(mp, "Master packets")
//{
//    terminal_io()->println(com_master_packets);
//    terminal_io()->println(com_master_channel_packets[0]);
//    terminal_io()->println(com_master_channel_packets[1]);
//    terminal_io()->println(com_master_channel_packets[2]);
//}

//TERMINAL_COMMAND(ms, "Master send")
//{
//    for (int k=0; k<PACKET_SIZE; k++) {
//        com_robots[3][k] = 0;
//    }
//    com_has_status[3] = false;
//    com_should_transmit[3] = true;
//    com_master_pos = -1;
//}

//TERMINAL_COMMAND(mr, "Master receive")
//{
//    if (com_has_status[3]) {
//        terminal_io()->println("Got answer");
//    } else {
//        terminal_io()->println("Got nothing");
//    }
//}


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

void print_config(int k){

    terminal_io()->print(k);
    terminal_io()->print(":");
    uint8_t conf=com_get_config(k);
    if (conf&CONFIG_PWR_UP)
        terminal_io()->print(" UP   ");
    else
        terminal_io()->print(" down ");

    if (conf&CONFIG_EN_CRC)
        terminal_io()->print(" CRC ");
    else
        terminal_io()->print(" crc ");
    if (conf&CONFIG_CRCO)
        terminal_io()->print(" CRCO ");
    else
        terminal_io()->print(" crco ");
    if (conf&CONFIG_MASK_MAX_RT)
        terminal_io()->print(" MAXRT ");
    else
        terminal_io()->print(" maxrt ");
    if (conf&CONFIG_MASK_RX_DR)
        terminal_io()->print(" RXDR ");
    else
        terminal_io()->print(" rxdr ");
    if (conf&CONFIG_MASK_TX_DS)
        terminal_io()->print(" TXDS ");
    else
        terminal_io()->print(" txds ");
    if (conf&CONFIG_PRIM_RX)
        terminal_io()->print(" PRIMRX ");
    else
        terminal_io()->print(" primrx ");
    terminal_io()->println("");
}

TERMINAL_COMMAND(config, "Display ack support")
{
    if (argc==1)
        print_config(atoi(argv[0]));
    else for(int k=0;k<3;++k)
        print_config(k);
}

void print_addr_term(int k){
    terminal_io()->print(k);
    terminal_io()->print(" : width ");
    terminal_io()->println(com_read_reg(k,REG_SETUP_AW)+2);
    terminal_io()->print("  tx:   ");
    uint8_t a[5];
    com_get_tx_addr(k,a);
    print_addr(a);
    terminal_io()->println();
    for(int pipe=0;pipe<6;pipe++){
        terminal_io()->print("  rx(");
        terminal_io()->print(pipe);
        terminal_io()->print("):");
        uint8_t a[5];
        com_get_rx_addr(k,pipe,a);
        print_addr(a);
        terminal_io()->println();
    }
}

TERMINAL_COMMAND(addr, "display or set com address: addr [id] nb.nb.nb.nb.nb")
{
    if (argc==0){
        for(int k=0;k<3;++k)
            print_addr_term(k);
    } else if (argc==1){
        print_addr_term(atoi(argv[0]));
    } else if (argc!=2){
        terminal_io()->println("invalid number of args");
    } else {
        int id=atoi(argv[0]);
        uint8_t a[5];
        int l=0;
        int s=0;
        int k=0;
        while((k<5) && (argv[1][l]!='\0')){
            if (argv[1][l]=='.') {
                argv[1][l]='\0';
                a[k]=atoi(argv[1]+s);
                s=l+1;
                k+=1;
            }
            l+=1;
        }
        if (k!=4){
            terminal_io()->println("invalid address");
        } else {
            a[k] = atoi(argv[1]+s);
            com_set_tx_addr(id, a);
            com_set_rx_addr(id,0, a);
        }
    }

}


void print_feature(int k){
    terminal_io()->print(k);
    terminal_io()->print(":");
    uint8_t fea=com_read_reg(k,REG_FEATURE);
    print_byte_as_hex(fea);
    terminal_io()->println();
}
TERMINAL_COMMAND(fea, "display feature reg")
{
    if (argc==1)
        print_feature(atoi(argv[0]));
    else
        for(int k=0;k<3;++k)
            print_feature(k);
}

void print_dynp(int k){
    terminal_io()->print(k);
    terminal_io()->print(":");
    uint8_t fea=com_read_reg(k,REG_DYNPD);
    print_byte_as_hex(fea);
    terminal_io()->println();
}
TERMINAL_COMMAND(dynp, "display dynp reg")
{
    if (argc==1)
        print_dynp(atoi(argv[0]));
    else
        for(int k=0;k<3;++k)
            print_dynp(k);
}
void print_rpd(int k){
    terminal_io()->print(k);
    terminal_io()->print(":");
    uint8_t fea=com_read_reg(k,REG_RPD);
    print_byte_as_hex(fea);
    terminal_io()->println();
}
TERMINAL_COMMAND(rpd, "display rpd reg")
{
    if (argc==1)
        print_rpd(atoi(argv[0]));
    else
        for(int k=0;k<3;++k)
            print_rpd(k);
}

TERMINAL_COMMAND(rst, "reset status rst [card]"){
    if (argc==0){
        for(int k=0;k<3;++k)
            com_reset_status(k);
    } else {
        int c=atoi(argv[1]);
        com_reset_status(c);
    }

}

void print_setup(int k){

    terminal_io()->print(k);
    terminal_io()->print(":");
    uint8_t s=com_get_rf_setup(k);

    if (s&REG_RF_CONT_WAVE)
        terminal_io()->print(" CONT ");
    else
        terminal_io()->print(" cont ");
    if (s&REG_RF_DR_LOW)
        terminal_io()->print(" DRLOW ");
    else
        terminal_io()->print(" drlow ");
    if (s&REG_RF_DR_LOW)
        terminal_io()->print(" PLL ");
    else
        terminal_io()->print(" pll ");
    if (s&REG_RF_DR_HIGH)
        terminal_io()->print(" DRHIG ");
    else
        terminal_io()->print(" drhig ");
    terminal_io()->print(s&REG_RF_PWR>>1);
    if (s&REG_RF_DR_LOW){
        if (s&REG_RF_DR_HIGH)
            terminal_io()->print(" error in data rate ");
    else
            terminal_io()->print(" 250kbps ");
    }
    else{
        if (s&REG_RF_DR_HIGH)
            terminal_io()->print(" 2Mbps ");
        else
           terminal_io()->print(" 1Mbps ");
    }
    if ((s&REG_RF_PWR>>1)==0)
        terminal_io()->print(" -18dBm ");
    else if ((s&REG_RF_PWR>>1)==1)
            terminal_io()->print(" -12dBm ");
    else if ((s&REG_RF_PWR>>1)==2)
            terminal_io()->print(" -6dBm ");
    else if ((s&REG_RF_PWR>>1)==3)
            terminal_io()->print(" 0dBm ");
    terminal_io()->println();
}


TERMINAL_COMMAND(setup, "display rf setup"){
    if (argc==1)
        print_setup(atoi(argv[0]));
    else
        for(int k=0;k<3;++k)
            print_setup(k);
}

void print_status(int k){
    terminal_io()->print(k);
    terminal_io()->print(": [STAT] ");
    uint8_t c=com_get_status(k);
    if (c&REG_STATUS_RX_DR)
        terminal_io()->print(" RXDR ");
    else
        terminal_io()->print(" rxdr ");
    if (c&REG_STATUS_TX_DS)
        terminal_io()->print(" TXDS ");
    else
        terminal_io()->print(" txds ");
    if (c&REG_STATUS_MAX_RT)
        terminal_io()->print(" MRT ");
    else
        terminal_io()->print(" mrt ");
    terminal_io()->print((c&REG_STATUS_RX_P_NO)>>1);
    if (c&REG_STATUS_TX_FULL)
        terminal_io()->print(" FULL ");
    else
        terminal_io()->print(" full ");
    terminal_io()->println();
    terminal_io()->print(" : [FIFO] ");
    c=com_get_fifo_status(k);
    if (c&REG_FSTAT_TX_REUSE)
        terminal_io()->print(" TXR ");
    else
        terminal_io()->print(" txr ");
    if (c&REG_FSTAT_TX_FULL)
        terminal_io()->print(" TXFULL ");
    else
        terminal_io()->print(" txfull ");
    if (c&REG_FSTAT_TX_EMPTY)
        terminal_io()->print(" TXEMPTY ");
    else
        terminal_io()->print(" txempty ");
    if (c&REG_FSTAT_RX_FULL)
        terminal_io()->print(" RXFULL ");
    else
        terminal_io()->print(" rxfull ");
    if (c&REG_FSTAT_RX_EMPTY)
        terminal_io()->print(" RXEMPTY ");
    else
        terminal_io()->print(" rxempty ");
    terminal_io()->println();
}

TERMINAL_COMMAND(status, "status")
{
    if (argc==0)
        for(int k=0;k<3;++k)
            print_status(k);
    else
        print_status(atoi(argv[0]));
}

void print_chan(int k){
    terminal_io()->print(k);
    terminal_io()->print(":");
    terminal_io()->println(com_get_channel(k));
}

TERMINAL_COMMAND(chan, "Display or set used channel: chan [id] [chan]")
{
    if (argc==0){
    for(int k=0;k<3;++k)
        print_chan(k);
    } else if (argc==1){
        print_chan(atoi(argv[0]));
    }
    else {
        int id=atoi(argv[0]);
        uint8_t chan=atoi(argv[1]);
        com_set_channel(id,chan);
    }
}

void print_ack(int k){
    terminal_io()->print(k);
    terminal_io()->print(":");
    print_byte_as_hex(com_get_ack(k));
    terminal_io()->println();
}

TERMINAL_COMMAND(ack, "Display or set ack support: ack card [O|1]")
{
    if (argc==0){
    for(int k=0;k<3;++k)
        print_ack(k);
    } else if (argc==1){
        print_ack(atoi(argv[0]));
    }
    else if (argc!=2){
        terminal_io()->println("wrong arg");
    } else {
        int c=atoi(argv[0]);
        bool b=atoi(argv[1]);
        com_set_ack(c,b);
    }
}

TERMINAL_COMMAND(recv, "check if data received")
{
    for(int k=0;k<3;++k){
        terminal_io()->print(k);
        terminal_io()->print(": ");
        int data = com_has_data(k);
        terminal_io()->print(data);
        if (data>=0){
            char pl[32];
            com_receive(k,(uint8_t *)pl,32);
            terminal_io()->print(" data[");
            terminal_io()->print(pl);
            terminal_io()->print("]");
            com_flush_rx(k);
            com_clear_status(k);
        }
        com_clear_status(k);
        terminal_io()->println();
    }
}

TERMINAL_COMMAND(send, "send card msg")
{
    if (argc!=2)
        terminal_io()->println("wrong number of args");
    else{
        int card=atoi(argv[0]);
        int s=com_get_pipe_payload(card,0);
        terminal_io()->print("payload is ");
        terminal_io()->println(s);
        char payload[32]={'\0'};
        for(int i=0;i<32 && argv[1][i]!='\0';++i) payload[i]=argv[1][i];
        com_send(card,(uint8_t *)payload,s);
    }
}

void print_retr(int card){
    terminal_io()->print(card);
    terminal_io()->print(": ");
    int d=com_get_retransmission_delay(card);
    terminal_io()->print(d);
    terminal_io()->print(" (");
    terminal_io()->print(250+250*d);
    terminal_io()->print(") ");
    terminal_io()->println(com_get_retransmission_count(card));
}

TERMINAL_COMMAND(retr, "Display or set retransmission: retr card delay count")
{
    if (argc==0){
    for(int k=0;k<3;++k){
        print_retr(k);
    }} else if (argc==1) {
        print_retr(atoi(argv[0]));
    } else if (argc!=3){
        terminal_io()->println("wrong arg");
    } else {
        int c=atoi(argv[0]);
        int b=atoi(argv[1]);
        int f=atoi(argv[2]);
        com_set_retransmission(c,b,f);
    }
}


void print_csnce(int card){
    terminal_io()->print("card ");
    terminal_io()->print(card);
    terminal_io()->print(" csn: ");
    if (digitalRead(com_csn_pins[card])==LOW)
        terminal_io()->print(" low ");
    else
        terminal_io()->print(" high ");
    terminal_io()->print(" ce: ");
    if (digitalRead(com_ce_pins[card])==LOW)
        terminal_io()->print(" low ");
    else
        terminal_io()->print(" high ");

    terminal_io()->println();
}

TERMINAL_COMMAND(csnce, "get csn pin and ce pin status")
{
    if (argc==1)
        print_csnce(atoi(argv[0]));
    else if (argc==2){
        int c=atoi(argv[0]);
        int s=atoi(argv[1]);
        if (s==0) com_ce_disable(c);
        else com_ce_enable(c);
    }
    else for(int i=0;i<3;++i)
            print_csnce(i);
}

void print_obs(int card){
    terminal_io()->print(card);
    terminal_io()->print(": ");
    terminal_io()->print(com_get_lost_count(card));
    terminal_io()->print(" lost ");
    terminal_io()->print(com_get_retransmitted_count(card));
    terminal_io()->println(" retransmitted ");
}

TERMINAL_COMMAND(obs, "display transmission stats")
{
    if (argc==1)
        print_obs(atoi(argv[0]));
    else
        for(int k=0;k<3;++k)
            print_obs(k);
}

static void print_rxp(int card){
    terminal_io()->print(card);
    terminal_io()->print(": ");
    for(int pipe=0;pipe<6;pipe++){
        terminal_io()->print(pipe);
        int pl=com_get_pipe_payload(card,pipe);
        if (pl==0) terminal_io()->print("[disabled] ");
        else {
            terminal_io()->print("[");
            terminal_io()->print(pl);
            terminal_io()->print("] ");
        }
    }
    terminal_io()->println();

}

TERMINAL_COMMAND(rxp, "display or set pipe payload, 0 means not active")
{
    if (argc==0){
        for(int k=0;k<3;++k)
            print_rxp(k);
    } else if (argc==1){
        print_rxp(atoi(argv[0]));
    } else if (argc!=3){
        terminal_io()->println("wrong number of arguments: card pipe payloadsize");
    }
    else{
        int c=atoi(argv[0]);
        int p=atoi(argv[1]);
        int pl=atoi(argv[2]);
        com_set_pipe_payload(c,p,pl);
    }
}


TERMINAL_COMMAND(flush, "flush rx and tx for all cards")
{
    for(int i=0;i<3;++i){
        com_flush_rx(i);
        com_flush_tx(i);
    }
}

TERMINAL_COMMAND(st, "get or set card state: 0 off , 1 rx, 2 tx")
{
    if (argc==0)
    for(int k=0;k<3;++k){
        terminal_io()->print(k);
        terminal_io()->print(" : ");
        if (com_current_state[k]==OFF)
            terminal_io()->println(" OFF ");
        else if (com_current_state[k]==RX)
            terminal_io()->println(" RX ");
        else
            terminal_io()->println(" TX ");
    }
    else {
        int c=atoi(argv[0]);
        int s=atoi(argv[1]);
        if (s==0) com_set_state(c,OFF);
        else if (s==1) com_set_state(c,RX);
        else if (s==2) com_set_state(c,TX);
    }
}


TERMINAL_COMMAND(dbg, "dump all informations")
{
    int n=0;
    char *arg[]={nullptr,nullptr};
    if (argc==1){
        n+=1;
        arg[0]=argv[0];
    }

    terminal_io()->println("csnce:");
    terminal_command_csnce(n,arg);
    terminal_io()->println("config:");
    terminal_command_config(n,arg);
    terminal_io()->println("ack:");
    terminal_command_ack(n,arg);
    terminal_io()->println("addr:");
    terminal_command_addr(n,arg);
    terminal_io()->println("retr:");
    terminal_command_retr(n,arg);
    terminal_io()->println("channel:");
    terminal_command_chan(n,arg);
    terminal_io()->println("setup:");
    terminal_command_setup(n,arg);
    terminal_io()->println("status:");
    terminal_command_status(n,arg);
    terminal_io()->println("observ:");
    terminal_command_obs(n,arg);
    terminal_io()->println("pipe pyload:");
    terminal_command_rxp(n,arg);
    terminal_io()->println("feature:");
    terminal_command_fea(n,arg);
    terminal_io()->println("dynp:");
    terminal_command_dynp(n,arg);
    terminal_io()->println("rpd:");
    terminal_command_rpd(n,arg);
}

void com_copy_addr(uint8_t dst[], uint8_t src[])
{
    for(int i=0;i<5;++i)
        dst[i]=src[i];
}
