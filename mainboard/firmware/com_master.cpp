#include "com_master.h"
#include "nrf24l01.h"
#include "com_proto.h"

#include <watchdog.h>
#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>



#define STATE_INIT     0
#define STATE_MAGIC1   1
#define STATE_SWITCH   2
#define STATE_ORDER    3

#define USBMODE_BIN 1
#define USBMODE_TERM 2
int usbcom_mode = USBMODE_TERM;

#define CARD_STATUS 0
#define CARD_ORDER  1
#define CARD_ICMP   2
bool card_status_ok=false;
bool card_order_ok=false;
bool card_icmp_ok=false;

static void com_usb_tick()
{
    static int state = 0;
    static uint8_t temp[1024];
    static size_t pos = 0;
    static int magic_pos=0;
    static char magic_term[]="term";
    static int magic_len=3;

    while (SerialUSB.available()) {
        watchdog_feed();
        uint8_t c = SerialUSB.read();
        if (state == STATE_INIT) {
            if (c == 0xaa) {
                state=STATE_MAGIC1;
            } else if (c==magic_term[0]){
                state=STATE_SWITCH;
                magic_pos=1;
            }
        } else if (state == STATE_MAGIC1) {
            if (c == 0x55) {
                state=STATE_ORDER;
            } else {
                state = STATE_INIT;
            }
        } else if (state==STATE_SWITCH){
            if (c == magic_term[magic_pos]){
                if (magic_pos == magic_len){
                    state=STATE_INIT;
                    magic_pos=0;
                    usbcom_mode=USBMODE_TERM;
                } else magic_pos+=1;
            } else {
                state=STATE_INIT;
                magic_pos=0;
            }
        } else if (state == STATE_ORDER) {
/*
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
*/
            state = 0;
        }
    }
}

//#define RECV_TEST 0

void com_master_init(){
    com_init(); // default card setup

    for(int card=0;card<3;++card){
        com_set_ack(card,true); // set ACK
        com_set_crc(card,2); // 2bytes for CRC
        com_set_rf(card,SPEED_2M,POW_0db);
        com_clear_status(card);
    }

    card_icmp_ok = com_is_ok(CARD_ICMP);
    card_status_ok = com_is_ok(CARD_STATUS);
    card_order_ok = com_is_ok(CARD_ORDER);

    // Set card 0 on receive mode with multiple pipe adresses
    if (card_status_ok){
        com_set_channel(CARD_STATUS,status_chan);
        uint8_t a[5]=addr_for_status;
        com_set_tx_addr(CARD_STATUS,a);
        com_set_rx_addr(CARD_STATUS,0,a);
        com_set_pipe_payload(CARD_STATUS,0,status_payload_size);
        uint8_t b[5]=addr_for_status;
        for(uint8_t pipe=1;pipe<6;pipe++){
            b[0]=(b[0]&0xF0) | pipe;
            com_set_rx_addr(CARD_STATUS,pipe,b);
            com_set_pipe_payload(CARD_STATUS,pipe,status_payload_size);//status_payload_size);//sizeof(struct packet_robot));
        }
        com_set_state(CARD_STATUS,ComState::RX);
    }
    if (card_order_ok){
        com_set_channel(CARD_ORDER,orders_chan);
        uint8_t x[5]=addr_for_orders; // address will change according to robot
        com_set_tx_addr(CARD_ORDER,x);
        com_set_rx_addr(CARD_ORDER,0,x);
        com_set_pipe_payload(CARD_ORDER,0,order_payload_size);
        com_set_state(CARD_ORDER,ComState::TX);
    }
    if (card_icmp_ok){
        // Set card 2 on rx mode, waiting for robot dhcp request
        com_set_channel(CARD_ICMP,icmp_chan);
        uint8_t y[5]=addr_for_icmp;
        com_set_tx_addr(CARD_ICMP,y);
        com_set_rx_addr(CARD_ICMP,0,y);
        com_set_pipe_payload(CARD_ICMP,0,icmp_payload_size);
        com_set_state(CARD_ICMP,ComState::TX);
    }
}


void com_master_icmp_tick(){
    if (com_has_data(CARD_ICMP)>=0){
        struct icmp_order o;
        com_receive(CARD_ICMP,(uint8_t *)&o,sizeof(icmp_order));

        SerialUSB.print("Receive an ICMP request: ");
        SerialUSB.println(o.icmp_type);
        com_clear_status(CARD_ICMP);
    }
}

void com_master_status_tick(){
    int data = com_has_data(CARD_STATUS);
    if (data>=0){
        packet_robot status;
//        char pl[32];
        com_receive(CARD_STATUS,(uint8_t *)&status,status_payload_size);
        SerialUSB.print("receive data STATUS CARD,  pipe ");
        SerialUSB.print(data);
        SerialUSB.print(" from ");
////        SerialUSB.print("data ");
        SerialUSB.print(status.id);
        SerialUSB.println(" ");
//        com_flush_rx(k);
        com_clear_status(0);
    }
}
void com_master_order_tick(){
    int data = com_has_data(CARD_ORDER);
    if (data>=0){
        char pl[32];
        com_receive(CARD_ORDER,(uint8_t *)pl,32);
        SerialUSB.print("receive data ORDER CARD,  pipe ");
        SerialUSB.print(data);
        SerialUSB.print(" content ");
////        SerialUSB.print("data ");
        SerialUSB.print(((int *)pl)[0]);
        SerialUSB.println(" ");
//        com_flush_rx(k);
        com_clear_status(0);
    }
}

void com_master_tick(){
    watchdog_feed();

    if (card_icmp_ok)
        com_master_icmp_tick();
    watchdog_feed();
    if (card_status_ok)
        com_master_status_tick();
    watchdog_feed();
    if (card_order_ok)
        com_master_order_tick();

    if (usbcom_mode == USBMODE_BIN)
        com_usb_tick();
    else {
        terminal_tick();
    }
}


TERMINAL_COMMAND(umb, "Set USB com in binary mode")
{
    terminal_io()->println("switch to binary usb com mode");
    usbcom_mode =  USBMODE_BIN;
}

TERMINAL_COMMAND(reinit, "reset master init")
{
    com_master_init();
}

TERMINAL_COMMAND(icmp, "send an icmp request")
{
    if (card_icmp_ok == false){
        terminal_io()->println("icmp card no available!");
        return;
    }
    struct icmp_order o;
    o.icmp_type = ICMP_ECHO;
    o.icmp_type = 1;
    if (com_send(CARD_ICMP,(uint8_t *)&o,icmp_payload_size)){
        terminal_io()->println("icmp packet acked");
    } else
        terminal_io()->println("icmp packet not acked");

}






