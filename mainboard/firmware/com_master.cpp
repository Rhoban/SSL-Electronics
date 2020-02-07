#include "com_master.h"
#include "nrf24l01.h"
#include "com_proto.h"

#include <watchdog.h>
#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>

#include "buzzer.h"
#include "hardware.h"

#define DEBUG_MASTER_COM

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

struct com_pipe_infos{
    uint8_t addr[5];
    bool available;
    packet_robot infos;
    uint32_t last_status; // last status received using millis()
    com_pipe_infos():available(true),last_status(millis()){}
};

struct com_pipe_infos com_status_pipes[6];

struct icmp_send_waiting{
    int stamp; // micros(), 0 means nothing to send
    struct icmp_order reply;
    uint8_t addr[5];
};

struct icmp_send_waiting icmp_to_send;

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
        com_copy_addr(com_status_pipes[0].addr,a);
        com_set_pipe_payload(CARD_STATUS,0,status_payload_size);
        uint8_t b[5]=addr_for_status;
        for(uint8_t pipe=1;pipe<6;pipe++){
            b[0]=(b[0]&0xF0) | pipe;
            com_copy_addr(com_status_pipes[pipe].addr,b);
            com_status_pipes[pipe].available = true;
            com_status_pipes[pipe].last_status = millis();
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
        // use stm32 uid to fix the address:
        uint8_t id[12];
        get_uid(id);
        unsigned int d=micros();
        uint8_t z[5];
        for(int i=0;i<5;++i)
            z[i]=id[0]^id[5+i];
        while (z[0]==y[0]) z[0]+=1; // First byte must be unique!!!!
        com_set_rx_addr(CARD_ICMP,1,z);
        com_set_pipe_payload(CARD_ICMP,0,icmp_payload_size);
        com_set_pipe_payload(CARD_ICMP,1,icmp_payload_size);
        com_set_state(CARD_ICMP,ComState::TX);
    }
}


void com_master_icmp_tick(){
    if (com_has_data(CARD_ICMP)==0){ // message arriving on pipe 1 are answers, ignore them
        struct icmp_order o;
        com_receive(CARD_ICMP,(uint8_t *)&o,sizeof(icmp_order));
#ifdef DEBUG_MASTER_COM
        SerialUSB.print("Receive an ICMP request: ");
        SerialUSB.println(o.icmp_type);
#endif
        com_clear_status(CARD_ICMP);
        if (o.icmp_type == ICMP_DHCP_REQUEST){
            uint8_t pipe_id=0;

            com_copy_addr(icmp_to_send.addr,o.icmp_addr);

            while((com_status_pipes[pipe_id].available==false) && (pipe_id<6))
                pipe_id+=1;
            if (pipe_id==6){
                icmp_to_send.reply.icmp_type=ICMP_DHCP_REPLY;
                icmp_to_send.reply.arg=ICMP_FULL;
            } else {
#ifdef DEBUG_MASTER_COM
                SerialUSB.print("find pipe ");
                SerialUSB.print(pipe_id);
                SerialUSB.print(" : ");
                print_addr(com_status_pipes[pipe_id].addr);
                SerialUSB.println();
#endif
                com_status_pipes[pipe_id].available=false;
                com_status_pipes[pipe_id].last_status=millis();
                icmp_to_send.reply.icmp_type=ICMP_DHCP_REPLY;
                icmp_to_send.reply.arg=ICMP_OK;
                com_copy_addr(icmp_to_send.reply.icmp_addr,com_status_pipes[pipe_id].addr);
            }
            icmp_to_send.stamp=micros();
        } else if (o.icmp_type == ICMP_ECHO){ // send message back
            icmp_to_send.reply = o;
            icmp_to_send.stamp = micros();
        } else if (o.icmp_type == ICMP_NOREPLY){
            com_set_state(CARD_ICMP,RX);
        }
    }

    if (icmp_to_send.stamp>0){
        SerialUSB.print("sending icmp reply to: ");
        print_addr(icmp_to_send.addr);
        SerialUSB.print(" with a reply addr: ");
        print_addr(icmp_to_send.reply.icmp_addr);
        SerialUSB.println();
        if ((micros()-icmp_to_send.stamp)>ICMP_TIMEOUT){
            icmp_to_send.stamp=0;
        }
        // send reply using icmp:
        uint8_t save[5];
        com_get_tx_addr(CARD_ICMP,save);
        com_set_tx_addr(CARD_ICMP,icmp_to_send.addr);
        com_set_state(CARD_ICMP,TX);
        bool r=com_send(CARD_ICMP,(uint8_t *)&(icmp_to_send.reply),sizeof(icmp_order));
        if (r) icmp_to_send.stamp=0;
        else com_flush_tx(CARD_ICMP);
        com_set_tx_addr(CARD_ICMP,save);
        com_set_state(CARD_ICMP,RX);
    }
}

void com_master_status_tick(){
    int data = com_has_data(CARD_STATUS);
    if (data>=0){
        com_receive(CARD_STATUS,(uint8_t *)&(com_status_pipes[data].infos),status_payload_size);
        com_status_pipes[data].last_status=millis();
        if (com_status_pipes[data].available == true){
            // we are not suppose to receive infos from this pipe!
            buzzer_beep(C6,500);
            buzzer_wait_play();
            watchdog_feed();
            buzzer_beep(C5,500);
            buzzer_wait_play();
        }
#ifdef DEBUG_MASTER_COM
        SerialUSB.print("receive data STATUS CARD,  pipe ");
        SerialUSB.print(data);
        SerialUSB.print(" from ");
////        SerialUSB.print("data ");
        SerialUSB.print(com_status_pipes[data].infos.id);
        SerialUSB.println(" ");
//        com_flush_rx(k);
#endif
        com_clear_status(0);
    }
}
void com_master_order_tick(){
    int data = com_has_data(CARD_ORDER);
    if (data>=0){
        char pl[32];
        com_receive(CARD_ORDER,(uint8_t *)pl,32);
#ifdef DEBUG_MASTER_COM
        SerialUSB.print("receive data ORDER CARD,  pipe ");
        SerialUSB.print(data);
        SerialUSB.print(" content ");
////        SerialUSB.print("data ");
        SerialUSB.print(((int *)pl)[0]);
        SerialUSB.println(" ");
#endif
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
    o.arg = 0;
    uint8_t icmp_addr[5]=addr_for_icmp;
    com_get_rx_addr(CARD_ICMP,1,o.icmp_addr);
    if (com_send(CARD_ICMP,(uint8_t *)&o,icmp_payload_size)){
        terminal_io()->println("icmp packet acked");
        com_set_state(CARD_ICMP,RX);
        print_addr(o.icmp_addr);
        int d=micros();
        int r=-1;
        while(((r=com_has_data(CARD_ICMP))==-1) && ((micros()-d)<500000)){
            watchdog_feed();
        }
        com_flush_rx(CARD_ICMP);
        if (r==-1){
            terminal_io()->println("icmp packet not reply");
        } else {
            terminal_io()->println("icmp packet reply");
        }
        com_set_tx_addr(CARD_ICMP,icmp_addr);
        com_set_state(CARD_ICMP,RX);
    } else
        terminal_io()->println("icmp packet not acked");

}


TERMINAL_COMMAND(dhcp, "send a dhcp request")
{
    if (card_icmp_ok == false){
        terminal_io()->println("icmp card no available!");
        return;
    }
    struct icmp_order o;
    o.icmp_type = ICMP_DHCP_REQUEST;
    o.arg = 0;
    uint8_t icmp_addr[5]=addr_for_icmp;
    com_get_rx_addr(CARD_ICMP,1,o.icmp_addr); // get internal pipe 1 address for reply

    if (com_send(CARD_ICMP,(uint8_t *)&o,icmp_payload_size)){
        terminal_io()->println("icmp packet acked");
        com_set_state(CARD_ICMP,RX);
        int d=micros();
        int r=-1;
        while(((r=com_has_data(CARD_ICMP))==-1) && ((micros()-d)<ICMP_TIMEOUT)){
            watchdog_feed();
        }
        if (r!=-1){
            com_receive(CARD_ICMP,(uint8_t *)&o,icmp_payload_size);
            if (o.icmp_type == ICMP_DHCP_REPLY){
                if (o.arg == ICMP_FULL){
                    terminal_io()->println("master is full, try later");
                } else {
                    terminal_io()->print("master givs us address: ");
                    print_addr(o.icmp_addr);
                    terminal_io()->println();
                }
            } else {
                terminal_io()->println("wrong icmp reply type");
            }
        } else {
            terminal_io()->println("no reply");
        }
        com_flush_rx(CARD_ICMP);
        com_set_tx_addr(CARD_ICMP,icmp_addr);
        com_set_state(CARD_ICMP,RX);
    } else
        terminal_io()->println("icmp packet not acked");

}






