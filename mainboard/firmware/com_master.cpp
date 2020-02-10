#include "com_master.h"
#include "nrf24l01.h"
#include "com_proto.h"

#include <watchdog.h>
#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>

#include "buzzer.h"
#include "hardware.h"

//#define DEBUG_MASTER_COM

#define STATE_INIT     0
#define STATE_MAGIC1   1
#define STATE_SWITCH   2
#define STATE_ORDER    3

#define USBMODE_BIN 1
#define USBMODE_TERM 2
int usbcom_mode = USBMODE_TERM;


#define STATE_PIPE_FREE  0
#define STATE_PIPE_USED  1
#define STATE_PIPE_RESET 2

struct com_pipe_infos{
    uint8_t addr[5];
    uint8_t order_addr[5];
    int state;
    bool new_status;
    packet_robot infos;
    bool new_order;
    packet_master order;
    uint32_t last_status; // last status received using millis()
    uint32_t last_order;
    com_pipe_infos():state(STATE_PIPE_FREE),last_status(millis()),new_status(false),new_order(false){}
};

struct com_pipe_infos com_pipes[6];

struct icmp_send_waiting{
    int stamp; // millis(), 0 means nothing to send
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
    static char order_message[sizeof(packet_robot)+3]={0xaa,0x55,0xff};


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
                pos=0;
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
            if (pos<sizeof(packet_master) && (pos<sizeof(temp))){
                temp[pos]==c;
            } else if ((c==0xff) && pos==sizeof(packet_master)){
                packet_master *p=(packet_master *)(temp);
                for(int pipe=0;pipe<6;pipe++){
                    if ((com_pipes[pipe].state==STATE_PIPE_USED)
                            &&
                            (com_pipes[pipe].infos.id==p->rid)){
                        com_pipes[pipe].order = *p;
                        com_pipes[pipe].new_order=true;
                    }
                }
            } else { // go back to init state, ignore data
                state=STATE_INIT;
            }
            state = 0;
        }
    }
    for(int pipe=0;pipe<6;++pipe){
        if (com_pipes[pipe].new_status == true){
            uint8_t *p=(uint8_t *)(&(com_pipes[pipe].infos));
            for(int i=0;i<sizeof(packet_robot);++i)
                order_message[2+i]=p[i];
            order_message[2+sizeof(packet_robot)]=0xFF;
            SerialUSB.write(order_message,sizeof(order_message));
            com_pipes[pipe].new_status=false;
        }
    }
}

//#define RECV_TEST 0

void com_master_init(){
    com_init(); // default card setup

    for(int card=0;card<3;++card){
        com_set_ack(card,true); // set ACK
        com_set_crc(card,2); // 2bytes for CRC
        //   com_set_rf(card,SPEED_2M,POW_0db);
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
        com_copy_addr(com_pipes[0].addr,a);
        com_set_pipe_payload(CARD_STATUS,0,status_payload_size);
        uint8_t b[5]=addr_for_status;
        for(uint8_t pipe=1;pipe<6;pipe++){
            b[0]=(b[0]&0xF0) | pipe;
            com_copy_addr(com_pipes[pipe].addr,b);
            com_pipes[pipe].state = true;
            com_pipes[pipe].last_status = millis();
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

    if (icmp_to_send.stamp>0){
#ifdef DEBUG_MASTER_COM
        SerialUSB.print("sending icmp reply ");
        SerialUSB.print(icmp_to_send.reply.icmp_type);
        SerialUSB.print(" / ");
        SerialUSB.print(icmp_to_send.reply.arg);
        SerialUSB.print(" to ");
        print_addr(icmp_to_send.addr);
        SerialUSB.print(" with a reply addr: ");
        print_addr(icmp_to_send.reply.icmp_addr);
        SerialUSB.println();
#endif
        if ((millis()-icmp_to_send.stamp)>ICMP_TIMEOUT_MS){
            icmp_to_send.stamp=0;
            uint8_t save[5]=addr_for_icmp;
            com_set_tx_addr(CARD_ICMP,save);
            com_set_rx_addr(CARD_ICMP,0,save);
            com_set_state(CARD_ICMP,RX);
        } else {
            // send reply using icmp:
            com_set_tx_addr(CARD_ICMP,icmp_to_send.addr);
            com_set_rx_addr(CARD_ICMP,0,icmp_to_send.addr);
            com_set_state(CARD_ICMP,TX);
            bool r=com_send(CARD_ICMP,(uint8_t *)&(icmp_to_send.reply),sizeof(icmp_order));
            if (r) {
                icmp_to_send.stamp=0;
                icmp_to_send.stamp=0;
                uint8_t save[5]=addr_for_icmp;
                com_set_tx_addr(CARD_ICMP,save);
                com_set_rx_addr(CARD_ICMP,0,save);
                com_set_state(CARD_ICMP,RX);
            }
            else com_flush_tx(CARD_ICMP);
            //        com_set_tx_addr(CARD_ICMP,save);
            //        com_set_rx_addr(CARD_ICMP,0,save);
            //        com_set_state(CARD_ICMP,RX);
        }
    } else {
        int p=com_has_data(CARD_ICMP);
        if (p==0){ // message arriving on pipe 1 are answers, ignore them
            struct icmp_order o;
            com_receive(CARD_ICMP,(uint8_t *)&o,sizeof(icmp_order));
#ifdef DEBUG_MASTER_COM
            SerialUSB.print("Receive an ICMP request: ");
            SerialUSB.println(o.icmp_type);
#endif
            com_clear_status(CARD_ICMP);
            if (o.icmp_type == ICMP_DHCP_REQUEST){
                int pipe_id=0;

                com_copy_addr(icmp_to_send.addr,o.icmp_addr);

                while( (pipe_id<6) && (com_pipes[pipe_id].state!=STATE_PIPE_FREE))
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
                    com_pipes[pipe_id].state=STATE_PIPE_USED;
                    com_pipes[pipe_id].last_status=millis();
                    com_copy_addr(com_pipes[pipe_id].order_addr,icmp_to_send.addr);
                    icmp_to_send.reply.icmp_type=ICMP_DHCP_REPLY;
                    icmp_to_send.reply.arg=ICMP_OK;
                    com_copy_addr(icmp_to_send.reply.icmp_addr,com_pipes[pipe_id].addr);
                }
                icmp_to_send.stamp=millis();
            } else if (o.icmp_type == ICMP_ECHO){ // send message back
                icmp_to_send.reply = o;
                icmp_to_send.stamp = millis();
            } else if (o.icmp_type == ICMP_NOREPLY){
                com_set_state(CARD_ICMP,RX);
            }
        } else if (p!=-1){
            com_flush_rx(CARD_ICMP);
        }
    }

}

void com_master_status_tick(){
    uint32_t now=millis();
    int pipe_error=-1;

    int data = com_has_data(CARD_STATUS);
    if (data>=0){
        com_receive(CARD_STATUS,(uint8_t *)&(com_pipes[data].infos),status_payload_size);
        if (com_pipes[data].state != STATE_PIPE_USED){
            pipe_error=data;
            SerialUSB.print("receive data on pipe ");
            SerialUSB.print(data);
            SerialUSB.print(" but pipe state is ");
            SerialUSB.println(com_pipes[data].state);
            // we are not suppose to receive infos from this pipe!
//            buzzer_beep(C6,500);
//            buzzer_wait_play();
//            watchdog_feed();
//            buzzer_beep(C5,500);
//            buzzer_wait_play();
        } else {
            com_pipes[data].last_status=now;
            com_pipes[data].new_status = true;
        }
#ifdef DEBUG_MASTER_COM
        SerialUSB.print("receive data STATUS CARD,  pipe ");
        SerialUSB.print(data);
        SerialUSB.print(" from ");
        ////        SerialUSB.print("data ");
        SerialUSB.print(com_status_pipes[data].infos.id);
        SerialUSB.println(" ");
#endif
        com_clear_status(CARD_STATUS);
    }
    // check for lost robots
    for(int pipe=0;pipe<6;++pipe){
        // disable ACK for this pipe for STATUS_TIMEOUT_MS
        if ((com_pipes[pipe].state==STATE_PIPE_RESET)
                && ((now - com_pipes[pipe].last_status) > STATUS_TIMEOUT_MS)){
            com_pipes[pipe].state=STATE_PIPE_FREE;
            // restore ACK functionnality for this pipe
            com_set_ack(CARD_STATUS,pipe,true);
        }
        if ((pipe==pipe_error)
                ||((com_pipes[pipe].state==STATE_PIPE_USED)
                   &&((now - com_pipes[pipe].last_status) > STATUS_TIMEOUT_MS))){
            com_pipes[pipe].state=STATE_PIPE_RESET;
            // disable ACK functionnality for this pipe
            // so possible connected robot to this pipe will reset
            com_set_ack(CARD_STATUS,pipe,false);
        }
    }

}
void com_master_order_tick(){

    for(int pipe=0;pipe<6;pipe++){
        if ((com_pipes[pipe].state==STATE_PIPE_USED) && (com_pipes[pipe].new_order)){
            com_set_tx_addr(CARD_ORDER,com_pipes[pipe].order_addr);
            com_set_rx_addr(CARD_ORDER,0,com_pipes[pipe].order_addr);
            if (com_send(CARD_ORDER,(uint8_t *)(&(com_pipes[pipe].order)),sizeof(packet_master))){
                com_pipes[pipe].new_order=false;
            }
        }
    }

    /*
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
    */
}

void terminal_status_infos_tick(){
    static int last=millis();
    if ((millis()-last)>2000) {
        last=millis();
        for(int i=0;i<6;++i){
            if (com_pipes[i].state==STATE_PIPE_USED && com_pipes[i].new_status){
                SerialUSB.print("have new status from pipe ");
                SerialUSB.println(i);
                com_pipes[i].new_status=false;
            }
        }
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
        terminal_status_infos_tick();
    }
}


TERMINAL_COMMAND(umb, "Set USB com in binary mode")
{
    terminal_io()->println("switch to binary usb com mode");
    usbcom_mode =  USBMODE_BIN;
}

TERMINAL_COMMAND(order, "Send an order packet on a pipe")
{
    int p=atoi(argv[0]);
    com_pipes[p].order.rid = com_pipes[p].infos.id;
    com_pipes[p].order.actions = ACTION_ON;
    com_pipes[p].order.x_speed = atoi(argv[1]);
    com_pipes[p].order.y_speed = atoi(argv[2]);
    com_pipes[p].order.t_speed = atoi(argv[3]);
    com_pipes[p].order.kickPower = 0;
    com_pipes[p].new_order=true;
}

TERMINAL_COMMAND(pipes, "Display pipes informations")
{
    terminal_io()->println("pipes:");
    for(int pipe=0;pipe<6;pipe++){
        terminal_io()->print(pipe);
        terminal_io()->print(" : ");
        if (com_pipes[pipe].state==STATE_PIPE_USED){
            terminal_io()->println("USED");
            terminal_io()->print("  addr:");
            print_addr(com_pipes[pipe].order_addr);
            terminal_io()->println();
            terminal_io()->print("  id: ");
            terminal_io()->println(com_pipes[pipe].infos.id);
            terminal_io()->print("  status: ");
            terminal_io()->println(com_pipes[pipe].infos.status);
            terminal_io()->print("  x,y: ");
            terminal_io()->print(com_pipes[pipe].infos.xpos);
            terminal_io()->print(" , ");
            terminal_io()->println(com_pipes[pipe].infos.ypos);
            terminal_io()->print("  voltage: ");
            terminal_io()->println(com_pipes[pipe].infos.voltage);
        } else if (com_pipes[pipe].state==STATE_PIPE_RESET){
            terminal_io()->println("RESET");
        } else {
             terminal_io()->println("NOT USED");
        }
    }
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
        while(((r=com_has_data(CARD_ICMP))==-1) && ((micros()-d)<ICMP_TIMEOUT_MS)){
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






