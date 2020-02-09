#include "com_robot.h"
#include "com_proto.h"
#include "hardware.h"
#include <stdlib.h>
#include <watchdog.h>
#include <wirish/wirish.h>
#include <terminal.h>

#define INIT 0
#define WAIT_DHCP 1
#define RUNNING 2
#define WAIT_AND_INIT 3

int last_status_ack;
int last_order_received;
int icmp_stamp;
int state=INIT;
int wait_init_timeout;

unsigned long int status_send_ok=0;
unsigned long int status_send_lost=0;
unsigned long int dhcp_failure=0;

struct icmp_order dhcp_request;

void com_robot_tick(){
    watchdog_feed();
    switch(state){
    case INIT:
        // Send a DHCP Request and wait for answer
        if (com_send(CARD_ICMP,(uint8_t *)&dhcp_request,icmp_payload_size)){
            com_set_state(CARD_ICMP,RX);
            state=WAIT_DHCP;
            icmp_stamp=millis();
            SerialUSB.println("wait for dhcp reply");
        } else {
            dhcp_failure+=1;
            com_flush_tx(CARD_ICMP);
            state=WAIT_AND_INIT;
            wait_init_timeout=millis()+10; // wait 10ms
        }
        break;
    case WAIT_AND_INIT:
        if (millis()>wait_init_timeout)
            state=INIT;
        break;
    case WAIT_DHCP:
        if ((millis()-icmp_stamp)>ICMP_TIMEOUT_MS){
            SerialUSB.println("dhcp timeout");
            state=INIT;
        } else {
            if ((com_has_data(CARD_ICMP))>=0){
                struct icmp_order o;
                com_receive(CARD_ICMP,(uint8_t *)&o,icmp_payload_size);
                SerialUSB.print("receive icmp reply type: ");
                SerialUSB.print(o.icmp_type);
                SerialUSB.print(" / ");
                SerialUSB.println(o.arg);
                if (o.icmp_type == ICMP_DHCP_REPLY){
                    if (o.arg == ICMP_FULL){
                        state=WAIT_AND_INIT;
                        wait_init_timeout=millis()+1000; // wait for 1s
                    } else {
                        com_set_tx_addr(CARD_STATUS,o.icmp_addr);
                        com_set_rx_addr(CARD_STATUS,0,o.icmp_addr);
                        com_set_pipe_payload(CARD_STATUS,0,status_payload_size);
                        com_set_state(CARD_STATUS,RX);
                        state=RUNNING;
                        status_send_ok=0;
                        status_send_lost=0;
                        last_status_ack=millis();
                    }
                } else {
                    SerialUSB.println("wrong icmp reply type");
                    state=INIT;
                }
                com_flush_rx(CARD_ICMP);
            }
        }
        break;
    case RUNNING:
        /*
        if (com_has_data(CARD_ORDER)!=-1){
            last_order_received = millis();
            struct packet_master order;
            com_receive(CARD_ORDER,(uint8_t *)&order,order_payload_size);
        }
        */

        struct packet_robot status;
        status.id=10;
        if (com_send(CARD_STATUS,(uint8_t *)&status,status_payload_size)){
            last_status_ack=millis();
            status_send_ok+=1;
        } else status_send_lost+=1;

//        if ((millis()-last_order_received)>ORDER_TIMEOUT_MS)
//            state=INIT;
        if ((millis()-last_status_ack)>STATUS_TIMEOUT_MS){
            SerialUSB.println("status timeout");
            state=INIT;
        }
        break;
    }
}

void com_robot_init(){
    com_init(); // default card setup
    state=INIT;
    for(int card=0;card<3;++card){
        com_set_ack(card,true); // set ACK
        com_set_crc(card,2); // 2bytes for CRC
        //com_set_rf(card,SPEED_2M,POW_0db);
        com_clear_status(card);
    }

    card_icmp_ok = com_is_ok(CARD_ICMP);
    card_status_ok = com_is_ok(CARD_STATUS);
    card_order_ok = com_is_ok(CARD_ORDER);


    // use stm32 uid to fix the address:
    uint8_t id[12];
    get_uid(id);
    unsigned int d=micros();
    uint8_t uaddr[5];
    uint8_t icmp_addr[5]=addr_for_icmp;
    for(int i=0;i<5;++i)
        uaddr[i]=id[0]^id[5+i];
    while(uaddr[0]==icmp_addr[0]) uaddr[0]+=1;// First byte must be unique!!!!

    // Set card 0 on receive mode with multiple pipe adresses
    if (card_status_ok){
        com_set_channel(CARD_STATUS,status_chan);
        com_set_state(CARD_STATUS,ComState::OFF);
    }
    if (card_order_ok){
        com_set_channel(CARD_ORDER,orders_chan);
        uint8_t x[5]=addr_for_orders; // address will change according to robot
        com_set_tx_addr(CARD_ORDER,x);
        com_set_rx_addr(CARD_ORDER,0,x);
        com_set_pipe_payload(CARD_ORDER,0,order_payload_size);
        com_set_state(CARD_ORDER,ComState::RX);
    }
    if (card_icmp_ok){
        // Set card 2 on rx mode, waiting for robot dhcp request
        com_set_channel(CARD_ICMP,icmp_chan);
        com_set_tx_addr(CARD_ICMP,icmp_addr);
        com_set_rx_addr(CARD_ICMP,0,icmp_addr);
        // use stm32 uid to fix the address:
        com_set_rx_addr(CARD_ICMP,1,uaddr);
        com_set_pipe_payload(CARD_ICMP,0,icmp_payload_size);
        com_set_pipe_payload(CARD_ICMP,1,icmp_payload_size);
        com_set_state(CARD_ICMP,ComState::TX);
    }

    dhcp_request.icmp_type = ICMP_DHCP_REQUEST;
    dhcp_request.arg = 0;
    com_get_rx_addr(CARD_ICMP,1,dhcp_request.icmp_addr); // get internal pipe 1 address for reply

}
TERMINAL_COMMAND(state, "Get robot state")
{
    terminal_io()->print("robot state is: ");
    terminal_io()->println(state);
    terminal_io()->print("stats: ");
    terminal_io()->print(status_send_ok);
    terminal_io()->print(" ok, ");
    terminal_io()->print(status_send_lost);
    terminal_io()->println(" lost!");
    terminal_io()->print("dhcp failure: ");
    terminal_io()->println(dhcp_failure);
}

