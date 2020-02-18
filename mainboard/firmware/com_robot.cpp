#include "com_robot.h"
#include "com_proto.h"
#include "hardware.h"
#include <stdlib.h>
#include <watchdog.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include "drivers.h"
#include "kicker.h"
#include "infos.h"
#include "ir.h"
#include "voltage.h"
#include "odometry.h"
#include "kinematic.h"
#include "buzzer.h"

#define INIT 0
#define WAIT_DHCP 1
#define RUNNING 2
#define WAIT_AND_INIT 3

int last_status_ack;
int last_order_received=0;
int icmp_stamp=0;
int state=INIT;
int wait_init_timeout;

static uint8_t my_actions=0x01;

unsigned long int status_send_ok=0;
unsigned long int status_send_lost=0;
unsigned long int dhcp_failure=0;

uint16_t last_order_id=0;

struct icmp_order dhcp_request;


struct packet_master order={0,ACTION_ON,0,0,0,0};

void build_status(struct packet_robot &packet)
{
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
    packet.last_order_id = last_order_id;
}


void apply_order(struct packet_master &master_packet)
{

    //SerialUSB.print("0 apply order:");
   // SerialUSB.println(master_packet.actions);

    last_order_id=max(master_packet.order_id,last_order_id);
    if (master_packet.rid != infos_get_id()){
      //  SerialUSB.print("ID ISSUE");
      //  SerialUSB.println(master_packet.rid);
        return;
    }
    // ensure there is at least 4ms between two order (250Hz order freq)
    if ((millis()-last_order_received)<4)
        return;


//    SerialUSB.print("apply order:");
//    SerialUSB.println(master_packet.actions);

    // Driving wheels
    if ((master_packet.actions & ACTION_ON)) {
        if(developer_mode == false){
            kicker_boost_enable(true);
        }
        if(master_packet.actions & ACTION_TARE_ODOM) {
            odometry_tare((master_packet.x_speed)/1000.0, (master_packet.y_speed)/1000.0, (master_packet.t_speed)/10000.0);
            //odometry_tare(0.0, 0.0, 0.0);

        }else{
            kinematic_set(master_packet.x_speed/1000.0, master_packet.y_speed/1000.0,
                          master_packet.t_speed/1000.0);
        }
        //uint8_t actions = master_packet.actions;

        // TODO !
        // If we want to drible only when IR is activated, we need
        // to uncomment
        // the following line
        //if ((master_packet->actions & ACTION_DRIBBLE) && (ir_present()) ) {
        if ((master_packet.actions & ACTION_DRIBBLE)) {
            drivers_set_safe(4, true, 1000);
        } else {
            drivers_set(4, false, 0);
        }

        // Charging
        if (master_packet.actions & ACTION_CHARGE) {
            kicker_boost_enable(true);
        } else {
            kicker_boost_enable(false);
        }




        // Kicking
        if ((my_actions & ACTION_KICK1) || (my_actions & ACTION_KICK2) || ir_present_now()) {
            if(kicker_cap_voltage() > 80.0){
                bool inverted = infos_kicker_inverted();
                if ((master_packet.actions & ACTION_KICK1) &&
                        !(my_actions & ACTION_KICK1)) {
                    SerialUSB.println("kick1");
                    kicker_kick(inverted ? 0 : 1, master_packet.kickPower*30);
                }

                if ((master_packet.actions & ACTION_KICK2) &&
                        !(my_actions & ACTION_KICK2)) {
                    kicker_kick(inverted ? 1 : 0, master_packet.kickPower*30);
                }

                my_actions = master_packet.actions;
            } else {

                my_actions = master_packet.actions;
                my_actions &= ~(ACTION_KICK1 | ACTION_KICK2);
            }
        }
    } else {
       // SerialUSB.println("ARG!");
        drivers_set(0, false, 0);
        drivers_set(1, false, 0);
        drivers_set(2, false, 0);
        drivers_set(3, false, 0);
        drivers_set(4, false, 0);
        my_actions = 0;
        kicker_boost_enable(false);
    }
}
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



void com_robot_tick(){
    static bool first_order_timeout=true;
    watchdog_feed();
    switch(state){
    case INIT:
        // Send a DHCP Request and wait for answer
        if (com_send(CARD_ICMP,(uint8_t *)&dhcp_request,icmp_payload_size)){
            com_set_state(CARD_ICMP,RX);
            state=WAIT_DHCP;
            icmp_stamp=millis();
          //  SerialUSB.println("wait for dhcp reply");
        } else {
            dhcp_failure+=1;
            com_flush_tx(CARD_ICMP);
            state=WAIT_AND_INIT;
            wait_init_timeout=millis()+100; // wait 100ms
        }
        break;
    case WAIT_AND_INIT:
        if (millis()>wait_init_timeout)
            state=INIT;
        break;
    case WAIT_DHCP:
        if ((millis()-icmp_stamp)>ICMP_TIMEOUT_MS){
            buzzer_play(MELODY_WARNING, false);
            dhcp_failure+=1;
            //SerialUSB.println("dhcp timeout");
            state=INIT;
        } else {
            if ((com_has_data(CARD_ICMP))>=0){
                struct icmp_order o;
                com_receive(CARD_ICMP,(uint8_t *)&o,icmp_payload_size);
                /*
                SerialUSB.print("receive icmp reply type: ");
                SerialUSB.print(o.icmp_type);
                SerialUSB.print(" / ");
                SerialUSB.println(o.arg);
                */
                if (o.icmp_type == ICMP_DHCP_REPLY){
                    if (o.arg == ICMP_FULL){
                        buzzer_play(MELODY_WARNING, false);
                        state=WAIT_AND_INIT;
                        wait_init_timeout=millis()+2000; // wait for 2s
                    } else {
                        buzzer_play(MELODY_BEGIN, false);
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
                    //SerialUSB.println("wrong icmp reply type");
                    state=INIT;
                }
                com_flush_rx(CARD_ICMP);
            }
        }
        break;
    case RUNNING:

//        static int counter=0;
//        if (counter<10){
//            counter+=1;
//            return;
//        }
//        counter=0;

        if (com_has_data(CARD_ORDER)!=-1){
            first_order_timeout=true;
            last_order_received = millis();
            com_receive(CARD_ORDER,(uint8_t *)&order,order_payload_size);

//            SerialUSB.print("order receive: action:");
//            print_byte_as_hex(order.actions);
//            SerialUSB.print(" | kickpow: ");
//            SerialUSB.print(order.kickPower);
//            SerialUSB.print(" | x : ");
//            SerialUSB.print(order.x_speed);
//            SerialUSB.print(" | y : ");
//            SerialUSB.print(order.y_speed);
//            SerialUSB.print(" | t : ");
//            SerialUSB.print(order.t_speed);
//            SerialUSB.println();

            com_flush_rx(CARD_ORDER);
            com_clear_status(CARD_ORDER);

        }

        struct packet_robot status;
        build_status(status);
        if (com_send(CARD_STATUS,(uint8_t *)&status,status_payload_size)){
            last_status_ack=millis();
            status_send_ok+=1;
        } else status_send_lost+=1;

        if ((millis()-last_order_received)>ORDER_TIMEOUT_MS){
            if (first_order_timeout){
                buzzer_play(MELODY_ALERT_FAST, false);
                first_order_timeout=false;
                SerialUSB.println("order timeout");

            }
            //state=INIT;
            order.rid = infos_get_id();
            order.actions=0;
            order.t_speed=0;
            order.x_speed=0;
            order.y_speed=0;
            order.kickPower=0;
        }
        if ((millis()-last_status_ack)>STATUS_TIMEOUT_MS){
            SerialUSB.println("status timeout");
            buzzer_play(MELODY_ALERT_FAST, false);
            state=INIT;
            order.actions=0;
            order.t_speed=0;
            order.x_speed=0;
            order.y_speed=0;
            order.kickPower=0;
        }
        apply_order(order);
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
        com_set_state(CARD_STATUS,OFF);
    }
    if (card_order_ok){
        com_set_channel(CARD_ORDER,orders_chan);
        com_set_tx_addr(CARD_ORDER,uaddr);
        com_set_rx_addr(CARD_ORDER,0,uaddr);
        com_set_pipe_payload(CARD_ORDER,0,order_payload_size);
        com_set_state(CARD_ORDER,RX);
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
        com_set_state(CARD_ICMP,TX);
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

TERMINAL_COMMAND(em, "Emergency")
{
        kinematic_stop();
        for (int k=0; k<5; k++) {
            drivers_set(k, false, 0.0);
        }

    kicker_boost_enable(false);
}

TERMINAL_COMMAND(rdiag,"")
{

    com_robot_init();
}

TERMINAL_COMMAND(order,"apply order on robot: charge kick dribble x y t")
{
    if (argc<6){
        terminal_io()->println("wrong args");
        return;
    }
    order.rid=infos_get_id();
    order.actions=ACTION_ON;
    if (atoi(argv[0])==1)
        order.actions |=ACTION_CHARGE;
    int k=atoi(argv[1]);
    if (k==1)
        order.actions |= ACTION_KICK1;
    if (k==2)
        order.actions |= ACTION_KICK2;
    if (atoi(argv[2])==1)
        order.actions |= ACTION_DRIBBLE;
    order.kickPower=100;
    order.x_speed=atoi(argv[3]);
    order.y_speed=atoi(argv[4]);
    order.t_speed=atoi(argv[5]);
    last_order_received=millis()-4;
//    apply_order(master_packet);
}


