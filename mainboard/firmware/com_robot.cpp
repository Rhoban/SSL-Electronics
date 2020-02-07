
void com_robot_init(){
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
