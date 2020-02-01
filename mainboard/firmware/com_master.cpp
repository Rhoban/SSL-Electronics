#include "com_master.h"
#include "nrf24l01.h"


#define STATE_INIT     0
#define STATE_MAGIC1   1
#define STATE_SWITCH   2
#define STATE_ORDER    3

#define USBMODE_BIN 1
#define USBMODE_TERM 2
int usbcom_mode = USBMODE_TERM;

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
/*
        terminal_io()->print("com usb tick read ");
        terminal_io()->println(c);
        terminal_io()->print("state is ");
        terminal_io()->println(state);
        terminal_io()->print("magic pos ");
        terminal_io()->println(magic_pos);
        terminal_io()->print("usb mode is  ");
        terminal_io()->println(usbcom_mode);
*/
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

#define RECV_TEST 0

void com_master_init(){
   com_init();
    for(int card=0;card<3;++card){
        com_set_reg(card, REG_CONFIG, 0x0C);
        com_set_reg(card, REG_SETUP_RETR, 0x5F);
        com_set_reg(card, REG_RF_SETUP, 0x21);
        com_set_reg(card, REG_RF_SETUP, 0x01);
        com_set_reg(card, REG_FEATURE, 0x00);
        com_set_reg(card, REG_DYNPD, 0x00);
        com_set_reg(card, REG_STATUS, 0x70);
        com_set_reg(card, REG_RF_CH, 0x4C);
        com_set_reg(card, REG_CONFIG, 0x0E);
        com_set_reg(card, REG_CONFIG, 0x0E);
        com_set_reg(card, REG_RF_CH, 0x6E);
        com_set_reg(card, REG_RF_SETUP, 0x09);
        com_set_reg(card, REG_RF_SETUP, 0x09);
        com_set_reg(card, REG_CONFIG, 0x0E);
        com_set_reg(card, REG_EN_AA, 0x3F);
        // com_set_reg(card, REG_RX_ADDR_P0, 0xE7);
        // com_set_reg(card, REG_RX_ADDR_P1, 0xE7);
        // com_set_reg(card, REG_RX_ADDR_P2, 0xE7);
        // com_set_reg(card, REG_RX_ADDR_P3, 0xE7);
        // com_set_reg(card, REG_RX_ADDR_P4, 0xE7);
        // com_set_reg(card, REG_TX_ADDR_P0, 0xE7);
        // com_set_reg(card, REG_TX_ADDR_P1, 0xE7);
        // com_set_reg(card, REG_TX_ADDR_P2, 0xE7);
        // com_set_reg(card, REG_TX_ADDR_P3, 0xE7);
        // com_set_reg(card, REG_TX_ADDR_P4, 0xE7);
        uint8_t addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
        com_set_reg5(card, REG_TX_ADDR, addr);
        com_set_reg(card, REG_RX_PW_P0, 0x20);
        if(RECV_TEST)
            com_set_reg(card, REG_CONFIG, 0x0F);
        else
            com_set_reg(card, REG_CONFIG, 0x0E);

        com_set_reg(card, REG_EN_RXADDR, 0x03);
        com_ce_enable(card);

        // set_ack(card,false);
        // com_ce_disable(card); // force standby I mode
        // set_ack(card,false); // set ACK
        // set_crc(card,2); // 2bytes for CRC
        // //for(int pipe=0;pipe<6;pipe++){ // enable all pipe with a payload of 32bytes
        // //    set_pipe_payload(card,pipe,32);
        // //}
        // set_pipe_payload(card,0,32);
        // set_retransmission(card,0,15);
        // set_channel(card,110);
        // clear_status(card);
    }

}


void com_master_tick(){
    watchdog_feed();

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

TERMINAL_COMMAND(chan, "Display or set used channel: chan [id] [chan]")
{
    if (argc!=2){
    for(int k=0;k<3;++k){
        terminal_io()->print(k);
        terminal_io()->print(":");
        terminal_io()->println(get_channel(k));
    }}
    else {
        int id=atoi(argv[0]);
        uint8_t chan=atoi(argv[1]);
        set_channel(id,chan);
    }
}
TERMINAL_COMMAND(ack, "Display or set ack support: ack card [O|1]")
{
    if (argc==0){
    for(int k=0;k<3;++k){
        terminal_io()->print(k);
        terminal_io()->print(":");
        terminal_io()->println(get_ack(k));
    }} else if (argc!=2){
        terminal_io()->println("wrong arg");
    } else {
        int c=atoi(argv[0]);
        bool b=atoi(argv[1]);
        set_ack(c,b);
    }
}

TERMINAL_COMMAND(recv, "check if data received")
{
    for(int k=0;k<3;++k){
        terminal_io()->print(k);
        terminal_io()->print(": ");
        int data = has_data(k);
        terminal_io()->print(data);
        if (data>=0){
            char pl[32];
            receive(k,(uint8_t *)pl,32);
            terminal_io()->print(" data[");
            terminal_io()->print(pl);
            terminal_io()->print("]");
            com_flush_rx(k);
            clear_status(k);
        }
        terminal_io()->println();
    }
}

TERMINAL_COMMAND(retr, "Display or set retransmission: retr card delay count")
{
    if (argc==0){
    for(int k=0;k<3;++k){
        terminal_io()->print(k);
        terminal_io()->print(": ");
        int d=get_retransmission_delay(k);
        terminal_io()->print(d);
        terminal_io()->print(" (");
        terminal_io()->print(250+250*d);
        terminal_io()->print(") ");
        terminal_io()->println(get_retransmission_count(k));
    }} else if (argc!=3){
        terminal_io()->println("wrong arg");
    } else {
        int c=atoi(argv[0]);
        int b=atoi(argv[1]);
        int f=atoi(argv[2]);
        set_retransmission(c,b,f);
    }
}

TERMINAL_COMMAND(send, "send card msg")
{
    if (argc!=2)
        terminal_io()->println("wrong number of args");
    else{
        int card=atoi(argv[0]);
        char payload[32]={'\0'};
        for(int i=0;i<32 && argv[1][i]!='\0';++i) payload[i]=argv[1][i];
        send(card,(uint8_t *)payload,32);
    }
}

TERMINAL_COMMAND(status, "status")
{
    for(int k=0;k<3;++k){
        terminal_io()->print(k);
        terminal_io()->print(":");
        uint8_t c=get_status(k);
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
        terminal_io()->print(" :");
        c=get_fifo_status(k);
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
}

TERMINAL_COMMAND(rst, "reset status rst [card]"){
    if (argc==0){
        for(int k=0;k<3;++k)
            reset_status(k);
    } else {
        int c=atoi(argv[1]);
        reset_status(c);
    }

}
TERMINAL_COMMAND(setup, "display rf setup"){
    for(int k=0;k<3;++k){
        terminal_io()->print(k);
        terminal_io()->print(":");
        uint8_t s=get_rf_setup(k);

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
}

TERMINAL_COMMAND(config, "Display ack support")
{
    for(int k=0;k<3;++k){
        terminal_io()->print(k);
        terminal_io()->print(":");
        uint8_t conf=get_config(k);
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
}

void print_byte_as_hex(uint8 v){
    uint8 a=(v>>4);
    if (a<10)
        terminal_io()->print(a);
    else
        terminal_io()->print((char )('A'+a-10));
    a=v&0x0F;
    if (a<10)
        terminal_io()->print(a);
    else
        terminal_io()->print((char )('A'+a-10));
}

void print_addr(struct addr5 a){
    for(int i=0;i<5;++i){
        print_byte_as_hex(a.addr[i]);
        if (i<4)
            terminal_io()->print("-");
    }
    terminal_io()->print("  ");
    for(int i=0;i<5;++i){
        terminal_io()->print(a.addr[i]);
        if (i<4)
            terminal_io()->print(".");
    }
}

TERMINAL_COMMAND(addr, "display or set com address: addr [id] nb.nb.nb.nb.nb")
{
    if (argc==0){
        for(int k=0;k<3;++k)
        {
            terminal_io()->print(k);
            terminal_io()->println(":");
            terminal_io()->print("  tx:   ");
            struct addr5 a=com_get_tx_addr(k);
            print_addr(a);
            terminal_io()->println();
            for(int pipe=0;pipe<6;pipe++){
                terminal_io()->print("  rx(");
                terminal_io()->print(pipe);
                terminal_io()->print("):");
                print_addr(com_get_rx_addr(k,pipe));
                terminal_io()->println();
            }
        }
    } else if (argc!=2){
        terminal_io()->println("invalid number of args");
    } else {
        int id=atoi(argv[0]);
        struct addr5 a;
        int l=0;
        int s=0;
        int k=0;
        while((k<5) && (argv[1][l]!='\0')){
            if (argv[1][l]=='.') {
                argv[1][l]='\0';
                a.addr[k]=atoi(argv[1]+s);
                s=l+1;
                k+=1;
            }
            l+=1;
        }
        if (k!=4){
            terminal_io()->println("invalid address");
        } else {
            a.addr[k] = atoi(argv[1]+s);
            com_set_tx_addr(id, a);
            com_set_rx_addr(id,0, a);
        }
    }

}

TERMINAL_COMMAND(obs, "display transmission stats")
{
    for(int k=0;k<3;++k)
    {
        terminal_io()->print(k);
        terminal_io()->print(": ");
        terminal_io()->print(get_lost_count(k));
        terminal_io()->print(" lost ");
        terminal_io()->print(get_retransmitted_count(k));
        terminal_io()->println(" retransmitted ");
}
}

TERMINAL_COMMAND(rxp, "display or set pipe payload, 0 means not active")
{
    if (argc==0){
        for(int k=0;k<3;++k)
        {
            terminal_io()->print(k);
            terminal_io()->print(": ");
            for(int pipe=0;pipe<6;pipe++){
                terminal_io()->print(pipe);
                int pl=get_pipe_payload(k,pipe);
                if (pl==0) terminal_io()->print("[disabled] ");
                else {
                    terminal_io()->print("[");
                    terminal_io()->print(pl);
                    terminal_io()->print("] ");
                }
            }
            terminal_io()->println();
        }
    } else if (argc!=3){
        terminal_io()->println("wrong number of arguments: card pipe payloadsize");
    }
    else{
        int c=atoi(argv[0]);
        int p=atoi(argv[1]);
        int pl=atoi(argv[2]);
        set_pipe_payload(c,p,pl);
    }
}

// TERMINAL_COMMAND(dbg, "dump all informations")
// {
// terminal_io()->println("setup:");
// terminal_command_setup(0,nullptr);
// terminal_io()->println("config:");
// terminal_command_config(0,nullptr);
// terminal_io()->println("status:");
// terminal_command_status(0,nullptr);
// terminal_io()->println("pipe pyload:");
// terminal_command_rxp(0,nullptr);
// terminal_io()->println("observ:");
// terminal_command_obs(0,nullptr);
// terminal_io()->println("addr:");
// terminal_command_addr(0,nullptr);
// }

