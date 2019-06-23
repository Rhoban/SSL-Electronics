#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include "hardware.h"
#include "com.h"
#include "servo.h"
#include "ssl.h"
#include "motor.h"
#include "security.h"

HardwareSPI slave(SLAVE_SPI);

TERMINAL_PARAMETER_INT(rcv, "Received byte", 0);
TERMINAL_PARAMETER_INT(irqed, "IRQed", 0);
TERMINAL_PARAMETER_INT(ssed, "Slave selected", 0);

static uint8_t frame_sizes[] = {
    sizeof(struct driver_packet_set),
    sizeof(struct driver_packet_params)

};
#define INSTRUCTIONS sizeof(frame_size)
static uint8_t frame[128];
static int frame_size = 0;
static int frame_pos = 0;
static int frame_type = 0xff;

static int last_receive = 0;
static bool controlling = false;

#define COM_READ_PACKET(type) \
    struct type *packet;      \
    packet = (struct type *)frame;

uint32_t save_pwm;

void com_frame_received()
{
    switch (frame_type) {
        case DRIVER_PACKET_SET: {
            // Setting the target speed
            COM_READ_PACKET(driver_packet_set)
            // servo_set(packet->enable, packet->targetSpeed, packet->pwm);
            save_pwm = packet->pwm;
            if(packet->enable){
                if( !motor_is_on() ){
                    motor_set(packet->enable, CONFIG_PWM);
                    launch_tare_motor();
                }
            }else{
                servo_set(false, 0);
            }
            servo_set_speed_consign( packet->targetSpeed );
        }
        break;
        case DRIVER_PACKET_PARAMS: {
            // Setting the PID parameters
            COM_READ_PACKET(driver_packet_params)
            //set_motor_speed_pid(packet->kp, packet->ki, packet->kd);
            const float k_speed_d = 0;
            servo_set_pid(K_SPEED_P, K_SPEED_I, k_speed_d);
        }
        break;
    }
}

// SPI answer
static struct driver_packet_ans answer;
uint8_t *answer_ptr;
size_t answer_pos = 0;

extern "C"
{
void __irq_spi1()
{
    irqed += 1;
    rcv = SPI1->regs->SR;

    if (spi_is_tx_empty(SPI1)) {
        spi_tx_reg(SPI1, answer_ptr[answer_pos++]);
    }
    if (spi_is_rx_nonempty(SPI1)) {
        rcv = spi_rx_reg(SPI1);

        if (frame_type == 0xff && rcv < INSTRUCTIONS) {
            // Reading frame type
            frame_type = rcv;
            frame_size = frame_sizes[frame_type];
        } else {
            // Reading frame data
            if (frame_pos < frame_size) {
                frame[frame_pos++] = rcv;
                if (frame_pos == frame_size) {
                    com_frame_received();

                    last_receive = millis();
                    controlling = true;
                    digitalWrite(LED_PIN, HIGH);
                }
            }
        }
    }
}
}

static void slave_irq()
{
    int is_slave = digitalRead(SLAVE_PIN) == LOW;

    if (is_slave) {
        ssed = 1;
        frame_pos = 0;
        frame_type = 0xff;
        slave.beginSlave(MSBFIRST, 0);

        // Sending the status
        if (security_get_error() == SECURITY_NO_ERROR) {
            answer.status = 0x55;
        } else {
            answer.status = 0x80|security_get_error();
        }
        answer.speed = servo_get_speed();
        answer.pwm = save_pwm;//motor_get_pwm();
        answer_ptr = (uint8_t*)&answer;
        answer_pos = 0;

        spi_tx_reg(SPI1, 0x00);
        spi_irq_enable(slave.c_dev(), SPI_RXNE_INTERRUPT|SPI_TXE_INTERRUPT);
    } else {
        spi_irq_disable(slave.c_dev(), SPI_INTERRUPTS_ALL);
        slave.end();
        pinMode(slave.misoPin(), INPUT_FLOATING);
    }
}


void com_init()
{
    // Enabling remap on SPI1
#if BOARD == GREG
    afio_remap(AFIO_REMAP_SPI1);
#endif
    pinMode(SLAVE_PIN, INPUT_PULLUP);
    attachInterrupt(SLAVE_PIN, slave_irq, CHANGE);

    // Turning led off
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
}

void com_tick()
{
    if (controlling) {
        if (millis() - last_receive > 100) {
            controlling = false;
            digitalWrite(LED_PIN, LOW);
            servo_set(false, 0);
        }
    }
}

TERMINAL_COMMAND(map, "MAPPING COM")
{
    terminal_io()->print("CS : ");
    terminal_io()->print(SLAVE_PIN);
    terminal_io()->print(" - ");
    terminal_io()->println(slave.nssPin());
    terminal_io()->print("MISO : ");
    terminal_io()->print(SLAVE_MISO_PIN);
    terminal_io()->print(" - ");
    terminal_io()->println(slave.misoPin());
    terminal_io()->print("MOSI : ");
    terminal_io()->print(SLAVE_MOSI_PIN);
    terminal_io()->print(" - ");
    terminal_io()->println(slave.mosiPin());
    terminal_io()->print("sck : ");
    terminal_io()->print(SLAVE_CLK_PIN);
    terminal_io()->print(" - ");
    terminal_io()->println(slave.sckPin());


}


TERMINAL_COMMAND(ap, "")
{
    terminal_io()->println(answer_pos);
}
