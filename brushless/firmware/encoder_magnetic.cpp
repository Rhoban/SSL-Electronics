#include "hardware.h"
#ifdef ENCODER_MAGNETIC
#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include <watchdog.h>
#include "motor.h"
#include "encoder.h"
#include "servo.h"

HardwareSPI encoder(ENCODER_SPI);

// Counter value
static uint32_t encoder_cnt = 0;
static uint16_t encoder_magnitude = 0;
static bool encoder_present = true;

bool encoder_is_ok()
{
    return encoder_magnitude > 500;
}

bool encoder_is_present()
{
    return encoder_present;
}


static int encoder_read_state = -1;
static uint16_t encoder_read_result = 0;
static uint16_t encoder_read_magnitude = 0;
static bool encoder_waiting_new_value = false;
static bool encoder_has_new_value = false;

extern "C"
{
void __irq_spi2()
{
    if (spi_is_tx_empty(SPI2)) {
        switch (encoder_read_state) {
            case 0: {
                encoder_read_result = (SPI2->regs->DR << 8);
                SPI2->regs->DR = 0xfe;
                encoder_read_state++;
                break;
            }
            case 1: {
                encoder_read_result |= SPI2->regs->DR;
                GPIOB->regs->BSRR = (1U << 12);
                delay_us(1);
                GPIOB->regs->BSRR = (1U << 12) << 16;
                delay_us(1);
                SPI2->regs->DR = 0xff;
                encoder_read_state++;
                break;
            }
            case 2: {
                encoder_read_magnitude = (SPI2->regs->DR << 8);
                SPI2->regs->DR = 0xff;
                encoder_read_state++;
                break;
            }
            case 3: {
                encoder_read_magnitude |= SPI2->regs->DR;
                encoder_magnitude = encoder_read_magnitude & 0x3fff;
                encoder_read_result &= 0x3fff;
                encoder_read_state = -1;
                GPIOB->regs->BSRR = (1U << 12);
                spi_irq_disable(SPI2, SPI_TXE_INTERRUPT);
                encoder_has_new_value = true;
                break;
            }
        }
    }
}
}

// Instruction
static uint16_t encoder_read_value()
{
    if (encoder_read_state == -1) {
        encoder_read_state = 0;
        GPIOB->regs->BSRR = (1U << 12) << 16;
        encoder_has_new_value = false;
        SPI2->regs->DR = 0x7f;
        spi_irq_enable(SPI2, SPI_TXE_INTERRUPT);
    }

    return 0;
}

TERMINAL_COMMAND(erv, "Encoder Read Value")
{
    while (!SerialUSB.available()) {
        encoder_read_value();
        while (encoder_read_state != -1) {
            watchdog_feed();
        }

        SerialUSB.print(encoder_read_result);
        SerialUSB.print(" (");
        SerialUSB.print(encoder_magnitude);
        SerialUSB.print(")");
        SerialUSB.println();

        delay(5);
        watchdog_feed();
    }
}

void encoder_irq()
{
    encoder_read();
}

static void init_timer()
{
    HardwareTimer timer(4);

    // Configuring timer
    timer.pause();
    timer.setPrescaleFactor(9);
    timer.setOverflow(1000); // 8Khz

    timer.setChannel4Mode(TIMER_OUTPUT_COMPARE);
    timer.setCompare(TIMER_CH4, 1);
    timer.attachCompare4Interrupt(encoder_irq);

    timer.refresh();
    timer.resume();
}

void encoder_init()
{
    // Initializing pins
    digitalWrite(ENCODER_SELECT_PIN, HIGH);
    encoder.begin(SPI_9MHZ, MSBFIRST, SPI_MODE_LOW_FALLING);
    pinMode(ENCODER_SELECT_PIN, OUTPUT);

    encoder_read();

    init_timer();
}

uint16_t magnetic_value = 0;
uint8_t sample = 0;
uint16_t average_sample = 0;

static int32_t encoder_deltas = 0;
static uint8_t encoder_delta_pos = 0;

int32_t encoder_compute_delta(uint16_t a, uint16_t b)
{
    int32_t delta = b - a;

    if (delta > 0x1fff) {
        delta -= 0x4000;
    }
    if (delta < -0x1fff) {
        delta += 0x4000;
    }

    return delta;
}

void encoder_read()
{
    // Trigger the reading of a new value
    encoder_has_new_value = false;
    encoder_waiting_new_value = true;
    encoder_read_value();
}


#define SPEED_FACTOR ( ( ( MAX_SPEED_ENCODER_CNT/ENCODER_CPR ) * (1000/SPEED_DT) * 5 ) / MAX_MOTOR_SPEED )

// Number of values stored in the speed ring buffer
#define SPEED_RB        (((SPEED_DT)/SERVO_DT)+1)
static int encoder_rb_1[SPEED_RB] = {0};
static int encoder_pos_1 = 0;
static int encoder_speed = 0;

void compute_rotor_velocity(){
    // Storing current value
    int current_value = encoder_value();
    encoder_rb_1[encoder_pos_1] = current_value;
    encoder_pos_1++;
    if (encoder_pos_1 >= SPEED_RB) {
        encoder_pos_1 = 0;
    }
    int past_value = encoder_rb_1[encoder_pos_1];

    // Updating current speed estimation [pulse per SPEED_DT]
    // XXX: Is there a problem when we overflowed?
    #ifdef REVERSE_PHASE
    int speed_pulse = -( current_value - past_value );  //REVERSE !
    #else
    int speed_pulse = ( current_value - past_value );  //REVERSE !
    #endif

    // Converting this into a speed [turn/s]
    // XXX: The discount was not tuned properly
    encoder_speed = ( 95 * encoder_speed + SPEED_FACTOR * speed_pulse )/100;
}

void encoder_tick()
{
    if (encoder_has_new_value) {
        encoder_has_new_value = false;
        encoder_deltas += encoder_compute_delta(magnetic_value, encoder_read_result);
        encoder_delta_pos++;

        if (encoder_delta_pos >= 8) {
            encoder_deltas /= 8;
            magnetic_value = (magnetic_value + encoder_deltas + 0x4000)%(0x4000);

            encoder_cnt -= encoder_deltas;
        
            compute_rotor_velocity();

            encoder_deltas = 0;
            encoder_delta_pos = 0;
            servo_set_flag();
        }
    }
}

uint32_t encoder_value()
{
    return encoder_cnt;
}

TERMINAL_COMMAND(eb, "Encoder benchmark")
{
    int start = micros();
    for (int k=0; k<10000; k++) {
        watchdog_feed();
        encoder_read();
    }
    terminal_io()->println((micros()-start)/10000.0);
}

TERMINAL_COMMAND(cnt, "Cnt debug")
{
    terminal_io()->println(encoder_cnt);
}

TERMINAL_COMMAND(spd, "Speed debug")
{
    float result = encoder_speed / ( (double) SPEED_NOMRALISATION );
    terminal_io()->print("speed cnt : ");
    terminal_io()->println(encoder_speed);
    terminal_io()->print("speed : ");
    terminal_io()->println(result);
}

float encoder_to_turn(){
    if( encoder_cnt >= HALF_MAX_ENCODER_CNT ){
        return (encoder_cnt - MAX_ENCODER_CNT)/16384.0;
    }else{
        return encoder_cnt/16384.0; 
    }
}

int encoder_to_speed(){
    return encoder_speed;
}

int encoder_to_int(){
    if( encoder_cnt >= HALF_MAX_ENCODER_CNT ){
        return (encoder_cnt - MAX_ENCODER_CNT);
    }else{
        return encoder_cnt;
    }
}

#endif
