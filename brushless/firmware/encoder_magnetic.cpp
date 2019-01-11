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
    int start = millis();

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


static uint32_t angle = 0;
static int32_t speed = 0;

static uint32_t filtered_encoder_cnt = 0;

// Timer frequence set to 8Kh
#define FREQUENCY_HZ 8000 
#define SAMPLE_FACTOR 10 
 
void low_pass_filter(){
    filtered_encoder_cnt = encoder_cnt;
}

static uint32_t sub_sample_cnt = 0;

void sub_sample_angle_and_speed(){
    if( sub_sample_cnt % SAMPLE_FACTOR ){
        sub_sample_cnt++;
        return;
    }
    sub_sample_cnt = 0;

    speed = angle;
    angle = filtered_encoder_cnt;
    speed = (angle - speed)*( FREQUENCY_HZ/SAMPLE_FACTOR );
}

void encoder_irq()
{
    encoder_read();
    low_pass_filter();
    sub_sample_angle_and_speed();
}

static void init_timer()
{
    HardwareTimer timer(4);

    // Configuring timer
    timer.pause();

    // Timer frequence set to FREQUENCY_HZ
    #define TIMER_OVERFLOW 1000
    timer.setPrescaleFactor(
        (CYCLES_PER_MICROSECOND*TIMER_OVERFLOW)/FREQUENCY_HZ
    );
    timer.setOverflow(TIMER_OVERFLOW);

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

#define MAX_ENCODER_CNT 0x100000000
#define HALF_MAX_ENCODER_CNT 0x80000000
float encoder_to_turn(){
    if( filtered_encoder_cnt >= HALF_MAX_ENCODER_CNT ){
        return (angle - MAX_ENCODER_CNT)/16384.0;
    }else{
        return angle/16384.0; 
    }
}

float encoder_to_speed(){
    if( filtered_encoder_cnt >= HALF_MAX_ENCODER_CNT ){
        return (speed - MAX_ENCODER_CNT)/16384.0;
    }else{
        return speed/16384.0; 
    }
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
#endif
