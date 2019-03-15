#include "hardware.h"
#ifdef ENCODER_MAGNETIC
#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include <watchdog.h>
#include "motor.h"
#include "encoder.h"
#include "servo.h"
#include "security.h"

HardwareSPI encoder(ENCODER_SPI);

// Counter value
static int32_t encoder_cnt = 0;
static uint16_t encoder_magnitude = 0;
static bool encoder_present = true;

inline bool parity_16_check( uint16_t x ){
    x ^= x >> 8;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    return (~x) & 1;
}


#define ENCODER_CNT_SCALE 16384
#define THETA_OUT_SCALE 1638
#define ENCODER_SPEED_SCALE 16384
static int theta_out_0 = 0*THETA_OUT_SCALE;
static int theta_out_1 = 0*THETA_OUT_SCALE;
static int theta_out_2 = 0*THETA_OUT_SCALE;
static int theta_out_3 = 0*THETA_OUT_SCALE;
static int theta_out_4 = 0*THETA_OUT_SCALE;
static int theta_out_5 = 0*THETA_OUT_SCALE;
static int theta_out_6 = 0*THETA_OUT_SCALE;
static int theta_out_7 = 0*THETA_OUT_SCALE;
static int theta_out_8 = 0*THETA_OUT_SCALE;
static int encoder_speed = 0*ENCODER_SPEED_SCALE;
static int low_2_speed = 0*ENCODER_SPEED_SCALE;
static int low_3_speed = 0*ENCODER_SPEED_SCALE;
static int low_4_speed = 0*ENCODER_SPEED_SCALE;
static int low_8_speed = 0*ENCODER_SPEED_SCALE;

bool encoder_is_ok()
{
    return encoder_magnitude > 500;
}

bool encoder_is_present()
{
    return encoder_present;
}


static bool encoder_has_new_value = false;

static int encoder_read_state = -1;
static uint16_t encoder_read_result = 0;
static uint16_t encoder_read_magnitude = 0;

uint16_t magnetic_value = 0;
static int32_t encoder_deltas = 0;

inline int32_t encoder_compute_delta(uint16_t a, uint16_t b)
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

static bool encoder_flag = false;


static float theta_3_d = 0.0;
static float theta_2_d = 0.0;
static float theta_1_d = 0.0;
static float theta_0_d = 0.0;

static float encoder_cnt_0_d = 0.0;
static float encoder_cnt_1_d = 0.0;
static float encoder_cnt_2_d = 0.0;
static float encoder_cnt_3_d = 0.0;

float num_d = 0.0;
float den_d = 0.0;

static bool is_writting = true;

static bool error_flag = false;


extern "C"
{
void __irq_spi2()
{
    if (spi_is_tx_empty(SPI2)) {
        switch (encoder_read_state) {
            case 0: {
                is_writting = true;
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
                
                // TODO ! Check thet errors !
                error_flag = false;
                #if 0
                if( encoder_read_magnitude & 0x4000 ){
                    security_set_warning(WARNING_ERROR_FLAG_ENCODER );
                    error_flag = true;
                }
                #endif
                if(
                    ! parity_16_check(encoder_read_magnitude & 0x7FFF) xor 
                    ( (encoder_read_magnitude & 0x8000) !=0 )
                ){
                    security_set_warning(WARNING_ERROR_PARITY_ENCODER);
                    error_flag = true;
                }

                encoder_magnitude = encoder_read_magnitude & 0x3fff;
                encoder_read_result &= 0x3fff;
                encoder_read_state = -1;
                GPIOB->regs->BSRR = (1U << 12);
                spi_irq_disable(SPI2, SPI_TXE_INTERRUPT);
                is_writting = false;

                encoder_has_new_value = true;
                break;
            }
        }
    }
}
}

unsigned int encoder_counter = 0;

// Instruction

static uint16_t encoder_read_value()
{
    if( encoder_flag ){
        security_set_warning(WARNING_ENCODER_LAG);
    }
    encoder_flag = true;
    if (encoder_read_state == -1) {
        encoder_read_state = 0;
        GPIOB->regs->BSRR = (1U << 12) << 16;
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
    encoder_read_value();
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

    encoder_read_value();

    init_timer();
}


static float theta_base_d = 0;

void encoder_tick()
{
    if( ! encoder_has_new_value ) return;

    uint16_t er = 0;
    if( is_writting ){
        return ;
    }else{
        if( ! error_flag ){
            er = encoder_read_result;
        }
    }
  
    // Sous échantillonage ! 
    encoder_counter ++;
    if( encoder_counter >= SUB_SAMPLE_FACTOR ){
        theta_out_8 = theta_out_7;
        theta_out_7 = theta_out_6;
        theta_out_6 = theta_out_5;
        theta_out_5 = theta_out_4;
        theta_out_4 = theta_out_3;
        theta_out_3 = theta_out_2;
        theta_out_2 = theta_out_1;
        theta_out_1 = theta_out_0;
        theta_out_0 = ( (int) (theta_0_d + theta_base_d)); // + theta_base;
        
        encoder_speed = (SPEED_SCALE*SUB_SAMPLE_FREQUENCE)*(
            theta_out_0 - theta_out_1
        );
        low_2_speed = (SPEED_SCALE*SUB_SAMPLE_FREQUENCE/2)*(
            theta_out_0 - theta_out_2
        );
        low_3_speed = (SPEED_SCALE*SUB_SAMPLE_FREQUENCE/3)*(
            theta_out_0 - theta_out_3
        );
        low_4_speed = (SPEED_SCALE*SUB_SAMPLE_FREQUENCE/4)*(
            theta_out_0 - theta_out_4
        );
        low_8_speed = (SPEED_SCALE*SUB_SAMPLE_FREQUENCE/8)*(
            theta_out_0 - theta_out_8
        );
        servo_set_flag();
        encoder_counter = 0;
    }

    encoder_deltas = encoder_compute_delta(magnetic_value, er);
    magnetic_value = (magnetic_value + encoder_deltas + 0x4000)%(0x4000);
    encoder_cnt -= encoder_deltas;

    const float d_0 = 2414.790782274951;

    const float n_0 = 1.0/d_0;
    const float n_1 = 3.0/d_0;
    const float n_2 = 3.0/d_0;
    const float n_3 = 1.0/d_0;

    const float d_1 = +6488.057607935079/d_0;
    const float d_2 = -5845.602032624118/d_0;
    const float d_3 = +1764.3352069639898/d_0;

    encoder_cnt_3_d = encoder_cnt_2_d;
    encoder_cnt_2_d = encoder_cnt_1_d;
    encoder_cnt_1_d = encoder_cnt_0_d;
    encoder_cnt_0_d = encoder_cnt;

    theta_3_d = theta_2_d;
    theta_2_d = theta_1_d;
    theta_1_d = theta_0_d;
    num_d = (
        n_0 * encoder_cnt_0_d +
        n_1 * encoder_cnt_1_d +
        n_2 * encoder_cnt_2_d +
        n_3 * encoder_cnt_3_d
    );

    den_d = (
        d_1 * theta_1_d +
        d_2 * theta_2_d +
        d_3 * theta_3_d
    );
    theta_0_d = num_d + den_d;

    #define MAX_SPEED_MOTOR 50 // turn.s-1
    #define MAX_SPEED_SECURITY_FACTOR 4
    static_assert(
        IS_POW_2(MAX_SPEED_SECURITY_FACTOR) && MAX_SPEED_SECURITY_FACTOR > 1, 
        ""
    );
    #define ANGLE_LIMIT_FACTOR 4
    // We want that the angle of rotation after Te times is biger thatn 
    // then admited frame limits of theta_0_d.
    // MAX_SPEED_MOTOR * MAX_SPEED_SECURITY_FACTOR / SUB_SAMPLE_FREQUENCE >= 1turn / ANGLE_LIMIT_FACTOR
    static_assert(MAX_SPEED_MOTOR*MAX_SPEED_SECURITY_FACTOR * ANGLE_LIMIT_FACTOR >= SUB_SAMPLE_FREQUENCE, " ");
#if 1
    const float angle_limit = ENCODER_CNT_SCALE/ANGLE_LIMIT_FACTOR;
    if( theta_0_d >= angle_limit ){
        encoder_cnt_3_d -= angle_limit;
        encoder_cnt_2_d -= angle_limit;
        encoder_cnt_1_d -= angle_limit;
        encoder_cnt_0_d -= angle_limit;
        encoder_cnt -= angle_limit;
        theta_3_d -= angle_limit;
        theta_2_d -= angle_limit;
        theta_1_d -= angle_limit;
        theta_0_d -= angle_limit;
        theta_base_d += angle_limit;
    }
    if( theta_0_d <= -angle_limit ){
        encoder_cnt_3_d += angle_limit;
        encoder_cnt_2_d += angle_limit;
        encoder_cnt_1_d += angle_limit;
        encoder_cnt_0_d += angle_limit;
        encoder_cnt += angle_limit;
        theta_3_d += angle_limit;
        theta_2_d += angle_limit;
        theta_1_d += angle_limit;
        theta_0_d += angle_limit;
        theta_base_d -= angle_limit;
    }
#endif
    encoder_has_new_value = false;
    encoder_flag = false;
}

uint32_t encoder_value()
{
    return ( encoder_cnt + HALF_MAX_ENCODER_CNT );
}

TERMINAL_COMMAND(eb, "Encoder benchmark")
{
    int start = micros();
    for (int k=0; k<10000; k++) {
        watchdog_feed();
        encoder_read_value();
    }
    terminal_io()->println((micros()-start)/10000.0);
}

TERMINAL_COMMAND(cnt, "Cnt debug")
{
    terminal_io()->println(encoder_cnt);
}

TERMINAL_COMMAND(spd, "Speed debug")
{
    float result = ( (float) encoder_speed) / SPEED_NOMRALISATION;

    terminal_io()->print("theta_0 : ");
    terminal_io()->println( ((float)theta_out_0)/0x4000);

    terminal_io()->print("encoder_cnt : ");
    terminal_io()->println(encoder_cnt);
    terminal_io()->print("speed cnt : ");
    terminal_io()->println(encoder_speed);
    terminal_io()->print("speed : ");
    terminal_io()->println(result);
    terminal_io()->print("low 2 speed : ");
    terminal_io()->println( ( (float) low_2_speed) / SPEED_NOMRALISATION );
    terminal_io()->print("low 3 speed : ");
    terminal_io()->println( ( (float) low_3_speed) / SPEED_NOMRALISATION );
    terminal_io()->print("low 4 speed : ");
    terminal_io()->println( ( (float) low_4_speed) / SPEED_NOMRALISATION );
    terminal_io()->print("low 8 speed : ");
    terminal_io()->println( ( (float) low_8_speed) / SPEED_NOMRALISATION );
}

float encoder_to_turn(){
    return encoder_cnt/16384.0; 
}

int encoder_to_speed(){
    return encoder_speed;
}

int encoder_position(){
    return theta_out_0;
}

#endif
