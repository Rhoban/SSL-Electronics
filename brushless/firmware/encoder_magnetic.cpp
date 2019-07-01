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

// #ifdef CARTE_CATIE
HardwareSPI encoder(ENCODER_SPI);

// Counter value
static int32_t encoder_cnt = 0;
static uint16_t encoder_magnitude = 0;
#ifdef ENCODER_IS_PRESENT
  static bool encoder_present = true;
#else
  static bool encoder_present = false;
#endif
static bool encoder_is_not_initialized = true;

inline bool parity_16_check( uint16_t x ){
    x ^= x >> 8;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    return (~x) & 1;
}



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

static int sum_speed = 0*ENCODER_SPEED_SCALE;
static int speed_0 = 0*ENCODER_SPEED_SCALE;
static int speed_1 = 0*ENCODER_SPEED_SCALE;
static int speed_2 = 0*ENCODER_SPEED_SCALE;
static int speed_3 = 0*ENCODER_SPEED_SCALE;
static int speed_4 = 0*ENCODER_SPEED_SCALE;
static int speed_5 = 0*ENCODER_SPEED_SCALE;
static int speed_6 = 0*ENCODER_SPEED_SCALE;
static int speed_7 = 0*ENCODER_SPEED_SCALE;


bool encoder_is_ok()
{
    // return encoder_magnitude > 500;
    return true;
}

bool encoder_is_present()
{
    // return encoder_present;
    return true;
}


static bool encoder_has_new_value = false;

static int encoder_read_state = -1;
static uint16_t encoder_read_result = 0;
static uint16_t encoder_read_magnitude = 0;

uint16_t magnetic_value = 0;
static int32_t encoder_deltas = 0;

inline int32_t encoder_compute_delta(uint16_t a, uint16_t b)
{
    int32_t delta = ((int32_t) b) - ((int32_t) a);

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

static bool error_spi_comunication = false;
static bool error_flag = false;
static uint16_t error_register = 0;

static unsigned int error_flag_count = 0;
static unsigned int error_parity_count = 0;
static unsigned int error_value_count = 0;
static unsigned int error_spi_count = 0;

#define SCALE_TO_CLEAR_GPIO_PIN 16
#define SCALE_TO_SET_GPIO_PIN 0

extern "C"
{
void __irq_spi2()
{
    // We just have received 8 bits in the MISO pin.
    // We need to get the data and send other ones in the MOSI pin.
    if (spi_is_tx_empty(SPI2)) {
        switch (encoder_read_state) {
            case 0: {
                is_writting = true;
                // We get the first 8 bits of the magnetic datas send by the 
                // encoder in the MOSI interface.
                encoder_read_magnitude = (SPI2->regs->DR << 8);
                // We send the last 8 bits of the command to 
                // request an angle (0xffff).
                SPI2->regs->DR = 0xff;
                encoder_read_state++;
                break;
            }
            case 1: {
                // We get the last 8 bits of the magnetic datas send by the 
                // encoder in the MOSI interface?
                encoder_read_magnitude |= SPI2->regs->DR;
                #define DATA_MASK 0x3fff
                // We collect the magnitude data (the last 13 bits)
                encoder_magnitude = encoder_read_magnitude & DATA_MASK;
                
                // We start a new communication for a new request 
                GPIOB->regs->BSRR = (1U << ENCODER_INDEX_PIN) << SCALE_TO_SET_GPIO_PIN;
                delay_us(1);
                GPIOB->regs->BSRR = (1U << ENCODER_INDEX_PIN) << SCALE_TO_CLEAR_GPIO_PIN;
                delay_us(1);

                // We want to ask for and magnetic value. The command is 
                // 0x7ffe (P=0 to have a even number of bits, R=1 for a read 
                // command and Address=x3FFE to ask for an magnetic value).
                // we send the first 8bits of the command in the MOSI
                SPI2->regs->DR = 0x7f;
                encoder_read_state++;
                break;
            }
            case 2: {
                // We get the first 8 bits of the angle datas send by the 
                // encoder in the MOSI interface.
                encoder_read_result = (SPI2->regs->DR << 8);
                // We send the last 8 bits of the command to 
                // request a magnetic value (0x7ffe).
                SPI2->regs->DR = 0xfe;
                encoder_read_state++;
                break;
            }
            case 3: {
                // We get the last 8 bits of the angle datas send by the 
                // encoder in the MOSI interface.
                encoder_read_result |= SPI2->regs->DR;
                
                error_spi_comunication = false;
                error_flag = false;
                #if 1
                // We check no error occurs when we sent all
                // the previous commands (angle and magetic read commands)
                // to the encoder
                //if( encoder_read_result & 0x4000 ){
                //    error_flag_count++;
                //    security_set_warning(WARNING_ERROR_FLAG_ENCODER );
                //    error_spi_comunication = true;
                //    error_flag = true;
                //}
                #endif
                // We check the parity of the received datas for the
                // angle value. The magnetic value is not used.
                if(
                    ! parity_16_check(encoder_read_result & 0x7FFF) xor 
                    ( (encoder_read_result & 0x8000) !=0 )
                ){
                    error_parity_count++;
                    security_set_warning(WARNING_ERROR_PARITY_ENCODER);
                    error_spi_comunication = true;
                }
                if( error_spi_comunication ){
                      error_spi_count++;
                }

                // We collect the angle data (the last 13 bits)
                encoder_read_result &= DATA_MASK;
                is_writting = false;
                
//                if( error_flag ){
//                    // We need to clear the error flag 
//                    // We close the comunication
//                    GPIOB->regs->BSRR = (1U << ENCODER_INDEX_PIN) << SCALE_TO_SET_GPIO_PIN;
//                    delay_us(1);
//                    // We start a new one :
//                    GPIOB->regs->BSRR = (1U << ENCODER_INDEX_PIN) << SCALE_TO_CLEAR_GPIO_PIN;
//                    delay_us(1);
//                    // We send the first 8 bits of the Clear Error Flag Command 0x4000 
//                    // (PAR=0, R=1, Add=x0001)
//                    SPI2->regs->DR = 0x40;
//                    encoder_read_state = 4;
//                }else{
                    encoder_read_state = -1;
                    // We close the spi comunication
                    GPIOB->regs->BSRR = (1U << ENCODER_INDEX_PIN) << SCALE_TO_SET_GPIO_PIN;
                    spi_irq_disable(SPI2, SPI_TXE_INTERRUPT);
//                }

                encoder_has_new_value = true;
                break;
            }
            case 4: {
                // We received the 8 first bits of the magnetic command
                encoder_read_magnitude = (SPI2->regs->DR << 8);
                // We send the last 8 bits of the Clear Error Flag Command 
                // (0x4000)
                SPI2->regs->DR = 0x01;
                encoder_read_state++;
                break;
            }
            case 5: {
                // We received the last 8 bits of the magentic command
                encoder_read_magnitude |= SPI2->regs->DR;
                // We collect the magnitude data (the last 13 bits)
                encoder_magnitude = encoder_read_magnitude & DATA_MASK;
                // We send the first 8 bits of the command to 
                // request a magnetic value (0x7ffe).
                SPI2->regs->DR = 0x7f;
                encoder_read_state++;
                break;
            }
            case 6: {
                // We received the first 8 bits of the error datas send by the 
                // encoder in the MOSI interface.
                error_register = (SPI2->regs->DR << 8);
                // We send the last 8 bits of the command to 
                // request a magnetic value (0x7ffe).
                SPI2->regs->DR = 0xfe;
                encoder_read_state++;
            }
            case 7: {
                // We received the last 8 bits of the error datas send by the 
                // encoder in the MOSI interface.
                error_register |= SPI2->regs->DR;
                // We close the spi communication.
                encoder_read_state = -1;
                GPIOB->regs->BSRR = (1U << ENCODER_INDEX_PIN) << SCALE_TO_SET_GPIO_PIN;
                spi_irq_disable(SPI2, SPI_TXE_INTERRUPT);
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
        
        // As described in the AS5048a datasheet 
        // (cf Figure 16, page 16, version V1-09 2016-nov-15)
        // To start the comunication, the Chip Select (CS) pin have to be set 
        // to down.
        GPIOB->regs->BSRR = (1U << ENCODER_INDEX_PIN) << SCALE_TO_CLEAR_GPIO_PIN;
       
        // We need to send a command to read an angle.
        // As described in the AS5048a datasheet 
        // (cf Figure 19 and 22, page 16, version V1-09 2016-nov-15)
        // A read angle command is 0xffff (P=1 to have a even number of bits,
        // R=1 for a read command and Address=x3fff to ask for an angle).
        //
        // As SPI is configured in 8 bits, we send the first 8bits of the
        // command in the MOSI
        SPI2->regs->DR = 0xFF;
        // We now enable an interupption, when the first 8bits will be received
        // from the MISO of the spi interface.
        // The continuation of the comunication is now managed by __irq_spi2()
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
    encoder.begin(SPI_18MHZ, MSBFIRST, SPI_MODE_LOW_FALLING);
    pinMode(ENCODER_SELECT_PIN, OUTPUT);

    init_timer();
    
    encoder_is_not_initialized = true;
    encoder_read_value();
}

static float theta_base_d = 0;

enum Speed_state {
    LOW_SPEED,
    NORMAL_SPEED
};
static Speed_state speed_state;
static float adaptative_encoder_speed = 0;

int encoder_to_speed(){
    //return encoder_speed;
    return adaptative_encoder_speed;
}

float encoder_to_float_speed(){
    return (1.0*encoder_to_speed())/ENCODER_SPEED_SCALE;
}

#define LOW_NORMAL_SPEED_MIN_FLOAT 1.0 
#define LOW_NORMAL_SPEED_MAX_FLOAT 1.5

#define LOW_NORMAL_SPEED_MIN 1048576
#define LOW_NORMAL_SPEED_MAX 1572864

static_assert( LOW_NORMAL_SPEED_MIN <= LOW_NORMAL_SPEED_MIN_FLOAT*ENCODER_SPEED_SCALE, "");
static_assert( LOW_NORMAL_SPEED_MIN_FLOAT*ENCODER_SPEED_SCALE < LOW_NORMAL_SPEED_MIN+1, "");
static_assert( LOW_NORMAL_SPEED_MAX <= LOW_NORMAL_SPEED_MAX_FLOAT*ENCODER_SPEED_SCALE, "");
static_assert( LOW_NORMAL_SPEED_MAX_FLOAT*ENCODER_SPEED_SCALE < LOW_NORMAL_SPEED_MAX+1, "");

void encoder_tick()
{
    if( ! encoder_has_new_value ) return;

    uint16_t er = 0;
    if( is_writting ){
        return ;
    }else{
        if( ! error_spi_comunication ){
            er = encoder_read_result;
        }
    }

    if( encoder_is_not_initialized ){
        if( error_spi_comunication ) return;
        magnetic_value = er;
        encoder_is_not_initialized = false;
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

        switch( speed_state ){
            case LOW_SPEED :
                if( abs(encoder_speed) >= LOW_NORMAL_SPEED_MAX ){
                    speed_state = NORMAL_SPEED;
                }
                break;
            case NORMAL_SPEED:
            default:
                if( abs(encoder_speed) <= LOW_NORMAL_SPEED_MIN ){
                    speed_state = LOW_SPEED;
                }
                break;
        }
        switch( speed_state ){
            case LOW_SPEED :
                adaptative_encoder_speed  = low_8_speed;
                break;
            case NORMAL_SPEED:
            default:
                adaptative_encoder_speed  = encoder_speed;
                break;
        }

        sum_speed -= speed_7;
        speed_7 = speed_6;
        speed_6 = speed_5;
        speed_5 = speed_4;
        speed_4 = speed_3;
        speed_3 = speed_2;
        speed_2 = speed_1;
        speed_1 = speed_0;
        speed_0 = encoder_to_speed();
        sum_speed += speed_0;


        servo_set_flag();
        encoder_counter = 0;
    }


    encoder_deltas = encoder_compute_delta(magnetic_value, er);
    
    #define MAX_DELTA 410 
    static_assert(
        ENCODER_CNT_SCALE * MAX_SPEED_MOTOR * MAX_SPEED_SECURITY_FACTOR 
        < ENCODER_FREQUENCE * MAX_DELTA,
        ""
    );
    static_assert( 2 * MAX_DELTA < ENCODER_CNT_SCALE, "" );
    if( abs(encoder_deltas) > MAX_DELTA ){
        // The angular delta is too big for the motor velocity
        security_set_warning(WARNING_INCOHERENT_PACKET_ENCODER);
        error_value_count++;
    }
    if( error_spi_comunication or abs(encoder_deltas) > MAX_DELTA ){
        // We guess an approximate value for delta using the speed.
        //encoder_deltas = encoder_to_speed()/(ENCODER_FREQUENCE*SPEED_SCALE);
        encoder_deltas = - encoder_to_speed()/(ENCODER_FREQUENCE*SPEED_SCALE);
    }
    magnetic_value = (magnetic_value + encoder_deltas + 0x4000)%(0x4000);
    // TODO Pourquoi -= ? 
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
    float result = ( (float) encoder_to_speed()) / ENCODER_SPEED_SCALE;
    float average = ( (float) sum_speed) / ( 8*ENCODER_SPEED_SCALE);

//    terminal_io()->print("theta_0 : ");
//    terminal_io()->println( ((float)theta_out_0)/0x4000);

//    terminal_io()->print("encoder_cnt : ");
//    terminal_io()->println(encoder_cnt);
//    terminal_io()->print("speed cnt : ");
//    terminal_io()->println(encoder_speed);
//    terminal_io()->print("speed : ");
    terminal_io()->println(result);
    terminal_io()->println(average);
//    terminal_io()->print("low 2 speed : ");
//    terminal_io()->println( ( (float) low_2_speed) / ENCODER_SPEED_SCALE );
//    terminal_io()->print("low 3 speed : ");
//    terminal_io()->println( ( (float) low_3_speed) / ENCODER_SPEED_SCALE );
//    terminal_io()->print("low 4 speed : ");
//    terminal_io()->println( ( (float) low_4_speed) / ENCODER_SPEED_SCALE );
//    terminal_io()->print("low 8 speed : ");
//    terminal_io()->println( ( (float) low_8_speed) / ENCODER_SPEED_SCALE );
}

float encoder_to_turn(){
    return encoder_cnt/(1.0*ONE_TURN_THETA); 
}

int encoder_position(){
    return theta_out_0;
}


void encoder_print_errors(){
    terminal_io()->print("f:");
    terminal_io()->print(error_flag_count);
    terminal_io()->print(" p:");
    terminal_io()->print(error_parity_count);
    terminal_io()->print(" v:");
    terminal_io()->print(error_value_count);
    terminal_io()->print(" s:");
    terminal_io()->print(error_spi_count);
}

TERMINAL_COMMAND(ec_err, "Encoder error")
{
    encoder_print_errors();
    terminal_io()->println("");
}

#else
    void encoder_init(){

}
int encoder_position(){
    return 0;
}

void encoder_tick(){

}

bool encoder_is_present(){
    return true;
}

bool encoder_is_ok(){
    return true;

}

// #endif
#endif


