#include "encoder.h"
#include "motor_foc.h"
#include "servo_foc.h"
#include "servo_hybrid.h"
#include <terminal.h>

static float speed_csg_float = 0;
static int theta_consign = 0;
static int delta_theta;

static int cnt = 0;

int update_angle(int angle){
    theta_consign += delta_theta;
    return theta_consign;
}

void servo_hybrid_init(){
    register_update_theta(update_angle);
    servo_foc_init();
}

static HybridState old_state = NO_STATE;
static HybridState state = NO_STATE;

void update_hybrid_state(){
    old_state = state;
    float speed = servo_hybrid_get_speed();
    float s_csg = fabs( speed_csg_float );
    switch( state ){
        case NO_STATE:
            if( s_csg <= MIN_NORMAL_HIGH_SPEED_HYSTERESIS ){
                if( s_csg <= MIN_LOW_NORMAL_SPEED_HYSTERESIS ){
                    state = LOW_SPEED;
                }else{
                    state = NORMAL_SPEED;
                }
            }else{
                state = HIGH_SPEED;
            }
            break;
        case LOW_SPEED :
            if( s_csg >= MAX_LOW_NORMAL_SPEED_HYSTERESIS ){
                if( s_csg >= MAX_NORMAL_HIGH_SPEED_HYSTERESIS ){
                    state = HIGH_SPEED;
                }else{
                    state = NORMAL_SPEED;
                }
            }
            break;
        case NORMAL_SPEED:
            if( s_csg <= MIN_LOW_NORMAL_SPEED_HYSTERESIS ){
                if(speed <= MIN_LOW_NORMAL_SPEED_HYSTERESIS){
                    state = LOW_SPEED;
                }
            }
            if( s_csg >= MAX_NORMAL_HIGH_SPEED_HYSTERESIS ){
                state = HIGH_SPEED;
            }
            break;
        case HIGH_SPEED:
            if( s_csg <= MIN_NORMAL_HIGH_SPEED_HYSTERESIS ){
                if( s_csg <= MIN_LOW_NORMAL_SPEED_HYSTERESIS ){
                    if(speed <= MIN_LOW_NORMAL_SPEED_HYSTERESIS){
                        state = LOW_SPEED;
                    }else if(speed <= MIN_NORMAL_HIGH_SPEED_HYSTERESIS){
                        state = NORMAL_SPEED;
                    }
                }else{
                    if(speed <= MIN_NORMAL_HIGH_SPEED_HYSTERESIS){
                        state = NORMAL_SPEED;
                    }
                }
            }
            break;
        default:
            break;
    }
}

void change_motor_mode(){
    if( old_state == state ) return;
    switch( state ){
        case LOW_SPEED:
            set_open_loop(true);
            set_fixed_theta(true);
            theta_consign = rotor_angle();
            direct_quadrature_voltage_set( REFERENCE_VOLTAGE/2, 0);
            break;
        case NORMAL_SPEED:
        case HIGH_SPEED:
            set_fixed_theta(false);
            set_open_loop(false);
            direct_quadrature_voltage_set( 0, 0);
            reset_asservissement();
            break;
        default:
            break;
    }
}

void servo_hybrid_tick(){
    if( ! motor_is_tared() ) return;
    update_hybrid_state();
    change_motor_mode();
    servo_foc_tick();
}
void servo_hybrid_set(bool enable, float targetSpeed, int16_t pwm){
    speed_csg_float = targetSpeed;
    delta_theta = (int)( ( targetSpeed * ENCODER_CNT_SCALE) / MOTOR_FREQUENCE );
    servo_foc_set(enable, targetSpeed, pwm);
}
float servo_hybrid_get_speed(){
    return servo_foc_get_speed();
}
int servo_hybrid_get_pwm(){
    return servo_foc_get_pwm();
}
void servo_hybrid_set_pid(float kp, float ki, float kd){
    servo_foc_set_pid(kp, ki, kd);
}
void servo_hybrid_set_speed_consign( float speed ){
    speed_csg_float = speed;
    delta_theta = (int)( ( speed * ENCODER_CNT_SCALE) / MOTOR_FREQUENCE );
    servo_set_speed_consign_foc(speed);
}
void servo_hybrid_set_flag(){
}
void servo_hybrid_emergency(){
    state = NO_STATE;
    servo_foc_emergency();
}
void servo_hybrid_stop(){
    state = NO_STATE;
    servo_foc_stop();
}

TERMINAL_COMMAND(hyb_set, "Set velocity for hybrid")
{
    servo_hybrid_set(true, atof(argv[0]), CONFIG_PWM);
}

TERMINAL_COMMAND(hyb, "Set velocity for hybrid")
{
    terminal_io()->print("speed_csg : " );
    terminal_io()->println(speed_csg_float);
    terminal_io()->print("state : ");
    terminal_io()->println( hyb_state_str(state) );
}