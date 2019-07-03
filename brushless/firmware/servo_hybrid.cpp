#include "encoder.h"
#include "motor_foc.h"
#include "servo_foc.h"
#include "servo_hybrid.h"
#include <terminal.h>

static float speed_csg_float = 0;
static int theta_origin = 0;
static unsigned int theta_update_cnt = 0;

void init_position(){
    theta_origin = rotor_angle();
    theta_update_cnt = 0;
}

inline int get_angular_position( int update_frequence ){
    static float delay_ratio = .5;
    return theta_origin + (int) (
        (theta_update_cnt+delay_ratio) * speed_csg_float *
        ONE_TURN_THETA
    ) / update_frequence;
}

static int update_frequence = MOTOR_FREQUENCE;

int compute_angular_position( int update_freq ){
    update_frequence = update_freq;
    theta_update_cnt ++;
    return get_angular_position( update_frequence );
}

void save_theta_origin(){
    theta_origin = get_angular_position( update_frequence );
    theta_update_cnt = 0;
}

int compute_angular_position_for_foc(int angle){
    return compute_angular_position( MOTOR_UPDATE_FREQUENCE );
}

void servo_hybrid_init(){
    servo_foc_init();

    ///servo_hall_init();

    ///#ifdef USE_OPEN_LOOP_FOR_HYBRID
    /// register_update_theta( compute_angular_position_for_foc );
    ///#else
    ///      servo_foc_register_get_theta_csg(compute_angular_position);
    ///#endif
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
        default:
            #ifdef USE_OPEN_LOOP_FOR_HYBRID
            set_fixed_theta(true);
            set_open_loop( true );
            direct_quadrature_voltage_set(REFERENCE_VOLTAGE/2, 0);
            #else
            servo_foc_set_manual_speed(false);
            set_foc_angular_position(theta_origin);
            reset_asservissement();
            #endif
            init_position();
            break;
        case NORMAL_SPEED:
        case HIGH_SPEED:
            #ifdef USE_OPEN_LOOP_FOR_HYBRID
            set_fixed_theta(false);
            set_open_loop( false );
            #else
            #endif
            servo_foc_set_manual_speed(true);
            servo_set_speed_consign_foc(speed_csg_float);
            reset_asservissement();
            break;
    }
}

void servo_hybrid_tick(){

  /// update_hybrid_state();
    /// change_motor_mode();
    servo_foc_tick();
    /// servo_hall_tick();
}
void servo_hybrid_set_speed_consign( float speed ){
  if( speed_csg_float != speed ){
    save_theta_origin();
    speed_csg_float = speed;
    servo_set_speed_consign_foc(speed);
  }
}
void servo_hybrid_set(bool enable, float targetSpeed, int16_t pwm){
    servo_hybrid_set_speed_consign(targetSpeed);
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
