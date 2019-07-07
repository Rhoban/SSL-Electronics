#include "hybrid.h"
#include "servo_hall.h"
#include "servo_foc.h"
#include "motor_hall.h"
#include "motor_foc.h"
#include "servo.h"
#include "ssl.h"
#include "terminal.h"
#include "encoder.h"

static HybridMod hybrid_mod = FOC;

#define MAX_ACC 0.1;
static float _max_acc = MAX_ACC; 

static inline float _quadramp(float target_speed, float dt){
	static float _current_speed = 0.0;
	
  if (dt == 0){ //just to be shure
		return _current_speed;
	}

	float acc = ((target_speed - _current_speed)) / (float)dt;	
	float max_acc = _max_acc;

#if 0
	//increase quadramp if stop required
	if (_enable) {
			max_acc = _max_acc;
	}
	else{
			max_acc = _max_acc *  MOTOR_CONTROL_QUAD_FACTOR ;
	}
#endif


	if ((acc > -max_acc) && (acc < max_acc)){//acceleration below limmit
		_current_speed = target_speed;
		return target_speed;
	}
	else{ //quadramp limit
		if (acc > 0){ //increse speed
			_current_speed += max_acc * (float)dt;
		}
		else{ //decrease speed
			_current_speed -= max_acc * (float)dt;
		}
		return _current_speed;
	}
	return 0.0; //never reached
}


void switch_to_hall(){
  enable_motor_foc(false);
  reset_hall_error();
  set_hall_motor_pwm( get_max_foc_pwm() );
  enable_motor_hall(true);
  hybrid_mod = HALL;
}

void switch_to_foc(){
  enable_motor_hall(false);
  reset_foc_error();
  enable_motor_foc(true);
  hybrid_mod = FOC;
}

float hybrid_get_speed(){
  if(hybrid_mod == HALL){ 
    return servo_hall_get_speed();
  }else{
    return servo_foc_get_speed();
  }
}

static int save_pwm = 0;

void motor_set(bool enable, int value)
{
  //if(hybrid_mod == HALL){ 
    motor_hall_set(enable, value);
  //}else{
    motor_foc_set(enable, value);
 // }
}

#define HIGH_SPEED 7.0
#define LOW_SPEED 4.0
#define LIMIT_SPEED 20.0

static bool enable_velocity = false;
static float hybrid_target_speed = 0;
int pwm_velocity = 0;
static float quad_speed;
void update_velocity(float dt){
    if( dt == 0 ) return ;
    quad_speed = _quadramp(hybrid_target_speed, dt);
    //quad_speed = hybrid_target_speed ; //, dt);

    if( fabs(quad_speed) >= HIGH_SPEED and hybrid_mod == FOC ){
        switch_to_hall();
    }
    if( fabs(quad_speed) <= LOW_SPEED and hybrid_mod == HALL ){
        switch_to_foc();
    }

    servo_hall_set(enable_velocity, quad_speed, pwm_velocity);
    save_pwm = pwm_velocity;
    //motor_hall_set(enable_velocity, pwm);
    //motor_set(enable_velocity, CONFIG_PWM);
    motor_foc_set(enable_velocity, CONFIG_PWM);
    servo_set_speed_consign_foc( quad_speed );
}

TERMINAL_COMMAND(hl, "Hybrid mode")
{
  terminal_io()->print("ts : ");
  terminal_io()->println(hybrid_target_speed);
  terminal_io()->print("qs : ");
  terminal_io()->println(quad_speed);
  if( hybrid_mod == HALL ){
    terminal_io()->println("HALL");
  }else{
    terminal_io()->println("FOC");
  }
}


void hybrid_process_packet( bool enable, float targetSpeed, int pwm){
    enable_velocity = enable;
    pwm_velocity = pwm;
    if( fabs(targetSpeed) > LIMIT_SPEED ){
      if( targetSpeed > 0 ){
        targetSpeed = LIMIT_SPEED;
      }else{
        targetSpeed = - LIMIT_SPEED;
      }
    }
    hybrid_target_speed = targetSpeed;
} 


int hybrid_get_pwm(){
  if(hybrid_mod == HALL){ 
    return servo_hall_get_pwm();
  }else{
    return get_max_foc_pwm();
  }
}


void hybrid_tick( ){

  static int last_time = millis();

  int time = millis();

  int dt = time-last_time;
  if( dt > 0 ){
    update_velocity(dt/1000.0);
    last_time = time;
  }
  motor_hall_tick();
  motor_foc_tick();
}


TERMINAL_COMMAND(toggle_hybrid, "Emergency")
{
  if(hybrid_mod == HALL){ 
    switch_to_foc();
  }else{
    switch_to_hall();
  }
}

TERMINAL_COMMAND(hyb_mod, "Hybrid mode")
{
  if( hybrid_mod == HALL ){
    terminal_io()->println("HALL");
  }else{
    terminal_io()->println("FOC");
  }
}
