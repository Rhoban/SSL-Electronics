#include "hybrid.h"
#include "servo_hall.h"
#include "servo_foc.h"
#include "motor_hall.h"
#include "motor_foc.h"
#include "servo.h"
#include "ssl.h"
#include "terminal.h"

static HybridMod hybrid_mod = FOC;


void switch_to_hall(){
  enable_motor_foc(false);
  enable_motor_hall(true);
  hybrid_mod = HALL;
}

void switch_to_foc(){
  enable_motor_hall(false);
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
  if(hybrid_mod == HALL){ 
    motor_hall_set(enable, value);
  }else{
    motor_foc_set(enable, value);
  }
}

#define HIGH_SPEED 7.0
#define LOW_SPEED 6.0

void hybrid_process_packet( bool enable, float targetSpeed, int pwm){
  if( targetSpeed >= HIGH_SPEED and hybrid_mod == FOC ){
      switch_to_hall();
  }
  if( targetSpeed <= LOW_SPEED and hybrid_mod == HALL ){
      switch_to_foc();
  }

  if(hybrid_mod == HALL){ 
    servo_hall_set(enable, targetSpeed, pwm);
  }else{
      // TODO : FAIRE AUTREMENT ! 
      save_pwm = pwm;
      if(enable){
        motor_set(enable, CONFIG_PWM);
      }else{
        servo_set(false, 0, 0);
      }
      servo_set_speed_consign_foc( targetSpeed );
  }
} 


int hybrid_get_pwm(){
  if(hybrid_mod == HALL){ 
    return servo_hall_get_pwm();
  }else{
    return save_pwm;
  }
}

void hybrid_tick( ){
  if(hybrid_mod == HALL){ 
    motor_hall_tick();
  }else{
    motor_foc_tick();
  }
}


TERMINAL_COMMAND(toggle_hybrid, "Emergency")
{
  if(hybrid_mod == HALL){ 
    switch_to_foc();
  }else{
    switch_to_hall();
  }
}


