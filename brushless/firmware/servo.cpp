#include <terminal.h>
#include "servo.h"
#include "servo_foc.h"
#include "servo_hall.h"

void servo_set(bool enable, float targetSpeed, int pwm){
  //servo_hall_set(enable, targetSpeed, pwm);
  servo_foc_set(enable, targetSpeed, pwm);
}

float servo_get_speed(){
    return servo_hall_get_speed();
    //return servo_foc_get_speed();
}

int servo_get_pwm(){
    return servo_hall_get_pwm();
    //return servo_foc_get_pwm();
}

void servo_set_pid(float kp, float ki, float kd){
    servo_hall_set_pid(kp, ki, kd);
    //servo_foc_set_pid(kp, ki, kd);
}

void servo_set_flag(){
    servo_hall_set_flag();
}

void servo_emergency(){
    servo_hall_emergency();
    //servo_foc_emergency();
}

void servo_stop(){
    servo_hall_stop();
    //servo_foc_stop();
}

void servo_set_speed_consign( float speed ){
    servo_set_speed_consign_foc(speed);
}

TERMINAL_COMMAND(em, "Emergency")
{
    servo_emergency();
}

TERMINAL_COMMAND(st, "Stop")
{
    servo_stop();
}

TERMINAL_COMMAND(term, "term test")
{
    terminal_io()->println("term works !");
}


