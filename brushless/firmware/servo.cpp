#include <terminal.h>
#include "servo.h"
#include "servo_foc.h"
#include "servo_hall.h"

void servo_set(bool enable, float targetSpeed, int pwm){
  servo_hall_set(enable, targetSpeed, pwm);
  servo_foc_set(enable, targetSpeed, pwm);
}

void servo_set_flag(){
    servo_hall_set_flag();
}

void servo_emergency(){
    servo_hall_emergency();
    servo_foc_emergency();
}

void servo_stop(){
    servo_hall_stop();
    servo_foc_stop();
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


