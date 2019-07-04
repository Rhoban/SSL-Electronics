#include "motor.h"
#include "motor_foc.h"
#include "motor_hall.h"


void motor_set(bool enable, int value)
{
    // motor_foc_set(enable, value);
    motor_hall_set(enable, value);
}
