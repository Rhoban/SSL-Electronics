#ifndef _MOTOR_H
#define _MOTOR_H

#include "ssl.h"

void motor_init();
void motor_set(bool enable, int value);
void motor_tick();
bool motor_is_on();

#endif
