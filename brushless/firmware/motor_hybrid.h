#ifndef _MOTOR_HYRID_H
#define _MOTOR_HYRID_H 

void motor_hybrid_init();
void motor_hybrid_set(bool enable, int value);
void motor_hybrid_tick();
bool motor_hybrid_is_on();
void launch_tare_motor_hybrid();

#endif
