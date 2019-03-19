#include "motor_foc.h"
#include "motor_hybrid.h"
#include <terminal.h>

void motor_hybrid_init(){
    motor_foc_init();
}
void motor_hybrid_set(bool enable, int value){
    motor_foc_set(enable, value);
}

void motor_hybrid_tick(){
    motor_foc_tick();
}
bool motor_hybrid_is_on(){
    return motor_foc_is_on();
}
void launch_tare_motor_hybrid(){
    launch_tare_motor_foc();
}

TERMINAL_COMMAND(hyb_tare, "Tare hybrid")
{
    launch_tare_motor_hybrid();
}

