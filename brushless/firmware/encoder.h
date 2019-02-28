#ifndef _ENCODER_H
#define _ENCODER_H

#include <stdint.h>

void encoder_init();
void encoder_read();
bool encoder_is_present();
bool encoder_is_ok();
uint32_t encoder_value();
void encoder_tick();
float encoder_to_turn();

#define MAX_ENCODER_CNT 0x100000000
#define HALF_MAX_ENCODER_CNT 0x8000000

//
// We want to use motor whose velocity < 200 tr/s so 
// velocity from [-200, 200] should be mapped in [-0x8000000,0x8000000] 
//
#define MAX_SPEED_ENCODER_CNT 0x8000000
#define MAX_MOTOR_SPEED 128 
#define SPEED_NOMRALISATION (MAX_SPEED_ENCODER_CNT/MAX_MOTOR_SPEED)

int encoder_to_int();
int encoder_to_speed();

#endif
