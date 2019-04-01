#ifndef _ENCODER_H
#define _ENCODER_H

#include "tools.h"

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
// TODO CORRIGE LE CHAMPS MAX_SPEED_ENCODER_CNT AVEC LA BONNE VALEUR !
#define ENCODER_CNT_SCALE 16384
#define MAX_SPEED_ENCODER_CNT 0x8000000
#define MAX_MOTOR_SPEED 128 
#define SPEED_NOMRALISATION (MAX_SPEED_ENCODER_CNT/MAX_MOTOR_SPEED)
#define SPEED_SCALE 64
static_assert( SPEED_SCALE*ENCODER_CNT_SCALE == SPEED_NOMRALISATION, "");

#define ENCODER_FREQUENCE 8000
#define SUB_SAMPLE_FREQUENCE 800
#define SUB_SAMPLE_FACTOR 10
static_assert( SUB_SAMPLE_FREQUENCE*SUB_SAMPLE_FACTOR == ENCODER_FREQUENCE, "Shanon version X10");
// TODO Mettre la contrainte de shanon concerant le sous échantillonage !

int encoder_position();
int encoder_to_speed();

#define MAX_SPEED_MOTOR 50 // turn.s-1
#define MAX_SPEED_SECURITY_FACTOR 4
static_assert(
    IS_POW_2(MAX_SPEED_SECURITY_FACTOR) && MAX_SPEED_SECURITY_FACTOR > 1, 
    ""
);

void encoder_print_errors();

#define ENCODER_CNT_SCALE 16384
#define THETA_OUT_SCALE 1638
#define ENCODER_SPEED_SCALE 16384

#endif
