#ifndef _ENCODER_H
#define _ENCODER_H

#include <stdint.h>

void encoder_init();
bool encoder_read();
bool encoder_is_present();
bool encoder_is_ok();
uint32_t encoder_value();

#endif
