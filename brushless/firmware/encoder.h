#ifndef _ENCODER_H
#define _ENCODER_H

#include <stdint.h>

void encoder_init();
bool encoder_read();
uint32_t encoder_value();

#endif
