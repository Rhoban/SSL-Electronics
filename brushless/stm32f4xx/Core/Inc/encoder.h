#pragma once

#include <stm32f4xx_hal.h>

typedef enum {
  ENCODER_VALUE_NOT_READY = 1,
  ENCODER_DO_NOT_START = 2,
  LAG_IN_ANGLE_COMPUTATION = 3,
  ANGLE_INCREASE_IS_TOO_BIG=4
} encoder_error_t;

void encoder_init(  
  SPI_HandleTypeDef* hspi, GPIO_TypeDef*  gpio_port_cs, uint16_t gpio_pin_cs
);
void encoder_tick();
void encoder_stop();
void encoder_start();
void start_read_encoder_position();
void encoder_set_origin();

float encoder_angle();
void encoder_spi_call_back();

void encoder_error_spi_call_back();

// This function is automatically called by an interruption with a given 
// priority
void encoder_compute_angle();
