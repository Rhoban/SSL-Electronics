#pragma once

#include <stm32f4xx_hal.h>

void encoder_init(  
  SPI_HandleTypeDef* hspi, GPIO_TypeDef*  gpio_port_cs, uint16_t gpio_pin_cs
);
void encoder_tick();
void start_read_encoder_position();

float encoder_angle();
void encoder_spi_call_back();

void encoder_error_spi_call_back();

// This function is automatically called by an interruption with a given 
// priority
void encoder_compute_angle();
