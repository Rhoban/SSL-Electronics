#include <encoder.h>
#include <as5047d.h>
#include <terminal.h>
#include <time.h>
#include <stdint.h>

#include "debug.h"

static as5047d_t encoder;

void encoder_init(  
  SPI_HandleTypeDef* hspi, GPIO_TypeDef*  gpio_port_cs, uint16_t gpio_pin_cs
){
  as5047d_init( &encoder, hspi, gpio_port_cs, gpio_pin_cs );
}

static volatile bool start = false;

void start_read_encoder_position(){
  if( start ){
    PRINTJ("Not finished");
    return;
  }
  start = true;
  if( !as5047d_start_reading_dynamic_angle(&encoder) ){
    terminal_println("Encoder is busy !" );
  }
}

void encoder_spi_call_back(){
  as5047d_spi_call_back(&encoder);
}

TERMINAL_COMMAND(deb, "deb"){
  PRINTT("-");
  PRINTT("b:%d", start);
  PRINTT("e:%d", encoder.error);
  PRINTT("s:%ld", encoder.state);
  PRINTT("r:%d", encoder.is_ready);
  PRINTT("c:%d", encoder.corrected_angle);
}

#include <main.h>
void encoder_tick(){
  if( start && encoder.is_ready ){
    if(!encoder.error){
      PRINTJ_PERIODIC(1000, enc, "%d", encoder.corrected_angle);
    }else{
      FREQJ(2000);
      //PRINTJ_PERIODIC(2000, errn, "E E : %d", encoder.error);
      PRINTJ("E E : %d", encoder.error);
    }
    start = false;
  }
}

TERMINAL_COMMAND(enc, "Read encoder")
{
  start_read_encoder_position();
}

TERMINAL_COMMAND(ev, "ev")
{
  terminal_println_int(encoder.corrected_angle);
}
