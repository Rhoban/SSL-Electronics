#include <encoder.h>
#include <as5047d.h>
#include <terminal.h>
#include <time.h>
#include <stdint.h>
#include <errors.h>
#include "debug.h"


//
// Clock computation
//
// At 10 Mhz (BaudRate = 10Mb/s, tclk=100ns), 1 packet of 16 bits is 
// received each : 
// 
//    T_2bits = TL + TclkH + 15 * Tclk + TH + Tcsn = 350 + 50 + 15*100 + 50 + 350 = 2300 ns
//
//    Time to a complete comunication :
//    T_com = T_2bits*3 = 6900 ns < 7us
//
// At our spi clock : SYCLK/PRESCALER = 84MHz/16 = 5.35MHz 
// (Baudrate 5.35Mb/s, tclk=190.476 ns)
//    TL = Tcsn = 1000 ns (what is implemented)
//    TclkH = TH = Tclk/2 = 95.238 ns
//     
//    T_2bits = 1000 + 95.238 + 15*190.476 + 95.238 + 1000 = 5048 ns
//    T_com = T_2bits*3 = 15142 ns < 16 us
//   
// The clock prescaler for a clock of 84 MHz is 16 : 84Mhz/16 = 5.25MHz
// (BaudRate = 5.25b/s)
//
static as5047d_t encoder;

void encoder_init(  
  SPI_HandleTypeDef* hspi, GPIO_TypeDef*  gpio_port_cs, uint16_t gpio_pin_cs
){
  as5047d_init( &encoder, hspi, gpio_port_cs, gpio_pin_cs );
}

typedef enum {
  ENCODER_VALUE_NOT_READY = 1,
  ENCODER_DO_NOT_START = 2
} encoder_error_t;

extern TIM_HandleTypeDef htim5;

void start_read_encoder_position(){
  if( !as5047d_start_reading_dynamic_angle(&encoder) ){
    raise_warning(WARNING_ENCODER_BUSY, ENCODER_DO_NOT_START);
  }
}

//
// This function should be as fast as possible, because its
// code is executed with the priority of a SPI interruption.
//
void as5047d_call_back_when_finished(as5047d_t* as5047d){
  if( as5047d == &encoder ){
    // We reset timer 5 to raise an interuption and to execute in a lower priority
    // the rest of the calculus.  
    htim5.Instance->CNT = 0;
  }
}

//
// This function is automatically called with an TIM5 interruption. This TIM5
// interuption is raised by chanel1. This channel is configured in an ouput 
// compare mode.
// This interuption is manually raised by reseting the counter of TIM5i by 
// using the callback function `as5047d_call_back_when_finished()`.
// 
// We don't execute this function inside the as5047d callback because the 
// execution is made under a SPI priority. We want to perform this computation
// calcul with a lower priority.
//
// So, DON'T CALL THIS FUNCTION BY YOURSELF.
// 
void encoder_compute_angle(){
  PRINTJ_PERIODIC(1000, "Calculus");
  if(encoder.error){
    if(
        encoder.error & 
        (AS5047D_ERROR | AS5047D_SPI_ERROR | AS5047D_SPI_CRASH)
    ){
      if(encoder.error & AS5047D_SPI_ERROR){
        raise_error(ERROR_ENCODER_SPI_TRANSMITRECEIVE, encoder.error);
      }
      if(encoder.error & AS5047D_SPI_CRASH){
        raise_error(ERROR_ENCODER_SPI_CRASH, encoder.error);
      }
      if( encoder.error & ~( AS5047D_SPI_ERROR | AS5047D_SPI_CRASH ) ){
        raise_error(ERROR_ENCODER, encoder.error);
      }
    }else{
      raise_warning(WARNING_ENCODER_ERROR_ON_AS5047D, encoder.error);
    }
    return;
  }  
  PRINTJ_PERIODIC(1000, "%d", as5047d_data_to_angle(&encoder) );
}

void encoder_error_spi_call_back(){
  as5047d_error_spi_call_back(&encoder);
}

void encoder_spi_call_back(){
  as5047d_spi_call_back(&encoder);
}

void encoder_tick(){
}

TERMINAL_COMMAND(enc, "Read encoder")
{
  start_read_encoder_position();
}

TERMINAL_COMMAND(diagnostic, "encoder diagnostic")
{
  as5047d_diagnostic_t diagnostic = {0, 0, 0, 0, 0};
  as5047d_start_reading_diagnostic(&encoder);

  while(!encoder.is_ready);
    
  if(!encoder.error){
    as5047d_data_to_diagnostic(&encoder, &diagnostic); 
    terminal_print( "magnetic field strength :" );
    if(diagnostic.mfs_too_low){
      terminal_println(" too low");
    }
    if(diagnostic.mfs_too_high){
      terminal_println(" too high");
    }
    if( ( ! diagnostic.mfs_too_high ) && ( ! diagnostic.mfs_too_low ) ){
      terminal_println(" OK");
    }
    terminal_print( "cordi : " );
    if(diagnostic.cordi_overflow){
      terminal_println("OVERFLOW (the measured angle is not reliable)");
    }else{
      terminal_println("OK");
    }
    terminal_print( "offset comensation : " );
    if(diagnostic.offset_compensation_is_ready){
      terminal_println("OK");
    }else{
      terminal_println("NOT READY");
    }
    terminal_print( "automatic gain control : " );
    terminal_println_int( diagnostic.automatic_gain_control );
  }else{
    terminal_print("Reading error : ");
    terminal_println_int(encoder.error);
  }
}
