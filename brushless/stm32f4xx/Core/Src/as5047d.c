/*
 * Copyright (C) 2019  Adrien Boussicault <adrien.boussicault@labri.fr>
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include <as5047d.h>
#include <time.h>
#include <terminal.h>
#include "debug.h"

// AM5 AS5047D SPI comunication

// Macro to compute the parity of a packet
#define ROP(u,v) (u^(u>>v))
#define REDUCE8(u) ROP(u,8)
#define REDUCE4(u) ROP(REDUCE8(u),4)
#define REDUCE2(u) ROP(REDUCE4(u),2)
#define REDUCE1(u) ROP(REDUCE2(u),1)
#define PARITY16(u) (REDUCE1(u) & 1)

// Macro to add ta parity bit in a packet
#define ADD_PARITY_BIT(packet, bit)  (packet | (PARITY16(packet)<<bit))

// The size in number of Bytes of a packet.
#define PACKET_SIZE 2

// Macro to extract the first 8 bits of a packet
#define FIRST(val) (uint8_t)(val >> 8)
// Macro to extract the last 8 bits of a packet
#define LAST(val) (uint8_t)(0xFF & val)
// Macro to concatenate the two byte of a packet
#define CONCATENATE(first, last) ((first << 8) | ( last )) 

#define RMASK(nb) ( ~( ((~0u) >> nb) << nb ) )
#define LMASK(nb) ( ~( ((~0u) << nb) >> nb ) )

//
// Bits defintion of the SPI Command Frame
//
// Those definition are usefull to build packet containing a read or write 
// command to transmit to the device.
// 
#define PARC_BIT 15 //Parity bit (even) calculated on the lower 15 bits of
                // command frame
#define RW_BIT 14 //Parity bit to define a R/W command. (0:Write, 1:Read)
  #define READ 1
  #define WRITE 0
#define ADRESS_SIZE 14 // Address to read or write


// Macro to build a packet containing SPI Read command.
#define RCMD(add) ADD_PARITY_BIT(add, PARC_BIT)
// Macro to build a packet containing SPI Read command.
#define WCMD(add) ADD_PARITY_BIT((add | (1u<<RW_BIT)), PARC_BIT)

//
// Bits defintion of the SPI Read Data  Frame
//
// Those definition are usefull to decode packet sent from the device.
// 
#define PARD_BIT 15 //Parity bit (even) calculated on the lower 15 bits of
                // command frame
#define EF_BIT 14 //Parity bit of error frame : 
              // 0: No command frame error occurred
              // 1: Error occurred
  #define FRAME_COMMAND_ERROR 1
  #define NO_FRAME_COMMAND_ERROR 0
#define DATA_SIZE 14 // Size of the data

//
// Bits defintion of the SPI Write Data  Frame
//
// Those definition are usefull to build packet contaning data followin a write
// command (see the part of SPI Command Frame).

// Parity bit : Same as PARD defined in SPI Read Data Frame
// data size : same ad DATA_SIZE defined in SPI Read Data Frame
  #define WRITE_LOW_BIT 14 // This bit should always be LOW.

// 
// Adress conatined in a SPI command Frame
//
#define NOP_ADD 0x0000 // No Operation
#define ERRFL_ADD 0x0001 // Error register
#define PROG_ADD 0x0003 // Programming register
#define DIAAGC_ADD 0x3FFC // Diagnostic and AGC
#define MAG_ADD 0x3FFD // CORDIC magnitude
#define ANGLEUNC_ADD 0x3FFF // Measure angle without dynamic 
                        // angle error compensation
#define ANGLECOM_ADD 0x3FFE // Measure angle with dynamic 
                        // angle error compensation

//
// Bits defintion of an ERRFL Read Data
//
#define PARERR_BIT 2 // Parity error (bit position)
#define INVCOMM_BIT 1
  // Invalid command error: set to 1 by reading or writing
  // an invalid register address
#define FRERR_BIT 0
  // Framing error: is set to 1 when a non-compliant SPI
  // frame is detected


//
// Bits defintion of an DIAAGC Read Data
//
#define MAGL_BIT 11 // Magnetic field strength too low; AGC=0xFF
#define MAGH_BIT 10 // Magnetic field strength too high; AGC=0x00
#define COF_BIT 9 // CORDIC overflow
#define LF_BIT 8 // Offset compensation : 
                 // 0:internal offset loops not ready regulated
                 // 1:internal offset loop finished
#define AGC_SIZE 8 // The size of the bit 7:0 containing the Automatic gain
                   // control value

//
// Bits defintion of a CMAG Read Data
//
#define CMAG_SIZE 14 // The size of the bits 13:0 containing the CORDIC 
                     // magnitude information

//
// Bits defintion of a ANGLE Read Data
//
#define CORDICANG_SIZE 14 // The size of the bits 13:0 containing the andle
                          // information without error compensation

//
// Bits defintion of a ANGLECOM Read Data
//
#define DAECANG_SIZE 14 // The size of the bits 13:0 containing the dynamic 
                        // angle error compensation

//
// If overloaded, this function should be as fast as possible, because its
// code is executed with the priority of a SPI interruption.
//
__weak void as5047d_call_back_when_finished(as5047d_t* as5047d){
  return;
}

static inline void activate_cs(as5047d_t* as5047d){
  HAL_GPIO_WritePin(
    as5047d->gpio_port_cs, as5047d->gpio_pin_cs, GPIO_PIN_RESET
  );
  DELAY_AT_LEAST_NS(350); // See datasheet of as5047d .
}
static inline void disactivate_cs(as5047d_t* as5047d){
  HAL_GPIO_WritePin(
    as5047d->gpio_port_cs, as5047d->gpio_pin_cs, GPIO_PIN_SET
  );
  DELAY_AT_LEAST_NS(350); // See datasheet of as5047d .
}

typedef enum {
  sleeping,
  sending_a_reading_command,
  waiting_data,
  sending_the_error_register_command,
  waiting_for_error_register
} Encoder_state; 

static inline void reset_comunication(as5047d_t* as5047d, as5047d_error_t error){
  as5047d->error |= (AS5047D_ERROR | error);

  // We abort any pending SPI communication 
  HAL_StatusTypeDef status = HAL_SPI_Abort_IT(as5047d->hspi);
  if( status != HAL_OK ){
    as5047d->error |= AS5047D_SPI_ABORT_FAIL;
  }

  // Perhaps the device it yet selected, so, we stop first disactivate the cs.
  disactivate_cs(as5047d);

  // The last command could be a write comand. So, we drop the next packet. 
  activate_cs(as5047d);
  disactivate_cs(as5047d);
  
  // We finish to reset the spi communication.
  as5047d->state = sleeping;

  // We start angle computation
  as5047d_call_back_when_finished(as5047d);
}

/*
 * Returns true ig the parity of 'x' is even.
 */
static inline bool even_parity_check( uint16_t x ){
    x ^= x >> 8;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    return !((~x) & 1);
}


static inline void raise_dma_error(DMA_HandleTypeDef *hdma, const char* text){
  uint32_t error = HAL_DMA_GetError(hdma);
  if( error != HAL_DMA_ERROR_NONE ){
    raise_error_string( text );
  }
  switch(error){
    case HAL_DMA_ERROR_NONE :
      raise_error_string("HAL_DMA_ERROR_NONE");
      break;
    case HAL_DMA_ERROR_TE :
      raise_error_string("HAL_DMA_ERROR_TE");
      break;
    case HAL_DMA_ERROR_FE :
      raise_error_string("HAL_DMA_ERROR_FE");
      break;
    case HAL_DMA_ERROR_DME :
      raise_error_string("HAL_DMA_ERROR_DME");
      break;
    case HAL_DMA_ERROR_TIMEOUT :
      raise_error_string("HAL_DMA_ERROR_TIMEOUT");
      break;
    case HAL_DMA_ERROR_PARAM :
      raise_error_string("HAL_DMA_ERROR_PARAM");
      break;
    case HAL_DMA_ERROR_NO_XFER :
      raise_error_string("HAL_DMA_ERROR_NO_XFER");
      break;
    case HAL_DMA_ERROR_NOT_SUPPORTED :
      raise_error_string("HAL_DMA_ERROR_NOT_SUPPORTED");
      break;
    default:
      raise_error_string("?");
  }
}
static inline void check_and_raise_dma_error(as5047d_t* as5047d){
  raise_dma_error(as5047d->hspi->hdmarx, "ERROR DMA RX");
  raise_dma_error(as5047d->hspi->hdmatx, "ERROR DMA TX");
}
static inline void check_and_raise_spi_error(as5047d_t* as5047d){
  uint32_t error = HAL_SPI_GetError(as5047d->hspi);
  if( error != HAL_SPI_ERROR_NONE ){
    RAISE_ERROR;
  }
  switch(error){
   case HAL_SPI_ERROR_NONE :
     break;
   case HAL_SPI_ERROR_MODF :
     raise_error_string( "HAL_SPI_ERROR_MODF" );
     break;
   case HAL_SPI_ERROR_CRC :
     raise_error_string( "HAL_SPI_ERROR_CRC" );
     break;
   case HAL_SPI_ERROR_OVR :
     raise_error_string( "HAL_SPI_ERROR_OVR" );
     break;
   case HAL_SPI_ERROR_FRE :
     raise_error_string( "HAL_SPI_ERROR_FRE" );
     break;
   case HAL_SPI_ERROR_DMA :
     raise_error_string( "HAL_SPI_ERROR_DMA" );
     check_and_raise_dma_error(as5047d);
     break;
   case HAL_SPI_ERROR_FLAG :
     raise_error_string( "HAL_SPI_ERROR_FLAG" );
     break;
   case HAL_SPI_ERROR_ABORT :
     raise_error_string( "HAL_SPI_ERROR_ABORT" );
     break;
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
   case HAL_SPI_ERROR_INVALID_CALLBACK :
     raise_error_string( "HAL_SPI_ERROR_INVALID_CALLBACK" );
     break;
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
    default:
     raise_error_string( "?" );
  }
}

static inline bool transmit_and_receipt_packet(as5047d_t* as5047d){
  // We open a comunication
  activate_cs(as5047d);

  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(
    as5047d->hspi, as5047d->pTxData, as5047d->pRxData, PACKET_SIZE
  );

  if( status != HAL_OK ){
    as5047d->error |= AS5047D_ERROR;
    if( status == HAL_ERROR ){
      as5047d->error |= AS5047D_SPI_ERROR;
    }
    if( status == HAL_BUSY ){
      as5047d->error |= AS5047D_SPI_BUSY;
    }
    #if DEBUG
    check_and_raise_spi_error(as5047d);
    #endif
    reset_comunication(as5047d, as5047d->error);
    return false;
  }
  return true;
}

bool as5047d_start_reading_dynamic_angle(as5047d_t* as5047d){
  if( as5047d->is_ready && as5047d->state != sleeping ) return false;
  as5047d->is_ready = false;

  as5047d->state = sending_a_reading_command;
  as5047d->error = AS5047D_OK;
 
  // We send a reading command to obtain an angle
  as5047d->pTxData[0] = FIRST(RCMD(ANGLECOM_ADD));
  as5047d->pTxData[1] = LAST(RCMD(ANGLECOM_ADD));
  return transmit_and_receipt_packet(as5047d);
}

bool as5047d_start_reading_diagnostic(as5047d_t* as5047d){
  if( as5047d->is_ready && as5047d->state != sleeping ) return false;
  as5047d->is_ready = false;

  as5047d->state = sending_a_reading_command;
  as5047d->error = AS5047D_OK;
 
  // We send a reading command to obtain an angle
  as5047d->pTxData[0] = FIRST(RCMD(DIAAGC_ADD));
  as5047d->pTxData[1] = LAST(RCMD(DIAAGC_ADD));
  return transmit_and_receipt_packet(as5047d);
}

void as5047d_error_spi_call_back(as5047d_t* as5047d){
  #ifdef DEBUG
  check_and_raise_spi_error(as5047d);
  #endif
 
  reset_comunication(as5047d, AS5047D_SPI_CRASH);
}

void as5047d_spi_call_back(as5047d_t* as5047d){
  // First, we release the chip select to allow the device to start the
  // calculus it have to perform
  disactivate_cs(as5047d);

  // We build the received pachet
  int16_t packet = CONCATENATE(as5047d->pRxData[0], as5047d->pRxData[1]);
  bool parity_error = even_parity_check( packet );
  bool command_error = (1u << EF_BIT) & packet;

  if(as5047d->error || parity_error || command_error){
    as5047d->error |= AS5047D_ERROR;
    switch( as5047d->state ){
      case sending_a_reading_command:
        // The reading command is sent
        // We need to wait the answer.
        // We build a command to optain the error registor to check 
        // all is ok.
        as5047d->pTxData[0] = FIRST(RCMD(ERRFL_ADD)); // This command is not usefull.
        as5047d->pTxData[1] = LAST(RCMD(ERRFL_ADD));

        // We change or state before transmiting some new data.
        as5047d->state = sending_the_error_register_command;
        break;
      case sending_the_error_register_command :
      case waiting_data :
        as5047d->pTxData[0] = FIRST(RCMD(NOP_ADD)); // Nothing to ask
        as5047d->pTxData[1] = LAST(RCMD(NOP_ADD)); // we just have to wait the 
                                                  // answer at the last error 
                                                  // register Command  

        // We change or state before transmiting some new data.
        as5047d->state = waiting_for_error_register;
        break;  
      case waiting_for_error_register :
        if( packet & (1u <<PARERR_BIT) ){
          as5047d->error |= AS5047D_TX_CMD_PARITY;
        }
        if( packet & (1u <<INVCOMM_BIT) ){
          as5047d->error |= AS5047D_TX_CMD_ADDRESS;
        }
        if( packet & (1u <<FRERR_BIT) ){
          as5047d->error |= AS5047D_TX_CMD_FRAME;
        }

        // We change or state before transmiting some new data.
        as5047d->state = sleeping;
        break;
      default: 
        ASSERT(false);
        break;
    }
  }else{
    switch( as5047d->state ){
      case sending_a_reading_command:   
        // The reading command is sent
        // We need to wait the answer.
        // We build a command to optain the error registor to check 
        // all is ok.
        as5047d->pTxData[0] = FIRST(RCMD(ERRFL_ADD)); // This command is not usefull.
        as5047d->pTxData[1] = LAST(RCMD(ERRFL_ADD));
   
        // We change or state before transmiting some new data.
        as5047d->state = waiting_data;
        break;
      case waiting_data :
        as5047d->data = packet & RMASK(DATA_SIZE);
        as5047d->state = sleeping;
        break;
      default:
        ASSERT(false);
        break;
    }
  }
  if(as5047d->state == sleeping){
    as5047d_call_back_when_finished(as5047d);
  }else{
    transmit_and_receipt_packet(as5047d);
  }
}

void as5047d_init(  
  as5047d_t* as5047d, 
  SPI_HandleTypeDef* hspi, GPIO_TypeDef*  gpio_port_cs, uint16_t gpio_pin_cs
){
  as5047d->hspi = hspi;
  as5047d->gpio_port_cs=gpio_port_cs;
  as5047d->gpio_pin_cs=gpio_pin_cs;

  disactivate_cs(as5047d);
  
  as5047d->error = AS5047D_OK;
  as5047d->state = sleeping;
  as5047d->is_ready = true;
}

void as5047d_data_to_diagnostic(
  as5047d_t* as5047d, as5047d_diagnostic_t * diagnostic
){
  diagnostic->mfs_too_low = as5047d->data & (1u << MAGL_BIT);
  diagnostic->mfs_too_high = as5047d->data & (1u << MAGH_BIT);
  diagnostic->cordi_overflow = as5047d->data & (1u << COF_BIT);
  diagnostic->offset_compensation_is_ready = as5047d->data & (1u << LF_BIT);
  diagnostic->automatic_gain_control = as5047d->data & RMASK(AGC_SIZE);
}

