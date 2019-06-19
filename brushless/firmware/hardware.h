#ifndef _HARDWARE_H
#define _HARDWARE_H

#include "ssl.h"

// Defines the driver type (see below)
#define DRIVER_TYPE     TYPE_70W

#define TYPE_30W        1
#define TYPE_70W        2
#define TYPE_DRIBBLER   3

#ifndef DRIVER_TYPE
#error "You should set a value for DRIVER_TYPE"
#endif

// Encoder configuration
#if DRIVER_TYPE == TYPE_30W
#define ENCODER_QUADRATURE          // Quadrature encoder A/B
#define ENCODER_CPR      1024
#endif

#if DRIVER_TYPE == TYPE_70W
#define ENCODER_MAGNETIC            // Magnetic AS5048 14bit encoder
// #define ENCODER_CPR      1024
#define ENCODER_CPR      16384
// #define ENCODER_NONE                // No encoder
#endif

#if DRIVER_TYPE == TYPE_DRIBBLER
#define ENCODER_NONE                // No encoder
#endif

#define GREG 1
#define CATIE 2

#ifdef BOARD_maple_mini
  #define BOARD GREG
#else
  #define BOARD CATIE
#endif

#if BOARD == CATIE
  // For SPI based encoders
  #define ENCODER_SPI         2
  #define ENCODER_SELECT_PIN  PB12
  #define ENCODER_INDEX_PIN   12 // Used to make a shift bit to define PB12

  // Current sensing
  #define CURRENT_U_PIN      PC0
  #define CURRENT_V_PIN      PC3
  #define CURRENT_W_PIN      PC1
  #define CURRENT_REF_PIN      PC2

  // Motors pins

  #ifdef REVERSE_PHASE
      #define U_SD_PIN       PC6  //use Timer 1 (Maple)
      #define U_IN_PIN       PA8  //use Timer 1 (Maple)

      #define V_SD_PIN       PC8  //use Timer 1 (Maple)
      #define V_IN_PIN       PA10  //use Timer 1 (Maple)
      
      #define W_SD_PIN       PC7 //use Timer 1 (Maple)
      #define W_IN_PIN       PA9 //use Timer 1 (Maple)
  #else
      #define U_SD_PIN       PC6 //use Timer 1 (Maple)
      #define U_IN_PIN       PA8 //use Timer 1 (Maple)

      #define V_SD_PIN       PC7 //use Timer 1 (Maple)
      #define V_IN_PIN       PA9 //use Timer 1 (Maple)

      #define W_SD_PIN       PC8  //use Timer 1 (Maple)
      #define W_IN_PIN       PA10 //use Timer 1 (Maple)
  #endif



  #define HALLU_PIN        PA0
  #define HALLV_PIN        PA1
  #define HALLW_PIN        PA2

  // SPI slave pin
  #define SLAVE_SPI           1
  #define SLAVE_PIN PA4
  #define SLAVE_CLK_PIN PA5
  #define SLAVE_MISO_PIN PA6
  #define SLAVE_MOSI_PIN PA7

  // Board LED pin
  #define LED_PIN     PB1
#endif

#if BOARD == GREG
  // For SPI based encoders
  #define ENCODER_SPI         2
  #define ENCODER_SELECT_PIN  31
  #define ENCODER_INDEX_PIN   12

  // Current sensing
  #define CURRENT_PIN      4

  // Motors pins

  #ifdef REVERSE_PHASE
      #define U_SD_PIN        10  //use Timer 2 (Maple Mini)
      #define U_IN_PIN       11  //use Timer 2 (Maple Mini)

      #define V_SD_PIN        33  //use Timer 3 (Maple Mini)
      #define V_IN_PIN       3  //use Timer 2 (Maple Mini)
      
      #define W_SD_PIN        8 //use Timer 2 (Maple Mini)
      #define W_IN_PIN       9 //use Timer 2 (Maple Mini)
  #else
      #define U_SD_PIN        10 //use Timer 2 (Maple Mini)
      #define U_IN_PIN       11 //use Timer 2 (Maple Mini)

      #define V_SD_PIN        8 //use Timer 2 (Maple Mini)
      #define V_IN_PIN       9 //use Timer 2 (Maple Mini)

      #define W_SD_PIN        33  //use Timer 3 (Maple Mini)
      #define W_IN_PIN       3 //use Timer 2 (Maple Mini)
  #endif



  #define HALLU_PIN        7
  #define HALLV_PIN        6
  #define HALLW_PIN        5

  // SPI slave pin
  #define SLAVE_SPI           1
  #define SLAVE_PIN           20

  // Board LED pin
  #define LED_PIN     22
  //#define LED_PIN     PB1



//  // For SPI based encoders
//  #define ENCODER_SPI         2
//  #define ENCODER_SELECT_PIN  PB12
//  #define ENCODER_INDEX_PIN   12 // Used to make a shift bit to define PB12
//
//  // Current sensing
//  #define CURRENT_PIN      PA7
//
//  // Motors pins
//
//  #ifdef REVERSE_PHASE
//      #define U_SD_PIN       PA1  //use Timer 2 (Maple Mini)
//      #define U_IN_PIN       PA0  //use Timer 2 (Maple Mini)
//
//      #define V_SD_PIN       PB1  //use Timer 3 (Maple Mini)
//      #define V_IN_PIN       PB0  //use Timer 2 (Maple Mini)
//      
//      #define W_SD_PIN       PA3 //use Timer 2 (Maple Mini)
//      #define W_IN_PIN       PA2 //use Timer 2 (Maple Mini)
//  #else
//      #define U_SD_PIN       PA1 //use Timer 2 (Maple Mini)
//      #define U_IN_PIN       PA0 //use Timer 2 (Maple Mini)
//
//      #define V_SD_PIN       PA3 //use Timer 2 (Maple Mini)
//      #define V_IN_PIN       PA2 //use Timer 2 (Maple Mini)
//
//      #define W_SD_PIN       PB1  //use Timer 3 (Maple Mini)
//      #define W_IN_PIN       PB0 //use Timer 2 (Maple Mini)
//  #endif
//
//
//
//  #define HALLU_PIN        PA4
//  #define HALLV_PIN        PA5
//  #define HALLW_PIN        PA6
//
//  // SPI slave pin
//  #define SLAVE_SPI           1
//  #define SLAVE_PIN BOARD_JTDI_PIN
//
//  // Board LED pin
//  #define LED_PIN BOARD_JTMS_SWDIO_PIN
#endif



// Servo configuration
#define SPEED_DT    10
#define SERVO_DT    1

// Current protection
#define CURRENT_LIMIT       2.5
#define CURRENT_DURATION    500
#define CURRENT_MAX         8

#define MAX_MOTOR_VOLTAGE 12
#define HALF_MAX_MOTOR_VOLTAGE MAX_MOTOR_VOLTAGE/2 

// XXX: Limiting PWM
#if DRIVER_TYPE == TYPE_70W
//#define CURRENT_DISABLE
#define PWM_MIN_PERCENT 0
#define PWM_MAX_PERCENT 95
#endif

#if DRIVER_TYPE == TYPE_30W
#define PWM_MIN_PERCENT 20
#define PWM_MAX_PERCENT 83
#endif

#if DRIVER_TYPE == TYPE_DRIBBLER
#define PWM_MIN_PERCENT 0
#define PWM_MAX_PERCENT 90
#endif

static_assert(0<=PWM_MIN_PERCENT, "");
static_assert(PWM_MIN_PERCENT<PWM_MAX_PERCENT, "");
static_assert(PWM_MAX_PERCENT<100, "");


// Limitting acceleration
#define ACC_MAX             10      // turn/s^2

#endif
