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

// Servo configuration
#define SPEED_DT    10
#define SERVO_DT    1

// Current protection
#define CURRENT_LIMIT       2.5
#define CURRENT_DURATION    500
#define CURRENT_MAX         8

#define MAX_MOTOR_VOLTAGE 12
#define HALF_MAX_MOTOR_VOLTAGE MAX_MOTOR_VOLTAGE/2 
#define PWM_SUPREMUM 3000

// XXX: Limiting PWM
#if DRIVER_TYPE == TYPE_70W
#define CURRENT_DISABLE
#define PWM_MIN             0     // 0-3000
#define PWM_MAX             2500    // 0-3000
#endif

#if DRIVER_TYPE == TYPE_30W
#define PWM_MIN             600     // 0-3000
#define PWM_MAX             2500    // 0-3000
#endif

#if DRIVER_TYPE == TYPE_DRIBBLER
#define PWM_MIN             0       // 0-3000
#define PWM_MAX             2700    // 0-3000
#endif

// Limitting acceleration
#define ACC_MAX             10      // turn/s^2

#endif
