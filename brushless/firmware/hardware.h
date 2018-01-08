#ifndef _HARDWARE_H
#define _HARDWARE_H

// Encoder configuration

#define ENCODER_QUADRATURE
// #define ENCODER_MAGNETIC

#define ENCODER_SPI 2
#define ENCODER_SELECT_PIN  31
#define ENCODER_INDEX_PIN   12

// Current sensing
#define CURRENT_PIN     4

// Motors pins
#define U_LOW_PIN        10
#define U_HIGH_PIN       11
#define V_LOW_PIN        8
#define V_HIGH_PIN       9
#define W_LOW_PIN        33
#define W_HIGH_PIN       3
#define HALLU_PIN   7
#define HALLV_PIN   6
#define HALLW_PIN   5

// SPI slave pin
#define SLAVE_SPI   1
#define SLAVE_PIN           20

// Board LED pin
#define LED_PIN     22

#endif
