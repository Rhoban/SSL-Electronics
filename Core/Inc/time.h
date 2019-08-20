#pragma once

#include <stdint.h>
#include <stm32f4xx_hal.h>
#include <stdbool.h>

#define CLK_HSE 8000000
#define CLK_PLLM 4
#define CLK_PLLN 168
#define CLK_PLLP 4
#define CLK_SYSCLK 84000000


#if defined(STM32F401xC)
  // This value is manually obtained with an oscilloscope to obtain 
  // for 1s precisely for delay_at_least(1000000)
  // we obtain when making a 1.15us for delay_at_least(1) 
  #define STM32_DELAY_US_MULT 28
  _Static_assert(
    (STM32_DELAY_US_MULT == 28) &&
    (CLK_HSE == 8000000) &&
    (CLK_PLLM == 4) &&
    (CLK_PLLN == 168) &&
    (CLK_PLLP == 4) &&
    (CLK_SYSCLK == 84000000),
    "SOME CLOCK PARAMETER HAVE CHANGED. You have to update STM32_DELAY_US_MULT expermintally !"
  );
#endif

_Static_assert(((CLK_HSE*CLK_PLLN/(CLK_PLLM*CLK_PLLP)) == CLK_SYSCLK), "");
_Static_assert(CLK_HSE==HSE_VALUE, "");


static inline void _delay_us_(uint32_t nb) {
    /* fudge for function call overhead  */
    nb--;
    asm volatile("   mov r0, %[nb]          \n\t"
                 "1: subs r0, #1            \n\t"
                 "   bhi 1b                 \n\t"
                 :
                 : [nb] "r" (nb)
                 : "r0");
}


/**
 * @brief Delay the given number of microseconds.
 *
 * // This code come from the Wirish library 
 * // (libmaple/include/libmaple/delay.h)
 * // of the libmaple library (https://github.com/leaflabs/libmaple)
 * // release under MIT License with :
 * //
 * // Copyright (c) 2010 Perry Hung.
 * // Copyright (c) 2011 LeafLabs, LLC.
 *
 * @param us Number of microseconds to delay.
 */
static inline void delay_us(uint32_t us) {
  us *= STM32_DELAY_US_MULT;
    /* fudge for function call overhead  */
  _delay_us_(us);
}

#define DELAY_AT_LEAST_NS(ns) _delay_us_( 1 + (ns*STM32_DELAY_US_MULT)/1000 );

//
// Be carefull ! The time of time_get_us() restarts at 0 when it try to reachs 
// 2^32 (the size of uint32_t).
// So the time restarts to 0 each 1H and 11/12 minutes.
//
// Do not use this function to compute and store a date.
//
// In fact, this function is usefull to compute small difference of times 
// (smaller than 1H11).
//
// Indeed, suppose that t1 and t2 are two times 
// obtained with time_get_us(). Suppose that the time has  been reseted 
// between t1 and t2, then, t2 - t1 is correct modulo 2^32, modulo 1H12 .
//
static inline uint32_t time_get_us(void) {
  register uint32_t ms, cycle_cnt;
  do {
    ms = HAL_GetTick();
    cycle_cnt = SysTick->VAL;
  } while (ms != HAL_GetTick());
  return 
    (ms * 1000)+
    (cycle_cnt == 0 ? 0 : (CLK_SYSCLK/1000) - cycle_cnt)/(CLK_SYSCLK/1000000)
  ;
}

typedef struct {
  uint32_t period_us;
  uint32_t counter;
  uint32_t time;
} countdown_t;


void init_countdown(countdown_t * countdown, uint32_t period_us);
void reset_countdown(countdown_t * countdown);
void tick_countdown(countdown_t * countdown);
bool update_countdown(countdown_t * countdown);

#define COUNTDOWN(name, period) \
  static countdown_t _countdown_ ##name = {.period_us=period, .counter=0, .time=0}; \
  if( update_countdown( & _countdown_ ##name ) )


#if 0 // Somm other implemenation :)
// 
// Wait some time in Micro seconds.
//
// All Interuption increase the final waiting time by the time of the
// interuption.
//
// We are sure that the function wait at least 'micros' micro sconds.
//
// When, this function is not interupted, this function is mare accurate than
// time_delay_us().
//
// For a clock of 84 Mhz, time_delay_at_least_us(1) wait 1.35 us.
// Time_delay_us(1) wait 1.6 us !
//
// This function is usefull when some electronics need we wait some short time
// (for example 500ns).
//   
static inline void time_delay_at_least_us(uint32_t micros) {
  register uint32_t acc=0;
  register int32_t val=SysTick->VAL, old=val;
  do {
    acc += ( ( (old -= SysTick->VAL) >= 0) ? old : old+(CLK_SYSCLK/1000) );
    old = (val -= old);
  }while( acc < micros*(CLK_SYSCLK/1000000) );
}

static inline uint32_t time_get_us(void) {
  register uint32_t ms, cycle_cnt;
  do {
    ms = HAL_GetTick();
    cycle_cnt = SysTick->VAL;
  } while (ms != HAL_GetTick());
  return 
    (ms * (CLK_SYSCLK/1000)) +
    //(cycle_cnt == 0 ? 0 : (CLK_SYSCLK/1000) - cycle_cnt)
    (CLK_SYSCLK/1000) - cycle_cnt
  ;
}
static inline void time_delay_us(uint32_t micros) {
  uint32_t deadline = time_get_us()+micros*(CLK_SYSCLK/1000000);
  while (time_get_us() < deadline ) {
    asm("nop");
  }
}
#endif


