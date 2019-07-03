#include "motor_foc.h"
#include "motor_hall.h"
#include "motor_hybrid.h"
#include "hardware.h"
#include <terminal.h>


static MotorMode motor_mode = FOC;

void motor_hall_hybrid_tick();

void init_hall_hybrid(){
    // Initializing hall sensors input
    pinMode(HALLU_PIN, INPUT_PULLUP);
    pinMode(HALLV_PIN, INPUT_PULLUP);
    pinMode(HALLW_PIN, INPUT_PULLUP);

    // Attach interrupts on phase change
    attachInterrupt(HALLU_PIN, motor_hall_hybrid_tick, CHANGE);
    attachInterrupt(HALLV_PIN, motor_hall_hybrid_tick, CHANGE);
    attachInterrupt(HALLW_PIN, motor_hall_hybrid_tick, CHANGE);
}

void init_hybrid_pwm_pins(){
    // Initalizing motor pins

  // Initalizing motor pins
  pinMode(U_SD_PIN, OUTPUT);
  digitalWrite(U_SD_PIN, LOW);
  pinMode(V_SD_PIN, OUTPUT);
  digitalWrite(V_SD_PIN, LOW);
  pinMode(W_SD_PIN, OUTPUT);
  digitalWrite(W_SD_PIN, LOW);

  pinMode(U_IN_PIN, PWM);
  pwmWrite(U_IN_PIN, 0);
  pinMode(V_IN_PIN, PWM);
  pwmWrite(V_IN_PIN, 0);
  pinMode(W_IN_PIN, PWM);
  pwmWrite(W_IN_PIN, 0);


}

void motor_hybrid_irq(){
  motor_irq();
}

void switch_to_hall(){
  motor_mode = HALL;

  enable_motor_foc(false);
  enable_motor_hall(true);
}

void switch_to_foc(){
  motor_mode = FOC;

  enable_motor_foc(true);
  enable_motor_hall(false);
}

static void _init_hybrid_timer(int number)
{
    HardwareTimer timer(number);

    // Configuring timer
    timer.pause();
    timer.setPrescaleFactor(PWM_PRESCALE_FACTOR);
    timer.setOverflow(PWM_OVERFLOW);
    timer.refresh();

    #if BOARD == GREG
    if (number == 3) {
        timer.setMode(TIMER_CH2, TIMER_OUTPUT_COMPARE);
        timer.setCompare(TIMER_CH2, 1);
        timer.attachInterrupt(TIMER_CH2, motor_hybrid_irq);
    }
    #endif
    #if BOARD == CATIE
    if (number == 1) {
        timer.setMode(TIMER_CH4, TIMER_OUTPUT_COMPARE);
        timer.setCompare(TIMER_CH4, 1);
        timer.attachInterrupt(TIMER_CH4, motor_hybrid_irq);
    }
    #endif
    timer.refresh();
    timer.resume();
}

void init_hybrid_timers(){
    // Configuring timers
    #if BOARD == GREG
    _init_hybrid_timer(2);
    _init_hybrid_timer(3);
    #endif
    #if BOARD == CATIE
    _init_hybrid_timer(1);
    #endif
}

void motor_hybrid_init(){
  ///   init_hall_hybrid();
  init_hybrid_timers();
  init_hybrid_pwm_pins();
  switch_to_foc();
}

void motor_hybrid_set(bool enable, int value){
    motor_foc_set(enable, value);
    motor_hall_set(enable, value);
}

void motor_hybrid_tick(){
    motor_foc_tick();
    /// motor_hall_tick();
}
bool motor_hybrid_is_on(){
    return motor_foc_is_on();
}
void launch_tare_motor_hybrid(){
    launch_tare_motor_foc();
}

void motor_hall_hybrid_tick(){
  motor_hall_irq();
}

TERMINAL_COMMAND(hyb_tare, "Tare hybrid")
{
    launch_tare_motor_hybrid();
}
