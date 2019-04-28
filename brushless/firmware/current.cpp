#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include "hardware.h"
#include "current.h"
#include "motor.h"
#include "security.h"

// Current sensing
static float current = 0.0;
static float current_ref = 0.0;

#if BOARD == CATIE
static bool current_flag = false;

void current_irq(){
  if( current_flag ){
    security_set_warning(WARNING_CURRENT_LAG);
  }
  current_flag = true;
}

static void init_timer()
{
  HardwareTimer timer(3);

  // Configuring timer
  timer.pause();
  timer.setPrescaleFactor(9);
  timer.setOverflow(1000); // 8Khz

  timer.setChannel4Mode(TIMER_OUTPUT_COMPARE);
  timer.setCompare(TIMER_CH1, 1);
  timer.attachCompare4Interrupt(current_irq);

  timer.refresh();
  timer.resume();
}
#endif

void current_init()
{
  #if BOARD == GREG
    pinMode(CURRENT_PIN, INPUT);
  #endif
  #if BOARD == CATIE
    pinMode(CURRENT_U_PIN, INPUT_ANALOG);
    pinMode(CURRENT_V_PIN, INPUT_ANALOG);
    pinMode(CURRENT_W_PIN, INPUT_ANALOG);
    pinMode(CURRENT_REF_PIN, INPUT_ANALOG);
    init_timer();
  #endif
}

static int samples = 0;

void current_resample()
{
    samples = 0;
    current = 0.0;
    current_ref = 0.0;
}

#if BOARD == GREG
inline void current_greg(){
    static int last_update = millis();
    static int last_limit = 0;

    if ((millis() - last_update) > 10) {
        last_update += 10;
        samples++;

        // Measured voltage [mV] on the current sensor output
        float voltage = (5.0/3.0)*analogRead(CURRENT_PIN)*3300.0/4096.0;

        if (samples == 1) {
            current = voltage;
        } else {
            current = current*0.98 + voltage*0.02;
        }

        if (samples == 100) {
            // XXX: We should re-estimate the reference sometime, when we know
            // that the motor is off for instance
            current_ref = current;
            last_limit = millis();
        }

        // Security
        if (samples > 100) {
            float amps = fabs(current_amps());

            if (amps > CURRENT_LIMIT) {
                // We are over CURRENT_LIMIT for more than CURRENT_DURATION ms
                if (millis() - last_limit > CURRENT_DURATION) {
                    security_set_error(SECURITY_CURRENT_MAX);
                }
            } else {
                last_limit = millis();
            }

            // We are over CURRENT_MAX
            if (amps > CURRENT_MAX) {
                security_set_error(SECURITY_CURRENT_LIMIT);
            }
        }
    }
}
#endif


#if BOARD == CATIE
int16_t current_u;
int16_t current_v;
int16_t current_w;
int16_t ref;

inline void current_catie(){
    if( !current_flag ) return;
    current_u = analogRead(CURRENT_U_PIN);  
    current_v = analogRead(CURRENT_V_PIN);  
    current_w = analogRead(CURRENT_W_PIN); 
    ref = analogRead(CURRENT_REF_PIN);  
    current_flag = false;
}
#endif

void current_tick()
{
#if BOARD == GREG
  current_greg();
#endif
#if BOARD == CATIE
  current_catie();
#endif
}

inline float convert_to_amps(int val, int ref){
#if BOARD == GREG
    return -20.0*((val - ref)/ref);
#endif
#if BOARD == CATIE
    #define AMPLIFIER_GAIN 20
    // Resistor : R = 5m homs / 2
    // inverse : 1/R = 400
    #define MEASURE_INV_RESISTOR 400
    #define VCC_VOLTAGE 3.3
    #define RESOLUTION_ANALOG_READ 4096
    return (
      (MEASURE_INV_RESISTOR*VCC_VOLTAGE)*(val-ref)
    )/( RESOLUTION_ANALOG_READ*AMPLIFIER_GAIN );
#endif
}

float current_amps()
{
#if BOARD == GREG
    return convert_to_amps(current, current_ref);
#endif
#if BOARD == CATIE
    int val = ( 
      abs(current_u - ref) + abs(current_v - ref) + abs(current_w - ref)
    ) / 2;
    return convert_to_amps(val+ref, ref);
#endif
}

TERMINAL_COMMAND(amps, "Current")
{
    terminal_io()->print(current_amps());
    terminal_io()->println(" A");
#if BOARD == GREG
    terminal_io()->print(current);
    terminal_io()->println(" V");
#endif
#if BOARD == CATIE
    terminal_io()->print(current_u- ref);
    terminal_io()->println(" U");
    terminal_io()->print(current_v-ref);
    terminal_io()->println(" V");
    terminal_io()->print(current_w-ref);
    terminal_io()->println(" W");
    terminal_io()->print(ref);
    terminal_io()->println(" R");
    terminal_io()->print(current_u+current_v+current_w- 3*ref);
    terminal_io()->println(" S");
#endif
}
