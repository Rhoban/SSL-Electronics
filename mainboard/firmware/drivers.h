#ifndef _DRIVERS_H
#define _DRIVERS_H

void drivers_init();

/**
 * Set the speed of the nth wheel [turn/s]
 */
uint8_t drivers_set(int index, bool enable, float target);
void drivers_set_safe(int index, bool enable, float target);

/**
 * Tick the drivers
 */
void drivers_tick();

#endif
