#ifndef _CURRENT_H
#define _CURRENT_H

/**
 * Initializes the current sensing device
 */
void current_init();

/**
 * Ticks the current sensing device
 */
void current_tick();

/**
 * The value measured by the current sensor [amp] *
 */
float current_amps();

#endif
