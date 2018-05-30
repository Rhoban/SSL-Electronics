#ifndef _KICKER_H
#define _KICKER_H

void kicker_init();

void kicker_boost_enable(bool enable);

/**
 * Run a kick, power is [us]
 */
void kicker_kick(int kicker, int power);

/**
 * Ticks the kicker
 */
void kicker_tick();

/**
 * Gets the kick capcitor voltage [V]
 */
float kicker_cap_voltage();


/**
 * Clear the capacitors
 */
void kicker_clear();

#endif
