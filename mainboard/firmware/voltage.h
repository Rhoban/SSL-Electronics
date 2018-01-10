#ifndef _VOLTAGE_H
#define _VOLTAGE_H

void voltage_init();

void voltage_tick();

/**
 * Voltage of two batteries [V]
 */
float voltage_bat1();
float voltage_bat2();

#endif
