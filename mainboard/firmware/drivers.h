#ifndef _DRIVERS_H
#define _DRIVERS_H

void drivers_init();

/**
 * Set the speed of the nth wheel [turn/s]
 */
void drivers_set(int index, bool enable, float target);

#endif
