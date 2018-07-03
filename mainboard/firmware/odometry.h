#ifndef _ODOMETRY_H
#define _ODOMETRY_H

#include "drivers.h"
#include "com.h"
#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include <watchdog.h>
#include "hardware.h"


struct position {
  double xpos;
  double ypos;
  double ang;
};


void odometry_init();


void odometry_tick();


struct position odometry_value();

#endif
