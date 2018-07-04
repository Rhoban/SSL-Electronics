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


void odometry_init(); //initialize odometry variables

void odometry_stop(); //Stop Odometry

void odometry_tick(); //Odometry routine

void odometry_tare(double _x, double _y, double _r); //Set an offset for the position of the robot

void odometry_fetch_encoders_value(); //Update value of current encoders after a tare

#endif
