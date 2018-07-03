#include "odometry.h"

#define WHEEL_RADIUS 0.0285
#define COS15        0.9659
#define SIN15        0.2588
#define ENC_TOUR     16438
#define DIST_TOUR    0.179
#define DIAMETER     0.18

#define PLOT_ODOM    1

struct position current_position;
int32_t current_encoder[4];
int32_t delta[4];
int32_t instantaneous_encoder[4];
bool odom_enable;
double last_tick;

void odometry_init(){
  odom_enable = false;
  current_position.xpos = 0.0;
  current_position.ypos = 0.0;
  current_position.ang = 0.0;

  for(int i = 0; i < 4; i++){
    instantaneous_encoder[i] = 0;
  }
}


void odometry_tick(){

  static int last_tick = millis();
  int elapsed = millis() - last_tick;

  if (elapsed >= 100) { // Every 100ms
    last_tick += elapsed;

    if(odom_enable == true){

      last_tick = millis();
      struct driver_odom tmp;

      for(int i = 0; i < 4; i++){
        tmp = drivers_ask_odom(i);
        instantaneous_encoder[i] = tmp.enc_cnt;
      }

      delta[0] = instantaneous_encoder[0] - current_encoder[0];
      delta[1] = instantaneous_encoder[1] - current_encoder[1];
      delta[2] = instantaneous_encoder[2] - current_encoder[2];
      delta[3] = instantaneous_encoder[3] - current_encoder[3];

      for(int i = 0; i < 4; i++){
          current_encoder[i] += delta[i];
      }

      float x_ref_bot = DIST_TOUR/(ENC_TOUR*(1+COS15+SIN15))*(delta[0]*COS15 - delta[3]*SIN15 - delta[2]); //Dist parcourue dans le sens du robot en m
      float y_ref_bot = DIST_TOUR/(ENC_TOUR*(1+COS15+SIN15))*(delta[0]*SIN15 - delta[3]*COS15 + delta[1]);
      float rot_ref_bot = DIST_TOUR/(ENC_TOUR*(4*DIAMETER))*(delta[0]+delta[1]+delta[2]+delta[3]);

      current_position.ang  += rot_ref_bot;
      current_position.xpos += x_ref_bot*cos(current_position.ang) - y_ref_bot*sin(current_position.ang);
      current_position.ypos += x_ref_bot*sin(current_position.ang) + y_ref_bot*cos(current_position.ang);

    }
  }

}

TERMINAL_COMMAND(odom, "Start of Odometry"){
  odom_enable = true;
}

TERMINAL_COMMAND(tare_odom, "Tare of Odometry"){
  odom_enable = false;
  current_position.xpos = 0.0;
  current_position.ypos = 0.0;
  current_position.ang = 0.0;

  struct driver_odom tmp;
  for(int i = 0; i < 4; i++){
    tmp = drivers_ask_odom(i);
    instantaneous_encoder[i] = tmp.enc_cnt;
  }

  current_encoder[0] = instantaneous_encoder[0];
  current_encoder[1] = instantaneous_encoder[1];
  current_encoder[2] = instantaneous_encoder[2];
  current_encoder[3] = instantaneous_encoder[3];
}

TERMINAL_COMMAND(enc, "Encoder value"){
  terminal_io()->print(delta[0]);
  terminal_io()->print(" - ");
  terminal_io()->print(delta[1]);
  terminal_io()->print(" - ");
  terminal_io()->print(delta[2]);
  terminal_io()->print(" - ");
  terminal_io()->println(delta[3]);
  terminal_io()->print("x : ");
  terminal_io()->println(current_position.xpos);
  terminal_io()->print("y : ");
  terminal_io()->println(current_position.ypos);
  terminal_io()->print("Ang : ");
  terminal_io()->println(current_position.ang);
}
