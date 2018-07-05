#include "odometry.h"

#define WHEEL_RADIUS 0.0285
#define COS15        0.9659
#define SIN15        0.2588
#define ENC_TOUR     16438
#define DIST_TOUR    0.179
#define DIAMETER     0.18
#define RADIUS     DIAMETER/2

struct position current_position;
int32_t current_encoder[4];
int32_t delta[4];
int32_t instantaneous_encoder[4];
bool odom_enable;
bool tare_round = true;


void odometry_init(){
  odom_enable = false;
  current_position.xpos = 0.0;
  current_position.ypos = 0.0;
  current_position.ang  = 0.0;
  tare_round = true;

  for(int i = 0; i < 4; i++){
    instantaneous_encoder[i] = 0;
  }
}

void odometry_stop(){
  odom_enable = false;
  odometry_init();
}


void odometry_tick(){

    if(odom_enable == true){

      //struct driver_odom tmp;
      if(tare_round == true){
        for(int i = 0; i < 4; i++){
          //tmp = drivers_ask_odom(i);
          current_encoder[i] = driver_answers[i].enc_cnt;
        }

        current_position.ang  = 0;
        current_position.xpos = 0;
        current_position.ypos = 0;

        tare_round = false;
      }
      else{

        for(int i = 0; i < 4; i++){
          //tmp = drivers_ask_odom(i);
          instantaneous_encoder[i] = driver_answers[i].enc_cnt;
        }

        delta[0] = instantaneous_encoder[0] - current_encoder[0];
        delta[1] = instantaneous_encoder[1] - current_encoder[1];
        delta[2] = instantaneous_encoder[2] - current_encoder[2];
        delta[3] = instantaneous_encoder[3] - current_encoder[3];

        for(int i = 0; i < 4; i++){
            current_encoder[i] += delta[i];
        }

        double x_ref_bot   = DIST_TOUR/(ENC_TOUR*(1 + COS15 + SIN15))*(delta[0]*COS15 - delta[3]*SIN15 - delta[2]); //Dist parcourue dans le sens du robot en m
        double y_ref_bot   = DIST_TOUR/(ENC_TOUR*(1 + COS15 + SIN15))*(delta[0]*SIN15 - delta[3]*COS15 + delta[1]);
        double rot_ref_bot = 2*PI/(ENC_TOUR*RADIUS*4)*(delta[0] + delta[1] + delta[2] + delta[3]);
        //double rot_ref_bot = 1/((4*DIAMETER))*(delta[0] + delta[1] + delta[2] + delta[3]);

        current_position.ang  += (rot_ref_bot);
        current_position.xpos += x_ref_bot*cos(current_position.ang) - y_ref_bot*sin(current_position.ang);
        current_position.ypos += x_ref_bot*sin(current_position.ang) + y_ref_bot*cos(current_position.ang);

        terminal_io()->print("x : ");
        terminal_io()->println(current_position.xpos);
        terminal_io()->print("y : ");
        terminal_io()->println(current_position.ypos);
        terminal_io()->print("Ang : ");
        terminal_io()->println(current_position.ang);
    }
  }
}


void odometry_tare(double _x = 0.0, double _y = 0.0, double _r = 0.0){
  odom_enable = false;
  current_position.xpos = _x;
  current_position.ypos = _y;
  current_position.ang  = _r;
}




TERMINAL_COMMAND(odom, "Start of Odometry"){
  odom_enable = true;
}

TERMINAL_COMMAND(st_odom, "Stop of Odometry"){
  odom_enable = false;
}

TERMINAL_COMMAND(tare_odom, "Tare of Odometry"){
  odometry_tare();
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
