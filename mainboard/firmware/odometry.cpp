#include "odometry.h"

#define WHEEL_RADIUS 0.0285
#define COS15        0.9659
#define SIN15        0.2588
#define ENC_TOUR     16438
#define RADIUS       0.079
#define PI           3.14159
#define ODOM_PLOT    0
#define DIV_PLOT     10

struct position current_position;
int32_t current_encoder[4];
double delta[4];
int32_t instantaneous_encoder[4];
bool odom_enable;
bool tare_round = true;
int compteur = 0;


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

      if(tare_round == true){//This round is supposed to setup the initial encoders values
        for(int i = 0; i < 4; i++){
          current_encoder[i] = driver_answers[i].enc_cnt;
        }

        tare_round = false;//It is only necessary once
      }
      else{

        for(int i = 0; i < 4; i++){
          instantaneous_encoder[i] = driver_answers[i].enc_cnt; //gathering the new encoders values
        }

        //How much each wheel has moved since last tick
        delta[0] = ((instantaneous_encoder[0] - current_encoder[0])*2*WHEEL_RADIUS*PI)/(ENC_TOUR);
        delta[1] = ((instantaneous_encoder[1] - current_encoder[1])*2*WHEEL_RADIUS*PI)/(ENC_TOUR);
        delta[2] = ((instantaneous_encoder[2] - current_encoder[2])*2*WHEEL_RADIUS*PI)/(ENC_TOUR);
        delta[3] = ((instantaneous_encoder[3] - current_encoder[3])*2*WHEEL_RADIUS*PI)/(ENC_TOUR);

        if((fabs(delta[0]) >= 1)||(fabs(delta[1]) >= 1)||(fabs(delta[2]) >= 1)||(fabs(delta[3]) >= 1)){
            terminal_io()->println("ORTIEEEE");
        }
        else{



            //Processing of the deplacement on the robot coordinate system
            double x_ref_bot   = -0.34641*delta[0] - 0.28284*delta[1] + 0.28284*delta[2] + 0.34641*delta[3];//Dist parcourue dans le sens du robot en m
            double y_ref_bot   =  0.41421*delta[0] - 0.41421*delta[1] - 0.41421*delta[2] + 0.41421*delta[3];
            double rot_ref_bot =  3.70751*delta[0] + 2.62160*delta[1] + 2.62160*delta[2] + 3.70751*delta[3];

            //Prrojection on the movement into the field coordinate system
            current_position.ang  += rot_ref_bot;
    	      current_position.xpos += x_ref_bot*cos(current_position.ang) - y_ref_bot*sin(current_position.ang);
            current_position.ypos += x_ref_bot*sin(current_position.ang) + y_ref_bot*cos(current_position.ang);

            //Update of the new encoders references values
            for(int i = 0; i < 4; i++){
                current_encoder[i] += (instantaneous_encoder[i] - current_encoder[i]);
            }


        }

        #if ODOM_PLOT == 1
        compteur++;
        if(compteur >= DIV_PLOT){
          compteur = 0;
          terminal_io()->print("x : ");
          terminal_io()->println(current_position.xpos);
          terminal_io()->print("y : ");
          terminal_io()->println(current_position.ypos);
          terminal_io()->print("Ang : ");
          terminal_io()->println(current_position.ang*360/(2*PI));
          terminal_io()->println("");
          /*terminal_io()->print(instantaneous_encoder[0]);
          terminal_io()->print(" | ");
          terminal_io()->print(instantaneous_encoder[1]);
          terminal_io()->print(" | ");
          terminal_io()->print(instantaneous_encoder[2]);
          terminal_io()->print(" | ");
          terminal_io()->println(instantaneous_encoder[3]);*/
        }

        #endif
    }
  }
}

void odometry_tare(double _x, double _y, double _r){
  odom_enable = false;
  current_position.xpos = _x;
  current_position.ypos = _y;
  current_position.ang  = _r;
  tare_round = true; //After a tare, a tare_round is necessary
}

struct position getOdometry(){
  return current_position;
}


TERMINAL_COMMAND(odom, "Start of Odometry"){
  odom_enable = true;
}

TERMINAL_COMMAND(st_odom, "Stop of Odometry"){
  odom_enable = false;
}

TERMINAL_COMMAND(tare_odom, "Tare of Odometry"){
  odometry_tare(0.0,0.0,0.0);
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
