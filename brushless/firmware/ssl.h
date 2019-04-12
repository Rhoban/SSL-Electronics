#ifndef _SSL_H
#define _SSL_H

//#define MOTEUR_BANC
//#define MOTEUR_90_12V
#define MOTEUR_ROBOT_4_0
//#define CYCLOIDE_PHASE_OPPOSITION
//#define CYCLOIDE12
//#define CYCLOIDE16
//#define CYCLOIDE16_PHASE_OPPOSITION

#define MAX_THETA_LIMIT 6.15
#define MIN_THETA_LIMIT -6.15

#ifdef CYCLOIDE16_PHASE_OPPOSITION
    #define PHASE_OPPOSITION 
    #define OPEN_LOOP_FOC false
    #define USE_FOC
    #define STOP_OUTSIDE_LIMITS 
    #define MOTOR_NAME "CYC. 16V PH. OP."
    #define CONFIG_PWM 95
    #define NB_POSITIVE_MAGNETS 7
    #define REVERSE_PHASE
    #define MANUAL_SPEED false //true
    // #define INITIAL_SPEED 1.0
    #define KFEM 55.0 //29.0 //42.0 //55.0 //110.0// //55.0 //29.6
    #define K_SPEED_P  180.0 //80.0 //270.0 //200.0 //180
    #define K_SPEED_I 2000.0 //1000.0 //3011.0 //80.0 //150
    #define K_POS_P 1.0 // 0.0 //6.66 //1.5 // 1.5
    #define K_POS_I 0.0 //5.0 //0.1 // 0.1
#endif


#ifdef CYCLOIDE16
    #define OPEN_LOOP_FOC false
    #define USE_FOC
    #define STOP_OUTSIDE_LIMITS 
    #define MOTOR_NAME "CYCLOIDE 16V"
    #define CONFIG_PWM 90
    #define NB_POSITIVE_MAGNETS 7
    #define REVERSE_PHASE
    #define MANUAL_SPEED false //true
    // #define INITIAL_SPEED 1.0
    #define KFEM 81.0 //29.0 //42.0 //55.0 //110.0// //55.0 //29.6
    #define K_SPEED_P  120.0 //270.0 //200.0 //180
    #define K_SPEED_I 3000.0 //3011.0 //80.0 //150
    #define K_POS_P 11.0 //6.66 //1.5 // 1.5
    #define K_POS_I 0.1 //5.0 //0.1 // 0.1
#endif


#ifdef CYCLOIDE12
    #define OPEN_LOOP_FOC false
    #define USE_FOC
    #define STOP_OUTSIDE_LIMITS 
    #define MOTOR_NAME "CYCLOIDE 12V"
    #define CONFIG_PWM 90
    #define NB_POSITIVE_MAGNETS 7
    #define REVERSE_PHASE
    #define MANUAL_SPEED false //true
    // #define INITIAL_SPEED 1.0
    #define KFEM 35.0 //42.0 //55.0 //110.0// //55.0 //29.6
    #define K_SPEED_P  110.0 //270.0 //200.0 //180
    #define K_SPEED_I 3000.0 //3011.0 //80.0 //150
    #define K_POS_P 10.0 //6.66 //1.5 // 1.5
    #define K_POS_I 0.1 //5.0 //0.1 // 0.1
#endif

#ifdef CYCLOIDE_PHASE_OPPOSITION
    #define PHASE_OPPOSITION 
    #define OPEN_LOOP_FOC false
    #define USE_FOC
    #define STOP_OUTSIDE_LIMITS 
    #define MOTOR_NAME "CYCLOIDE"
    //#define CONFIG_PWM 100
    #define CONFIG_PWM 80
    #define NB_POSITIVE_MAGNETS 7
    #define REVERSE_PHASE
    #define MANUAL_SPEED true
    #define KFEM 0.0// //55.0 //29.6
    #define K_SPEED_P 0.0 //200.0 //180
    #define K_SPEED_I 0.0 //80.0 //150
    #define K_POS_P 0.0// 6.66 //1.5 // 1.5
    #define K_POS_I 0.0 //5.0 //0.1 // 0.1
#endif

#ifdef CYCLOIDE_OLD
    #define OPEN_LOOP_FOC false
    #define USE_FOC
    #define STOP_OUTSIDE_LIMITS 
    #define MOTOR_NAME "CYCLOIDE"
    #define CONFIG_PWM 80
    #define NB_POSITIVE_MAGNETS 7
    #define REVERSE_PHASE
    #define MANUAL_SPEED true
    #define KFEM 55.0 //29.6
    #define K_SPEED_P 200.0 //180
    #define K_SPEED_I 80.0 //150
    #define K_POS_P 6.66 //1.5 // 1.5
    #define K_POS_I 5.0 //0.1 // 0.1
#endif

#ifdef MOTEUR_BANC
    #define OPEN_LOOP_FOC false //true
    #define USE_FOC
    //#define STOP_OUTSIDE_LIMITS 
    #define MOTOR_NAME "MOTEUR_BANC"
    #define CONFIG_PWM 30
    #define NB_POSITIVE_MAGNETS 8
    #define MANUAL_SPEED true //false
    #define KFEM 115.0 // 44.1
    #define K_SPEED_P 300 // 310
    #define K_SPEED_I 3000 // 150
    #define K_POS_P 0 // 1.5
    #define K_POS_I 0 // 0.1
#endif


#ifdef MOTEUR_90_12V
    //#define PHASE_OPPOSITION 
    //#define HIGH_IMPEDENCE_MODE
    #define OPEN_LOOP_FOC false //true
    #define USE_FOC
    //#define STOP_OUTSIDE_LIMITS 
    #define MOTOR_NAME "MOTEUR_90_12V"
    #define CONFIG_PWM 90
    #define NB_POSITIVE_MAGNETS 8
    #define MANUAL_SPEED false
    #define KFEM 52 //73.0 // 44.1
    #define K_SPEED_P 450  // 310
    #define K_SPEED_I 3000 // 150
    #define K_POS_P 11 // 1.5
    #define K_POS_I 0.1 // 0.1
#endif


#ifdef MOTEUR_ROBOT_4_0
    //#define PHASE_OPPOSITION 
    #define OPEN_LOOP_FOC false //true
    #define USE_HYBRID
    #define HYBRID_DIRECT_VOLTAGE 7*REFERENCE_VOLTAGE/8
    #define USE_FOC
    //#define STOP_OUTSIDE_LIMITS 
    #define MOTOR_NAME "MOTEUR_ROBOT_4_0"
    #define CONFIG_PWM 40
    #define NB_POSITIVE_MAGNETS 8
    #define MANUAL_SPEED true //false //true //false
    #define KFEM 120 //122 //115.0 // 44.1
    #define K_SPEED_P 425 //450 //300 // 310
    #define K_SPEED_I 3000 // 3000 // 3000 // 150
    #define K_POS_P 0.0 //11.0 // 1.5
    #define K_POS_I 0.0 //0.1 // 0.1
#endif

#ifndef INITIAL_SPEED
    #define INITIAL_SPEED 0
#endif

#endif
