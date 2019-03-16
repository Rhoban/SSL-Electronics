//#define MOTEUR_BANC
#define MOTEUR_ROBOT_4_0
//#define CYCLOIDE_PHASE_OPPOSITION
//#define CYCLOIDE


#define MAX_THETA_LIMIT 6.15
#define MIN_THETA_LIMIT -6.15


#ifdef CYCLOIDE
    #define OPEN_LOOP_FOC false
    #define DO_NOT_USE_HALL
    #define STOP_OUTSIDE_LIMITS 
    #define MOTOR_NAME "CYCLOIDE"
    #define CONFIG_PWM 30
    #define NB_POSITIVE_MAGNETS 7
    #define REVERSE_PHASE
    #define MANUAL_SPEED false //true
    #define KFEM 110.0// //55.0 //29.6
    #define K_SPEED_P  270.0 //200.0 //180
    #define K_SPEED_I 3011.0 //80.0 //150
    #define K_POS_P 6.66 //1.5 // 1.5
    #define K_POS_I 0.0 //5.0 //0.1 // 0.1
#endif

#ifdef CYCLOIDE_PHASE_OPPOSITION
    #define PHASE_OPPOSITION 
    #define OPEN_LOOP_FOC false
    #define DO_NOT_USE_HALL
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
    #define DO_NOT_USE_HALL
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
    #define DO_NOT_USE_HALL
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


#define MOTEUR_ROBOT_4_0
#ifdef MOTEUR_ROBOT_4_0
    #define OPEN_LOOP_FOC false //true
    #define DO_NOT_USE_HALL
    //#define STOP_OUTSIDE_LIMITS 
    #define MOTOR_NAME "MOTEUR_ROBOT_4_0"
    #define CONFIG_PWM 30
    #define NB_POSITIVE_MAGNETS 8
    #define MANUAL_SPEED true //false
    #define KFEM 115.0 // 44.1
    #define K_SPEED_P 300 // 310
    #define K_SPEED_I 3000 // 150
    #define K_POS_P 0 // 1.5
    #define K_POS_I 0 // 0.1
#endif
