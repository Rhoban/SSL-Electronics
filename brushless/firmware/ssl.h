#define MOTEUR_BANC
//#define CYCLOIDE


#define MAX_THETA_LIMIT 3.5
#define MIN_THETA_LIMIT -5.0


#ifdef CYCLOIDE
    #define OPEN_LOOP_FOC true
    #define DO_NOT_USE_HALL
    #define STOP_OUTSIDE_LIMITS 
    #define MOTOR_NAME "CYCLOIDE"
    #define CONFIG_PWM 100
    #define NB_POSITIVE_MAGNETS 7
    #define REVERSE_PHASE
    #define MANUAL_SPEED false
    #define KFEM 0 //29.6
    #define K_SPEED_P 0.0 //180
    #define K_SPEED_I 0.0 //150
    #define K_POS_P 0.0 //1.5 // 1.5
    #define K_POS_I 0.0 //0.1 // 0.1
#endif

#ifdef MOTEUR_BANC
    #define OPEN_LOOP_FOC false//true
    #define DO_NOT_USE_HALL
    //#define STOP_OUTSIDE_LIMITS 
    #define MOTOR_NAME "MOTEUR_BANC"
    #define CONFIG_PWM 100
    #define NB_POSITIVE_MAGNETS 8
    #define MANUAL_SPEED false
    #define KFEM 44.1
    #define K_SPEED_P 310
    #define K_SPEED_I 150
    #define K_POS_P 0 // 1.5
    #define K_POS_I 0 // 0.1
#endif
