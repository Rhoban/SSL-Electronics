//#define MOTEUR_BANC_60
#define CYCLOIDE_60


#define MAX_THETA_LIMIT 3.5
#define MIN_THETA_LIMIT -5.0


#ifdef CYCLOIDE_60
    #define DO_NOT_USE_HALL
    #define STOP_OUTSIDE_LIMITS 
    #define MOTOR_NAME "CYCLOIDE_60"
    #define CONFIG_PWM 60
    #define NB_POSITIVE_MAGNETS 7
    #define REVERSE_PHASE
    #define MANUAL_SPEED false
    #define KFEM 26.5 //29.6
    #define K_SPEED_P 110.0 //180
    #define K_SPEED_I 200.0 //150
    #define K_POS_P 1.5 //1.5 // 1.5
    #define K_POS_I 0.02 //0.1 // 0.1
#endif

#ifdef MOTEUR_BANC_60
    #define DO_NOT_USE_HALL
    #define STOP_OUTSIDE_LIMITS 
    #define MOTOR_NAME "MOTEUR_BANC_60"
    #define CONFIG_PWM 60
    #define NB_POSITIVE_MAGNETS 8
    #define MANUAL_SPEED false
    #define KFEM 29.6
    #define K_SPEED_P 180
    #define K_SPEED_I 150
    #define K_POS_P 1.5 // 1.5
    #define K_POS_I 0.1 // 0.1
#endif
