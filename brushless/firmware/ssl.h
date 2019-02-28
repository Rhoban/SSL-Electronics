#define MOTEUR_BANC_60
//#define CYCLOIDE_30

#ifdef CYCLOIDE_30
    #define STOP_OUTSIDE_LIMITS 
    #define MOTOR_NAME "CYCLOIDE_30"
    #define CONFIG_PWM 30
    #define NB_POSITIVE_MAGNETS 7
    #define REVERSE_PHASE
    #define MANUAL_SPEED false
#endif

#ifdef MOTEUR_BANC_60
    #define MOTOR_NAME "MOTEUR_BANC_60"
    #define CONFIG_PWM 60
    #define NB_POSITIVE_MAGNETS 8
    #define MANUAL_SPEED true //false
    #define KFEM 29.6
#endif
