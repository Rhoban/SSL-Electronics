#define MOTEUR_BANC_30
//#define CYCLOIDE_30

#ifdef CYCLOIDE_30
    #define MOTOR_NAME "CYCLOIDE_30"
    #define CONFIG_PWM 30
    #define NB_POSITIVE_MAGNETS 7
    #define REVERSE_PHASE
#endif

#ifdef MOTEUR_BANC_30
    #define MOTOR_NAME "MOTEUR_BANC_30"
    #define CONFIG_PWM 30
    #define NB_POSITIVE_MAGNETS 8
#endif
