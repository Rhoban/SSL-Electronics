// #define MOTEUR_BANC_20
//#define MOTEUR_BANC_30
//#define MOTEUR_BANC_40
//#define MOTEUR_BANC_60
//#define MOTEUR_ROBOT_4_1_30
//#define MOTEUR_ROBOT_4_2_30
//#define MOTEUR_ROBOT_4_3_30
//#define MOTEUR_ROBOT_4_4_30
#define CYCLOIDE_60
//#define CYCLOIDE_30

#define GLOBAL_MIN_SPEED_HYST 0.6
#define GLOBAL_MAX_SPEED_HYST 0.8

#define GLOBAL_LOW_SPEED_PWM 23
#define MAXIMAL_LOW_SPEED_PWM 60

#define MAX_SPEED_CONSIGN 10.0 // Nb turn . s^-1

#define MAX_THETA_LIMIT 4.4
#define MIN_THETA_LIMIT -5.5

#ifdef CYCLOIDE_60
    //#define USE_HIGH_SPEED_MODE
    #define STOP_OUTSIDE_LIMITS

    #define MOTOR_NAME "CYCLOIDE_60"
    #define CONFIG_PWM 60
    #define CONFIG_LOW_SPEED_PWM GLOBAL_LOW_SPEED_PWM
    #define ALPHA 0.36
    #define K_SPEED_P 1.1 //3.0
    #define K_SPEED_I 0.02 //0.001
    #define K_SPEED_D 0.0
    #define K_POS_P 10.0
    #define K_POS_I 0.002
    #define K_POS_D 0.0
    #define MANUAL_SPEED false
    #define INITIAL_SPEED 0.0
    #define MIN_SPEED_HYST GLOBAL_MIN_SPEED_HYST 
    #define MAX_SPEED_HYST GLOBAL_MAX_SPEED_HYST 
    #define NB_POSITIVE_MAGNETS 7
    //#define NB_POSITIVE_MAGNETS 8
    #define REVERSE_PHASE
#endif

#ifdef CYCLOIDE_30
    //#define USE_HIGH_SPEED_MODE
    #define STOP_OUTSIDE_LIMITS

    #define MOTOR_NAME "CYCLOIDE_30"
    #define CONFIG_PWM 30
    #define CONFIG_LOW_SPEED_PWM GLOBAL_LOW_SPEED_PWM
    #define ALPHA 0.87
    #define K_SPEED_P 2.0 //3.0
    #define K_SPEED_I 0.035 //0.001
    #define K_SPEED_D 0.0
    #define K_POS_P 30.0
    #define K_POS_I 0.4
    #define K_POS_D 0.0
    #define MANUAL_SPEED false
    #define INITIAL_SPEED 0.0
    #define MIN_SPEED_HYST GLOBAL_MIN_SPEED_HYST 
    #define MAX_SPEED_HYST GLOBAL_MAX_SPEED_HYST 
    #define NB_POSITIVE_MAGNETS 7
    //#define NB_POSITIVE_MAGNETS 8
    #define REVERSE_PHASE
#endif


#ifdef MOTEUR_ROBOT_4_1_30
    #define MOTOR_NAME "MOTEUR_ROBOT_4_1_30"
    #define CONFIG_PWM 30
    #define CONFIG_LOW_SPEED_PWM GLOBAL_LOW_SPEED_PWM
    #define ALPHA 0.451
    #define K_SPEED_P 1.5 //3.0
    #define K_SPEED_D 0.0
    #define K_SPEED_I 0.01 //0.001
    #define K_POS_P 0.0
    #define K_POS_I 0.0
    #define K_POS_D 0.0
    #define MANUAL_SPEED true //false
    #define INITIAL_SPEED 0.0
    #define MIN_SPEED_HYST GLOBAL_MIN_SPEED_HYST 
    #define MAX_SPEED_HYST GLOBAL_MAX_SPEED_HYST 
    #define NB_POSITIVE_MAGNETS 8
#endif

#ifdef MOTEUR_ROBOT_4_2_30
    #define MOTOR_NAME "MOTEUR_ROBOT_4_2_30"
    #define CONFIG_PWM 30
    #define CONFIG_LOW_SPEED_PWM GLOBAL_LOW_SPEED_PWM
    #define ALPHA 0.472
    #define K_SPEED_P 1.5
    #define K_SPEED_D 0.0
    #define K_SPEED_I 0.01
    #define K_POS_P 0.0
    #define K_POS_I 0.0
    #define K_POS_D 0.0
    #define MANUAL_SPEED true //false
    #define INITIAL_SPEED 0.0
    #define MIN_SPEED_HYST GLOBAL_MIN_SPEED_HYST 
    #define MAX_SPEED_HYST GLOBAL_MAX_SPEED_HYST 
    #define NB_POSITIVE_MAGNETS 8
#endif

#ifdef MOTEUR_ROBOT_4_3_30
    #define MOTOR_NAME "MOTEUR_ROBOT_4_3_30"
    #define CONFIG_PWM 30
    #define CONFIG_LOW_SPEED_PWM GLOBAL_LOW_SPEED_PWM
    #define ALPHA 0.453
    #define K_SPEED_P 1.5
    #define K_SPEED_D 0.0
    #define K_SPEED_I 0.01
    #define K_POS_P 0.0
    #define K_POS_I 0.0
    #define K_POS_D 0.0
    #define MANUAL_SPEED true //false
    #define INITIAL_SPEED 0.0
    #define MIN_SPEED_HYST GLOBAL_MIN_SPEED_HYST 
    #define MAX_SPEED_HYST GLOBAL_MAX_SPEED_HYST 
    #define NB_POSITIVE_MAGNETS 8
#endif

#ifdef MOTEUR_ROBOT_4_4_30
    #define MOTOR_NAME "MOTEUR_ROBOT_4_4_30"
    #define CONFIG_PWM 30
    #define CONFIG_LOW_SPEED_PWM GLOBAL_LOW_SPEED_PWM
    #define ALPHA 0.45
    #define K_SPEED_P 1.5
    #define K_SPEED_D 0.0
    #define K_SPEED_I 0.01
    #define K_POS_P 0.0
    #define K_POS_I 0.0
    #define K_POS_D 0.0
    #define MANUAL_SPEED true //false
    #define INITIAL_SPEED 0.0
    #define MIN_SPEED_HYST GLOBAL_MIN_SPEED_HYST 
    #define MAX_SPEED_HYST GLOBAL_MAX_SPEED_HYST 
    #define NB_POSITIVE_MAGNETS 8
#endif


#ifdef MOTEUR_BANC_20
    #define MOTOR_NAME "MOTEUR_BANC_20"
    #define CONFIG_PWM 20
    #define CONFIG_LOW_SPEED_PWM GLOBAL_LOW_SPEED_PWM
    #define ALPHA 1.36
    #define K_SPEED_P 3.0 //3.0
    #define K_SPEED_D 0.0
    #define K_SPEED_I 0.02 //0.001
    #define K_POS_P 0.0
    #define K_POS_I 0.0
    #define K_POS_D 0.0
    #define MANUAL_SPEED true //false
    #define INITIAL_SPEED 1.0
    #define MIN_SPEED_HYST GLOBAL_MIN_SPEED_HYST 
    #define MAX_SPEED_HYST GLOBAL_MAX_SPEED_HYST 
    #define NB_POSITIVE_MAGNETS 8
#endif

#ifdef MOTEUR_BANC_30
    #define MOTOR_NAME "MOTEUR_BANC_30"
    #define CONFIG_PWM 30
    #define CONFIG_LOW_SPEED_PWM GLOBAL_LOW_SPEED_PWM
    #define ALPHA 0.83
    #define K_SPEED_P 3.0 //3.0
    #define K_SPEED_D 0.0
    #define K_SPEED_I 0.01 //0.001
    #define K_POS_P 0.0
    #define K_POS_I 0.0
    #define K_POS_D 0.0
    #define MANUAL_SPEED true //false
    #define INITIAL_SPEED 1.0
    #define MIN_SPEED_HYST GLOBAL_MIN_SPEED_HYST 
    #define MAX_SPEED_HYST GLOBAL_MAX_SPEED_HYST 
    #define NB_POSITIVE_MAGNETS 8
#endif

#ifdef MOTEUR_BANC_40
    #define MOTOR_NAME "MOTEUR_BANC_40"
    #define CONFIG_PWM 40
    #define CONFIG_LOW_SPEED_PWM GLOBAL_LOW_SPEED_PWM
    #define ALPHA 0.673
    #define K_SPEED_P 3.0 //3.0
    #define K_SPEED_D 0.0
    #define K_SPEED_I 0.04 //0.001
    #define K_POS_P 0.0
    #define K_POS_I 0.0
    #define K_POS_D 0.0
    #define MANUAL_SPEED true //false
    #define INITIAL_SPEED 1.0
    #define MIN_SPEED_HYST GLOBAL_MIN_SPEED_HYST 
    #define MAX_SPEED_HYST GLOBAL_MAX_SPEED_HYST 
    #define NB_POSITIVE_MAGNETS 8
#endif

#ifdef MOTEUR_BANC_60
    #define MOTOR_NAME "MOTEUR_BANC_60"
    #define CONFIG_PWM 60
    #define CONFIG_LOW_SPEED_PWM GLOBAL_LOW_SPEED_PWM
    #define ALPHA 0.42
    #define K_SPEED_P 3.0 //3.0
    #define K_SPEED_D 0.0
    #define K_SPEED_I 0.01 //0.001
    #define K_POS_P 0.0
    #define K_POS_I 0.0
    #define K_POS_D 0.0
    #define MANUAL_SPEED true //false
    #define INITIAL_SPEED 1.0
    #define MIN_SPEED_HYST GLOBAL_MIN_SPEED_HYST 
    #define MAX_SPEED_HYST GLOBAL_MAX_SPEED_HYST 
    #define NB_POSITIVE_MAGNETS 8
#endif

#if MANUAL_SPEED == false
    #define USE_HIGH_SPEED_MODE
#endif
