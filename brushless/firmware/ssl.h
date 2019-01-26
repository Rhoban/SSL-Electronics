// #define MOTEUR_BANC_20
#define MOTEUR_BANC_30
//#define MOTEUR_BANC_40
//#define MOTEUR_BANC_60
//#define MOTEUR_ROBOT_4_1_30
//#define MOTEUR_ROBOT_4_2_30
//#define MOTEUR_ROBOT_4_3_30
//#define MOTEUR_ROBOT_4_4_30

#ifdef MOTEUR_ROBOT_4_1_30
    #define MOTOR_NAME "MOTEUR_ROBOT_4_1_30"
    #define CONFIG_PWM 30
    #define ALPHA 0.451
    #define K_SPEED_P 1.5 //3.0
    #define K_SPEED_D 0.0
    #define K_SPEED_I 0.01 //0.001
    #define K_POS_P 0.0
    #define K_POS_I 0.0
    #define K_POS_D 0.0
    #define MANUAL_SPEED true //false
    #define INITIAL_SPEED 0.0
#endif

#ifdef MOTEUR_ROBOT_4_2_30
    #define MOTOR_NAME "MOTEUR_ROBOT_4_2_30"
    #define CONFIG_PWM 30
    #define ALPHA 0.472
    #define K_SPEED_P 1.5
    #define K_SPEED_D 0.0
    #define K_SPEED_I 0.01
    #define K_POS_P 0.0
    #define K_POS_I 0.0
    #define K_POS_D 0.0
    #define MANUAL_SPEED true //false
    #define INITIAL_SPEED 0.0
#endif

#ifdef MOTEUR_ROBOT_4_3_30
    #define MOTOR_NAME "MOTEUR_ROBOT_4_3_30"
    #define CONFIG_PWM 30
    #define ALPHA 0.453
    #define K_SPEED_P 1.5
    #define K_SPEED_D 0.0
    #define K_SPEED_I 0.01
    #define K_POS_P 0.0
    #define K_POS_I 0.0
    #define K_POS_D 0.0
    #define MANUAL_SPEED true //false
    #define INITIAL_SPEED 0.0
#endif

#ifdef MOTEUR_ROBOT_4_4_30
    #define MOTOR_NAME "MOTEUR_ROBOT_4_4_30"
    #define CONFIG_PWM 30
    #define ALPHA 0.45
    #define K_SPEED_P 1.5
    #define K_SPEED_D 0.0
    #define K_SPEED_I 0.01
    #define K_POS_P 0.0
    #define K_POS_I 0.0
    #define K_POS_D 0.0
    #define MANUAL_SPEED true //false
    #define INITIAL_SPEED 0.0
#endif


#ifdef MOTEUR_BANC_20
    #define MOTOR_NAME "MOTEUR_BANC_20"
    #define CONFIG_PWM 20
    #define ALPHA 1.36
    #define K_SPEED_P 3.0 //3.0
    #define K_SPEED_D 0.0
    #define K_SPEED_I 0.02 //0.001
    #define K_POS_P 0.0
    #define K_POS_I 0.0
    #define K_POS_D 0.0
    #define MANUAL_SPEED true //false
    #define INITIAL_SPEED 1.0
#endif

#ifdef MOTEUR_BANC_30
    #define MOTOR_NAME "MOTEUR_BANC_30"
    #define CONFIG_PWM 30
    #define ALPHA 0.83
    #define K_SPEED_P 3.0 //3.0
    #define K_SPEED_D 0.0
    #define K_SPEED_I 0.1 //0.001
    #define K_POS_P 0.0
    #define K_POS_I 0.0
    #define K_POS_D 0.0
    #define MANUAL_SPEED true //false
    #define INITIAL_SPEED 1.0
#endif

#ifdef MOTEUR_BANC_40
    #define MOTOR_NAME "MOTEUR_BANC_40"
    #define CONFIG_PWM 40
    #define ALPHA 0.673
    #define K_SPEED_P 3.0 //3.0
    #define K_SPEED_D 0.0
    #define K_SPEED_I 0.04 //0.001
    #define K_POS_P 0.0
    #define K_POS_I 0.0
    #define K_POS_D 0.0
    #define MANUAL_SPEED true //false
    #define INITIAL_SPEED 1.0
#endif

#ifdef MOTEUR_BANC_60
    #define MOTOR_NAME "MOTEUR_BANC_60"
    #define CONFIG_PWM 60
    #define ALPHA 0.673
    #define K_SPEED_P 0.0 //3.0
    #define K_SPEED_D 0.0
    #define K_SPEED_I 0.0 //0.001
    #define K_POS_P 0.0
    #define K_POS_I 0.0
    #define K_POS_D 0.0
    #define MANUAL_SPEED true //false
    #define INITIAL_SPEED 1.0
#endif
