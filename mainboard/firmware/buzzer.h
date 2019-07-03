#ifndef _BUZZER_H
#define _BUZZER_H

// When the robot starts
#define MELODY_BOOT       0

// When the battery is low
#define MELODY_ALERT      1
#define MELODY_ALERT_FAST 2
// When there is a warning
#define MELODY_WARNING    3
// When we start the ID of the motors
#define MELODY_BEGIN      4
// When we start the ID of the motors
#define MELODY_END        5
// A custom melody used by beep
#define MELODY_CUSTOM     6
#define MELODY_BEETHOVEN     7
#define MELODY_BOOT_DEV       8
#define C5 523 //C note in Hz
#define E5 659
#define E5b 622
#define F5 698
#define G5 784
#define C6 1047
#define C7 2093
#define B5b 932
/**
 * Initializes the buzzer
 */
void buzzer_init();

/**
 * Plays a melody
 * @param melody The melody id (MELODY_*)
 * @param repeat Does the melody repeats continuously?
 */
void buzzer_play(unsigned int melody, bool repeat=false);

/**
 * Stops playing any sound
 */
void buzzer_stop();

/**
 * Ticking the buzzer
 */
void buzzer_tick();

/**
 * Is the buzzer plaing?
 */
bool buzzer_is_playing();

/**
 * Wait the end of the play
 */
void buzzer_wait_play();

/**
 * Plays a beep
 * @param freq     The frequency (Hz)
 * @param duration The duration (ms)
 */
void buzzer_beep(unsigned int freq, unsigned int duration);

void buzzer_play_note(int note);

#endif
