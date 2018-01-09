#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include "buzzer.h"

// Config
HardwareTimer           timer(2);

// Partitions
struct buzzer_note {
    unsigned int freq;
    unsigned int duration;
};

static struct buzzer_note melody_boot[] = {
    {523, 200/2},
    {659, 350/2},
    {523, 200/2},
    {698, 300/2},
    {659, 160/2},
    {0, 0}
};

static struct buzzer_note melody_alert[] = {
    {2000, 200},
    {200, 200},
    {2000, 200},
    {200, 200},
    {0, 0}
};

static struct buzzer_note melody_alert_fast[] = {
    {2000, 100},
    {200, 100},
    {2000, 100},
    {200, 100},
    {0, 0}
};

static struct buzzer_note melody_warning[] = {
    {800, 200},
    {400, 200},
    {200, 0},
    {200, 400},
    {0, 0}
};

static struct buzzer_note melody_begin[] = {
    {800, 200},
    {0, 200},
    {800, 200},
    {0, 0},
};

static struct buzzer_note melody_custom[] = {
    {0, 0},
    {0, 0}
};

// Status
static struct buzzer_note *melody;
static struct buzzer_note *melody_repeat;
static int melody_st;

void buzzer_init()
{
    melody = NULL;
    pinMode(BUZZER_PIN, PWM);
    pwmWrite(BUZZER_PIN, 0);
}

void buzzer_play_note(int note)
{
    timer.pause();
    timer.setPrescaleFactor(72000000 / (note * 100));
    timer.setOverflow(100);

    if (note == 0) {
        pinMode(BUZZER_PIN, OUTPUT);
        digitalWrite(BUZZER_PIN, LOW);
    } else {
        timer.refresh();
        timer.resume();
        pinMode(BUZZER_PIN, PWM);
        pwmWrite(BUZZER_PIN, 50);
    }
}

static void buzzer_enter(struct buzzer_note *note)
{
    buzzer_play_note(note->freq);
    melody = note;
    melody_st = millis();

    if (note->freq == 0 && note->duration == 0) {
        if (melody_repeat != NULL) {
            buzzer_enter(melody_repeat);
        } else {
            melody = NULL;
        }
    }
}

void buzzer_play(unsigned int melody_num, bool repeat)
{
    struct buzzer_note *to_play = NULL;

    if (melody_num == MELODY_BOOT) {
        to_play = &melody_boot[0];
    } else if (melody_num == MELODY_ALERT) {
        to_play = &melody_alert[0];
    } else if (melody_num == MELODY_ALERT_FAST) {
        to_play = &melody_alert_fast[0];
    } else if (melody_num == MELODY_WARNING) {
        to_play = &melody_warning[0];
    } else if (melody_num == MELODY_BEGIN) {
        to_play = &melody_begin[0];
    } else if (melody_num == MELODY_CUSTOM) {
        to_play = &melody_custom[0];
    } else {
        melody = NULL;
    }

    if (to_play) {
        melody_repeat = repeat ? to_play : NULL;
        buzzer_enter(to_play);
    }
}

void buzzer_tick()
{
    if (melody != NULL) {
        if (millis()-melody_st > melody->duration) {
            buzzer_enter(melody+1);
        }
    }
}

void buzzer_stop()
{
    buzzer_play_note(0);
    melody = NULL;
    melody_repeat = NULL;
}

bool buzzer_is_playing()
{
    return melody != NULL;
}

void buzzer_wait_play()
{
    while (buzzer_is_playing()) {
        buzzer_tick();
    }
}

void buzzer_beep(unsigned int freq, unsigned int duration)
{
    melody_custom[0].freq = freq;
    melody_custom[0].duration = duration;
    buzzer_play(MELODY_CUSTOM);
}

#ifdef HAS_TERMINAL
TERMINAL_COMMAND(play, "Play a melody")
{
    int melnum = atoi(argv[0]);
    terminal_io()->print("Playing melody ");
    terminal_io()->print(melnum);
    terminal_io()->println();
    buzzer_play(melnum);
}

TERMINAL_COMMAND(beep, "Plays a beep")
{
    if (argc == 2) {
        buzzer_beep(atoi(argv[0]), atoi(argv[1]));
    } else if (argc == 1) {
        buzzer_beep(atoi(argv[0]), 1000);
    } else {
        terminal_io()->println("Usage: beep freq [duration]");
    }
}
#endif
