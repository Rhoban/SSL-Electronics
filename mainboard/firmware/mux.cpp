#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include "hardware.h"
#include "mux.h"
#include "watchdog.h"

void mux_init()
{
    pinMode(MUX, INPUT_ANALOG);

    pinMode(ADDR1, OUTPUT);
    pinMode(ADDR2, OUTPUT);
    pinMode(ADDR3, OUTPUT);
}

int mux_sample(int addr)
{
    digitalWrite(ADDR1, ((addr>>0)&1) ? HIGH : LOW);
    digitalWrite(ADDR2, ((addr>>1)&1) ? HIGH : LOW);
    digitalWrite(ADDR3, ((addr>>2)&1) ? HIGH : LOW);

    return analogRead(MUX);
}

TERMINAL_COMMAND(mdb, "Mux debug")
{
    for (int k=0; k<8; k++) {
        terminal_io()->print("Mux #");
        terminal_io()->print(k);
        terminal_io()->print(": ");
        terminal_io()->println(mux_sample(k));
        watchdog_feed();
    }
}
