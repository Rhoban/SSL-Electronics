#include "assert.h"
#include "buzzer.h"
#include <watchdog.h>

int bassert_failed(){
    while(true){
        buzzer_play(MELODY_ASSERT, false);
        buzzer_wait_play();
    }
    return 0;
}
