#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include <flash_write.h>
#include "hardware.h"
#include "mux.h"
#include "infos.h"

// Flash address to write/read infos
#define INFOS_FLASH_ADDR    0x0801FC00

struct robot_infos
{
    int id;
};

static struct robot_infos infos;

void infos_init()
{
    infos.id = 1;

    flash_read(INFOS_FLASH_ADDR, (void *)&infos, sizeof(infos));
}


int infos_get_id()
{
    return infos.id;
}

TERMINAL_COMMAND(id, "ID")
{
    terminal_io()->println(infos.id);

    terminal_io()->print("Hall #1: ");
    terminal_io()->println(mux_sample(HALL1_ADDR));

    terminal_io()->print("Hall #2: ");
    terminal_io()->println(mux_sample(HALL2_ADDR));

    terminal_io()->print("Hall #3: ");
    terminal_io()->println(mux_sample(HALL3_ADDR));

    terminal_io()->print("Hall #4: ");
    terminal_io()->println(mux_sample(HALL4_ADDR));
}

void infos_set_id(int id)
{
    infos.id = id;
    flash_write(INFOS_FLASH_ADDR, (void *)&infos, sizeof(infos));
}

TERMINAL_COMMAND(setid, "Set ID in the persistent flash")
{
    if (argc) {
        infos_set_id(atoi(argv[0]));
    }
}
