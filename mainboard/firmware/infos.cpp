#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include <flash_write.h>
#include "hardware.h"
#include "mux.h"
#include "infos.h"

// Flash address to write/read infos
#define INFOS_FLASH_ADDR    0x0801FC00
extern bool barbu_mode;
bool developer_mode=false;

int IDs[11]={7,0,1,8,2,4,9,10,3,5,6}; //The coding of IDs from hall values

int id_from_hall(bool hall1, bool hall2, bool hall3, bool hall4)
{
  int index=(hall1) + (hall2<<1) + (hall3<<2) + (hall4<<3);
  if(index<11){
    return IDs[index];
    }
  else{
    return -1;
  }
}

struct robot_infos
{
  int id;
  bool kickerInverted;
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

bool infos_kicker_inverted()
{
    return infos.kickerInverted;
}

TERMINAL_COMMAND(infos, "ID")
{
    terminal_io()->println("ID:");
    terminal_io()->println(infos.id);

    terminal_io()->println("Kicker inverted:");
    terminal_io()->println((int)infos.kickerInverted);

}

void infos_set(int id, bool kickerInverted)
{
    infos.id = id;
    infos.kickerInverted = kickerInverted;
    flash_write(INFOS_FLASH_ADDR, (void *)&infos, sizeof(infos));
}
