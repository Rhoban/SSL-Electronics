#ifndef _INFOS_H
#define _INFOS_H

void infos_init();

void infos_set(int id, bool kickerInverted);
bool infos_kicker_inverted();
int infos_get_id();

#endif
