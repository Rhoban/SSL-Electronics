#pragma once

int bassert_failed();

#ifdef NDEBUG
#define bassert(cond) ((void) (0))
#else
#define bassert(cond) cond?0:bassert_failed()

#endif
