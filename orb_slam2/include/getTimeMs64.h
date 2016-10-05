#ifndef GETTIMEMS64_H
#define GETTIMEMS64_H

#include <sys/time.h>
#include <ctime>

/* Returns the amount of milliseconds elapsed since the UNIX epoch. */
unsigned long long GetTimeMs64();

#endif
