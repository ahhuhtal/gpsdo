#ifndef UNIXTIME_H_
#define UNIXTIME_H_

#include <inttypes.h>
#include <stdbool.h>

uint32_t unix_from_utc(int Y, int M, int D, int h, int m, int s);

#endif /* UNIXTIME_H_ */
