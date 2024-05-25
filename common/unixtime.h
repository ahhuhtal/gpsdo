#ifndef UNIXTIME_H_
#define UNIXTIME_H_

#include <inttypes.h>
#include <stdbool.h>

struct utc_time {
    int Y;
    int M;
    int D;
    int h;
    int m;
    int s;
};

uint32_t unix_from_utc(struct utc_time utc);
void utc_from_unix(uint32_t timestamp, struct utc_time* utc);

#endif /* UNIXTIME_H_ */
