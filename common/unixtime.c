#include "unixtime.h"

static const int DAYS_IN_MONTH[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
static const int DAYS_BEFORE_MONTH[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

#define SEC_IN_MINUTE INT32_C(60)
#define SEC_IN_HOUR INT32_C(3600)
#define SEC_IN_DAY INT64_C(86400)

static bool is_leap_year(uint16_t year) {
    return (year % 4) == 0 && ((year % 100) != 0 || (year % 400) == 0);
}

uint32_t unix_from_utc(int Y, int M, int D, int h, int m, int s) {

    D -= 1; // shift to [0, 30]
    M -= 1; // shift to [0, 11]

    // Check that the UTC time is valid.
    if (s >= 60 || m >= 60 || h >= 24 || M >= 12) {
        return 0;
    }

    bool leap_year = is_leap_year(Y);

    // Check that the days are valid.
    if (M == 1 && leap_year) {
        if (D >= 29) {
            return 0;
        }
    } else {
        if (D >= DAYS_IN_MONTH[M]) {
            return 0;
        }
    }

    // Compute number of full days since epoch (begin of 1970).
    uint32_t days_since_epoch = D;
    days_since_epoch += DAYS_BEFORE_MONTH[M];
    if (leap_year && M > 1) {
        days_since_epoch += 1;
    }
    for (int32_t y = 1970; y < Y; y++) {
        days_since_epoch += (is_leap_year(y) ? 366 : 365);
    }

    int64_t time = s + (SEC_IN_MINUTE * m) + (SEC_IN_HOUR * h);
    time += SEC_IN_DAY * (int64_t) days_since_epoch;

    if (time > UINT32_MAX) {
        return 0;
    }

    return time;
}
