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

/*
 * Move epoch from 01.01.1970 to 01.03.0000 (yes, Year 0) - this is the first
 * day of a 400-year long "era", right after additional day of leap year.
 * This adjustment is required only for date calculation, so instead of
 * modifying time_t value (which would require 64-bit operations to work
 * correctly) it's enough to adjust the calculated number of days since epoch.
 */
#define EPOCH_ADJUSTMENT_DAYS INT32_C(719468)
/* year to which the adjustment was made */
#define ADJUSTED_EPOCH_YEAR 0
/* there are 97 leap years in 400-year periods. ((400 - 97) * 365 + 97 * 366) */
#define DAYS_PER_ERA INT32_C(146097)
/* there are 24 leap years in 100-year periods. ((100 - 24) * 365 + 24 * 366) */
#define DAYS_PER_CENTURY INT32_C(36524)
/* there is one leap year every 4 years */
#define DAYS_PER_4_YEARS (3 * 365 + 366)
/* number of days in a non-leap year */
#define DAYS_PER_YEAR 365
/* number of years per era */
#define YEARS_PER_ERA 400

void utc_from_unix(uint32_t timestamp, struct utc_time *utc)
{
    int32_t days = timestamp / SEC_IN_DAY + EPOCH_ADJUSTMENT_DAYS;
    int32_t remainder = timestamp % SEC_IN_DAY;

    utc->h = remainder / SEC_IN_HOUR;
    remainder %= SEC_IN_HOUR;
    utc->m = remainder / SEC_IN_MINUTE;
    utc->s = remainder % SEC_IN_MINUTE;

    // http://howardhinnant.github.io/date_algorithms.html#civil_from_days
    int32_t era = days / DAYS_PER_ERA;
    uint32_t era_day = days - era * DAYS_PER_ERA; // [0, 146096]
    uint32_t era_year =
        (era_day - (era_day / (DAYS_PER_4_YEARS - 1)) + (era_day / DAYS_PER_CENTURY) - (era_day / (DAYS_PER_ERA - 1))) / 365; // [0, 399]
    uint32_t year_day = era_day - (DAYS_PER_YEAR * era_year + era_year / 4 - era_year / 100); // [0, 365]
    uint32_t month = month = (5 * year_day + 2) / 153; // [0, 11]
    uint32_t day = year_day - (153 * month + 2) / 5; // [0, 30]
    month += (month < 10) ? 2 : -10;
    int32_t year = ADJUSTED_EPOCH_YEAR + era_year + era * YEARS_PER_ERA + (month <= 1);

    utc->Y = year;
    utc->M = month + 1; // shift to [1, 12]
    utc->D = day + 1; // shift to [1, 31]
}
