#ifndef _FUNCONFIG_H
#define _FUNCONFIG_H

#ifndef CH32V003
#define CH32V003
#endif

#define FUNCONF_USE_DEBUGPRINTF 0

#define FUNCONF_DEBUGPRINTF_TIMEOUT (1<<31) // Wait for a very very long time.

#define FUNCONF_HSE_BYPASS 1
#define FUNCONF_USE_HSE 1
#define FUNCONF_USE_PLL 1
#define HSE_VALUE 10000000UL

#endif
