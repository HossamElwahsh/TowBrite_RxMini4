//
// Created by hossam on 3/3/2024.
//

#ifndef WDT_CONFIG_H
#define WDT_CONFIG_H

/** Typedefs */
typedef enum
{
    WD_PERIOD_1S = 0,
    WD_PERIOD_250MS = 1,
    WD_PERIOD_16MS = 2,
    WD_PERIOD_2MS = 3
}en_wdt_period_t;


/** Configs */
en_wdt_period_t en_wdt_period = WD_PERIOD_1S;

#endif //WDT_CONFIG_H
