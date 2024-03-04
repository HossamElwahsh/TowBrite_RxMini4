//
// Created by hossam on 3/3/2024.
//

#include "wdt_interface.h"
#include "wdt_private.h"
#include "wdt_config.h"

void wdt_init(void)
{
    /* initialize WDT */
    WDCTL = 0x00; /* clear control byte */

    /* set timer interval */
    WDCTL |= en_wdt_period;

    /* no need to set mode to watchdog not timer as bit MODE(2) is reset already to 0 in step 1 here */

    /* turn on watchdog timer */
    WDCTL |= WDCTL_EN;
}

void wdt_reset(void)
{
    /* reset sequence for watchdog */
    WDCTL |= WDT_RST_KEY1;
    WDCTL |= WDT_RST_KEY2;
}