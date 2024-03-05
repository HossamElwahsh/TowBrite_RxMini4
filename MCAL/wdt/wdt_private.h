//
// Created by hossam on 3/3/2024.
//

#ifndef WDT_PRIVATE_H
#define WDT_PRIVATE_H

#include "ioCC1110.h"
#include "../../ioCCxx10_bitdef.h"

#define WDT_RST_KEY1            (0xA0 | WDCTL_EN | WDCTL_INT_SEC_1)
#define WDT_RST_KEY2            (0x50 | WDCTL_EN | WDCTL_INT_SEC_1)

#define WDT_RST_KEY_STEP_1            (0xA0)
#define WDT_RST_KEY_STEP_2            (0x50)

#endif //WDT_PRIVATE_H
