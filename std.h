#ifndef TX_PRODUCTION_V3P04_MED_FARES_AISSA_STD_H
#define TX_PRODUCTION_V3P04_MED_FARES_AISSA_STD_H

/* @filename 	: 	std.h
 * @brief		:	Holds standard typedefs/macros/etc. and helping macros
 * @author		:	Hossam Elwahsh - https://github.com/HossamElwahsh
 *
 * */

typedef unsigned char boolean;
//typedef unsigned char uint8_t;
//typedef unsigned short uint16_t;
//typedef unsigned long   uint32_t;


#define STATIC static
#define BOOLEAN boolean

#define TRUE 	(1)
#define FALSE 	(0)
#define ZERO	(0)
#define NULL_PTR ((void *)0)
#define MOD(val, mod_with) (val % mod_with)
#define BYTE_MAX_VAL 0xFF
#define UINT32_MAX_VAL 0xFFFFFFFF

#define MAX_PERCENTAGE 						100UL

#define SIZE_ONE_BIT 1
#define TOGGLE(x) x = !x
#define INC(x) x = x + 1
#define DEC(x) x = x - 1

#define ENABLED     (1)
#define DISABLED    (0)


#endif //TX_PRODUCTION_V3P04_MED_FARES_AISSA_STD_H
