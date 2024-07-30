/**
 * @file    app_cfg
 * @author  Hossam Elwahsh - https://github.com/HossamElwahsh / hossam.e.elwahsh@gmail.com
 * @date    30/7/2024
 */


#ifndef RX_2024_APP_MAGNETIC_CFG_H
#define RX_2024_APP_MAGNETIC_CFG_H


/** --------------- project type --------------- */
/* options */
#define PROJECT_TYPE_BASE_OPT 1
#define PROJECT_TYPE_RX_MINI_4_IN_MAGNETIC_PROXIMITY_OPT 2

/* config */
#define PROJECT_TYPE_CFG PROJECT_TYPE_RX_MINI_4_IN_MAGNETIC_PROXIMITY_OPT


/** --------------- pcb type --------------- */
/* options */
/* sensor is on P0_1 */
#define MAGNETIC_PCB_TYPE_OLD_OPT 0
/* sensor is on P0_7 */
#define MAGNETIC_PCB_TYPE_NEW_OPT 1

/* selection */
#define MAGNETIC_PCB_TYPE MAGNETIC_PCB_TYPE_OLD_OPT


/** magnetic flash helping macros */
#define HIBYTE(a)     (uint8_t) ((uint16_t)(a) >> 8 )
#define LOBYTE(a)     (uint8_t)  (uint16_t)(a)

#define SET_WORD(regH, regL, word) \
   do{                             \
      (regH) = HIBYTE( word );     \
      (regL) = LOBYTE( word );     \
   }while(0)

/** magnetic flash data */
#define APP_MAGNETIC_MODE_FOLLOW_RIGHT      0x0055
#define APP_MAGNETIC_MODE_FOLLOW_LEFT       0xFFAA

#define APP_MAGNETIC_MODE_SIZE_IN_BYTES 2

#define FLASH_FWDATA_ADDR 0xDFAF
#define FLASH_APP_MAGNETIC_MODE_ADDR 0x5400  /* Page 21 start address */
#define FWT_FLASH_CLOCK_ADJUST 0x11 /* system clock is 26MHz */

#define RESTORE_APP_MODE()  flash_read_saved_app_mode()
#define SAVE_APP_MODE()     flash_write_app_mode()

/** --------------- defaults --------------- */
#define APP_MODE_DEFAULT APP_MAGNETIC_MODE_FOLLOW_RIGHT

#endif //RX_2024_APP_MAGNETIC_CFG_H
