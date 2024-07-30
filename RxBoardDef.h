#ifndef RXBOARDDEF_H
#define RXBOARDDEF_H

#include "ioCC1110.h"
#include "hal_types.h"
#include "ioCCxx10_bitdef.h"
#include "RADIO_CC1110_GFSK.h"
#include "sdcc_dma.h"
#include "MCAL/wdt/wdt_interface.h"
#include "app_magnetic_cfg.h"
#include "bit_math.h"
#include "std.h"

#define PROGRAMM_VERSION        3

//#define MY_MAC_ADDRESS           0x847BA8C0  /* moved to selfAddress[] below | oldTodo> change to random (new RX) and drop verification on signal destination */
#define NOTIFIER_TAG_MAC_ADDRESS          0x98765432 /* destination for heartbeat address - >>constant<< - matching notifier chip address */

#if PROJECT_TYPE_CFG == PROJECT_TYPE_BASE_OPT
#define LED_TXRX                0x01
#define LED_PAIR                0x02

#define RIGHT_TURN_SIGNAL       0x02
#define BACK_SIGNAL             0x04
#define BRAKE_SIGNAL            0x08
#define LEFT_TURN_SIGNAL        0x10
#define ALL_SIGNAL              0x1E

#elif PROJECT_TYPE_CFG == PROJECT_TYPE_RX_MINI_4_IN_MAGNETIC_PROXIMITY_OPT

/* Indicator leds for current follow mode
 * if rx is currently following left, left led should be always on
 * similarly for right led
 *
 * current mode can be changed using a magnet
 * held for more than 5 seconds during the first 15 sec of startup
 * */
#define IND_LED_MODE_LEFT_PIN 3
#define IND_LED_MODE_RIGHT_PIN 4
#define IND_LED_MODE_LEFT_PIN_MASK 0x08
#define IND_LED_MODE_RIGHT_PIN_MASK 0x10
#define IND_LED_MODE_LEFT_PORT P1
#define IND_LED_MODE_RIGHT_PORT P1

/* main output led that should output the current received signal from the tx
 * with respect to the current set mode
 * eg. if current mode is (follow right), the led should follow all received right signals
 * */
#define LED_MAIN_OUTPUT_PORT P0

#if MAGNETIC_PCB_TYPE == MAGNETIC_PCB_TYPE_OLD_OPT
#define LED_MAIN_OUTPUT_PIN 4
#define LED_MAIN_OUTPUT_PIN_MASK 0x10

#define PROX_SENSOR_PIN 1
#define PROX_SENSOR_PIN_MASK 0x02
#elif MAGNETIC_PCB_TYPE == MAGNETIC_PCB_TYPE_NEW_OPT
#define LED_MAIN_OUTPUT_PIN 5
#define LED_MAIN_OUTPUT_PIN_MASK 0x20

#define PROX_SENSOR_PIN 7
#define PROX_SENSOR_PIN_MASK 0x80
#endif
#define PROX_SENSOR_PORT P0

/* magnet is active low
 * low: magnetic field detected
 * high: magnetic field not detected
 * hence the xor with 1 so that the status be high when there's field and vice versa
 * */
#define GET_MAGNETIC_SENSOR_STATUS() (GET_BIT(PROX_SENSOR_PORT, PROX_SENSOR_PIN) ^ 0x01)

/* time to allow follow signal change (right/left) */
#define TIME_ALLOWED_MAGNETIC_CHANGE_10MS 1500 /* (in 10ms counts) therefore (1500) * 10ms = 15,000 = 15s */
#define TIME_HOLD_REQ_FOR_MAGNETIC_CHANGE_10MS 500 /* (in 10ms counts) therefore (500) * 10ms = 5,000 = 5s */

#endif

#define ACC_VCNTRL              0x04 /* verified - compatible with rx mini 4 */

#define WATCHDOG_MSK            0x10
#define WDT_ENABLE_1sec         (WDCTL_EN | WDCTL_INT_SEC_1)
#define WDT_RST_KEY1            (0xA0 | WDCTL_EN | WDCTL_INT_SEC_1)
#define WDT_RST_KEY2            (0x50 | WDCTL_EN | WDCTL_INT_SEC_1)
#define TIME_SLOT               55                                              // real time-slot = TIME_SLOT * 10 ms

#define ACC_25_PERCENT          738
#define ACC_50_PERCENT          750
#define ACC_75_PERCENT          760
#define FLASHING_DELAY          60000                                          // 130k value for 500 ms delay if Fclk = 26 MHz

#define HEART_BEAT_SIGNAL        0xA5

#define SYSMODE_RX              0
#define SYSMODE_TX              1

/* time to allow pairing at startup */
#define TIME_PAIRING_ALLOWED_10MS 1500 /* (in 10ms counts) therefore (1500) * 10ms = 15,000 = 15s */

#define ERASED_PAGE             7
#define PAGE_SIZE               1024
#define FLASH_PAGE_ADDR         (uint16_t)(ERASED_PAGE * PAGE_SIZE)
__no_init const uint8_t __code flashDataAddr[PAGE_SIZE] @FLASH_PAGE_ADDR;       // The area in flash where data written to flash will be placed.


#define DEFAULT_SELF_ADDRESS                    0xEF, 0xBE, 0xAD, 0xDE
#define FLASH_SELF_PAGE_ADDR  0x5000  /** start of page 20 - must be a start of a page */

//__no_init const uint8_t __code flashDataAddr[PAGE_SIZE] @FLASH_PAGE_ADDR;       // The area in flash where data written to flash will be placed.

/** static flash code */
/* default source pair address */
static const uint8_t __code selfAddress[] @FLASH_SELF_PAGE_ADDR = {DEFAULT_SELF_ADDRESS};

///* self address - page 9 */
//static const uint8_t selfAddress[] @0x2400 = {0xEF, 0xBE, 0xAD, 0xDE};

#define RAM_BUF_SIZE            128


/** Main Status Flags  ************************************************/
volatile struct {
  uint8_t  RF_TIME_EXPIRED      : 1;        
  uint8_t  RF_PACKET_DONE       : 1;
  uint8_t  RF_PACKET_ERROR      : 1;
  uint8_t  RF_DEVICE_PAIRED     : 1;
  uint8_t  HEART_BIT_ALARM      : 1;
  uint8_t  STORE_NEW_TX         : 1;
  uint8_t  FIRST_PACKET         : 1;
}Flags;

/* Types definitions **************************************************/ 
typedef struct {
  uint8_t   dstAddr[4];
  uint8_t   srcAddr[4];
  uint8_t   data_0;
  uint8_t   data_1;
} TRfPacket;

#define RX_PACKET_LENGTH   sizeof(TRfPacket)
#define TX_PACKET_LENGTH   sizeof(TRfPacket)

typedef enum {
  PS_ERROR,
  PS_NO_ERROR,
  PS_PAIR_ADDRESS
} TParseStatus;

typedef enum {
  SYSMODE_RX_WAIT,
  SYSMODE_RX_DELAY,
  SYSMODE_TX_PREPARE,
  SYSMODE_TX_TRANS,
  SYSMODE_TX_TRANS_WAIT
} TSystemStatus;


typedef struct {
  uint8_t lowByte;
  uint8_t highByte;
}TAdcResult;

union {
  TAdcResult  adcResult;
  uint16_t    adcData;
}unAcc;


TSystemStatus      sysMode;
volatile uint32_t  sysTick;
uint32_t           sysTickBuf;
uint8_t            blinkMask;
uint8_t            transRtry;

uint16_t accVoltage;
uint8_t  transmitterAddress[4];
uint8_t  rfdCount;
uint8_t  storeTxAddrCount;
uint8_t  rfBuffer[16];
uint16_t __xdata ramFuncAddr[RAM_BUF_SIZE];                              // RAM buffer for function
uint8_t  watchdogReset;

TRfPacket*  rfPacket = (TRfPacket*)rfBuffer;

/** flavored special typedefs / macros / functions */
#if PROJECT_TYPE_CFG == PROJECT_TYPE_RX_MINI_4_IN_MAGNETIC_PROXIMITY_OPT

/* typedefs */
//typedef enum
//{
//    FOLLOW_RIGHT_SIG = 0,
//    FOLLOW_LEFT_SIG,
//    FOLLOW_TOTAL
//}en_follow_mode_t;

typedef enum
{
    MAGNET_STATUS_NOT_PRESENT = 0,
    MAGNET_STATUS_PRESENT,
    MAGNET_STATUS_TOTAL
}en_magnet_status_t;

typedef enum
{
    INT_FALLING_EDGE = 0,
    INT_RISING_EDGE
}en_current_mag_sensor_interrupt_mode_t;

typedef struct
{
    uint8_t low_byte;
    uint8_t high_byte;
}st_app_mag_mode_bytes_t;

typedef union
{
    uint8_t app_mode_arr[APP_MAGNETIC_MODE_SIZE_IN_BYTES];
    uint16_t u16_app_mode;
    st_app_mag_mode_bytes_t st_app_mode_bytes;
}un_mag_app_mode_t;

/* private variables */
static en_current_mag_sensor_interrupt_mode_t en_gl_current_mag_sensor_interrupt_mode = INT_FALLING_EDGE;

/* macros */
#define ENABLE_INT_P0()     IEN1 |= IEN1_P0IE
#define DISABLE_INT_P0()    IEN1 &= ~IEN1_P0IE

#define INT_P0_ON_FALLING_EDGE()    DISABLE_INT_P0(); \
                                    PICTL |= PICTL_P0ICON; \
                                    ENABLE_INT_P0();  \
                                    en_gl_current_mag_sensor_interrupt_mode = INT_FALLING_EDGE

#define INT_P0_ON_RISING_EDGE()    DISABLE_INT_P0(); \
                                    PICTL &= ~PICTL_P0ICON; \
                                    ENABLE_INT_P0(); \
                                    en_gl_current_mag_sensor_interrupt_mode = INT_RISING_EDGE

/* functions */
static void init_magnetic_sensor_interrupts();
static void app_switch_follow_signal();

/* flash APIs - app mode */
static void flash_read_saved_app_mode(void);
static void flash_erase_app_mode_page(void);
static void flash_write_app_mode(void);
static void app_defaults_init();
static void dma0_init(void);
static void update_magnetic_mode_ui(void);
#endif


/** Functions prototypes *****************************************************************/
extern void setSystemClock(void);
extern void mcuPinInit(void);
extern void radioInit(void);
extern void adcInit(void);
extern void systemStart(void);
extern void switchToIdle(void);
extern void getAccVoltage(void);
extern void eraseFlashPage(uint16_t pageAddr);
extern void writeTxAddressToFlash(uint16_t dataAddr);
extern void runFunctionFromRam(void (*func)(uint16_t param), uint16_t __xdata *ramAddr, uint16_t funcLength, uint16_t funcParam);
extern void accumVoltageIndication(void);
extern void programVersionIndication(void);
extern void readTxAddrFromFlash(uint32_t* pTxAddr, uint32_t* pFlashAddr);
extern void storeNewTxToFlash(void);
#endif