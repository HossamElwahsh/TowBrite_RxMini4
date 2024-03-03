#ifndef RXBOARDDEF_H
#define RXBOARDDEF_H

#define PROGRAMM_VERSION        3

//#define MY_MAC_ADDRESS           0x847BA8C0  /* moved to selfAddress[] below | oldTodo> change to random (new RX) and drop verification on signal destination */
#define NOTIFIER_TAG_MAC_ADDRESS          0x98765432 /* destination for heartbeat address - >>constant<< - matching notifier chip address */

#define LED_TXRX                0x01
#define LED_PAIR                0x02
#define ACC_VCNTRL              0x04

#define WATCHDOG_MSK            0x10
#define WDT_ENABLE_1sec         (WDCTL_EN | WDCTL_INT_SEC_1)
#define WDT_RST_KEY1            (0xA0 | WDCTL_EN | WDCTL_INT_SEC_1)
#define WDT_RST_KEY2            (0x50 | WDCTL_EN | WDCTL_INT_SEC_1)
#define TIME_SLOT               55                                              // real time-slot = TIME_SLOT * 10 ms

#define ACC_25_PERCENT          738
#define ACC_50_PERCENT          750
#define ACC_75_PERCENT          760
#define FLASHING_DELAY          60000                                          // 130k value for 500 ms delay if Fclk = 26 MHz

#define RIGHT_TURN_SIGNAL       0x02
#define BACK_SIGNAL             0x04
#define BRAKE_SIGNAL            0x08
#define LEFT_TURN_SIGNAL        0x10
#define ALL_SIGNAL              0x1E
#define HEART_BIT_SIGNAL        0xA5

#define SYSMODE_RX              0
#define SYSMODE_TX              1

/* time to allow pairing at startup */
#define TIME_PAIRING_ALLOWED_10MS 1500 /* (in 10ms counts) therefore (1500) * 10ms = 15,000 = 15s */

#define ERASED_PAGE             7
#define PAGE_SIZE               1024
#define FLASH_PAGE_ADDR         (uint16_t)(ERASED_PAGE * PAGE_SIZE)
__no_init const uint8_t __code flashDataAddr[PAGE_SIZE] @FLASH_PAGE_ADDR;       // The area in flash where data written to flash will be placed.


#define DEFAULT_PAIR_ADDRESS                    0xEF, 0xBE, 0xAD, 0xDE
#define FLASH_SELF_PAGE_ADDR  0x5000  /** start of page 20 - must be a start of a page */

//__no_init const uint8_t __code flashDataAddr[PAGE_SIZE] @FLASH_PAGE_ADDR;       // The area in flash where data written to flash will be placed.

/** static flash code */
/* default source pair address */
static const uint8_t __code selfAddress[PAGE_SIZE] @FLASH_SELF_PAGE_ADDR = {DEFAULT_PAIR_ADDRESS};

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

uint32_t tempCnt;


uint16_t accVoltage;
uint8_t  transmitterAddress[4];
uint8_t  rfdCount;
uint8_t  storeTxAddrCount;
uint8_t  rfBuffer[16];
uint16_t __xdata ramFuncAddr[RAM_BUF_SIZE];                              // RAM buffer for function
uint8_t  watchdogReset;

TRfPacket*  rfPacket = (TRfPacket*)rfBuffer;



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
extern void programmVersionIndication(void);
extern void readTxAddrFromFlash(uint32_t* pTxAddr, uint32_t* pFlashAddr);
extern void storeNewTxToFlash(void);

#endif