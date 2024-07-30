/**************************************************************************************************
  Filename:       main.c
  Revised:        $Date: 2020-06-17 $
  Revision:       $Revision: 3.02 $
  Author:         $Author: Yakov Churinov $

  Description:    This file supports the CarLamp CC1110 Receiver.

  Should you have any questions to use this Software, contact e-mail: goodbug@ukr.net
**************************************************************************************************/


#include "RxBoardDef.h"

/** global variables */
uint16_t gl_u16_pair_timeout = TIME_PAIRING_ALLOWED_10MS;

#if PROJECT_TYPE_CFG == PROJECT_TYPE_RX_MINI_4_IN_MAGNETIC_PROXIMITY_OPT
uint16_t gl_u16_follow_change_timeout = TIME_ALLOWED_MAGNETIC_CHANGE_10MS;
uint16_t gl_u16_hold_counter_for_magnetic_switch = TIME_HOLD_REQ_FOR_MAGNETIC_CHANGE_10MS;
un_mag_app_mode_t un_gl_mag_app_mode;
uint8_t u8_gl_mag_follow_bit_num = APP_MAG_RECEIVED_DEFAULT_BIT_POS;
en_magnet_status_t en_gl_current_magnet_status = MAGNET_STATUS_NOT_PRESENT; // todo initialize from realtime reading
boolean bool_gl_switch_mode_required = FALSE;

/** dma config - saved in XDATA region */
static __xdata DMA_DESC dma0_config;
static __xdata uint8_t data_to_flash[APP_MAGNETIC_MODE_SIZE_IN_BYTES];


/** initializes app defaults for global variables/flags if any
 * - after initializing all peripherals*/
static void app_defaults_init()
{
    un_gl_mag_app_mode.u16_app_mode = APP_MODE_DEFAULT; /* default */

    data_to_flash[0] = un_gl_mag_app_mode.st_app_mode_bytes.low_byte;
    data_to_flash[1] = un_gl_mag_app_mode.st_app_mode_bytes.high_byte;
}
#endif
/*******************************************************************************
* LOCAL FUNCTIONS
*/
void halFlashStartErase(void); // Implemented in assembly

static void switchToIDLE(void)
{
  rfdCount = 0;
  RFST = RFST_SIDLE;                                                            // send IDLE strobe    
  while(MARCSTATE != MARC_STATE_IDLE) {;}                                        // wait while RF not idle state
}

static void switchToRX(void)
{
  switchToIDLE();
  RFST = RFST_SRX;                                                              
  while(MARCSTATE != MARC_STATE_RX) {;}                                        
}

static void switchToTX(void)
{
  switchToIDLE();
  RFST = RFST_STX;                                                              
  while(MARCSTATE != MARC_STATE_TX) {;}                                        
}

#if PROJECT_TYPE_CFG == PROJECT_TYPE_BASE_OPT

static void LEDPairIndicationHandler(void)
{
  uint8_t dataBuf = (sysTick >> 3) & blinkMask;
  if(!dataBuf) P1 &= ~LED_PAIR;         // on
  else         P1 |= LED_PAIR;          // offs
}

#endif


static void generateNewHeartPacket(void)
{
//   uint32_t* pSrcAddr    = (uint32_t*)&rfPacket->srcAddr;
   uint32_t* pDestAddr   = (uint32_t*)&rfPacket->dstAddr;
    rfPacket->data_0 = HEART_BEAT_SIGNAL;
    rfPacket->data_1 = HEART_BEAT_SIGNAL;
//   *pSrcAddr    = MY_MAC_ADDRESS;

    rfPacket->srcAddr[0] = selfAddress[0];
    rfPacket->srcAddr[1] = selfAddress[1];
    rfPacket->srcAddr[2] = selfAddress[2];
    rfPacket->srcAddr[3] = selfAddress[3];

    *pDestAddr   = NOTIFIER_TAG_MAC_ADDRESS;
}


/********************************************************************************************************
 *  HANDLER FOR RFRXTX INTERRUPT
 ********************************************************************************************************/
#pragma vector = RFTXRX_VECTOR
__interrupt void RFTXRX_ISR (void) {

#if PROJECT_TYPE_CFG != PROJECT_TYPE_RX_MINI_4_IN_MAGNETIC_PROXIMITY_OPT
    P1 &= ~LED_TXRX;
#endif

  switch(sysMode)
  {
  case SYSMODE_RX_WAIT:
      rfBuffer[rfdCount++] = RFD;                                                   // store byte from radio into buffer   
    break;
  case SYSMODE_TX_TRANS_WAIT:
  case SYSMODE_TX_TRANS:
      RFD = rfBuffer[rfdCount++];                                                   // store byte from radio into buffer   
    break;
  default:
    break;
  }
  
}
/********************************************************************************************************
 *  HANDLER FOR TIMER1 INTERRUPT
 ********************************************************************************************************/
/* runs every 10ms */
#pragma vector = T1_VECTOR
__interrupt void T1_ISR (void) {
  
  sysTick ++;

  if(!Flags.RF_DEVICE_PAIRED)
  {
      if(gl_u16_pair_timeout)
      {
          gl_u16_pair_timeout--;
      }
  }

  if(gl_u16_follow_change_timeout)
  {
      gl_u16_follow_change_timeout--;

      if(ZERO == gl_u16_follow_change_timeout)
      {
          // disable mode magnetic switching interrupt (timeout)
          DISABLE_INT_P0();
      }
  }

  if(MAGNET_STATUS_PRESENT == en_gl_current_magnet_status)
  {
      if(gl_u16_hold_counter_for_magnetic_switch)
      {
          gl_u16_hold_counter_for_magnetic_switch--;

          if(ZERO == gl_u16_follow_change_timeout)
          {
              // turn off interrupts
              DISABLE_INT_P0();

              // switch mode
              bool_gl_switch_mode_required = TRUE;
          }
      }
  }

  wdt_reset();
}

/********************************************************************************************************
 *  HANDLER FOR RF INTERRUPT
 ********************************************************************************************************/
#pragma vector = RF_VECTOR
__interrupt void RF_ISR (void) {
  
  S1CON &= ~S1CON_RFIF_FLAGS;                                                   // Clear CPU interrupt flag
#if PROJECT_TYPE_CFG != PROJECT_TYPE_RX_MINI_4_IN_MAGNETIC_PROXIMITY_OPT
  P1 |= LED_TXRX;
#endif
  if(RFIF & (RFIF_IRQ_RXOVF | RFIF_IRQ_TXUNF ))                                 // if RX OVERFLOW or TX UNDERFLOW flag is present
  {                                
    Flags.RF_PACKET_ERROR = 1;                                                  // set packet error flag
  } 
  else if(RFIF & RFIF_IRQ_DONE){ 
    switchToIDLE();
    Flags.RF_PACKET_DONE = 1;                                                   // if transaction is normal done set packet done flag
  }                   
  RFIF = 0;                                                                     // clear flags
  rfdCount = 0;
}

 static void updateTrasnmitDataPacket(void)
 {
//   uint32_t* pSrcAddr    = (uint32_t*)&rfPacket->srcAddr;
     uint32_t *pDestAddr = (uint32_t *) &rfPacket->dstAddr;

//   *pSrcAddr    = MY_MAC_ADDRESS;

     rfPacket->srcAddr[0] = selfAddress[0];
     rfPacket->srcAddr[1] = selfAddress[1];
     rfPacket->srcAddr[2] = selfAddress[2];
     rfPacket->srcAddr[3] = selfAddress[3];

     *pDestAddr = NOTIFIER_TAG_MAC_ADDRESS;
 }
 static void updatePairedFlag(void)
 {
    Flags.RF_DEVICE_PAIRED = 1;
    blinkMask = 0x1F;
 }

 static uint8_t channelCleared(void)
 {
  uint8_t rssiDec = RSSI;                                                       // get data from RSSI register
  uint8_t rssiDb = (rssiDec < 128) ? (73 - (rssiDec >> 1)) : (73 +((256-rssiDec) >> 1)); // calc RSSI in dB
  if(rssiDb < 70) return 0;                                              
  return 1;
 }
 
 static void diableRFInterrupts(void)
 {
   IEN2 &= ~IEN2_RFIE;
   RFTXRXIE = 0;
   RFIF = 0;                                                                     // clear flags
   rfdCount = 0;
 }
 static void enableRFInterrupts(void)
 {
   IEN2 |= IEN2_RFIE;                                                            // enable RF interrupt
   RFTXRXIE = 1;
   RFIF = 0;
   RFTXRXIF = 0;
   
 }
 
 static void transData(void)
{
  switchToIDLE();
  rfdCount = 0;
  enableRFInterrupts();
  switchToTX();
}
static void sysMode2RXWait(void)
{
  enableRFInterrupts();
  switchToRX();  
  sysMode = SYSMODE_RX_WAIT;
}
static void sysMode2RXDelay(void)
{
//  wdt_reset();
  sysTickBuf = sysTick;
  sysMode    = SYSMODE_RX_DELAY;
  diableRFInterrupts();
  switchToRX();
}
static void checkRXmode(void)
{
  if (MARCSTATE == MARC_STATE_RX) return;
  sysMode2RXWait();
}
static void rssiChange(void)
{
  if(sysTick & 0x2)  switchToRX();
  
}
 /********************************************************************************************************
 *  CHECK RECEIVED PACKET
 ********************************************************************************************************/
static TParseStatus packetParse(void){

//  wdt_reset();

  uint32_t* pSrcAddrBuf = (uint32_t*)transmitterAddress;                           // set pointers to destination and source addresses
  uint32_t* pSrcAddr    = (uint32_t*)&rfPacket->srcAddr;
  uint32_t* pDestAddr   = (uint32_t*)&rfPacket->dstAddr;
 
  if((LQI & 0x80) == 0)            return PS_ERROR;                           
//  if(*pDestAddr != MY_MAC_ADDRESS) return PS_ERROR;
  if(rfPacket->data_0 != rfPacket->data_1) return PS_ERROR;
  
  if(*pSrcAddrBuf == *pSrcAddr) {
      Flags.RF_DEVICE_PAIRED = 1; /* set to paired */
      return PS_NO_ERROR;
  }

  if(Flags.RF_DEVICE_PAIRED)    return PS_ERROR;  

  if(!gl_u16_pair_timeout) return PS_ERROR;                         /* block pairing if pair time has passed */

  if(rfPacket->data_1 & LEFT_TURN_SIGNAL)                           // if the source address is not equal to the destination address, but signal is LEFT_TURN_SIGNAL
  {
        *pSrcAddrBuf = *pSrcAddr;                                   // remember new source address
        return PS_PAIR_ADDRESS;
  }
  return PS_ERROR;  
}

/********************************************************************************************************
 *  PROGRAM MAIN SECTION
 ********************************************************************************************************/
int main( void )
{
  setSystemClock();
  mcuPinInit();
  init_magnetic_sensor_interrupts();
  adcInit();
  if(watchdogReset == 0){
    programVersionIndication();                                                // all lamp ON for control and LEFT_TURN flashing for version indication
    accumVoltageIndication();                                                   // RIGHT_TURN flashing for voltage indication
  } else {
    watchdogReset = 0;
  }
  readTxAddrFromFlash((uint32_t*)transmitterAddress, (uint32_t*)flashDataAddr); 
  radioInit();

  systemStart();
  sysMode2RXWait();

    /* init watchdog */
    //wdt_init();

#if PROJECT_TYPE_CFG == PROJECT_TYPE_RX_MINI_4_IN_MAGNETIC_PROXIMITY_OPT
    app_defaults_init();
    dma0_init();

    RESTORE_APP_MODE();

    /* enable global interrupts */
    IEN0 |= IEN0_EA;                                                              // enable global interrupt
#endif

  while(1){

      /* reset watchdog */
//      wdt_reset();
#if PROJECT_TYPE_CFG == PROJECT_TYPE_RX_MINI_4_IN_MAGNETIC_PROXIMITY_OPT
    while(1)
    {
        boolean bool_l_magnet_state = GET_MAGNETIC_SENSOR_STATUS();
        boolean test = 5;

        if(bool_gl_switch_mode_required) app_switch_follow_signal();
    }
#endif


#if PROJECT_TYPE_CFG != PROJECT_TYPE_RX_MINI_4_IN_MAGNETIC_PROXIMITY_OPT
      LEDPairIndicationHandler();
#endif
      switch(sysMode)
      {
        case SYSMODE_RX_WAIT:
          checkRXmode();
           if(Flags.RF_PACKET_DONE)
           {
             Flags.RF_PACKET_DONE = 0;
             switch(packetParse())
             {
             case PS_PAIR_ADDRESS:
                 storeNewTxToFlash();  
             case PS_NO_ERROR:
                 sysMode2RXDelay();
#if PROJECT_TYPE_CFG == PROJECT_TYPE_BASE_OPT
                 P0 &= ~ALL_SIGNAL; // off;
                 P0 |= rfPacket->data_0;
#elif PROJECT_TYPE_CFG == PROJECT_TYPE_RX_MINI_4_IN_MAGNETIC_PROXIMITY_OPT
                 CLR_BIT(LED_MAIN_OUTPUT_PORT, LED_MAIN_OUTPUT_PIN);
                 WRITE_BIT(LED_MAIN_OUTPUT_PORT, LED_MAIN_OUTPUT_PIN, GET_BIT((rfPacket->data_0), u8_gl_mag_follow_bit_num));
#endif
                 transRtry = 3;
               break;
             default:
                sysMode2RXWait();
               break;
             }
           }else
           if(Flags.RF_PACKET_ERROR) 
           {
             Flags.RF_PACKET_ERROR = 0;
             sysMode2RXWait();
           }
           else 
           if((sysTickBuf + 500) < sysTick)
           {
                 generateNewHeartPacket();
                 sysMode2RXDelay();       
                 transRtry = 3;
           }
          break;
      case SYSMODE_RX_DELAY:   
   //         if((sysTickBuf + 5) < sysTick)
            {
               updateTrasnmitDataPacket();
               updatePairedFlag();                 
               sysMode = SYSMODE_TX_PREPARE;
            }
        break;
      case SYSMODE_TX_PREPARE:
          if(channelCleared())
          {
            sysMode = SYSMODE_TX_TRANS;
          }
          rssiChange();
          if((sysTickBuf + 20) < sysTick)
          {
            enableRFInterrupts();
            sysMode = SYSMODE_RX_WAIT;
          }
        break;
      case SYSMODE_TX_TRANS:
          transData();
          sysMode = SYSMODE_TX_TRANS_WAIT;
        break;
      case SYSMODE_TX_TRANS_WAIT:
          if(Flags.RF_PACKET_DONE)
          {
            Flags.RF_PACKET_DONE = 0;
            sysTickBuf = sysTick;
            sysMode2RXWait();
          }
          if(Flags.RF_PACKET_ERROR){ 
            Flags.RF_PACKET_ERROR = 0;  
            if(--transRtry)
            {
              sysMode2RXDelay();
            }
            else
            {
              sysTickBuf = sysTick;
              sysMode2RXWait();
            }
          }   
        break;
        default:
            sysMode2RXWait();
          break;
        }    
      if((sysTickBuf + 60) < sysTick)
      {
#if PROJECT_TYPE_CFG == PROJECT_TYPE_BASE_OPT
          P0 &= ~ALL_SIGNAL; // off;
#elif PROJECT_TYPE_CFG == PROJECT_TYPE_RX_MINI_4_IN_MAGNETIC_PROXIMITY_OPT
          CLR_BIT(LED_MAIN_OUTPUT_PORT, LED_MAIN_OUTPUT_PIN);
#endif
      }
  }
      
}

/********************************************************************************************************
*   SET SYSTEM CLOCK
*   Set system clock source to HS XOSC, with no pre-scaling  = 26 MHZ
*   Set timers clock source to (HS XOSC div 16) = 1,625 MHz
********************************************************************************************************/
void setSystemClock(void){

  watchdogReset = SLEEP & WATCHDOG_MSK; /* 1 if last reset was due to watchdog */
  /*** Set system clock source to HS XOSC, with no pre-scaling ***/
  SLEEP &= ~SLEEP_OSC_PD;           /* requesting both oscillators up */
  while( !(SLEEP & SLEEP_XOSC_S) ); /* checking high speed OSC is powered up */

  /* update watchdog crystal source to high-speed stable crystal osc 32.768 kHz */
  while(!(SLEEP & SLEEP_HFRC_S)); /* check HS RCOSC stable before changing OSC32 bit */
  CLKCON = (CLKCON & ~(CLKCON_OSC32)); /* enable 32.768kHz crystal */

  CLKCON = (CLKCON & ~(CLKCON_TICKSPD | CLKCON_CLKSPD | CLKCON_OSC)) | TICKSPD_DIV_16; /* adjust clock - choose HS-X (crystal) */
  while (CLKCON & CLKCON_OSC);      /* wait for oscillator calibration to be finished */
  SLEEP |= SLEEP_OSC_PD;            /* power down oscillators not selected by CLKCON.OSC */
}
/********************************************************************************************************
 *  INIT MCU PINS
 ********************************************************************************************************/
void mcuPinInit(void){

#if PROJECT_TYPE_CFG == PROJECT_TYPE_BASE_OPT
    /** port 1 init */
    P1DIR |= LED_TXRX | LED_PAIR | ACC_VCNTRL;                                    // set pins for leds and control ACC as outputs
    P1 &= ~ACC_VCNTRL;                                                            // set LOW on ACC_VCNTRL pin

    /** port 0 init */
    P0DIR |= (RIGHT_TURN_SIGNAL | BACK_SIGNAL | BRAKE_SIGNAL | LEFT_TURN_SIGNAL); // set pins for light signals as outputs
    P0 &= ~(RIGHT_TURN_SIGNAL | BACK_SIGNAL | BRAKE_SIGNAL | LEFT_TURN_SIGNAL);   // set LOW for light signals;

#elif PROJECT_TYPE_CFG == PROJECT_TYPE_RX_MINI_4_IN_MAGNETIC_PROXIMITY_OPT
    /** port 1 init */
    P1DIR |= ACC_VCNTRL | IND_LED_MODE_LEFT_PIN_MASK | IND_LED_MODE_RIGHT_PIN_MASK;                                    // set pins for leds and control ACC as outputs
    P1 &= ~(ACC_VCNTRL | IND_LED_MODE_LEFT_PIN_MASK | IND_LED_MODE_RIGHT_PIN_MASK);                                        // set LOW on ACC_VCNTRL pin

    /** port 0 init */
    P0DIR |= (LED_MAIN_OUTPUT_PIN_MASK); // set pins for light signals as outputs
    P0 &= ~(LED_MAIN_OUTPUT_PIN_MASK);   // set LOW for light signals;

#endif
}


/********************************************************************************************************
 *  INIT RADIO
 ********************************************************************************************************/
void radioInit(void){

  RFST = RFST_SIDLE;
  
  CHANNR = SMARTRF_SETTING_CHANNR;
  FSCTRL1 = SMARTRF_SETTING_FSCTRL1;                                            // Frequency synthesizer control      
  FREQ2 = SMARTRF_SETTING_FREQ2;                                                // Frequency control word, high byte
  FREQ1 = SMARTRF_SETTING_FREQ1;                                                // Frequency control word, middle byte
  FREQ0 = SMARTRF_SETTING_FREQ0;                                                // Frequency control word, low byte
  MDMCFG4 = SMARTRF_SETTING_MDMCFG4;                                            // Modem configuration
  MDMCFG3 = SMARTRF_SETTING_MDMCFG3;                                            // Modem configuration
  MDMCFG2 = SMARTRF_SETTING_MDMCFG2;                                            // Modem configuration
  MDMCFG1 = SMARTRF_SETTING_MDMCFG1;                                            // Modem configuration
  MDMCFG0 = SMARTRF_SETTING_MDMCFG0;                                            // Modem configuration 
  DEVIATN = SMARTRF_SETTING_DEVIATN;                                            // Modem deviation setting
  MCSM0 = SMARTRF_SETTING_MCSM0;                                                // Main Radio Control State Machine configuration
  FOCCFG = SMARTRF_SETTING_FOCCFG;                                              // Frequency Offset Compensation configuration 
  BSCFG = SMARTRF_SETTING_BSCFG;                                                // Bit Synchronization configuration
  AGCCTRL2 = SMARTRF_SETTING_AGCCTRL2;                                          // AGC control
  AGCCTRL1 = SMARTRF_SETTING_AGCCTRL1;
  AGCCTRL0 = SMARTRF_SETTING_AGCCTRL0;
  FREND1 = SMARTRF_SETTING_FREND1;                                              // Front end RX configuration
  FSCAL3 = SMARTRF_SETTING_FSCAL3;                                              // Frequency synthesizer calibration
 // FSCAL2 = SMARTRF_SETTING_FSCAL2;
  FSCAL1 = SMARTRF_SETTING_FSCAL1;
  FSCAL0 = SMARTRF_SETTING_FSCAL0;
  TEST1 = SMARTRF_SETTING_TEST1;                                                // Various Test Settings
  TEST0 = SMARTRF_SETTING_TEST0;
  PA_TABLE0 = SMARTRF_SETTING_PA_TABLE0;                                        // PA output power setting 0
  IOCFG0 = SMARTRF_SETTING_IOCFG0;                                              // Radio test signal configuration (P1_5)
  VERSION = SMARTRF_SETTING_VERSION;                                            // Chip ID[7:0]
  
  PKTLEN = RX_PACKET_LENGTH;
  PKTCTRL0 = 0;
  PKTCTRL1 = 0;
                                                                                // the status bytes contain RSSI and LQI values, as well as the CRC OK flag
  MDMCFG1 |= MDMCG1_NUM_PREAMBLE_4;                                             // sets the minimum 4 of preamble bytes to be transmitted
  MCSM0 |= FS_AUTOCAL_FROM_IDLE;                                                // select calibration when going from to IDLE RX or TX automatically 
  IEN0 |= IEN0_RFTXRXIE;                                                        // enable RFTXRX interrupt
  IEN2 |= IEN2_RFIE;                                                            // enable RF interrupt
  RFIM |= RFIM_IM_TXUNF | RFIM_IM_RXOVF | RFIM_IM_DONE;                         // activate RF interrupts mask
  RFTXRXIF = 0;                                                                 // clear interrupts flags
  RFIF = 0;
}



/********************************************************************************************************
 *  SYSTEM START
 ********************************************************************************************************/
void systemStart(void){  
  
  /*** init timer 1  ***************************/  
  T1CTL &= ~T1CTL_DIV;                                                          // set T1 preskaler by 1,625 MHz div_1. One timer tick = 0,615 mks. for 10000 mks (10 ms) need 16260 ticks = 0x3F84 
  T1CC0H = 0x3F;                                                                // load T1CC0H:T1CC0L for overflow every 10 ms
  T1CC0L = 0x84;
  T1CTL &= ~(T1CTL_CH2IF | T1CTL_CH1IF | T1CTL_CH0IF | T1CTL_OVFIF);            // clear all interrupt flags for TIMER1 
  IEN1 |= IEN1_T1IE;                                                            // TIMER1 interrupt enable
  T1CTL |= T1CTL_MODE_MODULO;                                                   // start timer in Modulo Mode
 
  blinkMask = (Flags.RF_DEVICE_PAIRED == 1) ? 0x7: 0x1F;    // update blink period 

  sysTick = 0;
  sysTickBuf = 0;
  EA = 1;                                                                       // enable global interrupt
}

/********************************************************************************************************
 *  INIT ADC
 ********************************************************************************************************/
void adcInit(void){
  P0SEL |= P0SEL_P0_ALTERNATE;                                                  // set P0_0 as ADC input
  P0INP |= P0INP_PDUP0;                                                         // set P0 input in three-state (no pull-up/pull-down)
  ADCCFG |= ADCCFG_0;
} 
/********************************************************************************************************
 *  ACC VOLTAGE MEASURE
 ********************************************************************************************************/
void getAccVoltage(void){
  
  P1 |= ACC_VCNTRL;                                                             // set HIGH on ACC_VCNTRL pin
  ADCCON3 = ADCCON3_EREF_AVDD | ADCCON3_EDIV_256 | ADCCON3_ECH_AIN0;            // start extra conversion for ADC0
  while (!(ADCCON1 & ADCCON1_EOC)) {}                                           // wait end of conversion
  unAcc.adcResult.lowByte = ADCL;                                               // read data_1 from ADC registers
  unAcc.adcResult.highByte = ADCH; 
  unAcc.adcData >>= 4;
  P1 &= ~ACC_VCNTRL;                                                            // set LOW on ACC_VCNTRL pin
}

/********************************************************************************************************
 *  READ TX ADDRESS FROM FLASH 
 ********************************************************************************************************/
void readTxAddrFromFlash(uint32_t* pTxAddr, uint32_t* pFlashAddr){
  uint8_t i = 1;
  storeTxAddrCount = 0;
  while (i){
    if (*pFlashAddr == 0xFFFFFFFF) {
      i = 0;
    } else { 
        storeTxAddrCount++;
        *pTxAddr = *pFlashAddr;
        if (storeTxAddrCount < 128){
          pFlashAddr++;     
        } else {i = 0;}	      
      }
  }
//  if(storeTxAddrCount != 0) Flags.RF_DEVICE_PAIRED = 1;
//  else Flags.RF_DEVICE_PAIRED = 0;
}
/***********************************************************************************************************
* @fn          runFunctionFromRam
*
* @brief       Copies another function from flash to RAM and executes the function. 
*              Does check whether the space provided in RAM is enough, this must be done prior to the call.
*
* @param       void (*func)(void) - address of function to be run from RAM.
*              uint8_t __xdata *ramAddr - adress of function location in RAM.
*              uint16_t funcLength - size of buffer to place function in [bytes].
*
* @return      void
************************************************************************************************************/
void runFunctionFromRam(void (*func)(uint16_t param), uint16_t __xdata *ramAddr, uint16_t funcLength, uint16_t funcParam) {
  EA = 0;                                                                       // disable global interrupt
  uint16_t __code *flashFuncAddr = (uint16_t __code*)(uint16_t)func;            // flashFuncAddr is a pointer to where in flash the function is
  VFPTR f = (VFPTR)(uint16_t)ramAddr;                                           // f is a function pointer to the address in RAM where the function will be placed.
  for (uint16_t i = 0; i < funcLength; i++){ ramAddr[i] = flashFuncAddr[i];}    // Copy the function from flash to RAM
  (*f)(funcParam);                                                              // Run function from RAM
  EA = 1;                                                                       // enable global interrupt
  return;
}
/********************************************************************************************************
 *  ERASE PAGE 
 ********************************************************************************************************/
void eraseFlashPage(uint16_t pageAddr){
  while (FCTL & FCTL_BUSY){}                                                    // Wait for the flash controller to be ready
  FADDRH = pageAddr >> 9;                                                       // Setup FADDRH, FADDRL
  FADDRL = (pageAddr >> 1) & ~0xFF00;
  FWT = 0x22;                                                                   // Setup FWT setting, matches 26 MHz clock frequency
  halFlashStartErase();                                                         // Erase the page that later will be written to. (2-byte alignment.) 
  while (FCTL & FCTL_BUSY){}
}
/***********************************************************************************************************
* @fn          flashWriter
*
* @brief       Writes contents of the array "transmitterAddress" to flash. Must be run from RAM.
* @param       void
* @return      void
************************************************************************************************************/
void writeTxAddressToFlash(uint16_t dataAddr){
    
  while (FCTL & FCTL_BUSY){}                                                    // Waiting for the flash controller to be ready
  FADDRH = dataAddr >> 9;                                                       // Setup FADDRH, FADDRL
  FADDRL = (dataAddr >> 1) & ~0xFF00;
  FWT = 0x22;                                                                   // Setup FWT setting, matches 26 MHz clock frequency
  FCTL |= FCTL_WRITE;                                                           // Enabling flash write.
  uint8_t i = 0;
  while (i < sizeof transmitterAddress){
    FWDATA = transmitterAddress[i++];                                           // Write two bytes to FWDATA. Flash write starts after second byte is written to the register.
    FWDATA = transmitterAddress[i++];                                           // The first byte written to FWDATA is the LSB of the 16-bit word
    while (FCTL & FCTL_SWBSY){}
  }
  return;
}
/********************************************************************************************************
 *  STORE NEW TX ADDRESS TO FLASH 
 ********************************************************************************************************/
void storeNewTxToFlash(void){
  if (storeTxAddrCount >= 128){storeTxAddrCount = 0;}
  if(storeTxAddrCount == 0){                                                    // if counter overrun
    runFunctionFromRam(eraseFlashPage, ramFuncAddr, RAM_BUF_SIZE, FLASH_PAGE_ADDR); // erase FLASH page for new records
  }
  uint16_t addressIntoPage = FLASH_PAGE_ADDR + (storeTxAddrCount * 4);          // calculate new FLASH address
  runFunctionFromRam(writeTxAddressToFlash, ramFuncAddr, RAM_BUF_SIZE, addressIntoPage); // store new TX address into FLASH  
  storeTxAddrCount++;                                                           // increase storeTxAddrCount 
}

/********************************************************************************************************
 *  ACCUM VOLTAGE INDICATION 
 ********************************************************************************************************/
void accumVoltageIndication(void){

  getAccVoltage();
  
  uint8_t   flashCount = 4;                                                     // if accum > 75% 
  if(unAcc.adcData < ACC_25_PERCENT) {                                          // if accum < 25%
    flashCount = 1;
  } else if (unAcc.adcData < ACC_50_PERCENT) {                                  // if accum < 50%
    flashCount = 2;
  } else if (unAcc.adcData < ACC_75_PERCENT) {                                  // if accum < 75%
    flashCount = 3;
  } 
  
  uint32_t delay = FLASHING_DELAY;

#if PROJECT_TYPE_CFG == PROJECT_TYPE_BASE_OPT

    while (flashCount){
    P0 |= RIGHT_TURN_SIGNAL;                                                     // RIGHT_TURN on;
    while(delay){delay--;}
    P0 &= ~RIGHT_TURN_SIGNAL;                                                    // RIGHT_TURN off
    while(delay < FLASHING_DELAY){delay++;}
    flashCount--;
  }

#elif PROJECT_TYPE_CFG == PROJECT_TYPE_RX_MINI_4_IN_MAGNETIC_PROXIMITY_OPT

    while (flashCount){
        LED_MAIN_OUTPUT_PORT |= LED_MAIN_OUTPUT_PIN_MASK;      // Main LED on;
        while(delay){delay--;}
        LED_MAIN_OUTPUT_PORT &= ~LED_MAIN_OUTPUT_PIN_MASK;     // Main LED off
        while(delay < FLASHING_DELAY){delay++;}
        flashCount--;
    }

#endif


}

/********************************************************************************************************
 *  PROGRAMM VERSION INDICATION 
 ********************************************************************************************************/
void programVersionIndication(void){

#if PROJECT_TYPE_CFG == PROJECT_TYPE_BASE_OPT

  uint32_t delay = FLASHING_DELAY * 2;
  P0 |= RIGHT_TURN_SIGNAL | BACK_SIGNAL | BRAKE_SIGNAL | LEFT_TURN_SIGNAL;      // all lamps on
  while(delay){delay--;}
  P0 &= ~(RIGHT_TURN_SIGNAL | BACK_SIGNAL | BRAKE_SIGNAL | LEFT_TURN_SIGNAL);   // all lamps off
  while(delay < FLASHING_DELAY){delay++;}

  uint16_t version = PROGRAMM_VERSION;

    while (version){
    P0 |= LEFT_TURN_SIGNAL;                                                     // LEFT_TURN on;
    while(delay){delay--;}
    P0 &= ~LEFT_TURN_SIGNAL;                                                    // LEFT_TURN off
    while(delay < FLASHING_DELAY){delay++;}
    version--;
  }

#elif PROJECT_TYPE_CFG == PROJECT_TYPE_RX_MINI_4_IN_MAGNETIC_PROXIMITY_OPT

    uint32_t delay = FLASHING_DELAY * 2;
    LED_MAIN_OUTPUT_PORT |= LED_MAIN_OUTPUT_PIN_MASK;      // all lamps on
    while(delay){delay--;}
    LED_MAIN_OUTPUT_PORT &= ~LED_MAIN_OUTPUT_PIN_MASK;   // all lamps off
    while(delay < FLASHING_DELAY){delay++;}

    uint16_t version = PROGRAMM_VERSION;

    while (version){
        LED_MAIN_OUTPUT_PORT |= LED_MAIN_OUTPUT_PIN_MASK;                // LEFT_TURN on;
        while(delay){delay--;}
        LED_MAIN_OUTPUT_PORT &= ~LED_MAIN_OUTPUT_PIN_MASK;               // LEFT_TURN off
        while(delay < FLASHING_DELAY){delay++;}
        version--;
    }

#endif
}

//region Special Board Flavors Functions
#if PROJECT_TYPE_CFG == PROJECT_TYPE_RX_MINI_4_IN_MAGNETIC_PROXIMITY_OPT

/** init DMA channel 0 for flash write */
static void dma0_init(void)
{
    /* Configure DMA channel 0 for flash write */
    dma0_config.SRCADDRH  = (((uint16_t)(__xdata uint16_t *)data_to_flash) >> 8) & 0x00FF;
    dma0_config.SRCADDRL   = ((uint16_t)(__xdata uint16_t *)data_to_flash) & 0x00FF;
    dma0_config.DESTADDRH  = (FLASH_FWDATA_ADDR >> 8) & 0x00FF;
    dma0_config.DESTADDRL   = FLASH_FWDATA_ADDR & 0x00FF;
    dma0_config.LENH  = (APP_MODE_SIZE_IN_BYTES >> 8) & 0x00FF;
    dma0_config.VLEN      = DMA_VLEN_USE_LEN;
    dma0_config.LENL   = APP_MODE_SIZE_IN_BYTES & 0x00FF;
    dma0_config.WORDSIZE  = DMA_WORDSIZE_BYTE;
    dma0_config.TMODE     = DMA_TMODE_SINGLE;
    dma0_config.TRIG      = DMA_TRIG_FLASH;
    dma0_config.SRCINC    = DMA_SRCINC_1;
    dma0_config.DESTINC   = DMA_DESTINC_0;
    dma0_config.IRQMASK   = DMA_IRQMASK_DISABLE;
    dma0_config.M8        = DMA_M8_USE_8_BITS;
    dma0_config.PRIORITY  = DMA_PRI_HIGH;

    // Point DMA controller at our DMA descriptor
    DMA0CFGH = ((uint16_t)&dma0_config >> 8) & 0x00FF;
    DMA0CFGL = (uint16_t)&dma0_config & 0x00FF;

    /* Channel ready - waiting for trigger */
}

static void init_magnetic_sensor_interrupts()
{
    /*** init port 0 interrupt - @HossamElwahsh */
    /* > Enable interrupt for required inputs on port 0 - @HossamElwahsh */

    /* interrupt priority for P0INT */
    IP0 |= IP0_IPG5;
    IP1 |= IP1_IPG5;

    /* 1. Clear interrupt flags */
    P0IFG = 0x00; /* Module Status Flag */
    CLR_BIT(IRCON, 5); /* clear CPU status flag */

    /* 2. set individual interrupt enable bit in IEN0, IEN1, IEN2 to 1 */
//    P0IEN |= PROX_SENSOR_PIN_MASK; /* Enable interrupts for Port 0 bits PROX SENSOR */
#if MAGNETIC_PCB_TYPE == MAGNETIC_PCB_TYPE_OLD_OPT
    PICTRL |= PICTL_P0IENL;
#elif MAGNETIC_PCB_TYPE == MAGNETIC_PCB_TYPE_NEW_OPT
    PICTRL |= PICTL_P0IENH;
#endif

    INT_P0_ON_FALLING_EDGE(); /* Port 0 magnetic sensor detection set to falling edge mode (1) as magnetic sensor is active low */

    /* 4. Enable global interrupt by setting IEN0.EA = 1 - already done in main loop */

}

static void app_switch_follow_signal()
{
    if(!bool_gl_switch_mode_required) return;

    // disable interrupt just in case
    DISABLE_INT_P0();

    /* hold time passed - change app mode */
    un_gl_mag_app_mode.u16_app_mode ^= 0xFFFF; /* toggles app mode */

    /* save to flash */
    SAVE_APP_MODE();

    // clear flag
    bool_gl_switch_mode_required = FALSE;
}

//region Flash Management


void halFlashStartWrite(void) __attribute__((aligned(2)));

#pragma segment="CUSTOM_SEGMENT" // Define a custom segment

#pragma location = "CUSTOM_SEGMENT" // Specify the location for the following function
#pragma optimize=no_inline
void halFlashStartWrite(void) {
    // trigger flash write. this generates a DMA trigger
    // this was moved to a separate function as sdcc does not
    // optimize functions that include asm code (!)
    asm("ORL FCTL, #0x02");
    asm("NOP");
}

static void flash_read_saved_app_mode(void)
{
    uint16_t temp_read_value = ZERO;
    uint8_t __code * flashPtr = (uint8_t __code *)FLASH_APP_MAGNETIC_MODE_ADDR;

    temp_read_value = (flashPtr[0] & 0xFF) | ((flashPtr[1] & 0xFF) << 8);

    switch (temp_read_value)
    {
        case APP_MAGNETIC_MODE_FOLLOW_RIGHT:
        case APP_MAGNETIC_MODE_FOLLOW_LEFT:
        {
            un_gl_mag_app_mode.u16_app_mode = temp_read_value;
            update_magnetic_mode_ui(); // Update UI
            break;
        }
        default:
        {
            /* invalid mode - leave global mode as default (APP_MODE_DEFAULT) */
            // save default
            SAVE_APP_MODE();
            break;
        }
    }
}

static void flash_erase_app_mode_page(void)
{
    while (FCTL & FCTL_BUSY); /* wait until not busy (not busy erasing if any) */

    /* set clock speed */
    FWT = FWT_FLASH_CLOCK_ADJUST;


    /* Flash page to be erased is selected by FADDRH[5:1] bits
     * - write page number only in [5:1] NOT page address */
    SET_WORD(FADDRH, FADDRL, ((uint16_t)FLASH_APP_MAGNETIC_MODE_ADDR) >> 1);

    // Erase the page that will be written to
    FCTL |=  FCTL_ERASE; /* start erasing */
    asm ("nop"); /* mandatory NOP after erase */

    // Wait for the erase operation to complete
    while (FCTL & FCTL_BUSY);
}

static void flash_write_app_mode(void)
{
    /* update data to be flashed */
    data_to_flash[0] = un_gl_mag_app_mode.app_mode_arr[0];
    data_to_flash[1] = un_gl_mag_app_mode.app_mode_arr[1];

    /* erase before writing */
    flash_erase_app_mode_page();

    /* wait erase completion */
    while (FCTL & FCTL_BUSY);

    // Configure the flash controller
    FWT = FWT_FLASH_CLOCK_ADJUST;
    SET_WORD(FADDRH, FADDRL, ((uint16_t)FLASH_APP_MAGNETIC_MODE_ADDR) >> 1);

    /* ARM DMA channel */
    DMAARM |= DMAARM0;

    /* initiate flash write with DMA */
    halFlashStartWrite();

    // Wait for DMA transfer to complete
    while (!(DMAIRQ & DMAIRQ_DMAIF0));

    // Wait until flash controller not busy
    while (FCTL & (FCTL_BUSY | FCTL_SWBSY));

    // By now, the transfer is completed, so the transfer count is reached.
    // The DMA channel 0 interrupt flag is then set, so we clear it here.
    DMAIRQ &= ~DMAIRQ_DMAIF0;

    /* reset DMA */
    dma0_init();

    /* blink user */
    update_magnetic_mode_ui();
}

//endregion

static void update_magnetic_mode_ui()
{
    if(APP_MAGNETIC_MODE_FOLLOW_RIGHT == un_gl_mag_app_mode.u16_app_mode)
    {
        CLR_BIT(IND_LED_MODE_LEFT_PORT, IND_LED_MODE_LEFT_PIN);
        SET_BIT(IND_LED_MODE_RIGHT_PORT, IND_LED_MODE_RIGHT_PIN);

        u8_gl_mag_follow_bit_num = APP_MAG_RECEIVED_RIGHT_BIT_POS;
    }

    else if(APP_MAGNETIC_MODE_FOLLOW_LEFT == un_gl_mag_app_mode.u16_app_mode)
    {
        CLR_BIT(IND_LED_MODE_RIGHT_PORT, IND_LED_MODE_RIGHT_PIN);
        SET_BIT(IND_LED_MODE_LEFT_PORT, IND_LED_MODE_LEFT_PIN);

        u8_gl_mag_follow_bit_num = APP_MAG_RECEIVED_LEFT_BIT_POS;
    }
}

/** handler for port0 interrupt - @HossamElwahsh */
#pragma vector = P0INT_VECTOR
__interrupt void P0_ISR (void) {

    if(GET_BIT(P0IFG, PROX_SENSOR_PIN))
    {
        // process mag sensor pin
        en_gl_current_magnet_status = GET_MAGNETIC_SENSOR_STATUS();

        if(INT_FALLING_EDGE == en_gl_current_mag_sensor_interrupt_mode)
        {
            // start of hold

            // reset hold timer
            gl_u16_hold_counter_for_magnetic_switch = TIME_HOLD_REQ_FOR_MAGNETIC_CHANGE_10MS;

            // switch to rising edge
            INT_P0_ON_RISING_EDGE();
        }
        else if(INT_RISING_EDGE == en_gl_current_mag_sensor_interrupt_mode)
        {
            // end of hold

            // check timer
            if(ZERO == gl_u16_hold_counter_for_magnetic_switch)
            {
                // hold time OK
                // disable interrupt
                DISABLE_INT_P0();

                // switch mode
                bool_gl_switch_mode_required = TRUE;
            }
            else
            {
                // do nothing ?
            }

            // switch to falling edge
            INT_P0_ON_FALLING_EDGE();
        }
    }
    // else (or both) clear pending interrupts

    /* clear IFG */
    P0IFG = 0x00; /* Module Status Flag */
    CLR_BIT(IRCON, 5); /* Clear CPU status flag - IRCON.P0IF */
}

#endif
//endregion