/***************************************************************
 *  SmartRF Studio(tm) Export
 *
 *  Radio register settings specifed with C-code
 *  compatible #define statements.
 *
 *  RF device: CC1110
 *
 ***************************************************************/

#ifndef RADIO_CC1110_GFSK_H
#define RADIO_CC1110_GFSK_H

#define SMARTRF_RADIO_CC1110
#define SMARTRF_SETTING_PKTCTRL0    0x05

/* Frequency Setting   */
#define SMARTRF_SETTING_FREQ2       0x10    	// The base frequency for the frequency synthesizer calculated as Fclk * FrequencyControl / 65536 
#define SMARTRF_SETTING_FREQ1       0xA7	// Fbase = 26000000 Hz * 0x10A762 / 65536 = 26000000 * 1091426 / 65536
#define SMARTRF_SETTING_FREQ0       0x62	// Fbase = 432999816,894 Hz

#define SMARTRF_SETTING_MDMCFG0     0x00	// The channel spacing is multiplied by the channel number CHAN and added to the base frequency.
                                                // It is unsigned and has the format: Foffset = Fclk / 262144 * (256 + CHANSPC)
						// Foffset = 26000000/262144 * (256 + 0) = 25390,625 Hz

#define SMARTRF_SETTING_CHANNR      0x06	// The 8-bit unsigned channel number, which is multiplied by the channel spacing setting and added to the base frequency												 
						// Fcarrier = Fbase + (CHANNR * Foffset) = 432999816,894 + (6 * 25390,625) = 433202941,894 = 433,202 kHz
												
						// Writing a value from 2 to 12 in the CHANNR register, allow the Fcarrier to be set from 433.05 to 433.15 MHz. 
#define SMARTRF_SETTING_FSCTRL1     0x06
#define SMARTRF_SETTING_MDMCFG4     0xCA
#define SMARTRF_SETTING_MDMCFG3     0x83
#define SMARTRF_SETTING_MDMCFG2     0x13
#define SMARTRF_SETTING_MDMCFG1     0x20

#define SMARTRF_SETTING_DEVIATN     0x35
#define SMARTRF_SETTING_MCSM0       0x18
#define SMARTRF_SETTING_FOCCFG      0x1D
#define SMARTRF_SETTING_BSCFG       0x1C
#define SMARTRF_SETTING_AGCCTRL2    0x43
#define SMARTRF_SETTING_AGCCTRL1    0x00
#define SMARTRF_SETTING_AGCCTRL0    0xB0
#define SMARTRF_SETTING_FREND1      0xB6
#define SMARTRF_SETTING_FSCAL3      0xE9
#define SMARTRF_SETTING_FSCAL1      0x00
#define SMARTRF_SETTING_FSCAL0      0x1F
#define SMARTRF_SETTING_TEST1       0x31
#define SMARTRF_SETTING_TEST0       0x09
#define SMARTRF_SETTING_PA_TABLE0   0x60        // 0x60 = 0 dB; 0x84 = 5 dB; 0xC8 = 7 dB; 0xC0 = 10 dB 
#define SMARTRF_SETTING_IOCFG0      0x06
#define SMARTRF_SETTING_VERSION     0x04

#endif
