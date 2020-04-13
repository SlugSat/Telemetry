/***************************************************************
 *  SmartRF Studio(tm) Export
 *
 *  Radio register settings specifed with C-code
 *  compatible #define statements.
 *
 *  RF device: CC1200
 *
 ***************************************************************/

#ifndef SMARTRF_CC1200_H
#define SMARTRF_CC1200_H

#define SMARTRF_RADIO_CC1200
#define SMARTRF_SETTING_IOCFG2           0x06
#define SMARTRF_SETTING_DEVIATION_M      0xD1
#define SMARTRF_SETTING_MODCFG_DEV_E     0x00
#define SMARTRF_SETTING_DCFILT_CFG       0x5D
#define SMARTRF_SETTING_PREAMBLE_CFG0    0x8A
#define SMARTRF_SETTING_IQIC             0xCB
#define SMARTRF_SETTING_CHAN_BW          0xA6
#define SMARTRF_SETTING_MDMCFG1          0x40
#define SMARTRF_SETTING_MDMCFG0          0x05
#define SMARTRF_SETTING_SYMBOL_RATE2     0x3F
#define SMARTRF_SETTING_SYMBOL_RATE1     0x75
#define SMARTRF_SETTING_SYMBOL_RATE0     0x10
#define SMARTRF_SETTING_AGC_REF          0x20
#define SMARTRF_SETTING_AGC_CS_THR       0xEC
#define SMARTRF_SETTING_AGC_CFG1         0x51
#define SMARTRF_SETTING_AGC_CFG0         0x87
#define SMARTRF_SETTING_FIFO_CFG         0x00
#define SMARTRF_SETTING_FS_CFG           0x14
#define SMARTRF_SETTING_PKT_CFG2         0x00
//#define SMARTRF_SETTING_PKT_CFG0         0x20
//#define SMARTRF_SETTING_PKT_LEN          0xFF
#define SMARTRF_SETTING_IF_MIX_CFG       0x1C
#define SMARTRF_SETTING_FREQOFF_CFG      0x22
#define SMARTRF_SETTING_MDMCFG2          0x0C
#define SMARTRF_SETTING_FREQ2            0x56
#define SMARTRF_SETTING_FREQ1            0xCC
#define SMARTRF_SETTING_FREQ0            0xCC
#define SMARTRF_SETTING_IF_ADC1          0xEE
#define SMARTRF_SETTING_IF_ADC0          0x10
#define SMARTRF_SETTING_FS_DIG1          0x07
#define SMARTRF_SETTING_FS_DIG0          0xAF
#define SMARTRF_SETTING_FS_CAL1          0x40
#define SMARTRF_SETTING_FS_CAL0          0x0E
#define SMARTRF_SETTING_FS_DIVTWO        0x03
#define SMARTRF_SETTING_FS_DSM0          0x33
#define SMARTRF_SETTING_FS_DVC0          0x17
#define SMARTRF_SETTING_FS_PFD           0x00
#define SMARTRF_SETTING_FS_PRE           0x6E
#define SMARTRF_SETTING_FS_REG_DIV_CML   0x1C
#define SMARTRF_SETTING_FS_SPARE         0xAC
#define SMARTRF_SETTING_FS_VCO0          0xB5
#define SMARTRF_SETTING_XOSC5            0x0E
#define SMARTRF_SETTING_XOSC1            0x03

// Custom configration values to have a fixed length packet
#define SMARTRF_SETTING_PKT_CFG0         0x00
#define SMARTRF_SETTING_PKT_LEN          0x13
#endif
