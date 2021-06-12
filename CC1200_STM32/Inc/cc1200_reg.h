/**
* @file cc1200_reg.h
* @author Cameron Moreno
* @date Feb 2 2021
* @brief Defines all the register addresses of the cc1200
* @see https://www.ti.com/lit/ug/swru346b/swru346b.pdf?ts=1612317166591&ref_url=https%253A%252F%252Fwww.google.com%252F \n
* Section 3 Microncontroller Interface \n
* Section 12 Register Description
*
*/
#ifndef CC1200_REG_H
#define CC1200_REG_H

#define CC1200_IOCFG3						0x0000 ///< GPIO3 Pin COnfiguration
#define CC1200_IOCFG2                   	0x0001 ///< GPIO2 Pin COnfiguration
#define CC1200_IOCFG1                   	0x0002 ///< GPIO1 Pin COnfiguration
#define CC1200_IOCFG0                   	0x0003 ///< GPIO0 Pin COnfiguration

#define CC1200_SYNC3                    	0x0004 ///< Sync Word Configuration [31:24]
#define CC1200_SYNC2                    	0x0005 ///< Sync Word Configuration [23:16]
#define CC1200_SYNC1                    	0x0006 ///< Sync Word Configuration [15:8]
#define CC1200_SYNC0                    	0x0007 ///< Sync Word Configuration [7:0]

#define CC1200_SYNC_CFG1                	0x0008 ///< Sync Word Configuration Reg. 1
#define CC1200_SYNC_CFG0                	0x0009 ///< Sync Word Configuration Reg. 0

#define CC1200_DEVIATION_M              	0x000A ///< Frequency Deviation Configuration
	
#define CC1200_MODCFG_DEV_E             	0x000B ///< Modulation Format and Frequency Deviation Configuration
	
#define CC1200_DCFILT_CFG               	0x000C ///< Digital DC Removal Configuration

#define CC1200_PREAMBLE_CFG1            	0x000D ///< Preamble Configuration Reg. 1
#define CC1200_PREAMBLE_CFG0            	0x000E ///< Preamble Configuration Reg. 0
	
#define CC1200_IQIC                     	0x000F ///< Digital Image Channel Compensation Configuration
	
#define CC1200_CHAN_BW                  	0x0010 ///< Channel Filter Configuration
	
#define CC1200_MDMCFG1                  	0x0011 ///< General Modem Parameter Configuration Reg. 1
#define CC1200_MDMCFG0                  	0x0012 ///< General Modem Parameter Configuration Reg. 0
	
#define CC1200_SYMBOL_RATE2             	0x0013 ///< Symbol Rate Configuration Exponent and Mantissa [19:16]
#define CC1200_SYMBOL_RATE1             	0x0014 ///< Symbol Rate Configuration Exponent and Mantissa [15:8]
#define CC1200_SYMBOL_RATE0             	0x0015 ///< Symbol Rate Configuration Exponent and Mantissa [7:0]
	
#define CC1200_AGC_REF                  	0x0016 ///< AGC Reference Level Configuration

#define CC1200_AGC_CS_THR               	0x0017 ///< Carrier Sense Threshold Configuration
	
#define CC1200_AGC_GAIN_ADJUSTMENT      	0x0018 ///< RSSI Offset Configuration
	
#define CC1200_AGC_CFG3                 	0x0019 ///< Automatic Gain Control Configuration Reg. 3
#define CC1200_AGC_CFG2                 	0x001A ///< Automatic Gain Control Configuration Reg. 2
#define CC1200_AGC_CFG1                 	0x001B ///< Automatic Gain Control Configuration Reg. 1
#define CC1200_AGC_CFG0                 	0x001C ///< Automatic Gain Control Configuration Reg. 0
	
#define CC1200_FIFO_CFG                 	0x001D ///< FIFO Configuration
	
#define CC1200_DEV_ADDR                 	0x001E ///< Device Address Configuration
	
#define CC1200_SETTLING_CFG             	0x001F ///< Frequency Synthesizer Calibration and Settling Configuration

#define CC1200_FS_CFG                   	0x0020 ///< Frequency Synthesizer Configuration

#define CC1200_WOR_CFG1                 	0x0021 ///< Enhanced Wake On Radio Configuration Reg. 1
#define CC1200_WOR_CFG0                 	0x0022 ///< Enhanced Wake On Radio Configuration Reg. 0
#define CC1200_WOR_EVENT0_MSB           	0x0023 ///< Event 0 Configuration MSB
#define CC1200_WOR_EVENT0_LSB           	0x0024 ///< Event 0 Configuration LSB

#define CC1200_RXDCM_TIME               	0x0025 ///< RX Duty Cycle Mode Configuration	
	
#define CC1200_PKT_CFG2                 	0x0026 ///< Packet Configuration Reg. 2
#define CC1200_PKT_CFG1                 	0x0027 ///< Packet Configuration Reg. 1
#define CC1200_PKT_CFG0                 	0x0028 ///< Packet Configuration Reg. 0
	
#define CC1200_RFEND_CFG1               	0x0029 ///< RFEND Configuration Reg. 1
#define CC1200_RFEND_CFG0               	0x002A ///< RFEND Configuraton Reg. 0
	
#define CC1200_PA_CFG1                  	0x002B ///< Power Amplifier Configuration Reg. 1
#define CC1200_PA_CFG0                  	0x002C ///< Power Amplifier Configuration Reg. 0

#define CC1200_ASK_CFG                  	0x002D ///< Amplitude Shift Keying Configuration
	
#define CC1200_PKT_LEN                  	0x002E /// < Packet Length Configuration
	
#define CC1200_EXT_ADDR                 	0x2F00 ///< Extended Address

/*************************************************************/
/*					EXTENDED REGISTERS     					 */
/*************************************************************/

#define CC1200_IF_MIX_CFG					0x2F00 ///< IF Mix Configuratoin
	
#define CC1200_FREQOFF_CFG					0x2F01 ///< Frequency Offset Correction Configuration
	
#define CC1200_TOC_CFG						0x2F02 ///< Timing Offset Correction Configuration
	
#define CC1200_MARC_SPARE					0x2F03 ///< MARC Spare
	
#define CC1200_ECG_CFG						0x2F04 ///< External Clock Frequency Configuration
	
#define CC1200_MDMCFG2						0x2F05 ///< General Modem Parameter Configuration
	
#define CC1200_EXT_CTRL						0x2F06 ///< External Control Configuration
	
#define CC1200_RCCAL_FINE					0x2F07 ///< RC Oscillator Calibration Fine
	
#define CC1200_RCCAL_COURSE					0x2F08 ///< RC Oscillator Calibration Course
	
#define CC1200_RCCAL_OFFSET					0x2F09 ///< RC Oscillator Calibration Clock Offset
	
#define CC1200_FREQOFF1						0x2F0A ///< Frequency Offset MSB
#define CC1200_FREQOFF0						0x2F0B ///< Frequency Offset LSB
	
#define CC1200_FREQ2						0x2F0C ///< Frequency Configuration [23:16]
#define CC1200_FREQ1						0x2F0D ///< Frequency Configuraton [15:8]
#define CC1200_FREQ0						0x2F0E ///< Frequency Configuraton [7:0]
	
#define CC1200_IF_ADC2						0x2F0F ///< Analog to Digital Converter Configuration Reg. 2
#define CC1200_IF_ADC1						0x2F10 ///< Analog to Digital Converter Configuration Reg. 1
#define CC1200_IF_ADC0						0x2F11 ///< Analog to Digital Converter Configuration Reg. 0
	
#define CC1200_FS_DIG1						0x2F12 ///< Frequency Synthesizer Digital Reg. 1
#define CC1200_FS_DIG0						0x2F13 ///< Frequency Synthesizer Digital Reg. 0
	
#define CC1200_FS_CAL3						0x2F14 ///< Frequency Synthesizer Calibration Reg. 3
#define CC1200_FS_CAL2						0x2F15 ///< Frequency Synthesizer Calibration Reg. 2
#define CC1200_FS_CAL1						0x2F16 ///< Frequency Synthesizer Calibration Reg. 1
#define CC1200_FS_CAL0						0x2F17 ///< Frequency Synthesizer Calibration Reg. 0
	
#define CC1200_FS_CHP						0x2F18 ///< Frequency Synthesizer Charge Pump Configuration
	
#define CC1200_FS_DIVTWO					0x2F19 ///< Frequency Synthesizer Divide by 2
	
#define CC1200_FS_DSM1						0x2F1A ///< FS Digital Synthesizer Module Configuration Reg. 1
#define CC1200_FS_DSM0						0x2F1B ///< FS Digital Synthesizer Module Configuration Reg. 0
	
#define CC1200_FS_DVC1						0x2F1C ///< FS Digital Synthesizer Divider Chain Configuration Reg. 1
#define CC1200_FS_DVC0						0x2F1D ///< FS Digital Synthesizer Divider Chain Configuration Reg. 0
	
#define CC1200_FS_LBI						0x2F1E ///< Frequency Synthesizer Local Bias Configuration
	
#define CC1200_FS_PFD						0x2F1F ///< Frequency Synthesizer Phase Frequency Detector Configuration
	
#define CC1200_FS_PRE						0x2F20 ///< Frequency Synthesizer Prescaler Configuration
	
#define CC1200_FS_REG_DIV_CML				0x2F21 ///< Frequency Synthesizer Divider Regulator Configuration
	
#define CC1200_FS_SPARE						0x2F22 ///< Frequency Synthesizer Spare
	
#define CC1200_FS_VCO4						0x2F23 ///< FS Voltage Controlled Oscillator Configuration Reg. 4
#define CC1200_FS_VCO3						0x2F24 ///< FS Voltage Controlled Oscillator Configuration Reg. 3
#define CC1200_FS_VCO2						0x2F25 ///< FS Voltage Controlled Oscillator Configuration Reg. 2
#define CC1200_FS_VCO1						0x2F26 ///< FS Voltage Controlled Oscillator Configuration Reg. 1
#define CC1200_FS_VCO0						0x2F27 ///< FS Voltage Controlled Oscillator Configuration Reg. 0
	
#define CC1200_GBIAS6						0x2F28 ///< Global Bias Configuration Reg. 6
#define CC1200_GBIAS5						0x2F29 ///< Global Bias Configuration Reg. 5
#define CC1200_GBIAS4						0x2F2A ///< Global Bias Configuration Reg. 4
#define CC1200_GBIAS3						0x2F2B ///< Global Bias Configuration Reg. 3
#define CC1200_GBIAS2						0x2F2C ///< Global Bias Configuration Reg. 2
#define CC1200_GBIAS1						0x2F2D ///< Global Bias Configuration Reg. 1
#define CC1200_GBIAS0						0x2F2E ///< Global Bias Configuration Reg. 0
	
#define CC1200_IFAMP						0x2F2F ///< Intermediate Frequency Amplifier Configuration

#define CC1200_LNA							0x2F30 ///< Low Noise Amplifier Configuration
	
#define CC1200_RXMIX						0x2F31 ///< RX Mixer Configuration
	
#define CC1200_XOSC5						0x2F32 ///< Crystal Oscillator Configuration Reg. 5
#define CC1200_XOSC4						0x2F33 ///< Crystal Oscillator Configuration Reg. 4
#define CC1200_XOSC3						0x2F34 ///< Crystal Oscillator Configuration Reg. 3
#define CC1200_XOSC2						0x2F35 ///< Crystal Oscillator Configuration Reg. 2
#define CC1200_XOSC1						0x2F36 ///< Crystal Oscillator Configuration Reg. 1
#define CC1200_XOSC0						0x2F37 ///< Crystal Oscillator Configuration Reg. 0
	
#define CC1200_ANALOG_SPARE					0x2F38 ///< Analog Spare
	
#define CC1200_PA_CFG3						0x2F39 ///< Power Amplifier Configuration

#define CC1200_WOR_TIME1					0x2F64 ///< eWOR Timer Counter Value MSB
#define CC1200_WOR_TIME0					0x2F65 ///< eWOR Timer Counter Value LSB
	
#define CC1200_WOR_CAPTURE1					0x2F66 ///< eWOR Timer Capture Value MSB
#define CC1200_WOR_CAPTURE0					0x2F67 ///< eWOR Timer Capture Value LSB
	
#define CC1200_BIST							0x2F68 ///< MARC Built-In Self-Test
	
#define CC1200_DCFILTOFFSET_I1				0x2F69 ///< DC Filter Offset I MSB
	
#define CC1200_DCFILTOFFSET_I0				0x2F6A ///< DC Filter Offset I LSB
	
#define CC1200_DCFILTOFFSET_Q1				0x2F6B ///< DC Filter Offset Q MSB
	
#define CC1200_DCFILTOFFSET_Q0				0x2F6C ///< DC Filter Offset Q LSB
	
#define CC1200_IQIE_I1						0x2F6D ///< IQ Imbalance Value I MSB
#define CC1200_IQIE_I0						0x2F6E ///< IQ Imbalance Value I LSB
	
#define CC1200_IQIE_Q1						0x2F6F ///< IQ Imbalance Value Q MSB
#define CC1200_IQIE_Q0						0x2F70 ///< IQ Imbalance Value Q LSB
	
#define CC1200_RSSI1						0x2F71 ///< Receive Signal Strength Indicator Reg. 1
#define CC1200_RSSI0						0x2F72 ///< Receive Signal Strength Indicator Reg. 0
	
#define CC1200_MARCSTATE					0x2F73 ///< MARC State
	
#define CC1200_LQI_VAL						0x2F74 ///< Link Quality Indicator Value
	
#define CC1200_PQT_SYNC_ERR					0x2F75 ///< Preamble and Sync Word Error
	
#define CC1200_DEM_STATUS					0x2F76 ///< Demodulator Status
	
#define CC1200_FREQOFF_EST1					0x2F77 ///< Frequency Offset Estimate MSB
	
#define CC1200_FREQOFF_EST0					0x2F78 ///< Frequency Offset Estimate LSB
	
#define CC1200_AGC_GAIN3					0x2F79 ///< Automatic Gain Control Reg. 3
#define CC1200_AGC_GAIN2					0x2F7A ///< Automatic Gain Control Reg. 2
#define CC1200_AGC_GAIN1					0x2F7B ///< Automatic Gain Control Reg. 1
#define CC1200_AGC_GAIN0					0x2F7C ///< Automatic Gain Control Reg. 0
	
#define CC1200_CFM_RX_DATA_OUT				0x2F7D ///< Custom Frequency Modulation RX Data
	
#define CC1200_CFM_TX_DATA_IN				0x2F7E ///< Custom Frequency Modulation TX Data
	
#define CC1200_ASK_SOFT_RX_DATA				0x2F7F ///< ASK Soft Decision Output
	
#define CC1200_RNDGEN						0x2F80 ///< Random Number Generator Value
	
#define CC1200_MAGN2						0x2F81 ///< Signal Magnitude after CORDIC [16]
#define CC1200_MAGN1						0x2F82 ///< Signal Magnitude after CORDIC [15:8]
#define CC1200_MAGN0						0x2F83 ///< Signal Magnitude after CORDIC [7:0]
	
#define CC1200_ANG1							0x2F84 ///< Signal Angular after CORDIC [9:8]
#define CC1200_ANG0							0x2F85 ///< Signal Angular after CORDIC [7:0]
	
#define CC1200_CHFILT_I2					0x2F86 ///< Channel Filter Data Real Part [16]		 							
#define CC1200_CHFILT_I1					0x2F87 ///< Channel Filter Data Real Part [15:8]
#define CC1200_CHFILT_I0					0x2F88 ///< Channel Filter Data Real Part [7:0]
	
#define CC1200_CHFILT_Q2					0x2F89 ///< Channel Filter Data Imaginary Part [16]
#define CC1200_CHFILT_Q1					0x2F8A ///< Channel Filter Data Imaginary Part [15:8]
#define CC1200_CHFILT_Q0					0x2F8B ///< Channel Filter Data Imaginary Part [7:0]
	
#define CC1200_GPIO_STATUS					0x2F8C ///< General Purpose Input/Output Status
	
#define CC1200_FSCAL_CTRL					0x2F8D ///< Frequency Synthesizer Calibration Control
	
#define CC1200_PHASE_ADJUST					0x2F8E ///< Frequency Synthesizer Phase Adjust
	
#define CC1200_PARTNUMBER					0x2F8F ///< Part Number
	
#define CC1200_PARTVERSION					0x2F90 ///< Part Revision
	
#define CC1200_SERIAL_STATUS				0x2F91 ///< Serial Status
	
#define CC1200_MODEM_STATUS1				0x2F92 ///< Modem Status Reg. 1
#define CC1200_MODEM_STATUS0				0x2F93 ///< Modem Status Reg. 0
	
#define CC1200_MARC_STATUS1					0x2F94 ///< MARC Status Reg. 1
#define CC1200_MARC_STATUS0					0x2F95 ///< MARC Status Reg. 0
	
#define CC1200_PA_IFAMP_TEST				0x2F96 ///< Power Amplifier Intermediate Frequency Amplifier Test
	
#define CC1200_FSRF_TEST					0x2F97 ///< Frequency Synthesizer Test
	
#define CC1200_PRE_TEST						0x2F98 ///< Frequency Synthesizer Prescaler Test
	
#define CC1200_PRE_OVR						0x2F99 ///< Frequency Synthesizer Prescaler Override
	
#define CC1200_ADC_TEST						0x2F9A ///< Analog to Digital Converter Test
	
#define CC1200_DVC_TEST						0x2F9B ///< Digital Divide Chain Test
	
#define CC1200_ATEST						0x2F9C ///< Analog Test
	
#define CC1200_ATEST_LVDS					0x2F9D ///< Analog Test LVDS
	
#define CC1200_ATEST_MODE					0x2F9E ///< Analog Test Mode
	
#define CC1200_XOSC_TEST1					0x2F9F ///< Crystal Oscillator Test Reg. 1
#define CC1200_XOSC_TEST0					0x2FA0 ///< Crystal Oscillator Test Reg. 0
	
#define CC1200_AES							0x2FA1 ///< Advanced Encryption Standard Status
	
#define CC1200_MDM_TEST						0x2FA2 ///< Modem Test
	
#define CC1200_RXFIRST						0x2FD2 ///< RX FIFO Pointer First Entry
	
#define CC1200_TXFIRST						0x2FD3 ///< TX FIFO Pointer First Entry
	
#define CC1200_RXLAST						0x2FD4 ///< RX FIFO Pointer Last Entry
	
#define CC1200_TXLAST						0x2FD5 ///< TX FIFO Point Last Entry
	
#define CC1200_NUM_TXBYTES					0x2FD6 ///< TX FIFO Status
	
#define CC1200_NUM_RXBYTES					0x2FD7 ///< RX FIFO Status
	
#define CC1200_FIFO_NUM_TXBYTES				0x2FD8 ///< TX FIFO Status
	
#define CC1200_FIFO_NUM_RXBYTES				0x2FD9 ///< RX FIFO Status
	
#define CC1200_RXFIFO_PRE_BUF				0x2FDA ///< RX FIFO First Byte

/*************************************************************/
/*					COMMAND STROBES     					 */
/*************************************************************/
#define CC1200_COMMAND_SRES					0x30 ///< Reset Chip
#define CC1200_COMMAND_SFSTXON				0x31 ///< Enable and calibrate frequency synthesizer
#define CC1200_COMMAND_SXOFF				0x32 ///< Enter XOFF state when CSn is de-asserted
#define CC1200_COMMAND_SCAL					0x33 ///< Calibrate frequency synthesizer
#define CC1200_COMMAND_SRX					0x34 ///< Enable RX
#define CC1200_COMMAND_STX					0x35 ///< Enable TX
#define CC1200_COMMAND_SIDLE				0x36 ///< Exit RX/TX, turn of freqency synthesizer and exit eWOR mode
#define CC1200_COMMAND_SAFC					0x37 ///< Automatic Frequency Compensation
#define CC1200_COMMAND_SWOR					0x38 ///< Start automatic RX polling sequence (eWOR) as shown in Sec 9.6
#define CC1200_COMMAND_SPWD					0x39 ///< Enter SLEEP mode when CSN is de-asserted
#define CC1200_COMMAND_SFRX					0x3A ///< Flush the RX FIFO
#define CC1200_COMMAND_SFTX					0x3B ///< Flush the TX FIFO
#define CC1200_COMMAND_SWORRST				0x3C ///< Reset the eWOR timer to the Event 1 value
#define CC1200_COMMAND_NOP					0x3D ///< No operation

/*************************************************************/
/*						SPI MEMORY ACCESS   				 */
/*************************************************************/
#define CC1200_SPI_SINGLE_READ				0x8000 ///< R/WR bit of the header = 1
#define CC1200_SPI_SINGLE_WRITE				0x0000 ///< R/WR bit of the header = 0
#define CC1200_SPI_BURST_ON					0x4000 ///< Burst bit of the header = 1
#define CC1200_SPI_BURST_OFF				0x0000 ///< Burst bit of the header = 0

/*************************************************************/
/*						STATUS BYTES    					 */
/*************************************************************/
#define CC1200_STATE_IDLE              		0x00 ///< IDLE state
#define CC1200_STATE_RX                		0x10 ///< Receive mode
#define CC1200_STATE_TX                		0x20 ///< Transmit mode
#define CC1200_STATE_FSTXON 	           	0x30 ///< Fast TX ready
#define CC1200_STATE_CALIBRATE         		0x40 ///< Frequency synthesizer calibration is running
#define CC1200_STATE_SETTLING          		0x50 ///< PLL is settling
#define CC1200_STATE_RXFIFO_ERROR      		0x60 ///< RX FIFO has over/underflowed
#define CC1200_STATE_TXFIFO_ERROR      		0x70 ///< TX FIFO has over/underflowed

/*************************************************************/
/*						MARC_STATUS1	    				 */
/*************************************************************/
#define CC1200_MARC_STATUS1_NO_FAILURE				0x00 ///< No failure detected
#define CC1200_MARC_STATUS1_RX_TIMEOUT				0x01 ///< RX timeout occurred
#define CC1200_MARC_STATUS1_RX_TERM					0x02 ///< RX termination based on CS or PQT
#define CC1200_MARC_STATUS1_EWOR_SYNC_LOST			0x03 ///< eWOR sync lost (16 slots with no successful reception)
#define CC1200_MARC_STATUS1_MAX_LENGTH_FILTER		0x04 ///< Packet discarded due to maximum length filtering
#define CC1200_MARC_STATUS1_ADDRESS_FILTER			0x05 ///< Packet discarded due to address filtering
#define CC1200_MARC_STATUS1_CRC_FILTER				0x06 ///< Packet discarded due to CRC filtering
#define CC1200_MARC_STATUS1_TX_OVERFLOW				0x07 ///< TX FIFO overflow error occurred
#define CC1200_MARC_STATUS1_TX_UNDERFLOW			0x08 ///< TX FIFO underflow occurred
#define CC1200_MARC_STATUS1_RX_OVERFLOW				0x09 ///< RX FIFO overflow error occurred
#define CC1200_MARC_STATUS1_RX_UNDERFLOW			0x0A ///< RX FIFO underflow error occurred
#define CC1200_MARC_STATUS1_TX_ON_CCA_FAIL			0x0B ///< TX ON CCA failed
#define CC1200_MARC_STATUS1_TX_SUCCESS				0x40 ///< TX finished successfully
#define CC1200_MARC_STATUS1_RX_SUCCSES				0x80 ///< RX finished successfully

/*************************************************************/
/*						MARC 2 PIN STATE    				 */
/*************************************************************/
#define CC1200_MARC_STATE_SETTLING					0x00 ///< Settling
#define CC1200_MARC_STATE_TX						0x01 ///< TX
#define CC1200_MARC_STATE_IDLE						0x02 ///< Idle
#define CC1200_MARC_STATE_RX						0x03 ///< RX

/*************************************************************/
/*							MARC_STATE	    				 */
/*************************************************************/
#define CC1200_MARC_PIN2_STATE_SLEEP				0x00 ///< Depends on GPIO pins used
#define CC1200_MARC_PIN2_STATE_IDLE					0x01 ///< IDLE
#define CC1200_MARC_PIN2_STATE_XOFF					0x02 ///< SETTLING
#define CC1200_MARC_PIN2_STATE_BIAS_SETTLE_MC		0x03 ///< SETTLING
#define CC1200_MARC_PIN2_STATE_REG_SETTLE_MC		0x04 ///< SETTLING
#define CC1200_MARC_PIN2_STATE_MANCAL				0x05 ///< SETTLING
#define CC1200_MARC_PIN2_STATE_BIAS_SETTLE			0x06 ///< SETTLING
#define CC1200_MARC_PIN2_STATE_REG_SETTLE			0x07 ///< SETTLING
#define CC1200_MARC_PIN2_STATE_STARTCAL				0x08 ///< SETTLING
#define CC1200_MARC_PIN2_STATE_BWBOOST				0x09 ///< SETTLING
#define CC1200_MARC_PIN2_STATE_FS_LOCK				0x0A ///< SETTLING
#define CC1200_MARC_PIN2_STATE_IFADCON				0x0B ///< SETTLING
#define CC1200_MARC_PIN2_STATE_ENDCAL				0x0C ///< SETTLING
#define CC1200_MARC_PIN2_STATE_RX					0x0D ///< RX
#define CC1200_MARC_PIN2_STATE_RX_END				0x0E ///< RX
#define CC1200_MARC_PIN2_STATE_RXDCM				0x0F ///< RX
#define CC1200_MARC_PIN2_STATE_TXRX_SWITCH			0x10 ///< SETTLING
#define CC1200_MARC_PIN2_STATE_RX_FIFO_ERR			0x11 ///< SETTLING
#define CC1200_MARC_PIN2_STATE_FSTXON				0x12 ///< SETTLING
#define CC1200_MARC_PIN2_STATE_TX					0x13 ///< TX
#define CC1200_MARC_PIN2_STATE_TX_END				0x14 ///< TX
#define CC1200_MARC_PIN2_STATE_RXTX_SWITCH			0x15 ///< SETTLING
#define CC1200_MARC_PIN2_STATE_TX_FIFO_ERR			0x16 ///< SETTLING
#define CC1200_MARC_PIN2_STATE_IFADCON_TXRX			0x17 ///< SETTLING

/*************************************************************/
/*						RESERVED REGISTERS    				 */
/*************************************************************/
// Reserved registers		
#define CC1200_RESERVED_LOWER_1				0x2F40
#define CC1200_RESERVED_UPPER_1				0x2F63
#define CC1200_RESERVED_LOWER_2				0x2FA3
#define CC1200_RESERVED_UPPER_2				0x2FD1

#endif /*CC1200_REG_H*/