/**
* @file cc1200.c
* @author Cameron Moreno
* @brief Provides code for interacting with the CC1200
* @date April 3 2021
*
* @todo The following tasks should be complete by the end of Spring quarter 2021:
*		- [ ]	Implement a circuit buffer for TX and RX which are separate from the SPI buffer.
*		- [ ]	Currently, CC1200_cofigure() and CC1200_verify_configure() are separte functions.
*				this means that the CC1200 address space is iterated over twice. It would be better
*				to just read the register immedately after writing to verify that it was written to
*				correctly.
*		- [ ]	Fix the awkward logic in the CC1200 read and write functions for checking if the address
*				is readable and writable. I needed to invert isWriteable for some reason. Find out why.
*		- [ ]	Some variables are made just becuase the HAL functions take pointers to variables. Determine
*				if there is a better way to do this and implement if there is.
*		- [ ]	Go through all functions and set cc1200 status bits accordingly. The struct for this
*				is defined in cc1200.h.
*/
#include "cc1200.h"
#include <string.h>

/**
 *  Register settings copied from the SmartRF Studio software
 */

// TX Parameters (Experimental)
//
//const registerSetting_t preferredSettings[]=
//{
//  {CC1200_IOCFG2,            0x06},
//  {CC1200_SYNC_CFG1,         0xA8},
//  {CC1200_MODCFG_DEV_E,      0x29},
//  {CC1200_DCFILT_CFG,        0x1E},
//  {CC1200_PREAMBLE_CFG1,     0x00},
//  {CC1200_PREAMBLE_CFG0,     0x8A},
//  {CC1200_IQIC,              0x00},
//  {CC1200_CHAN_BW,           0x45},
//  {CC1200_MDMCFG1,           0x42},
//  {CC1200_MDMCFG0,           0x05},
//  {CC1200_SYMBOL_RATE2,      0x96},
//  {CC1200_SYMBOL_RATE1,      0xF0},
//  {CC1200_SYMBOL_RATE0,      0x07},
//  {CC1200_AGC_REF,           0x29},
//  {CC1200_AGC_CS_THR,        0xF8},
//  {CC1200_AGC_CFG2,          0x60},
//  {CC1200_AGC_CFG1,          0x12},
//  {CC1200_AGC_CFG0,          0x84},
//  {CC1200_FIFO_CFG,          0x00},
//  {CC1200_FS_CFG,            0x14},
//  {CC1200_PKT_CFG2,          0x00},
//  {CC1200_PKT_CFG0,          0x20},
//  {CC1200_PA_CFG1,           0x5F},
//  {CC1200_PA_CFG0,           0x53},
//  {CC1200_PKT_LEN,           0xFF},
//  {CC1200_FREQOFF_CFG,       0x23},
//  {CC1200_MDMCFG2,           0x00},
//  {CC1200_FREQ2,             0x57},
//  {CC1200_FREQ1,             0x4C},
//  {CC1200_FREQ0,             0xCC},
//  {CC1200_IF_ADC1,           0xEE},
//  {CC1200_IF_ADC0,           0x10},
//  {CC1200_FS_DIG1,           0x04},
//  {CC1200_FS_DIG0,           0xA3},
//  {CC1200_FS_CAL1,           0x40},
//  {CC1200_FS_CAL0,           0x0E},
//  {CC1200_FS_DIVTWO,         0x03},
//  {CC1200_FS_DSM0,           0x33},
//  {CC1200_FS_DVC1,           0xF7},
//  {CC1200_FS_DVC0,           0x0F},
//  {CC1200_FS_PFD,            0x00},
//  {CC1200_FS_PRE,            0x6E},
//  {CC1200_FS_REG_DIV_CML,    0x1C},
//  {CC1200_FS_SPARE,          0xAC},
//  {CC1200_FS_VCO0,           0xB5},
//  {CC1200_IFAMP,             0x0D},
//  {CC1200_XOSC5,             0x0E},
//  {CC1200_XOSC1,             0x03},
//  {CC1200_PARTNUMBER,        0x20},
//  {CC1200_PARTVERSION,       0x11},
//  {CC1200_MODEM_STATUS1,     0x10},
//};



//RX Parameters (Experimental)
//
//const registerSetting_t preferredSettings[]=
//{
//  {CC1200_IOCFG2,            0x06},
//  {CC1200_SYNC_CFG1,         0xA8},
//  {CC1200_MODCFG_DEV_E,      0x29},
//  {CC1200_DCFILT_CFG,        0x5D},
//  {CC1200_PREAMBLE_CFG0,     0x8A},
//  {CC1200_IQIC,              0xCB},
//  {CC1200_CHAN_BW,           0x45},
//  {CC1200_MDMCFG1,           0x40},
//  {CC1200_MDMCFG0,           0x05},
//  {CC1200_SYMBOL_RATE2,      0x96},
//  {CC1200_SYMBOL_RATE1,      0xF0},
//  {CC1200_SYMBOL_RATE0,      0x07},
//  {CC1200_AGC_REF,           0x3B},
//  {CC1200_AGC_CS_THR,        0xEC},
//  {CC1200_AGC_CFG1,          0x51},
//  {CC1200_AGC_CFG0,          0x87},
//  {CC1200_FIFO_CFG,          0x00},
//  {CC1200_FS_CFG,            0x14},
//  {CC1200_PKT_CFG2,          0x00},
//  {CC1200_PKT_CFG0,          0x20},
//  {CC1200_PKT_LEN,           0xFF},
//  {CC1200_IF_MIX_CFG,        0x1C},
//  {CC1200_FREQOFF_CFG,       0x22},
//  {CC1200_MDMCFG2,           0x0C},
//  {CC1200_FREQ2,             0x57},
//  {CC1200_FREQ1,             0x4C},
//  {CC1200_FREQ0,             0xCC},
//  {CC1200_IF_ADC1,           0xEE},
//  {CC1200_IF_ADC0,           0x10},
//  {CC1200_FS_DIG1,           0x07},
//  {CC1200_FS_DIG0,           0xAF},
//  {CC1200_FS_CAL1,           0x40},
//  {CC1200_FS_CAL0,           0x0E},
//  {CC1200_FS_DIVTWO,         0x03},
//  {CC1200_FS_DSM0,           0x33},
//  {CC1200_FS_DVC0,           0x17},
//  {CC1200_FS_PFD,            0x00},
//  {CC1200_FS_PRE,            0x6E},
//  {CC1200_FS_REG_DIV_CML,    0x1C},
//  {CC1200_FS_SPARE,          0xAC},
//  {CC1200_FS_VCO0,           0xB5},
//  {CC1200_XOSC5,             0x0E},
//  {CC1200_XOSC1,             0x03},
//  {CC1200_PARTNUMBER,        0x20},
//  {CC1200_PARTVERSION,       0x11},
//  {CC1200_MODEM_STATUS1,     0x10},
//};


// SmartRF Studio Presets
const registerSetting_t preferredSettings[]=
{
  {CC1200_IOCFG2,            0x06},
  {CC1200_SYNC_CFG1,         0xA8},
  {CC1200_DEVIATION_M,       0x47},
  {CC1200_MODCFG_DEV_E,      0x2F},
  {CC1200_DCFILT_CFG,        0x1E},
  {CC1200_PREAMBLE_CFG0,     0x8A},
  {CC1200_IQIC,              0x00},
  {CC1200_CHAN_BW,           0x01},
  {CC1200_MDMCFG1,           0x42},
  {CC1200_MDMCFG0,           0x05},
  {CC1200_SYMBOL_RATE2,      0xC9},
  {CC1200_SYMBOL_RATE1,      0x99},
  {CC1200_SYMBOL_RATE0,      0x99},
  {CC1200_AGC_REF,           0x2F},
  {CC1200_AGC_CS_THR,        0xF8},
  {CC1200_AGC_CFG2,          0x60},
  {CC1200_AGC_CFG1,          0x12},
  {CC1200_AGC_CFG0,          0x84},
  {CC1200_FIFO_CFG,          0x00},
  {CC1200_FS_CFG,            0x14},
  {CC1200_PKT_CFG2,          0x00},
  {CC1200_PKT_CFG0,          0x20},
  {CC1200_PKT_LEN,           0xFF},
  {CC1200_FREQOFF_CFG,       0x23},
  {CC1200_MDMCFG2,           0x00},
  {CC1200_FREQ2,             0x57},
  {CC1200_FREQ1,             0x4C},
  {CC1200_FREQ0,             0xCC},
  {CC1200_IF_ADC1,           0xEE},
  {CC1200_IF_ADC0,           0x10},
  {CC1200_FS_DIG1,           0x04},
  {CC1200_FS_DIG0,           0xA3},
  {CC1200_FS_CAL1,           0x40},
  {CC1200_FS_CAL0,           0x0E},
  {CC1200_FS_DIVTWO,         0x03},
  {CC1200_FS_DSM0,           0x33},
  {CC1200_FS_DVC1,           0xF7},
  {CC1200_FS_DVC0,           0x0F},
  {CC1200_FS_PFD,            0x00},
  {CC1200_FS_PRE,            0x6E},
  {CC1200_FS_REG_DIV_CML,    0x1C},
  {CC1200_FS_SPARE,          0xAC},
  {CC1200_FS_VCO0,           0xB5},
  {CC1200_IFAMP,             0x0D},
  {CC1200_XOSC5,             0x0E},
  {CC1200_XOSC1,             0x03},
};

//eWor Test
//const registerSetting_t preferredSettings[]=
//{
//	{CC1200_IOCFG2,            0x06},
//	{CC1200_IOCFG0,            0x14},
//	{CC1200_SYNC_CFG1,         0xA8},
//	{CC1200_DEVIATION_M,       0x47},
//	{CC1200_MODCFG_DEV_E,      0x2F},
//	{CC1200_DCFILT_CFG,        0x1E},
//	{CC1200_PREAMBLE_CFG0,     0x8A},
//	{CC1200_IQIC,              0x00},
//	{CC1200_CHAN_BW,           0x01},
//	{CC1200_MDMCFG1,           0x42},
//	{CC1200_MDMCFG0,           0x05},
//	{CC1200_SYMBOL_RATE2,      0xC9},
//	{CC1200_SYMBOL_RATE1,      0x99},
//	{CC1200_SYMBOL_RATE0,      0x99},
//	{CC1200_AGC_REF,           0x2F},
//	{CC1200_AGC_CS_THR,        0xF8},
//	{CC1200_AGC_CFG2,          0x60},
//	{CC1200_AGC_CFG1,          0x12},
//	{CC1200_AGC_CFG0,          0x84},
//	{CC1200_FIFO_CFG,          0x00},
//	{CC1200_FS_CFG,            0x12},
//	{CC1200_WOR_CFG1,          0x43},
//	{CC1200_WOR_CFG0,		   0x24},
//	{CC1200_WOR_EVENT0_MSB,    0x17},
//	{CC1200_WOR_EVENT0_LSB,    0xD8},
//	{CC1200_PKT_CFG2,          0x00},
//	{CC1200_PKT_CFG0,          0x20},
//	{CC1200_RFEND_CFG1,		   0x0F},
//	{CC1200_PA_CFG1,           0x5F},
//	{CC1200_PKT_LEN,           0xFF},
//	{CC1200_FREQOFF_CFG,       0x23},
//	{CC1200_MDMCFG2,           0x00},
//	{CC1200_FREQ2,             0x56},
//	{CC1200_FREQ1,             0xCC},
//	{CC1200_FREQ0,             0xCC},
//	{CC1200_IF_ADC1,           0xEE},
//	{CC1200_IF_ADC0,           0x10},
//	{CC1200_FS_DIG1,           0x04},
//	{CC1200_FS_DIG0,           0xA3},
//	{CC1200_FS_CAL1,           0x40},
//	{CC1200_FS_CAL0,           0x0E},
//	{CC1200_FS_DIVTWO,         0x03},
//	{CC1200_FS_DSM0,           0x33},
//	{CC1200_FS_DVC1,           0xF7},
//	{CC1200_FS_DVC0,           0x0F},
//	{CC1200_FS_PFD,            0x00},
//	{CC1200_FS_PRE,            0x6E},
//	{CC1200_FS_REG_DIV_CML,    0x1C},
//	{CC1200_FS_SPARE,          0xAC},
//	{CC1200_FS_VCO0,           0xB5},
//	{CC1200_IFAMP,             0x0D},
//	{CC1200_XOSC5,             0x0E},
//	{CC1200_XOSC1,             0x03},
//};

//const registerSetting_t preferredSettings[]=
//{
//  {CC1200_IOCFG2,            0x06},
//  {CC1200_SYNC_CFG1,         0xA8},
//  {CC1200_DEVIATION_M,       0x47},
//  {CC1200_MODCFG_DEV_E,      0x2F},
//  {CC1200_DCFILT_CFG,        0x1E},
//  {CC1200_PREAMBLE_CFG0,     0x8A},
//  {CC1200_IQIC,              0x00},
//  {CC1200_CHAN_BW,           0x01},
//  {CC1200_MDMCFG1,           0x42},
//  {CC1200_MDMCFG0,           0x05},
//  {CC1200_SYMBOL_RATE2,      0xC9},
//  {CC1200_SYMBOL_RATE1,      0x99},
//  {CC1200_SYMBOL_RATE0,      0x99},
//  {CC1200_AGC_REF,           0x2F},
//  {CC1200_AGC_CS_THR,        0xF8},
//  {CC1200_AGC_CFG2,          0x60},
//  {CC1200_AGC_CFG1,          0x12},
//  {CC1200_AGC_CFG0,          0x84},
//  {CC1200_FIFO_CFG,          0x00},
//  {CC1200_FS_CFG,            0x14},
//  {CC1200_WOR_CFG1,          0x43},
//  {CC1200_WOR_CFG0,		     0x24},
//  {CC1200_WOR_EVENT0_MSB,    0x17},
//  {CC1200_WOR_EVENT0_LSB,    0xD8},
//  {CC1200_PKT_CFG2,          0x00},
//  {CC1200_PKT_CFG0,          0x20},
//  {CC1200_PKT_LEN,           0xFF},
//  {CC1200_FREQOFF_CFG,       0x23},
//  {CC1200_MDMCFG2,           0x00},
//  {CC1200_FREQ2,             0x57},
//  {CC1200_FREQ1,             0x4C},
//  {CC1200_FREQ0,             0xCC},
//  {CC1200_IF_ADC1,           0xEE},
//  {CC1200_IF_ADC0,           0x10},
//  {CC1200_FS_DIG1,           0x04},
//  {CC1200_FS_DIG0,           0xA3},
//  {CC1200_FS_CAL1,           0x40},
//  {CC1200_FS_CAL0,           0x0E},
//  {CC1200_FS_DIVTWO,         0x03},
//  {CC1200_FS_DSM0,           0x33},
//  {CC1200_FS_DVC1,           0xF7},
//  {CC1200_FS_DVC0,           0x0F},
//  {CC1200_FS_PFD,            0x00},
//  {CC1200_FS_PRE,            0x6E},
//  {CC1200_FS_REG_DIV_CML,    0x1C},
//  {CC1200_FS_SPARE,          0xAC},
//  {CC1200_FS_VCO0,           0xB5},
//  {CC1200_IFAMP,             0x0D},
//  {CC1200_XOSC5,             0x0E},
//  {CC1200_XOSC1,             0x03},
//};

/**
 *  \brief Sets up the CS pins and pulls high
 *
 *  \param [in] xcvr Pointer to a CC1200 transceiver
 *  \param [in] port The STM32 port used for the chip select pin of the xcvr
 *  \param [in] pin The port pin used as the chip select for the xcvr
 *  \return Returns 1, but needs to return success or failure
 *
 *  \details This is necessary if you are using multiple SPI devices as you want to insure only one devices CS is pulled low when communicating
 */
int8_t CC1200_ready_pin(cc1200_t* xcvr, GPIO_TypeDef* port, uint16_t pin){
	// Assign the designated chip select GPIO port and pin to the transceiver
	xcvr->CS_PORT = port;
	xcvr->CS_PIN = pin;

	 // Set the CS pin high
	HAL_GPIO_WritePin(xcvr->CS_PORT, xcvr->CS_PIN, GPIO_PIN_SET);

	return 1;
}

/**
 *  \brief Resets the CC1200 and configures it with the provided register settings
 *
 *  \param [in] xcvr Pointer to a CC1200 transceiver
 *  \param [in] hspi Pointer so the STM32 SPI handler being used from communication with the xcvr
 *  \param [in] spi_buf SPI buffer where MISO data from the CC1200 is stored
 *  \param [in] setting The SmartRF settings to load in the CC1200 registers
 *  \return The return value pf CC1200_verify_config()
 *
 *  \details CC1200_read_pin() must be used for the xcvr before using this function
 */
int8_t CC1200_Init(cc1200_t* xcvr, SPI_HandleTypeDef* hspi, char* spi_buf, const registerSetting_t* setting){
	// Assign the spi handler and buffer to the transceiver
	xcvr->hspi = hspi;
	xcvr->miso_data = (uint8_t*)spi_buf;
	
	// Initialize the circular buffers for transmission and reception
	//circ_buf_init(&(xcvr->rx_buf));
	//circ_buf_init(&(xcvr->tx_buf));

	// Reset the CC1200
	CC1200_command_strobe(xcvr, CC1200_COMMAND_SRES);
	
	// Configure the CC1200
	CC1200_configure(xcvr, setting);
	
	// Verify configuration
	return CC1200_verify_configure(xcvr, setting);
}

/**
 *  \brief Configure the CC1200 with the given settings
 *
 *  \param [in] xcvr Pointer to a CC1200 transceiver
 *  \param [in] setting The SmartRF settings to load in the CC1200 registers
 *  \return Return void, but should return success or failure
 *
 *  \details This is called from within CC1200_Init and should not be called directly
 */
void CC1200_configure(cc1200_t* xcvr, const registerSetting_t* setting){
	uint16_t addr;
	uint8_t configIndex = 0;
	
	// Iterate through every configuration address on the CC1200
	for(addr = 0x0000; addr < 0x3000; addr++){
		// Remember that the iterations are being done as if the CC1200 address space were
		// 2-byte addressable, but it is actually 1-byte addressable. The extended address space is
		// accessed with the 1-byte address 0x2F, followed by the 1-byte address of the extended
		// address space. The following if-statement looks for when the current address is 0x2F
		// and then sets the current address as 0x2F00 in order to begin iteration through
		// the extended address space
		if((addr < 0x2F00) && ((addr >> 8) == 0x002F))
			addr = 0x2F00;
		
		// If at the next desired address to configure, then configure it
		if(addr == setting[configIndex].addr){
			CC1200_write_register(xcvr, addr, setting[configIndex].val);
			configIndex++;
		}
	}
}

/**
 *  \brief Read cc1200 registers and print out preferred settings
 *
 *  \param [in] xcvr Pointer to a CC1200 transceiver
 *  \param [in] setting The SmartRF settings that were loaded in the CC1200 registers
 *  \return Return 1 or -1, but should return success or failure
 *
 *  \details This is called by CC1200_Init() after CC1200_configure() and should not be called directly
 */
int8_t CC1200_verify_configure(cc1200_t* xcvr, const registerSetting_t* setting){
	uint16_t addr;
	uint8_t configIndex = 0;
	
	// Iterate through every configuration address on the CC1200
	for(addr = 0x0000; addr < 0x3000; addr++){
		if((addr < 0x2F00) && ((addr >> 8) == 0x002F))
			addr = 0x2F00;
		
		// If at the next configured address, verify that it's current setting is correct
		if(addr == setting[configIndex].addr){
			CC1200_read_register(xcvr, addr);
			// If the register value is not correct then return failure 
			if(*(xcvr->miso_data) != setting[configIndex].val)
				return -1;
			configIndex++;
		}
	}

	return 1;
}

/**
 *  \brief Trigger a command strobe
 *
 *  \param [in] xcvr Pointer to a CC1200 transceiver
 *  \param [in] command A CC1200 command strobe
 *  \return Return void, but should return success or failure
 *
 *  \details The command strobe defines are listed in cc1200_reg.h under the COMMAND STROBE secion. Please
 *  refer to the command strobe table of the CC120x user manual
 */
void CC1200_command_strobe(cc1200_t* xcvr, uint8_t command){
	// Pull CS low
	HAL_GPIO_WritePin(xcvr->CS_PORT, xcvr->CS_PIN, GPIO_PIN_RESET);
	
	// Transmit the command strobe
	HAL_SPI_TransmitReceive(xcvr->hspi, &command, xcvr->miso_data, 1, 100);
	
	// Pull CS high
	HAL_GPIO_WritePin(xcvr->CS_PORT, xcvr->CS_PIN, GPIO_PIN_SET);
}

/**
 *  \brief Write a value to a specified register
 *
 *  \param [in] xcvr Pointer to a CC1200 transceiver
 *  \param [in] addr The register address to write to
 *  \param [in] val The value to write to the register
 *  \return Return void, but should return success or failure
 *
 *  \details More details
 */
void CC1200_write_register(cc1200_t* xcvr, uint16_t addr, uint8_t val){
	uint8_t isExtendedAddr = (addr > 0x002F);
	// The register is writeable if it is not one of the reserved address spaces specified in
	// the CC120x user manual
	uint8_t isWriteable = (addr > CC1200_RESERVED_LOWER_1) && (addr < CC1200_RESERVED_UPPER_1);
	isWriteable = isWriteable && ((addr > CC1200_RESERVED_LOWER_2) && (addr < CC1200_RESERVED_UPPER_2));
	isWriteable = !isWriteable;
	uint16_t data_to_send;
	uint8_t data;
	
	// Get the LSB of addr which contains the actual address to be written to
	addr = (addr&0x00FF);
	
	// Return an error if the address can not be written.
	if(isExtendedAddr && !isWriteable)
		return;
	
	// Pull CS low
	HAL_GPIO_WritePin(xcvr->CS_PORT, xcvr->CS_PIN, GPIO_PIN_RESET);
	
	// Send sequence of data depending on whether the accessed address space is extended or not
	if(isExtendedAddr){
		// For the extended address send (R/WR bit | burst bit | extended address 0x2F | address to
		// be accessed).
		data_to_send = CC1200_SPI_SINGLE_WRITE | CC1200_SPI_BURST_OFF| CC1200_EXT_ADDR | addr;
		
		// Send the first byte
		data = (uint8_t)(data_to_send >> 8);
		HAL_SPI_TransmitReceive(xcvr->hspi, &data, xcvr->miso_data, 1, 100);
		
		// Send the second byte
		data = (uint8_t)data_to_send;
		HAL_SPI_TransmitReceive(xcvr->hspi, &data, xcvr->miso_data, 1, 100);
	}else{
		// For the normal register space send (R/WR bit | burst bit | address to access)
		// For normal register access, all these bits are contained within the first byte
		data_to_send = CC1200_SPI_SINGLE_WRITE | CC1200_SPI_BURST_OFF | (addr << 8);
		
		// Send the first byte
		data = (uint8_t)(data_to_send >> 8);
		HAL_SPI_TransmitReceive(xcvr->hspi, &data, xcvr->miso_data, 1, 100);
	}
	
	// Send the data byte to be written to the address
	HAL_SPI_TransmitReceive(xcvr->hspi, &val, xcvr->miso_data, 1, 100);
	
	// Pull CS high
	HAL_GPIO_WritePin(xcvr->CS_PORT, xcvr->CS_PIN, GPIO_PIN_SET);
}

/**
 *  \brief Read the current value of a specified register
 *
 *  \param [in] xcvr Pointer to a CC1200 transceiver
 *  \param [in] addr The register address to read from
 *  \return Return void, but should return success or failure
 *
 *  \details More details
 */
void CC1200_read_register(cc1200_t* xcvr, uint16_t addr){
	uint8_t isExtendedAddr = (addr > 0x002F);
	// The register is writeable if it is not one of the reserved address spaces specified in
	// the CC120x user manual
	uint8_t isReadable = (addr > CC1200_RESERVED_LOWER_1) && (addr < CC1200_RESERVED_UPPER_1);
	isReadable = isReadable && ((addr > CC1200_RESERVED_LOWER_2) && (addr < CC1200_RESERVED_UPPER_2));
	isReadable = !isReadable;
	uint16_t data_to_send;
	uint8_t data;
	
	// Get the LSB of addr which contains the actual address to be read from
	addr = (addr&0x00FF);
	
	// Return an error if the address can not be written.
	if(isExtendedAddr && !isReadable)
		return;

	// Pull CS low
	HAL_GPIO_WritePin(xcvr->CS_PORT, xcvr->CS_PIN, GPIO_PIN_RESET);
	
	// Send sequence of data depending on whether the accessed address space is extended or not
	if(isExtendedAddr){
		// For the extended address send (R/WR bit | burst bit | extended address 0x2F | address to
		// be accessed).
		data_to_send = CC1200_SPI_SINGLE_READ | CC1200_SPI_BURST_OFF| CC1200_EXT_ADDR | addr;
		
		// Send the first byte
		data = (uint8_t)(data_to_send >> 8);
		HAL_SPI_TransmitReceive(xcvr->hspi, &data, xcvr->miso_data, 1, 100);
		// Store the chip ready and status bits
	//	xcvr->status.chipRDY = *xcvr->miso_data >> 7;
		//xcvr->status.state = (*xcvr->miso_data >> 4) & (0x07);
		
		// Send the second byte
		data = (uint8_t)data_to_send;
		HAL_SPI_TransmitReceive(xcvr->hspi, &data, xcvr->miso_data, 1, 100);
	}else{
		// For the normal register space send (R/WR bit | burst bit | address to access)
		// For normal register access, all these bits are contained within the first byte
		data_to_send = CC1200_SPI_SINGLE_READ | CC1200_SPI_BURST_OFF | (addr << 8);
		
		// Send the first byte
		data = (uint8_t)(data_to_send >> 8);
		HAL_SPI_TransmitReceive(xcvr->hspi, &data, xcvr->miso_data, 1, 100);
	}
	
	// Send anything just to read the address
	data = (uint8_t)data_to_send;
	HAL_SPI_TransmitReceive(xcvr->hspi, &data, xcvr->miso_data, 1, 100);
	
	// Pull CS high
	HAL_GPIO_WritePin(xcvr->CS_PORT, xcvr->CS_PIN, GPIO_PIN_SET);
}

/**
 *  \brief Transmit packets via Standard FIFO Access
 *  
 *  \param [in] xcvr Pointer to a CC1200 transceiver
 *  \param [in] packet The packet to load into the TX FIFO
 *  \return Return 1, but should return success or failure
 *  
 *  \details Currently this function only handles standard fifo burst transmission
 *  with variable packet length. In the future, this function should use a separate function that formats
 *  the packet according to the current settings.
 */
int8_t CC1200_transmit(cc1200_t* xcvr, uint8_t* packet){
	// For transmitting, need to first load the FIFO registers while in idle, then strobe the STX register
	// Must wait long enough for the data to actaully finish transmitting
	uint8_t packet_len = strlen((char*)packet); // Max packet length of 127
	uint8_t STANDARD_FIFO = 0x7F;
	int i;
	
	// Transceiver should be in IDLE before trying to transmit
	CC1200_read_register(xcvr, CC1200_MARCSTATE);
	if(*xcvr->miso_data != 0x41)
		return -1;
	CC1200_command_strobe(xcvr, CC1200_COMMAND_SFSTXON);

	// Pull CS low
	HAL_GPIO_WritePin(xcvr->CS_PORT, xcvr->CS_PIN, GPIO_PIN_RESET);

	// Write xcvr->bit, burst xcvr->bit, and FIFO address
	HAL_SPI_TransmitReceive(xcvr->hspi, &STANDARD_FIFO, xcvr->miso_data, 1, 100);

	// Send packet length as the CC1200 is set for variable packet length
	HAL_SPI_TransmitReceive(xcvr->hspi, &packet_len, xcvr->miso_data, 1, 100);
	
	// Send the payload
	for(i = 0; i < packet_len; i++){
		HAL_SPI_TransmitReceive(xcvr->hspi, &packet[i], xcvr->miso_data, 1, 100);
	}
	
	// Pull CS high and send the transmit command strobe
	HAL_GPIO_WritePin(xcvr->CS_PORT, xcvr->CS_PIN, GPIO_PIN_SET);
	CC1200_command_strobe(xcvr, CC1200_COMMAND_STX);

	return 1;
}

/**
 *  \brief Receive packets via Standard FIFO Access
 *  
 *  \param [in] xcvr Pointer to a CC1200 transceiver
 *  \param [in] rx_buf A pointer to the buffer to hold the data read from the RX FIFO
 *  \return Return 1, but should return success or failure
 *  
 *  \details More details
 */
int8_t CC1200_receive(cc1200_t* xcvr, char* rx_buf){
	// Experimentation:
	// MOSI: 0xEF D7 00		MISO: 0x0F 00 10
	// Reads extended address 0xD7 RX_NUMBYTES which returns 0x10 = 16 (13 byte payload + 1 byte length + 2 byte CRC)
	// MOSI: 0xFF 00		MISO: 0x0F 0D
	// Reads the packet length byte which returns ox0D = 13
	// MOSI: 0xFF ... 		MISO: 0x0F ...
	// Then reads the 13 payload bytes and the 2 CRC bytes
	// MOSI: 0x34			MISO: 0x0F
	// Sends SRX command strobe possibly because chip returns to IDLE once the full packet has been read
	uint8_t rx_numbytes;
	uint8_t payload_len;
	uint8_t STANDARD_FIFO = 0xFF;
	uint8_t i;
	uint8_t NOP = CC1200_COMMAND_NOP;
	CC1200_read_register(xcvr, CC1200_NUM_RXBYTES);
	rx_numbytes = *(xcvr->miso_data);

	if(rx_numbytes == 0){
		CC1200_read_register(xcvr, CC1200_MARCSTATE);
		if(*xcvr->miso_data == 0x41){
			CC1200_command_strobe(xcvr,CC1200_COMMAND_SRX);
		}
		else if(*xcvr->miso_data == 0x11){
			CC1200_command_strobe(xcvr,CC1200_COMMAND_SFRX);
			CC1200_command_strobe(xcvr,CC1200_COMMAND_SRX);
		}
		rx_buf[0] = '\0';
		return -1;
	}

	/*
	if(rx_numbytes > 0){
		CC1200_read_register(xcvr, CC1200_MARCSTATE);
		if(*xcvr->miso_data != 0x41){
			if(*xcvr->miso_data == 0x11){
				CC1200_command_strobe(xcvr,CC1200_COMMAND_SFRX);
				CC1200_command_strobe(xcvr,CC1200_COMMAND_SRX);
			}

			return -1;
		}
	}
	*/
	
	// Pull CS low
	HAL_GPIO_WritePin(xcvr->CS_PORT, xcvr->CS_PIN, GPIO_PIN_RESET);
	
	// Send command to access the RX FIFO
	HAL_SPI_TransmitReceive(xcvr->hspi, &STANDARD_FIFO, xcvr->miso_data, 1, 100);
	
	// Read the first byte containing the payload length
	HAL_SPI_TransmitReceive(xcvr->hspi, &NOP, xcvr->miso_data, 1, 100);
	payload_len = *(xcvr->miso_data);

	//circ_buf_add(&(xcvr->rx_buf), payload_len);

	// Read the rest of the data in the RX FIFO, including the two CRC bits on the end
	for(i = 0; i < payload_len+2; i++){
		HAL_SPI_TransmitReceive(xcvr->hspi, &NOP, xcvr->miso_data, 1, 100);
		if(i < payload_len){
			rx_buf[i] = *(xcvr->miso_data);
			//circ_buf_add(&(xcvr->rx_buf),*(xcvr->miso_data));
		}

		// Check the PKT_CFG1 register for CRC_CFG and APPEND_STATUS
		// This determines whether or not two extra status bytes are appended to
		// the end of the packet.
		// The first byte is the RSSI values and the second byte is CRC_OK and LQI
		// Bit 7 of the second byte indicated CRC_OK.
	}
	
	// Add the newline character and null characters so the rx_buf can be written to
	// a serial terminal for user debugging. These should be removed for end application
	rx_buf[payload_len] = '\0';

	// Pull CS high and send the receive command strobe
	HAL_GPIO_WritePin(xcvr->CS_PORT, xcvr->CS_PIN, GPIO_PIN_SET);
	//CC1200_command_strobe(xcvr, CC1200_COMMAND_SFSTXON);
	CC1200_command_strobe(xcvr, CC1200_COMMAND_SRX);

	return 1;
}
