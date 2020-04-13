// This file includes all of the functions required to communicate via SPI to the CC1200

/* Included files */
#include "CC1200_SPI_Functions.h"

/* Preprocessor Macros */

/* Private variables */


/* Function definitions */
uint8_t ReadWriteExtendedReg (SPI_HandleTypeDef *hspi, uint8_t accessType, uint16_t address, uint8_t value)
{
	// Chip select low
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	
	uint8_t addrHI = address>>8;
	uint8_t addrLO = (uint8_t) address;
	uint8_t readValue = 0;
	
	
	if(accessType) { 	// Read from address
		
		if (addrHI)
		{
			addrHI |= CC1200_READ_BIT;
			HAL_SPI_Transmit(hspi,&addrHI, 1, 10);	// Access address	
			while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY){};
			HAL_SPI_Transmit(hspi,&addrLO, 1, 10);
		}
		else
		{
			addrLO |= CC1200_READ_BIT;
			HAL_SPI_Transmit(hspi,&addrLO, 1, 10);
			while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY){};
		}
		HAL_SPI_Transmit(hspi,0x0, 1, 10); // Transmit 0 for read
		HAL_SPI_Receive(hspi, &readValue, 1, 10); // Read value
			
			
	} else { // Write to address
			
		addrHI |= CC1200_WRITE_BIT;
		
		if (addrHI)
		{
			HAL_SPI_Transmit(hspi,&addrHI, 1, 10);	// Access address	
			while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY){};
		}
		HAL_SPI_Transmit(hspi,&addrLO, 1, 10);
		while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY){};

		HAL_SPI_Transmit(hspi, &value, 1, 10);		// Write value
	}
	
	// Chip select high	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	
	return readValue;
}

uint8_t ReadWriteCommandReg (SPI_HandleTypeDef *hspi, uint8_t address)
{
	// Chip select low
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	
  uint8_t readValue  = 0;
	
	HAL_SPI_Transmit(hspi,&address, 1, 10);
	while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY){};
	HAL_SPI_Receive(hspi, &readValue, 1, 10);
		
	// Chip select high	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	
	return readValue;
}

void CC1200_INIT(SPI_HandleTypeDef *hspi)
{
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_IOCFG2, SMARTRF_SETTING_IOCFG2);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_DEVIATION_M, SMARTRF_SETTING_DEVIATION_M);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_MODCFG_DEV_E, SMARTRF_SETTING_MODCFG_DEV_E);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_DCFILT_CFG, SMARTRF_SETTING_DCFILT_CFG);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_PREAMBLE_CFG0, SMARTRF_SETTING_PREAMBLE_CFG0);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_IQIC, SMARTRF_SETTING_IQIC);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_CHAN_BW, SMARTRF_SETTING_CHAN_BW);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_MDMCFG1, SMARTRF_SETTING_MDMCFG1);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_MDMCFG0, SMARTRF_SETTING_MDMCFG0);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_SYMBOL_RATE2, SMARTRF_SETTING_SYMBOL_RATE2);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_SYMBOL_RATE1, SMARTRF_SETTING_SYMBOL_RATE1);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_SYMBOL_RATE0, SMARTRF_SETTING_SYMBOL_RATE0);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_AGC_REF, SMARTRF_SETTING_AGC_REF);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_AGC_CS_THR, SMARTRF_SETTING_AGC_CS_THR);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_AGC_CFG1, SMARTRF_SETTING_AGC_CFG1);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_AGC_CFG0, SMARTRF_SETTING_AGC_CFG0);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_FIFO_CFG, SMARTRF_SETTING_FIFO_CFG);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_FS_CFG, SMARTRF_SETTING_FS_CFG);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_PKT_CFG2, SMARTRF_SETTING_PKT_CFG2);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_PKT_CFG0, SMARTRF_SETTING_PKT_CFG0);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_PKT_LEN, SMARTRF_SETTING_PKT_LEN);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_IF_MIX_CFG, SMARTRF_SETTING_IF_MIX_CFG);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_FREQOFF_CFG, SMARTRF_SETTING_FREQOFF_CFG);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_MDMCFG2, SMARTRF_SETTING_MDMCFG2);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_FREQ2, SMARTRF_SETTING_FREQ2);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_FREQ1, SMARTRF_SETTING_FREQ1);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_FREQ0, SMARTRF_SETTING_FREQ0);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_IF_ADC1, SMARTRF_SETTING_IF_ADC1);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_IF_ADC0, SMARTRF_SETTING_IF_ADC0);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_FS_DIG1, SMARTRF_SETTING_FS_DIG1);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_FS_DIG0, SMARTRF_SETTING_FS_DIG0);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_FS_CAL1, SMARTRF_SETTING_FS_CAL1);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_FS_CAL0, SMARTRF_SETTING_FS_CAL0);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_FS_DIVTWO, SMARTRF_SETTING_FS_DIVTWO);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_FS_DSM0, SMARTRF_SETTING_FS_DSM0);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_FS_DVC0, SMARTRF_SETTING_FS_DVC0);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_FS_PFD, SMARTRF_SETTING_FS_PFD);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_FS_PRE, SMARTRF_SETTING_FS_PRE);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_FS_REG_DIV_CML, SMARTRF_SETTING_FS_REG_DIV_CML);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_FS_SPARE, SMARTRF_SETTING_FS_SPARE);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_FS_VCO0, SMARTRF_SETTING_FS_VCO0);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_XOSC5, SMARTRF_SETTING_XOSC5);
	ReadWriteExtendedReg(hspi, CC1200_WRITE_BIT, CC1200_XOSC1, SMARTRF_SETTING_XOSC1);
}



////////////////////////////// TI Reference Code, Not currently being used/////////////////////
static void trxReadWriteBurstSingle(SPI_HandleTypeDef *hspi, uint8_t addr, uint8_t *pData, int len)
{
    int i;
    uint8_t dummy;
    /* Communicate len number of bytes: if RX - the procedure sends 0x00 to push bytes from slave*/
    if (addr & CC1200_READ_BIT) {
        if (addr & CC1200_BURST_BIT) {
            for (i = 0; i < len; i++) {
								HAL_SPI_Transmit(hspi,0x0, 1, 10);
                while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY);
                HAL_SPI_Receive(hspi, pData, 1, 10);
                pData++;
				
            }
        } else {
						HAL_SPI_Transmit(hspi,0x0, 1, 10);
            while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY);
            HAL_SPI_Receive(hspi, pData, 1, 10);
        }
    } else {
        if (addr & CC1200_BURST_BIT) {
            /* Communicate len number of bytes: if TX - the procedure doesn't overwrite pData */
            for (i = 0; i < len; i++) {
								HAL_SPI_Transmit(hspi,pData, 1, 10);
                while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY);
								HAL_SPI_Receive(hspi, &dummy, 1, 10);
                pData++;
            }
        } else {
						HAL_SPI_Transmit(hspi,pData, 1, 10);
            while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY);
						HAL_SPI_Receive(hspi, &dummy, 1, 10);
        }
    }
}

uint8_t trx8BitRegAccess(SPI_HandleTypeDef *hspi, uint8_t accessType, uint8_t addrByte, uint8_t *pData, int len)
{
	uint8_t addr;
	uint8_t readValue  = 0;
	addr = accessType | addrByte;
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	
	HAL_SPI_Transmit(hspi,&addr, 1, 10);
	while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY){};
	HAL_SPI_Receive(hspi, &readValue, 1, 10);
	trxReadWriteBurstSingle(hspi, accessType | addrByte, pData, len);
		
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

	return (readValue);
}
