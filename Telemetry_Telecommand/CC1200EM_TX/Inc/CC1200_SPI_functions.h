#ifndef CC1200_SPI_H
#define CC1200_SPI_H

/* Included files */
#include "main.h"
#include "CC1200_reg.h"
#include "smartrf_CC1200.h"

/* Preprocessor Macros */

/* Private variables */

/* Function Prototypes */
void trxRfSpiInterfaceInit(SPI_HandleTypeDef *hspi, uint8_t prescalerValue);
uint8_t trx8BitRegAccess(SPI_HandleTypeDef *hspi, uint8_t accessType, uint8_t addrByte, uint8_t *pData, int len);
uint8_t ReadWriteExtendedReg (SPI_HandleTypeDef *hspi, uint8_t accessType, uint16_t address, uint8_t value);
uint8_t ReadWriteCommandReg (SPI_HandleTypeDef *hspi, uint8_t address);
void CC1200_INIT(SPI_HandleTypeDef *hspi);




#endif // CC1200_SPI_H
