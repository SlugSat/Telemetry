/* 
 * File:   hal_spi.h
 * Author: Andrew
 *
 * Created on June 4, 2018, 8:09 PM
 */

#ifndef HAL_SPI_H
#define	HAL_SPI_H

/*
 * CONSTANTS
 */
#define SPI_CS_N_PIN_DIR  TRISDbits.TRISD4
#define SPI_CS_N_PIN      LATDbits.LATD4

#define RADIO_BURST_ACCESS   0x40
#define RADIO_SINGLE_ACCESS  0x00
#define RADIO_READ_ACCESS    0x80
#define RADIO_WRITE_ACCESS   0x00

/*
 * MACROS
 */
#define SPI_BEGIN()       SPI_CS_N_PIN = 0                // Pull CSn low to start communication
#define SPI_TX(x)         SPI2BUF = x                     // Load x into SPI buffer  
#define SPI_WAIT_READY()  while (!SPI2STATbits.SPIRBF);   // Wait for receive buffer to fill
#define SPI_RX()          SPI2BUF                         // SPI buffer to read
#define SPI_END()         SPI_CS_N_PIN = 1                // Pull CSn high to end communication

/*
 * PROTOTYPES
 */
void trxRfSpiInterfaceInit(int prescalerValue);
int trx8BitRegAccess(int accessType, int addrByte, int *pData, int len);

#endif	/* HAL_SPI_H */

