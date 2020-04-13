/*
 * File:   hal_spi.c
 * Author: Andrew
 *
 * Created on June 4, 2018, 8:21 PM
 */


#include "xc.h"
#include "hal_spi.h"

/*
 * LOCAL FUNCTIONS
 */
static void trxReadWriteBurstSingle(int addr, int *pData, int len);

/*
 *FUNCTIONS
 */

/******************************************************************************
 * @fn          trxRfSpiInterfaceInit
 *
 * @brief       Function to initialize TRX SPI. CC1101/CC112x is currently
 *              supported. The supported prescalerValue must be set so that
 *              SMCLK/prescalerValue does not violate radio SPI constraints.
 *
 * input parameters
 *
 * @param       prescalerValue - SMCLK/prescalerValue gives SCLK frequency
 *
 * output parameters
 *
 * @return      void
 */
void trxRfSpiInterfaceInit(int prescalerValue)
{
    // CC1200 Slave Select pin set high
    SPI_CS_N_PIN_DIR = 0;
    SPI_CS_N_PIN = 1;

    /* Configuration
     * -  SPI ON
     * -  8-bit mode
     * -  Master Mode
     * -  Non-Framed mode
     * -  CKP = 0 (Inactive state is low)
     * -  CKE = 1 (Sample data on rising edge)
     * -  SMP = 0 (rx data in phase with tx data)
     */
    IEC1CLR = 0x03800000; // Disable all interrupts
    SPI2CON = 0; // Stop and reset SPI2
    int rdData = SPI2BUF; // Clear the receive buffer
    SPI2BRG = prescalerValue; // SCLK = Fpb / (2*(prescalerValue + 1))
    SPI2STATCLR = 0x40; // Clear the overflow  
    SPI2CON = 0x8120; // Configuration 
}

/*******************************************************************************
 * @fn          trx8BitRegAccess
 *
 * @brief       This function performs a read or write from/to a 8bit register
 *              address space. The function handles burst and single read/write
 *              as specfied in addrByte. Function assumes that chip is ready.
 *
 * input parameters
 *
 * @param       accessType - Specifies if this is a read or write and if it's
 *                           a single or burst access. Bitmask made up of
 *                           RADIO_BURST_ACCESS/RADIO_SINGLE_ACCESS/
 *                           RADIO_WRITE_ACCESS/RADIO_READ_ACCESS.
 * @param       addrByte - address byte of register.
 * @param       pData    - data array
 * @param       len      - Length of array to be read(TX)/written(RX)
 *
 * output parameters
 *
 * @return      chip status
 */
int trx8BitRegAccess(int accessType, int addrByte, int *pData, int len)
{
    SPI_BEGIN();
    SPI_TX(accessType | addrByte);
    SPI_WAIT_READY();
    int readValue = SPI_RX();
    trxReadWriteBurstSingle(accessType | addrByte, pData, len);
    SPI_END();

    return (readValue);
}

/*******************************************************************************
 * @fn          trxReadWriteBurstSingle
 *
 * @brief       When the address byte is sent to the SPI slave, the next byte
 *              communicated is the data to be written or read. The address
 *              byte that holds information about read/write -and single/
 *              burst-access is provided to this function.
 *
 *              Depending on these two bits this function will write len bytes to
 *              the radio in burst mode or read len bytes from the radio in burst
 *              mode if the burst bit is set. If the burst bit is not set, only
 *              one data byte is communicated.
 *
 *              NOTE: This function is used in the following way:
 *
 *              TRXEM_SPI_BEGIN();
 *              while(TRXEM_PORT_IN & TRXEM_SPI_MISO_PIN);
 *              ...[Depending on type of register access]
 *              trxReadWriteBurstSingle(uint8 addr,uint8 *pData,uint16 len);
 *              TRXEM_SPI_END();
 *
 * input parameters
 *
 * @param       none
 *
 * output parameters
 *
 * @return      void
 */
static void trxReadWriteBurstSingle(int addr, int *pData, int len)
{
    int i;
    int dummy;
    /* Communicate len number of bytes: if RX - the procedure sends 0x00 to push bytes from slave*/
    if (addr & RADIO_READ_ACCESS) {
        if (addr & RADIO_BURST_ACCESS) {
            for (i = 0; i < len; i++) {
                SPI_TX(0); /* Possible to combining read and write as one access type */
                SPI_WAIT_READY();
                *pData = SPI_RX(); /* Store pData from last pData RX */
                pData++;
            }
        } else {
            SPI_TX(0);
            SPI_WAIT_READY();
            *pData = SPI_RX();
        }
    } else {
        if (addr & RADIO_BURST_ACCESS) {
            /* Communicate len number of bytes: if TX - the procedure doesn't overwrite pData */
            for (i = 0; i < len; i++) {
                SPI_TX(*pData);
                SPI_WAIT_READY();
                dummy = SPI_RX();
                pData++;
            }
        } else {
            SPI_TX(*pData);
            SPI_WAIT_READY();
            dummy = SPI_RX();
        }
    }
}