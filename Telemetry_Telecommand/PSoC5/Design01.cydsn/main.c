#include "project.h"

#define RADIO_BURST_ACCESS   0x40
#define RADIO_SINGLE_ACCESS  0x00
#define RADIO_READ_ACCESS    0x80
#define RADIO_WRITE_ACCESS   0x00

#define SPI_WAIT_READY() while (!(SPI_ReadRxStatus() & SPI_STS_RX_FIFO_NOT_EMPTY))
#define SPI_TX SPI_WriteByte
#define SPI_RX SPI_ReadByte

#define RADIO_IDLE 0x36
#define RADIO_NOP 0x3D
#define RADIO_RX 0x34
#define RADIO_TX 0x35

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

int trx8BitRegAccess(int accessType, int addrByte, int *pData, int len)
{
    SPI_WriteTxData(accessType | addrByte);
    
    while (!(SPI_ReadRxStatus() & SPI_STS_RX_FIFO_NOT_EMPTY));
    int readValue = SPI_ReadByte();
    trxReadWriteBurstSingle(accessType | addrByte, pData, len);

    return (readValue);
}

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    SPI_Start();
    VDAC8_1_Start();
    Opamp_1_Start();
    
    // set reset high
    RESET_Write(1);
    RESET_Write(0);
    RESET_Write(1);

    volatile uint8 stByte;
    int wrData;
    int addr;
    for(;;)
    {
        stByte = trx8BitRegAccess(RADIO_WRITE_ACCESS | RADIO_SINGLE_ACCESS, RADIO_IDLE, &wrData, sizeof (wrData));
        CyDelay(1000);
        stByte = trx8BitRegAccess(RADIO_WRITE_ACCESS | RADIO_SINGLE_ACCESS, RADIO_NOP, &wrData, sizeof (wrData));
        CyDelay(1000);
        stByte = trx8BitRegAccess(RADIO_WRITE_ACCESS | RADIO_SINGLE_ACCESS, RADIO_TX, &wrData, sizeof (wrData));
        CyDelay(1000);
        stByte = trx8BitRegAccess(RADIO_WRITE_ACCESS | RADIO_SINGLE_ACCESS, RADIO_NOP, &wrData, sizeof (wrData));
        CyDelay(1000);
        stByte = trx8BitRegAccess(RADIO_WRITE_ACCESS | RADIO_SINGLE_ACCESS, RADIO_RX, &wrData, sizeof (wrData));
        CyDelay(1000);
        stByte = trx8BitRegAccess(RADIO_WRITE_ACCESS | RADIO_SINGLE_ACCESS, RADIO_NOP, &wrData, sizeof (wrData));
        CyDelay(1000);
    }
}

/* [] END OF FILE */
