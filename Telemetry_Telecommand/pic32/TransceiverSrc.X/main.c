/*
 * File:   main.c
 * Author: admatthe
 *
 * Created on May 7, 2018, 1:33 PM
 */

#include "uCConfig.h"
#include "hal_spi.h"
#include <stdio.h>
#include <plib.h>

int main(void)
{
    int addr;
    int wrData;
    int stByte;
    int rdData;
    int flag;

    BOARD_Init();

    // CC1200 Reset Pin set high
    TRISDbits.TRISD3 = 0;
    LATDbits.LATD3 = 1;

    trxRfSpiInterfaceInit(1);


    while (1) {
        flag = 0;

        // enter hex value to be written over SPI
        printf("Enter an address: 0x");
        scanf("%x", &addr);

        // if true write data to register else read data from register
        if ((addr < 0x30) && (addr >> 7 == 0)) {
            printf("Enter data to write: 0x");
            scanf("%x", &wrData);
            printf("address entered: 0x%04x\n", addr);
            printf("data entered: 0x%02x\n", wrData);
            flag = 1;

            stByte = trx8BitRegAccess(RADIO_WRITE_ACCESS | RADIO_SINGLE_ACCESS, addr, &wrData, sizeof (wrData));
        } else {
            printf("address entered: 0x%04x\n", addr);

            stByte = trx8BitRegAccess(RADIO_READ_ACCESS | RADIO_SINGLE_ACCESS, addr, &rdData, sizeof (rdData));
        }

        //        printf("address entered: 0x%04x\n", addr);
        //        
        //        
        //        
        //        // bring SS pin low
        //        SPI_BEGIN();
        //        
        //        // get status byte
        //        // send data on master channel to cc1200
        //        SPI_TX(addr);
        //        // wait for cc1200 data to arrive in master SPI rx buffer
        //        //while (!SpiChnDataRdy(2));
        //        SPI_WAIT_READY();
        //        // store SPI rx buffer data in rdData
        //        stByte = SPI_RX();
        //        
        //        // get data
        //        // send data on master channel to cc1200
        //        if (flag == 1) {
        //            SPI_TX(wrData);
        //        } else {
        //            SPI_TX(0);
        //        }
        //        // wait for cc1200 data to arrive in master SPI rx buffer
        //        //while (!SpiChnDataRdy(2));
        //        SPI_WAIT_READY();
        //        // store SPI rx buffer data in rdData
        //        rdData = SPI_RX();
        //        
        //        // bring SS pin high
        //        SPI_END();

        // print status byte received from cc1200
        printf("status byte received from cc1200: 0x%02x\n", stByte);
        // print data received from cc1200
        printf("data received from cc1200: 0x%02x\n", rdData);
    }

    while (1);
}
