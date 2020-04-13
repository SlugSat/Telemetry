/* 
 * File:   uCConfig.h
 * Author: admatthe
 *
 * Created on May 21, 2018, 11:52 AM
 */

#ifndef UCCONFIG_H
#define	UCCONFIG_H

/*******************************************************************************
 * PUBLIC #INCLUDES                                                           *
 ******************************************************************************/
#include <GenericTypeDefs.h>
#include <stdint.h>
#include <xc.h>

/*******************************************************************************
 * PUBLIC #DEFINES                                                             *
 ******************************************************************************/

//suppresses various warnings that we don't need to worry about for CMPE13
#ifndef _SUPPRESS_PLIB_WARNING
#define _SUPPRESS_PLIB_WARNING
#endif

#ifndef _DISABLE_OPENADC10_CONFIGPORT_WARNING
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING
#endif

// Specify the default UART to use. UART1 is selected the Uno32, as it's the one on the USB port.
#define UART_USED UART1

// Set the baud rate for use with the UART. This is chosen as it's the default baud rate for Tera
// Term.
#define UART_BAUD_RATE 115200

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                     *
 ******************************************************************************/

/**
 * Function: BOARD_Init(void)
 * @param None
 * @return None
 * @brief Set the clocks up for the board, initializes the serial port, and turns
 * on the A/D subsystem for battery monitoring
 * @author Max Dunne, 2013.09.15  */
void BOARD_Init();


/**
 * Function: BOARD_End(void)
 * @param None
 * @return None
 * @brief shuts down all peripherals except for serial and A/D. Turns all pins
 * into input
 * @author Max Dunne, 2013.09.20  */
void BOARD_End();

/**
 * Function: BOARD_GetPBClock(void)
 * @param None
 * @return PB_CLOCK - speed the peripheral clock is running in hertz
 * @brief returns the speed of the peripheral clock.  Nominally at 20Mhz
 * @author Max Dunne, 2013.09.01  */
unsigned int BOARD_GetPBClock();


/**
 * Function: BOARD_GetSysClock(void)
 * @param None
 * @return SYS_CLOCK - speed the main clock is running at
 */
unsigned int BOARD_GetSysClock();

#endif	/* UCCONFIG_H */

