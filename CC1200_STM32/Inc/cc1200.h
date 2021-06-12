/**
* @file cc1200.h
* @author Cameron Moreno
* @date April 3 2021
* @brief Header file for cc1200.c
*/
#ifndef CC1200_H
#define CC1200_H

#include <stdio.h>
#include "stm32l1xx_hal.h"

#include "cc1200_reg.h"
#include "CircularBuffer.h"
  
/**
 *  Holds the register addresses and desired values for configuring the cc1200
 */
typedef struct cc1200_registerSetting{
	uint16_t addr;	///< The register address to access
	uint8_t val;	///< The value to be loaded at the register address
} registerSetting_t;

/**
 * Structure which holds the status bits returned by the CC1200
 */
typedef struct cc1200_status{
	uint8_t chipRDY : 1;	///< chipRDY bit signaling that the crystal is stable
	uint8_t state : 3;	///< Current state of the CC1200
} status;

/**
 *  Main cc1200 struct for communicating with a specific xcvr
 */
typedef struct cc1200{
	status status;	///< Structure for the status byte returned by the CC1200
	uint8_t* miso_data;	///< MISO data returned by the CC1200
	circ_buf_t rx_buf; ///< Circular buffer for the receiver
	circ_buf_t tx_buf; ///< Circular buffer for the transmitter
	GPIO_TypeDef* CS_PORT;	///< GPIO port use for the chip select
	uint16_t CS_PIN;	///< GPIO pin used for the chip select
	SPI_HandleTypeDef* hspi;	///< SPI handler
} cc1200_t;

extern const registerSetting_t preferredSettings[];

/**
 *  \brief Sets up the CS pins and pulls high
 */
int8_t CC1200_ready_pin(cc1200_t* xcvr, GPIO_TypeDef* port, uint16_t pin);

/**
 *  \brief Resets the CC1200 and configures it with the provided register settings
 */
int8_t CC1200_Init(cc1200_t* xcvr, SPI_HandleTypeDef* hspi, char* spi_buf, const registerSetting_t* setting);

/**
 *  \brief Configure the CC1200 with the given settings
 */
void CC1200_configure(cc1200_t* xcvr, const registerSetting_t* setting);

/**
 *  \brief Read cc1200 registers and print out preferred settings
 */
int8_t CC1200_verify_configure(cc1200_t* xcvr, const registerSetting_t* setting);

/**
 *  \brief Trigger a command strobe
 */
void CC1200_command_strobe(cc1200_t* xcvr, uint8_t command);

/**
 *  \brief Write a value to a specified register
 */
void CC1200_write_register(cc1200_t* xcvr, uint16_t addr, uint8_t val);

/**
 *  \brief Read the current value of a specified register
 */
void CC1200_read_register(cc1200_t* xcvr, uint16_t addr);

/**
 * @brief Transmit packets via Standard FIFO Access
 */
int8_t CC1200_transmit(cc1200_t* xcvr, uint8_t* packet);

/**
 * @brief Receive packets via Standard FIFO Access
 */
int8_t CC1200_receive(cc1200_t* xcvr, char* rx_buf);

#endif /*CC1200_H*/
