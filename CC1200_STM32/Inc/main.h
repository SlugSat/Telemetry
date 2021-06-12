/**
* @file main.h
* @author Cameron Moreno
* @date April 3 2021
* @brief Header file for main.c
*
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"


void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13	///< Pin for the blue button on the Nucleo 64 board
#define B1_GPIO_Port GPIOC	///< Port for the blue button on the Nucleo 64 board

#define USART_TX_Pin GPIO_PIN_2 
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

#define MCU_WAKEUP_Pin GPIO_PIN_2	///< Pin for the MCU_WAKEUP signal from the CC1200 GPIO
#define MCU_WAKEUP_GPIO_Port GPIOB	///< Port for the MCU_WAKEUP signal from the CC1200 GPIO

/* PROGRAM TEST DEFINES *******************************************************/
/** @defgroup ProgramTestDefines
  *  @brief    Defines used for testing CC1200 functions and implementations
  *  in separate source files.
  * @{
  */
#define MAIN_TEST
//#define TX_TEST
//#define RX_TEST
//#define EWOR_TEST
//#define STATE_MACHINE_TEST
/**
  * @}
  */
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
