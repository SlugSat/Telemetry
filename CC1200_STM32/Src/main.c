/**
* @file main.c
* @author Cameron Moreno
* @brief Main system file
* @date April 3 2021
*
*/
#include "main.h"
#include <stdio.h>
//#include "CircularBuffer.h"
#include "cc1200_reg.h"
#include "cc1200.h"

#ifdef MAIN_TEST

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

#define CS_GPIO_PORT GPIOA
#define CS_PIN GPIO_PIN_5

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);

//enum states {MCU_SLEEP, MCU_IDLE, MCU_WAKEUP, MCU_ERROR} state;

int main(void){
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_SPI2_Init();
	MX_USART2_UART_Init();

	char message[50];
	//char packet[] = "Hello World!\n";
	char spi_buf[127];
	//char rx_buf[127];
	uint16_t message_len;
	cc1200_t xcvr1;

	CC1200_ready_pin(&xcvr1, CS_GPIO_PORT, CS_PIN);
	CC1200_Init(&xcvr1, &hspi2, spi_buf, preferredSettings);

	//state = MCU_SLEEP;

	//CC1200_command_strobe(&xcvr1, CC1200_COMMAND_SCAL);
	//CC1200_command_strobe(&xcvr1, CC1200_COMMAND_SRX);
	CC1200_command_strobe(&xcvr1, CC1200_COMMAND_SWOR);
	while (1){
		if(mcu_wakeup == 1){
			mcu_wakeup = 0;

			CC1200_read_register(&xcvr1, CC1200_MARC_STATUS1);
			message_len = sprintf(message, "MS1 = 0x%x\n",*(xcvr1.miso_data));
			HAL_UART_Transmit(&huart2, (uint8_t*)message, message_len, 100);

			if(*(xcvr1.miso_data) == 0x80){
				CC1200_receive(&xcvr1, spi_buf);
				message_len = sprintf(message, "%s\n",spi_buf);
				HAL_UART_Transmit(&huart2, (uint8_t*)message, message_len, 100);
			}

			//CC1200_command_strobe(&xcvr1, CC1200_COMMAND_SWOR);
		}
		//CC1200_receive(&xcvr1, spi_buf);
		//message_len = sprintf(message, "%s",spi_buf);
		//HAL_UART_Transmit(&huart2, (uint8_t*)message, message_len, 100);

		//CC1200_read_register(&xcvr1, CC1200_MARC_STATUS1);
		//message_len = sprintf(message, "MS1 = 0x%x\n",*(xcvr1.miso_data));
		//HAL_UART_Transmit(&huart2, (uint8_t*)message, message_len, 100);
//		switch(state){
//			case MCU_SLEEP:
//				// Here the MCU does nothing and wait for the MCU_WAKEUP signal from the CC1200
//				break;
//			case MCU_IDLE:
//				// This is the state the MCU is in when awaiting instructions from the ground station
//				CC1200_read_register(CC1200_MARC_STATUS1);
//				// IF RX received then begin reading data from the buffer
//				// Decode the packet and carry out the instructions from the ground station
//
//				// Code should be added here for a timer. If a certain amount of time passes without any signal
//				// for the MCU to do something, then the MCU should return to sleep mode.
//				break;
//			case MCU_WAKEUP:
//				// This state is reached once the MCU gets the MCU_WAKEUP signal. The MCU should
//				// read MARC_STATUS_OUT to determine the cause of the signal. If a good pack was
//				// received, the MCU should take appropriate action, if it was signaled because of
//				// a bad picket, it means 16 consecutive bad packets were received. The MCU should
//				// then enter the MCU_ERROR state to handle this.
//				CC1200_read_register(&xcvr1, CC1200_MARC_STATUS1);
//				if(*xcvr1.miso_data == 0x00) // Change 0x00 to whatever it is for a good packet
//					state = MCU_IDLE;
//				else if(*xcvr1.miso_data == 0x00) // Change 0x00 to whatever it is for a bad packet
//					state = MCU_ERROR;
//				break;
//			case MCU_ERROR:
//				// This state should handle the error which was the cause of the MCU_WAKEUP
//				break;
//			default:
//				// Do some stuff
//		}

		//CC1200_read_register(&xcvr1, CC1200_IF_ADC0);
		//message_len = sprintf(message, "MS1 = 0x%x\n",*(xcvr1.miso_data));
		//HAL_UART_Transmit(&huart2, (uint8_t*)message, message_len, 100);
	}
}

/**
*	@brief System Clock Configuration
*	@retval None
*/
void SystemClock_Config(void){
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	*/
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK){
		Error_Handler();
	}
}

/**
*	@brief SPI2 Initialization Function
*	@param None
*	@retval None
*/
static void MX_SPI2_Init(void){
	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK){
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */
}

/**
*	@brief USART2 Initialization Function
*	@param None
*	@retval None
*/
static void MX_USART2_UART_Init(void){
	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK){
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */
}

/**
*	@brief GPIO Initialization Function
*	@param None
*	@retval None
*/
static void MX_GPIO_Init(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : MCU_WAKEUP_Pin */
	GPIO_InitStruct.Pin = MCU_WAKEUP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MCU_WAKEUP_GPIO_Port, &GPIO_InitStruct);

	 /* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void){
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1){
	}
	/* USER CODE END Error_Handler_Debug */
}
#endif
