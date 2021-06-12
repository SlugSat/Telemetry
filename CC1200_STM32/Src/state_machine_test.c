/**
* @file state_machine_test.c
* @author Cameron Moreno
* @brief Test file for implementing a basic state machine
* @date May 5 2021
*
*/
#include "main.h"
#include <stdio.h>
//#include "CircularBuffer.h"
#include "cc1200_reg.h"
#include "cc1200.h"

#ifdef STATE_MACHINE_TEST

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

#define CS_GPIO_PORT GPIOA
#define CS_PIN GPIO_PIN_5

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);

/*********************************
 * PACKET PROTOCOL NOTES HERE
**********************************/
/**
 *  The following packet information is taken from the Year 3
 *  Spring Quarter Report located on the SlugSat Google Drive at:
 *  
 *  "SlugSat/Year3/Quaterly Deliverables/Spring 2019/Spring Quarter Report/
 *  Chapter Final PDFs/F. Command & Control.pdf"
 *  
 *  COMMAND			|	HEADER BYTE	| Additional data (bytes)	|	Description
 *  ----------------------------------------------------------------------------
 *  Kill			|	1111 0000	|			N/A				|	Tell the satellite to completely
 *  				|				|							|	shut down (per FCC regulations) 
 *  ---------------------------------------------------------------------------------------------------------
 *  Log Science		|	0001 0X00	|	2 optional bytes of time|	Command the satellite to log a
 *  				|				|	information				|	science event at the current time
 *  				|				|							|	at a time in the future, specified by X
 *  ---------------------------------------------------------------------------------------------------------
 *  Request Status	|	0010 XXXX	|			N/A				|	Request status information about various
 *  				|				|							|	subsystems on the satellite, specified by
 *  				|				|							|	the XXXX bits
 *  ---------------------------------------------------------------------------------------------------------
 *  Request Science	|	0100 0X00	|	6 bytes of time and/or	|	Request stored Science Payload data points.
 * 	Data			|				|	chunk size				|	Can either be specified from a start time to
 *					|				|							|	an end time, or with a start time and chunk size,
 *					|				|							|	specifying the number of data points. The mode
 *					|				|							|	is specified by the X bit
 *	--------------------------------------------------------------------------------------------------------- 				|				|							|
 *  Request			|	0101 0000	|			N/A				|	Request the current location of the satellite,
 *  Location		|				|							|	based on SGP4 and Keplerian orbital elements
 */


/******************************
 *  FSM FUNCTIONS GO HERE
*******************************/
// An enumeration of the states of the FSM.
typedef enum states{
	SLEEP_STATE,	///< MCUs low power sleep state
	IDLE_STATE,	///< The MCU is awake and waiting for the MCU_WAKEUP signal
	MCU_WAKEUP_STATE,	///< The MCU_WAKEUP received and now reading MARC_STATUS_OUT
	PARSE_PACKET_STATE,	///< A good packet received and now reading the packet from the RX FIFO
	ERROR_HANDLER_STATE,	///< A bad packet was received and now need to figure out what is wrong
	UNKNOWN_STATE	///< Added as a default state during testing 
} state;

// An enumeration of events that cause state transitions.
typedef enum events{
	NILEVENT,	///< Nothing happened
	MCU_WAKEUP_EVENT,	///< MCU_WAKEUP signal was asserted
	GOOD_PACKET_EVENT,	///< A good packet was recieved
	BAD_PACKET_EVENT	///< A bad packet was detected
} event;

// Declarations of action functions to be called on state transitions
// go here.
state mcu_wakeup_action(cc1200_t* xcvr);	///< Read MARC_STATUS_OUT
state parse_packet_action(cc1200_t* xcvr);	///< Read packet from RX FIFO and parse payload
state error_handler_action(cc1200_t* xcvr);	///< Fix a detected error
state unknown_action(cc1200_t* xcvr);	///< Print something to tester know something went wrong
state test_requested_action(cc1200_t* xcvr); ///< A basic user defined action

// Declarations of actions defined by the user packet protocol
void test_print(UART_HandleTypeDef* huart); ///< Will simple print to the serial terminal for testing

typedef struct{
	state nextState; ///< Next state to transition to
	state (*action)(cc1200_t* xcvr); ///< The action to be done on transition to the next state
} stateElement;

/**
 *  TRANSITION TABLE
 *  
 *  stateMatrix[3][3] = {
 *  	{STATE, ACTION}, {STATE, ACTION}, {STATE, ACTION},
 *  	{STATE, ACTION}, {STATE, ACTION}, {STATE, ACTION},
 *  	{STATE, ACTION}, {STATE, ACTION}, {STATE, ACTION}
 *  }
 *  
 *  In the array above, the row index indentifies the current state and
 *  the column index identifies the event that occurred.
 *  
 *  Within the array, the STATE is the next state to go to and the action
 *  should be a pointer to the function that should be called on transition.
 */
 stateElement stateMatrix[6][4] = {
	{{SLEEP_STATE, unknown_action}, {MCU_WAKEUP_STATE, mcu_wakeup_action}, {MCU_WAKEUP_STATE, mcu_wakeup_action}, {MCU_WAKEUP_STATE, mcu_wakeup_action}},
	{{IDLE_STATE, unknown_action}, {MCU_WAKEUP_STATE, mcu_wakeup_action}, {MCU_WAKEUP_STATE, mcu_wakeup_action}, {MCU_WAKEUP_STATE, mcu_wakeup_action}},
	{{MCU_WAKEUP_STATE, unknown_action}, {MCU_WAKEUP_STATE, unknown_action} ,{PARSE_PACKET_STATE, parse_packet_action}, {PARSE_PACKET_STATE, parse_packet_action}},
	{{PARSE_PACKET_STATE, unknown_action}, {PARSE_PACKET_STATE, unknown_action}, {PARSE_PACKET_STATE, unknown_action}, {PARSE_PACKET_STATE, unknown_action}},
	{{ERROR_HANDLER_STATE, unknown_action}, {ERROR_HANDLER_STATE, unknown_action}, {ERROR_HANDLER_STATE, unknown_action}, {ERROR_HANDLER_STATE, unknown_action}},
	{{UNKNOWN_STATE, unknown_action}, {UNKNOWN_STATE, unknown_action}, {UNKNOWN_STATE, unknown_action}, {UNKNOWN_STATE, unknown_action}}
 };
 
 

int main(void){
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_SPI2_Init();
	MX_USART2_UART_Init();

	//char message[50];
	char spi_buf[127];
	//uint8_t error;
	//uint16_t message_len;
	cc1200_t xcvr1;

	CC1200_ready_pin(&xcvr1, CS_GPIO_PORT, CS_PIN);
	CC1200_Init(&xcvr1, &hspi2, spi_buf, preferredSettings);

	state currentState = IDLE_STATE;
	event lastEvent = NILEVENT;
	stateElement currentElement = stateMatrix[currentState][lastEvent];

	while (1){
		stateMatrix[currentState][lastEvent].action(&xcvr1);
		currentElement = stateMatrix[currentState][lastEvent];
	}
}

/***********************************************
 *  DEFINITIONS OF ACTION FUNCTIONS GO HERE
************************************************/
/**
 *  \brief Function called when MCU_WAKEUP signal asserted
 *  
 *  \param [in] xcvr Pointer to a CC1200 transceiver
 *  \return Returns the next state
 *  
 *  \details This function determines the reason for the MCU_WAKEUP_SIGNAL.
 */
state mcu_wakeup_action(cc1200_t* xcvr){
	state nextState;
	CC1200_read_register(xcvr, CC1200_MARC_STATUS1);
	uint8_t marc_status_out = *xcvr->miso_data;

	if(marc_status_out == 0x80){
		nextState = PARSE_PACKET_STATE;
	}else if(marc_status_out == 0x20){
		nextState = ERROR_HANDLER_STATE;
	}else{
		nextState = UNKNOWN_STATE;
	}
	return nextState;
}

/**
 *  \brief Retrieve the packet from the RX FIFO and parse
 *  
 *  \param [in] xcvr Pointer to a CC1200 transceiver
 *  \return Returns the next state
 *  
 *  \details This function should cause a transition to some defined user function 
 */
state parse_packet_action(cc1200_t* xcvr){
	state nextState;
	char rx_buf[128];
	
	CC1200_receive(xcvr, rx_buf);
	
	nextState = IDLE_STATE;

	return nextState;
}

/**
 *  \brief Basic error handling function for CC1200
 *  
 *  \param [in] xcvr Pointer to a CC1200 transceiver
 *  \return Returns the next state
 *  
 *  \details More details
 */
state error_handler_action(cc1200_t* xcvr){
	state nextState;
	uint8_t error_code = 0x00;
	
	switch(error_code){
		// No failure
		case 0x00:
			nextState = UNKNOWN_STATE;
			break;
		// RX timeout occurred.
		case 0x01:
			nextState = UNKNOWN_STATE;
			break;
		// TX termination base on CS and PQT.
		case 0x02:
			nextState = UNKNOWN_STATE;
			break;
		// eWOR sync lost.
		case 0x03:
			nextState = UNKNOWN_STATE;
			break;
		// Packet discarded due to maximum length filtering
		case 0x04:
			nextState = UNKNOWN_STATE;
			break;
		// Packet discarded due to address filtering
		case 0x05:
			nextState = UNKNOWN_STATE;
			break;
		// Packet discarded due to CRC filtering
		case 0x06:
			nextState = UNKNOWN_STATE;
			break;
		// TX FIFO overflow error occurred
		case 0x07:
			CC1200_command_strobe(xcvr,CC1200_COMMAND_SFTX);
			nextState = IDLE_STATE;
			break;
		// TX FIFO underflow error occurred
		case 0x08:
			CC1200_command_strobe(xcvr,CC1200_COMMAND_SFTX);
			nextState = IDLE_STATE;
			break;
		// RX FIFO overflow error occurred
		case 0x09:
			CC1200_command_strobe(xcvr,CC1200_COMMAND_SFRX);
			nextState = IDLE_STATE;
			break;
		// RX FIFO underflow error occurred
		case 0x0A:
			CC1200_command_strobe(xcvr,CC1200_COMMAND_SFRX);
			nextState = IDLE_STATE;
			break;
		// TX on CCA failed
		case 0x0B:
			nextState = UNKNOWN_STATE;
			break;
		// TX finished successfully
		case 0x40:
			nextState = UNKNOWN_STATE;
			break;
		// RX finished successfully
		case 0x80:
			nextState = UNKNOWN_STATE;
			break;
		default:
			nextState = UNKNOWN_STATE;
			break;
	};

	nextState = ERROR_HANDLER_STATE;
	
	return nextState;
}

state unknown_action(cc1200_t* xcvr){
	state nextState;
	
	nextState = UNKNOWN_STATE;

	return nextState;
}

state test_requested_action(cc1200_t* xcvr){
	state nextState;

	nextState = UNKNOWN_STATE;

	return nextState;
}

void test_print(UART_HandleTypeDef* huart){
	uint8_t message[] = "Code made it here.\n";
	uint8_t message_len = sizeof(message)/sizeof(message[0]);

	HAL_UART_Transmit(&huart2, message, message_len, 100);
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
