/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  *
  *  CC1200 SPI Test Harness
	* 
	* Wiring:
	* Reset -- D7 (PA8)
	* CSN   -- D8 (PA9)
	* SCLK  -- D3 (PB3)
	* MOSI  -- D4 (PB5)
	* MISO  -- D5 (PB4)
	*	
	* Use this test harness to check the operation of the CC1200 module
	*
	* The following tests and settings can be configured:
	*
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "string.h"
#include "CC1200_SPI_Functions.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Pins
#define CSN GPIO_PIN_9
#define RESET GPIO_PIN_8

// Tests
#define READ_WRITE_TEST
//#define COMMAND_STROBE_TEST

#define TX_TEST (1)
#define RX_TEST (0)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
///////////////////////////////////////////////////////////////////////
	// CSN and RESET setup
	///////////////////////////////////////////////////////////////////////
	 
	//Set CS high
	HAL_GPIO_WritePin(GPIOA, CSN, GPIO_PIN_SET);

	//Set reset high, low, high to begin
	HAL_GPIO_WritePin(GPIOA, RESET, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA, RESET, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA, RESET, GPIO_PIN_SET);
	HAL_Delay(10);
	
	///////////////////////////////////////////////////////////////////////
	// Start the test
	///////////////////////////////////////////////////////////////////////
	
	uint16_t address = 0x2F02;
	uint8_t value = 0xa;
	uint8_t readValue = 0;
	char Msg1[100] = {0};
	char Msg2[100] = {0};

	///////////////////////////////////////////////////////////////////////
	// Test: send a packet continuously and receive with the dev board
	///////////////////////////////////////////////////////////////////////
  // Configure the chip
	CC1200_INIT(&hspi1);
	
	
	#if TX_TEST
	readValue = 0;
	address = CC1200_TXFIFO;
	uint32_t sendAmt = 6*50;
	uint8_t txValue = 0;
	
	uint16_t count = 0;
	uint16_t countErr = 0;

	for(int i = 0; i < sendAmt; i++)
	{
		// Read num_tx_bytes
		txValue = ReadWriteExtendedReg (&hspi1, CC1200_READ_BIT, CC1200_NUM_TXBYTES, value);  
		if (txValue < 128) 
		{	
			ReadWriteExtendedReg(&hspi1, CC1200_WRITE_BIT, address, i);
			HAL_Delay(1);
		}
		else
		{
			HAL_Delay(1);
			i--;
			//continue;
		}

		readValue = ReadWriteCommandReg(&hspi1, CC1200_SNOP); // Seems to need HAL_Delay and a NOP to produce the correct status bit
		HAL_Delay(8);
		
		if (readValue == 0x7f)
		{
			countErr++;
			ReadWriteCommandReg(&hspi1, CC1200_SFTX); // Flush if FIFO error
			continue;
		}
		else if ((readValue & 0xf0) != 0x20)
		{
			ReadWriteCommandReg(&hspi1, CC1200_STX);
			count++;
			HAL_Delay(1);
		} 
	}
	
	// Check for errors again to make sure everything transmits properly
	txValue = ReadWriteExtendedReg (&hspi1, CC1200_READ_BIT, CC1200_NUM_TXBYTES, value);  
	while (txValue != 0)
	{
		memcpy(Msg1, Msg2, 70);
		snprintf((char *)Msg1, sizeof(Msg1), "\r\nNumber of bytes in the transmit buffer: 0x%02x\r\n", txValue);
		HAL_UART_Transmit(&huart2, (uint8_t *) Msg1, sizeof(Msg1), 1);
		
		readValue = ReadWriteCommandReg(&hspi1, CC1200_SNOP); // Seems to need HAL_Delay and a NOP to produce the correct status bit
		HAL_Delay(8);
		
		if ((readValue & 0xf0) != 0x20)
		{
			ReadWriteCommandReg(&hspi1, CC1200_STX);
			count++;
			HAL_Delay(1);
		} 
		txValue = ReadWriteExtendedReg (&hspi1, CC1200_READ_BIT, CC1200_NUM_TXBYTES, value);
	}
	#endif
	
	
	// Test receiving
	#if RX_TEST
	
	uint8_t rxBuffer[70] = {0}; // Initialize a buffer to read the data
	ReadWriteCommandReg(&hspi1, CC1200_SFRX); // Flush RX FIFO
	ReadWriteCommandReg(&hspi1, CC1200_SRX);  // Enter receive mode
	
	uint8_t doneCount = 0;
	uint16_t numRXBytes = 0;
	address = CC1200_RXFIFO;
	
	while (doneCount < 1) // less than 1 because we expect only one packet
	{
		// Get the number of bytes in the RX FIFO
		numRXBytes = ReadWriteExtendedReg (&hspi1, CC1200_READ_BIT, CC1200_NUM_RXBYTES, value);  
		memcpy(Msg1, Msg2, 100);
		snprintf((char *)Msg1, 100, "\r\nNumber of bytes in the receive buffer: 0x%02x    State of CC1200: 0x%02x\r\n", numRXBytes, readValue);
		HAL_UART_Transmit(&huart2, (uint8_t *) Msg1, sizeof(Msg1), 1);
		
		// If there is a full packet, read it
		if (numRXBytes >= 19)
		{
			doneCount++;
			for (int i = 0; i < 19; i++)
			{
				rxBuffer[i * doneCount] = ReadWriteExtendedReg(&hspi1, CC1200_READ_BIT, address, 0);
			}
		}
		
		// Check the state of the chip in the event that an error occurred
		readValue = ReadWriteCommandReg(&hspi1, CC1200_SNOP); 
		HAL_Delay(8);
		
		if (readValue == 0x6f) // Flush if FIFO error
		{
			ReadWriteCommandReg(&hspi1, CC1200_SFRX); 
		}
		else if ((readValue & 0xf0) != 0x10) // Re-enter transmit mode if there was a FIFO error
		{
			ReadWriteCommandReg(&hspi1, CC1200_SRX);
		}
	}	
	
	// Print out each value read
	memcpy(Msg1, Msg2, 100);
	int j = 0;
	int i = 0;
	# if 0 // This is for decoding length 6 hex packets
	for (i = 0; i < 90; i += 15)
	{
		snprintf((char *) &Msg1[i], 15, "\r\nData: 0x%02x\r\n", rxBuffer[j++]);
	}
	HAL_UART_Transmit(&huart2, (uint8_t *) Msg1, 100*sizeof(uint8_t), 1);
	#else // This is for decoding length 19 text packets
	memcpy(Msg1, Msg2, 100);
	snprintf((char *) &Msg1[i], 10, "\r\nPacket:         ");
	for (i = 11; i < 90; i++)
	{
		snprintf((char *) &Msg1[i], 2, "%c", (char) rxBuffer[j++]);
	}
	snprintf((char *) &Msg1[++i], 2, "\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *) Msg1, 100*sizeof(uint8_t), 1);
	
	#endif
	#endif
	
	
	// End the test
	memcpy(Msg1, Msg2, 100);
	snprintf((char *)Msg1, sizeof(Msg1), "\n\nEND OF TRANSCEIVER TEST \n");
	HAL_UART_Transmit(&huart2, (uint8_t *) Msg1, sizeof(Msg1), 1);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA6 PA7 PA8 
                           PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
