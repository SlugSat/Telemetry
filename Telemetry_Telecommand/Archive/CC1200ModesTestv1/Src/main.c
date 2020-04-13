/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "string.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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

void trxRfSpiInterfaceInit(uint8_t prescalerValue);
uint8_t trx8BitRegAccess(uint8_t accessType, uint8_t addrByte, uint8_t *pData, int len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//#define SPI_BEGIN()       SPI_CS_N_PIN = 0                // Pull CSn low to start communication
#define SPI_TX(x)         HAL_SPI_Transmit(&hspi1, x, 1, 10)                    // Load x into SPI buffer  
#define SPI_WAIT_READY()  while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)   // Wait for receive buffer to fill
#define SPI_RX(x)         HAL_SPI_Receive(&hspi1, x, 1, 10)                        // SPI buffer to read
//#define SPI_END()         SPI_CS_N_PIN = 1                // Pull CSn high to end communication

#define RADIO_BURST_ACCESS   0x40
#define RADIO_SINGLE_ACCESS  0x00
#define RADIO_READ_ACCESS    0x80
#define RADIO_WRITE_ACCESS   0x00	


//#define SPI_TX SPI_WriteByte
//#define SPI_RX SPI_ReadByte

#define RADIO_IDLE 0x36
#define RADIO_NOP 0x3D
#define RADIO_RX 0x34
#define RADIO_TX 0x35

uint8_t pDataRead;

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


/* Start CC1200 SPI Test Harness
	 * 
	 * Wiring:
	 * Reset -- D7 (PA8)
	 * CSN   -- D8 (PA9)
	 * SCLK  -- D3 (PB3)
	 * MOSI  -- D4 (PB5)
	 * MISO  -- D5 (PB4)
	 */
	 
	 
//	//Set CS high
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

	//Set reset high, low, high to begin
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_Delay(10);


//  //Simple tutorial mode switch
//	//Read Data
//	//1. Set CS low
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
//	//2. Transmit Command Strobe Address
//	HAL_SPI_Transmit(&hspi1, spiIn, 1, 10);
//	//3. Transmit a NOP for the one operation delay
//	HAL_SPI_Receive(&hspi1, &spiIn[1], 1, 10);
//	//4. Set CS high
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);


	volatile uint8_t stByte;
  uint8_t wrData = 0x55;
	uint8_t correct = 0;
	uint8_t testOpener [35] = "\r\n\r\nCC1200 Mode Switching Test...\r\n";
	uint8_t idleOpener [14] = "Idle Mode...  ";
	uint8_t txOpener [14] = "TX Mode...    ";
	uint8_t rxOpener [14] = "RX Mode...    ";
	uint8_t statusCheck [28];
	snprintf(statusCheck, sizeof(statusCheck), "Status byte is 0x.2%u, [%u/3]\r\n", stByte, correct);
	
//    uint8_t addr;
  
	HAL_UART_Transmit(&huart2, testOpener, sizeof(testOpener), 1);


	// IDLE test
	HAL_UART_Transmit(&huart2, idleOpener, sizeof(idleOpener), 1);
	stByte = trx8BitRegAccess(RADIO_WRITE_ACCESS | RADIO_SINGLE_ACCESS, RADIO_IDLE, &wrData, sizeof (wrData));
	HAL_Delay(10);
	stByte = trx8BitRegAccess(RADIO_WRITE_ACCESS | RADIO_SINGLE_ACCESS, RADIO_NOP, &wrData, sizeof (wrData));
	HAL_Delay(10);
	if((stByte>>4) == 0){
		correct++;
	}
	
	// TX test
	snprintf(statusCheck, sizeof(statusCheck), "Status byte is 0x%.2X, [%u/3]\r\n", stByte, correct);
	HAL_UART_Transmit(&huart2, statusCheck, sizeof(statusCheck), 1);

	HAL_UART_Transmit(&huart2, txOpener, sizeof(txOpener), 1);
	stByte = trx8BitRegAccess(RADIO_WRITE_ACCESS | RADIO_SINGLE_ACCESS, RADIO_TX, &wrData, sizeof (wrData));
	HAL_Delay(10);
	stByte = trx8BitRegAccess(RADIO_WRITE_ACCESS | RADIO_SINGLE_ACCESS, RADIO_NOP, &wrData, sizeof (wrData));
	HAL_Delay(10);
	
	if((stByte>>4) == 2){
		correct++;
	}
		
	// RX test
	snprintf(statusCheck, sizeof(statusCheck), "Status byte is 0x%.2X, [%u/3]\r\n", stByte, correct);
	HAL_UART_Transmit(&huart2, statusCheck, sizeof(statusCheck), 1);

	HAL_UART_Transmit(&huart2, rxOpener, sizeof(rxOpener), 1);
	stByte = trx8BitRegAccess(RADIO_WRITE_ACCESS | RADIO_SINGLE_ACCESS, RADIO_RX, &wrData, sizeof (wrData));
	HAL_Delay(10);
	stByte = trx8BitRegAccess(RADIO_WRITE_ACCESS | RADIO_SINGLE_ACCESS, RADIO_NOP, &wrData, sizeof (wrData));
	HAL_Delay(10);
	
	// TX test
	snprintf(statusCheck, sizeof(statusCheck), "Status byte is 0x%.2X, [%u/3]\r\n", stByte, correct);
	HAL_UART_Transmit(&huart2, statusCheck, sizeof(statusCheck), 1);

	HAL_UART_Transmit(&huart2, txOpener, sizeof(txOpener), 1);
	stByte = trx8BitRegAccess(RADIO_WRITE_ACCESS | RADIO_SINGLE_ACCESS, RADIO_TX, &wrData, sizeof (wrData));
	HAL_Delay(10);
	stByte = trx8BitRegAccess(RADIO_WRITE_ACCESS | RADIO_SINGLE_ACCESS, RADIO_NOP, &wrData, sizeof (wrData));
	HAL_Delay(10);
	
	// RX test
	snprintf(statusCheck, sizeof(statusCheck), "Status byte is 0x%.2X, [%u/3]\r\n", stByte, correct);
	HAL_UART_Transmit(&huart2, statusCheck, sizeof(statusCheck), 1);

	HAL_UART_Transmit(&huart2, rxOpener, sizeof(rxOpener), 1);
	stByte = trx8BitRegAccess(RADIO_WRITE_ACCESS | RADIO_SINGLE_ACCESS, RADIO_RX, &wrData, sizeof (wrData));
	HAL_Delay(10);
	stByte = trx8BitRegAccess(RADIO_WRITE_ACCESS | RADIO_SINGLE_ACCESS, RADIO_NOP, &wrData, sizeof (wrData));
	HAL_Delay(10);
	
	if((stByte>>4) == 2){
		correct++;
	}
	
	if((stByte>>4) == 1){
		correct++;
	}
	
	
	 
	// TEST RESULTS
	
	snprintf(statusCheck, sizeof(statusCheck), "Status byte is 0x%.2X, [%u/3]\r\n", stByte, correct);
	HAL_UART_Transmit(&huart2, statusCheck, sizeof(statusCheck), 1);

	if (correct == 6){
		uint8_t finishMsg [48] = "[3/3] Success! Mode switching is operational\r\n";
		HAL_UART_Transmit(&huart2, finishMsg, sizeof(finishMsg), 1);
	} else {
		uint8_t finishMsg [76];
		snprintf(finishMsg, sizeof(finishMsg), "[%u/3] Mode switching failure... check your wiring and pin initialization\r\n", correct);
		HAL_UART_Transmit(&huart2, finishMsg, sizeof(finishMsg), 1);
	}


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

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  HAL_GPIO_WritePin(GPIOA, LED_Pin|RESET_Pin|CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin RESET_Pin CSN_Pin */
  GPIO_InitStruct.Pin = LED_Pin|RESET_Pin|CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



static void trxReadWriteBurstSingle(uint8_t addr, uint8_t *pData, int len)
{
    int i;
    uint8_t dummy;
	pDataRead = *pData;
    /* Communicate len number of bytes: if RX - the procedure sends 0x00 to push bytes from slave*/
    if (addr & RADIO_READ_ACCESS) {
        if (addr & RADIO_BURST_ACCESS) {
            for (i = 0; i < len; i++) {
				
								HAL_SPI_Transmit(&hspi1,0x0, 1, 10);
                while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
                HAL_SPI_Receive(&hspi1, pData, 1, 10);
                pData++;
				
            }
        } else {
						HAL_SPI_Transmit(&hspi1,0x0, 1, 10);
            while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
            HAL_SPI_Receive(&hspi1, pData, 1, 10);
        }
    } else {
        if (addr & RADIO_BURST_ACCESS) {
            /* Communicate len number of bytes: if TX - the procedure doesn't overwrite pData */
            for (i = 0; i < len; i++) {
								HAL_SPI_Transmit(&hspi1,pData, 1, 10);
                while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
								HAL_SPI_Receive(&hspi1, &dummy, 1, 10);
                pData++;
            }
        } else {
						HAL_SPI_Transmit(&hspi1,pData, 1, 10);
            while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
						HAL_SPI_Receive(&hspi1, &dummy, 1, 10);
        }
    }
}

uint8_t trx8BitRegAccess(uint8_t accessType, uint8_t addrByte, uint8_t *pData, int len)
{
	uint8_t addr;
	uint8_t readValue;
	addr = accessType | addrByte;
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	
	HAL_SPI_Transmit(&hspi1,&addr, 1, 10);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){};
	HAL_SPI_Receive(&hspi1, &readValue, 1, 10);
  trxReadWriteBurstSingle(accessType | addrByte, pData, len);
		
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		
  return (readValue);
}



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
