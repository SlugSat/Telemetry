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
#include "CC1200_reg.h"
#include "smartrf_CC1200.h"

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
//#define READ_WRITE_TEST
//#define COMMAND_STROBE_TEST
#define SPI_FUNCTION_TEST


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
uint8_t ReadWriteExtendedReg (uint8_t accessType, uint16_t address, uint8_t value);
uint8_t ReadWriteCommandReg (uint8_t address);
void CC1200_INIT(void);


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
	uint8_t readValue;
	char Msg1[70] = {0};
	char Msg2[70] = {0};
	
	
	snprintf((char *)Msg1, sizeof(Msg1), "\n\nSTARTING THE CC1200 TRANSCEIVER TEST\n\n");
	HAL_UART_Transmit(&huart2, (uint8_t *) Msg1, sizeof(Msg1), 1);
	memcpy(Msg1, Msg2, 100);
	
	
	///////////////////////////////////////////////////////////////////////
  // Test reading and writing to a configuration register
	///////////////////////////////////////////////////////////////////////
	#ifdef SPI_FUNCTION_TEST
	
	uint8_t testString[] = {0x53, 0x6c, 0x75, 0x67, 0x53, 0x61, 0x74}; //"SlugSat" in hex	
	while(1){
			HAL_SPI_Transmit(&hspi1, testString, 7, 100);			
			HAL_Delay(100);
			HAL_GPIO_TogglePin(GPIOA, RESET);
	}
	#endif
	
	#ifdef READ_WRITE_TEST
	
	/*
	Read the value at address 0x2F02 (configuration register CC1200_TOC_CFG) then set 
	the value of this register to value 0xa.  Before the write this register should
	be reset to 0xb.
	*/
	
	// Read initial reg value
  readValue = ReadWriteExtendedReg (CC1200_READ_BIT, address, value); 
		
	snprintf((char *)Msg1, sizeof(Msg1), "\r\nTest: read from register 0x%x: 0x%x\r\n", address, readValue);
	HAL_UART_Transmit(&huart2, (uint8_t *) Msg1, sizeof(Msg1), 1);
	
	snprintf((char *)Msg1, sizeof(Msg1), "\r\nTest: read from register should be '0xb', check init function if not \r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *) Msg1, sizeof(Msg1), 1);
	
	// Write to register
  readValue = ReadWriteExtendedReg (CC1200_WRITE_BIT, address, value); 
	
	// Read new value
	readValue = ReadWriteExtendedReg (CC1200_READ_BIT, address, value);  
		
	snprintf((char *)Msg1, sizeof(Msg1), "\r\nTest: read from register 0x%x after writing to the register: 0x%x\r\n", address, readValue);
	HAL_UART_Transmit(&huart2, (uint8_t *) Msg1, sizeof(Msg1), 1);
	
	snprintf((char *)Msg1, sizeof(Msg1), "\r\nTest: read from register should be '0xa', check write function if not \r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *) Msg1, sizeof(Msg1), 1);
	#endif
	
	///////////////////////////////////////////////////////////////////////
	// Test command strobes
	///////////////////////////////////////////////////////////////////////
  
	#ifdef COMMAND_STROBE_TEST
	
	memcpy(Msg1, Msg2, 100);
	snprintf((char *)Msg1, sizeof(Msg1), "\r\nMode Test Expected Bytes: transmit 0x2F, idle 0x0F, receive 0x1F\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *) Msg1, sizeof(Msg1), 1);
	
	readValue = ReadWriteCommandReg(CC1200_SFTX); // Flush tx fifo
	HAL_Delay(10);
	readValue = ReadWriteCommandReg(CC1200_SFRX); // Flush RXfifo
	HAL_Delay(10);
	readValue = ReadWriteCommandReg(CC1200_SNOP); // Seems to need HAL_Delay and a NOP to produce the correct status bit
	//HAL_Delay(1000);
		
	readValue = 0;
	readValue = ReadWriteCommandReg(CC1200_SIDLE); // Idle
	HAL_Delay(10);
	readValue = ReadWriteCommandReg(CC1200_SNOP); // Seems to need HAL_Delay and a NOP to produce the correct status bit
	
	memcpy(Msg1, Msg2, 100);
	snprintf((char *)Msg1, sizeof(Msg1), "\r\nMode Test: switch to idle: 0x%02x\r\n", readValue);
	HAL_UART_Transmit(&huart2, (uint8_t *) Msg1, sizeof(Msg1), 1);
	
	readValue = 0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	readValue = ReadWriteCommandReg(CC1200_STX); // Transmit
	HAL_Delay(10);
	readValue = ReadWriteCommandReg(CC1200_SNOP); // Seems to need HAL_Delay and a NOP to produce the correct status bit

	memcpy(Msg1, Msg2, 100);
	snprintf((char *)Msg1, sizeof(Msg1), "\r\nMode Test: switch to transmit: 0x%02x\r\n", readValue);
	HAL_UART_Transmit(&huart2, (uint8_t *) Msg1, sizeof(Msg1), 1);
	
	#endif

	///////////////////////////////////////////////////////////////////////
	// Configure all registers
	///////////////////////////////////////////////////////////////////////
	//HAL_Delay(100);
	CC1200_INIT();
	//HAL_Delay(100);
	
	// Check a register 
	address = CC1200_AGC_CFG0;
  readValue = ReadWriteExtendedReg (CC1200_READ_BIT, address, value); 
	
	memcpy(Msg1, Msg2, 100);
	snprintf((char *)Msg1, sizeof(Msg1), "\r\nTest: read from register 0x%x: 0x%x\r\n", address, readValue);
	HAL_UART_Transmit(&huart2, (uint8_t *) Msg1, sizeof(Msg1), 1);
	
	// Check a register
	address = CC1200_AGC_CFG1;
	uint8_t randRead = ReadWriteExtendedReg (CC1200_READ_BIT, address, value); 
		
	memcpy(Msg1, Msg2, 100);
	snprintf((char *)Msg1, sizeof(Msg1), "\r\nTest: read from register 0x%x: 0x%x\r\n", address, randRead);
	HAL_UART_Transmit(&huart2, (uint8_t *) Msg1, sizeof(Msg1), 1);
	
	
	///////////////////////////////////////////////////////////////////////
	// Test: send a packet continuously and receive with the dev board
	///////////////////////////////////////////////////////////////////////

	readValue = 0;
	//address = 0x00;
	address = CC1200_TXFIFO;
	uint32_t sendAmt =114;
	
	uint16_t count = 0;
	uint16_t correct =0;
	uint16_t weird =0;
	uint16_t countErr = 0;
	//char Msg1[100] = {0};
	uint8_t txValuebefire = ReadWriteExtendedReg (CC1200_READ_BIT, CC1200_NUM_TXBYTES, value);  
		
	memcpy(Msg1, Msg2, 100);
	snprintf((char *)Msg1, sizeof(Msg1), "\r\n befire TX bytes:0x%02x\r\n", txValuebefire);
	HAL_UART_Transmit(&huart2, (uint8_t *) Msg1, sizeof(Msg1), 1);
	for(int i = 0; i < sendAmt; i++)
	{
		// Read num_tx_bytes
		uint8_t txValue = ReadWriteExtendedReg (CC1200_READ_BIT, CC1200_NUM_TXBYTES, value);  
		if (txValue < 128) 
		{	
			ReadWriteExtendedReg(CC1200_WRITE_BIT, address, i);
			HAL_Delay(1);
		}
		else
		{
			HAL_Delay(1);
			continue;
		}

		readValue = ReadWriteCommandReg(CC1200_SNOP); // Seems to need HAL_Delay and a NOP to produce the correct status bit
		HAL_Delay(8);
		
		if (readValue == 0x7f)
		{
			countErr++;
			ReadWriteCommandReg(CC1200_SFTX); // Flush if FIFO error
			i--;
			continue;
		}
		else if ((readValue & 0xf0) != 0x20)
		{
			ReadWriteCommandReg(CC1200_STX);
//			memcpy(Msg1, Msg2, 100);
//				
//			snprintf((char *)Msg1, sizeof(Msg1), "\r\nStatus byte: 0x%02x\tNum TX bytes:0x%02x\r\n", readValue, txValue);
//			HAL_UART_Transmit(&huart2, (uint8_t *) Msg1, sizeof(Msg1), 1);
			count++;
			HAL_Delay(1);
		} 
		
//		else if ((readValue & 0xf0) == 0x20)
//		{
//			correct++;
//		}
//		else
//		{
//		weird++;
//		}
	}
	
	// Check for errors again to make sure everything transmits properly
	// TO DO: make this code below into a while loop that checks if there are still bytes left in the TX FIFO
	for(int i = 0; i < sendAmt; i++)
	{
		readValue = ReadWriteCommandReg(CC1200_SNOP); // Seems to need HAL_Delay and a NOP to produce the correct status bit
		HAL_Delay(8);
		
		if (readValue == 0x7f)
		{
			countErr++;
			ReadWriteCommandReg(CC1200_SFTX); // Flush if FIFO error
			i--;
			continue;
		}
		else if ((readValue & 0xf0) != 0x20)
		{
			ReadWriteCommandReg(CC1200_STX);
//			memcpy(Msg1, Msg2, 100);
//				
//			snprintf((char *)Msg1, sizeof(Msg1), "\r\nStatus byte: 0x%02x\tNum TX bytes:0x%02x\r\n", readValue, txValue);
//			HAL_UART_Transmit(&huart2, (uint8_t *) Msg1, sizeof(Msg1), 1);
			count++;
			HAL_Delay(1);
		} 
		HAL_Delay(20);
	}
	
	
//	memcpy(Msg1, Msg2, 100);
//	snprintf((char *)Msg1, sizeof(Msg1), "\nRestrobing TX: %u\tFIFO Errors: %u\tCorrectly sent:%u\tWeird:%u\n", count, countErr,correct,weird);
//	HAL_UART_Transmit(&huart2, (uint8_t *) Msg1, sizeof(Msg1), 1);

	HAL_Delay(10);
	
//	readValue = ReadWriteExtendedReg(CC1200_READ_BIT, address, readValue);
//	memcpy(Msg1, Msg2, 100);
//	snprintf((char *)Msg1, sizeof(Msg1), "\nValue at register 0x%02x should be %u, is: %u\n", address, sendAmt - 1 , readValue);
//	HAL_UART_Transmit(&huart2, (uint8_t *) Msg1, sizeof(Msg1), 1);

	// Test receiving
	HAL_Delay(100);
//	
//	readValue = ReadWriteCommandReg(CC1200_SFRX); // Flush RXfifo
//	HAL_Delay(10);
//	
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
//	readValue = 0;
//	readValue = ReadWriteCommandReg(CC1200_SRX); // Recieve
//	HAL_Delay(1000);
//	readValue = ReadWriteCommandReg(CC1200_SNOP); // Seems to need HAL_Delay and a NOP to produce the correct status bit
//	memcpy(Msg1, Msg2, 100);
//	snprintf((char *)Msg1, sizeof(Msg1), "\r\nMode Test: receive status byte: 0x%x\r\n", readValue);
//	HAL_UART_Transmit(&huart2, (uint8_t *) Msg1, sizeof(Msg1), 1);
//	
//	address = CC1200_FIFO_NUM_RXBYTES;
//	readValue = ReadWriteExtendedReg(CC1200_READ_BIT, address, readValue);
//	memcpy(Msg1, Msg2, 100);
//	snprintf((char *)Msg1, sizeof(Msg1), "\r\nNumber of bytes in the RX fifo: 0x%x\r\n", readValue);
//	HAL_UART_Transmit(&huart2, (uint8_t *) Msg1, sizeof(Msg1), 1);
//	
	
	
//	memcpy(Msg1, Msg2, 100);
//	snprintf((char *)Msg1, sizeof(Msg1), "\n\nEND OF TRANSCEIVER TEST %02x\n", readValue);
//	HAL_UART_Transmit(&huart2, (uint8_t *) Msg1, sizeof(Msg1), 1);
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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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


uint8_t ReadWriteExtendedReg (uint8_t accessType, uint16_t address, uint8_t value)
{
	// Chip select low
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	
	uint8_t addrHI = address>>8;
	uint8_t addrLO = (uint8_t) address;
	uint8_t readValue = 0;
	
	
	if(accessType) { 	// Read from address
		
		if (addrHI)
		{
			addrHI |= CC1200_READ_BIT;
			HAL_SPI_Transmit(&hspi1,&addrHI, 1, 10);	// Access address	
			while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){};
			HAL_SPI_Transmit(&hspi1,&addrLO, 1, 10);
		}
		else
		{
			addrLO |= CC1200_READ_BIT;
			HAL_SPI_Transmit(&hspi1,&addrLO, 1, 10);
			while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){};
		}
		HAL_SPI_Transmit(&hspi1,0x0, 1, 10); // Transmit 0 for read
		HAL_SPI_Receive(&hspi1, &readValue, 1, 10); // Read value
			
			
	} else { // Write to address
			
		addrHI |= CC1200_WRITE_BIT;
		
		if (addrHI)
		{
			HAL_SPI_Transmit(&hspi1,&addrHI, 1, 10);	// Access address	
			while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){};
		}
		HAL_SPI_Transmit(&hspi1,&addrLO, 1, 10);
		while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){};

		HAL_SPI_Transmit(&hspi1, &value, 1, 10);		// Write value
	}
	
	// Chip select high	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	
	return readValue;
}

uint8_t ReadWriteCommandReg (uint8_t address)
{
	// Chip select low
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	
  uint8_t readValue  = 0;
	
	HAL_SPI_Transmit(&hspi1,&address, 1, 10);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){};
	HAL_SPI_Receive(&hspi1, &readValue, 1, 10);
		
	// Chip select high	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	
	return readValue;
}

void CC1200_INIT(void)
{
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_IOCFG2, SMARTRF_SETTING_IOCFG2);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_DEVIATION_M, SMARTRF_SETTING_DEVIATION_M);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_MODCFG_DEV_E, SMARTRF_SETTING_MODCFG_DEV_E);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_DCFILT_CFG, SMARTRF_SETTING_DCFILT_CFG);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_PREAMBLE_CFG0, SMARTRF_SETTING_PREAMBLE_CFG0);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_IQIC, SMARTRF_SETTING_IQIC);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_CHAN_BW, SMARTRF_SETTING_CHAN_BW);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_MDMCFG1, SMARTRF_SETTING_MDMCFG1);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_MDMCFG0, SMARTRF_SETTING_MDMCFG0);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_SYMBOL_RATE2, SMARTRF_SETTING_SYMBOL_RATE2);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_SYMBOL_RATE1, SMARTRF_SETTING_SYMBOL_RATE1);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_SYMBOL_RATE0, SMARTRF_SETTING_SYMBOL_RATE0);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_AGC_REF, SMARTRF_SETTING_AGC_REF);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_AGC_CS_THR, SMARTRF_SETTING_AGC_CS_THR);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_AGC_CFG1, SMARTRF_SETTING_AGC_CFG1);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_AGC_CFG0, SMARTRF_SETTING_AGC_CFG0);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_FIFO_CFG, SMARTRF_SETTING_FIFO_CFG);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_FS_CFG, SMARTRF_SETTING_FS_CFG);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_PKT_CFG2, SMARTRF_SETTING_PKT_CFG2);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_PKT_CFG0, SMARTRF_SETTING_PKT_CFG0);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_PKT_LEN, SMARTRF_SETTING_PKT_LEN);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_IF_MIX_CFG, SMARTRF_SETTING_IF_MIX_CFG);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_FREQOFF_CFG, SMARTRF_SETTING_FREQOFF_CFG);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_MDMCFG2, SMARTRF_SETTING_MDMCFG2);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_FREQ2, SMARTRF_SETTING_FREQ2);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_FREQ1, SMARTRF_SETTING_FREQ1);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_FREQ0, SMARTRF_SETTING_FREQ0);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_IF_ADC1, SMARTRF_SETTING_IF_ADC1);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_IF_ADC0, SMARTRF_SETTING_IF_ADC0);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_FS_DIG1, SMARTRF_SETTING_FS_DIG1);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_FS_DIG0, SMARTRF_SETTING_FS_DIG0);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_FS_CAL1, SMARTRF_SETTING_FS_CAL1);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_FS_CAL0, SMARTRF_SETTING_FS_CAL0);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_FS_DIVTWO, SMARTRF_SETTING_FS_DIVTWO);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_FS_DSM0, SMARTRF_SETTING_FS_DSM0);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_FS_DVC0, SMARTRF_SETTING_FS_DVC0);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_FS_PFD, SMARTRF_SETTING_FS_PFD);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_FS_PRE, SMARTRF_SETTING_FS_PRE);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_FS_REG_DIV_CML, SMARTRF_SETTING_FS_REG_DIV_CML);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_FS_SPARE, SMARTRF_SETTING_FS_SPARE);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_FS_VCO0, SMARTRF_SETTING_FS_VCO0);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_XOSC5, SMARTRF_SETTING_XOSC5);
	ReadWriteExtendedReg(CC1200_WRITE_BIT, CC1200_XOSC1, SMARTRF_SETTING_XOSC1);
}



////////////////////////////// TI Reference Code, Not currently being used/////////////////////
static void trxReadWriteBurstSingle(uint8_t addr, uint8_t *pData, int len)
{
    int i;
    uint8_t dummy;
    /* Communicate len number of bytes: if RX - the procedure sends 0x00 to push bytes from slave*/
    if (addr & CC1200_READ_BIT) {
        if (addr & CC1200_BURST_BIT) {
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
        if (addr & CC1200_BURST_BIT) {
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
	uint8_t readValue  = 0;
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
