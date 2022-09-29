/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include <math.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
 
/* USER CODE END Includes */
 
/* Private typedef -----------------------------------------------------------*/
typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hour ;
	uint8_t dayofweek ;
	uint8_t dayofmonth;
	uint8_t month;
	uint8_t year;
} TIME;
/* USER CODE BEGIN PTD */
#define DELAY1 9900
#define DELAY2 200
/* USER CODE END PTD */
 
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */
#define DS3231_ADDRESS 0xD0
#define FIRST_REG 0x00
#define REG_SIZE 0x01
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
 
#define EPOCH_2022 1640988000
/* USER CODE END PM */
 
/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
 
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
 
/* USER CODE BEGIN PV */
char buffer[14];
uint8_t data [] = "Hello from STM32!\r\n";
TIME time;
/* USER CODE END PV */
 
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
void HAL_UART_TxCpltCllback(UART_HandleTypeDef *huart);
void pause_sec(float x);
int bcdToDec(uint8_t val);
uint8_t decToBcd(int val);
void getTime (void);
void setTime (uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year);
int epochFromTime(TIME time);
void BCDtoDecimalTesting(uint8_t val);
void decimalToBCDTesting(int val);
 
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
  MX_I2C1_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
 
  /* USER CODE END 2 */
 
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  setTime(45, 00, 15, 4, 29, 9, 22); // Set the time to a specific time
 
  uint_t valtoBCD = 24;
  int BCDtoDECValue = 64;
  decimalToBCDTesting(valtoBCD); // method to test decimalToBCD function
  BCDtoDecimalTesting(BCDtoDECValue);	// method to test BCDtoDecimal function
  while (1)
  {
    /* USER CODE END WHILE */
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8); // toggle the GPIO pin(blue one)
	  pause_sec(1);		// create a delay of one second
	  getTime();		// Get time from the RTC
	  int epoch = epochFromTime(time); // from time calculate epoch time for that time
	  sprintf(buffer, "%d-%d-%d:%d \r\n", time.year, time.month, time.dayofmonth, time.seconds); //send time to the serial port/com#
	  HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), 1000); //send time to the serial port/com#
	  sprintf(buffer, "%d \r\n\n", epoch);
	  HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), 1000); // print epoch to serial port
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
 
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
 
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
 
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}
 
/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
 
  /* USER CODE BEGIN I2C1_Init 0 */
 
  /* USER CODE END I2C1_Init 0 */
 
  /* USER CODE BEGIN I2C1_Init 1 */
 
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
 
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
 
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
 
  /* USER CODE END I2C1_Init 2 */
 
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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
 
  /* USER CODE END USART2_Init 2 */
 
}
 
/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
 
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
 
  /* DMA interrupt init */
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);
 
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
 
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);
 
  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
 
  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
 
}
 
/* USER CODE BEGIN 4 */
void pause_sec(float x)
{
	/* Delay program execution for x seconds */
	//TO DO:
	//TASK 2
	//Make sure you've defined DELAY1 and DELAY2 in the private define section
 
	//YOUR CODE HERE
	for(int i = 0; i < DELAY1; i++){
 
		for(int j = 0; j < DELAY2; j++){}
	}
 
}
 
uint8_t decToBcd(int val)
{
//    /* Convert normal decimal numbers to binary coded decimal*/
//	//TO DO:
//	//TASK 3
 
    uint8_t bcd = 0;
 
    int i = 0;
 
    while(val != 0){
 
        int rem = val % 10; // Get each value from the parameter given
 
        val = val / 10;		// remove the value {rem} from the bcd parameter given
        					// the val variable is reduced and used in the next iteration.
 
        bcd |= (rem << i);	// we shift the rem parameter acording to i, shift i units to the left
 
        i = i + 4;			// increment i by 4, 4 bits.
 
    }
 
    return bcd;
 
}
 
 
int bcdToDec(uint8_t val)
{
//    /* Convert binary coded decimal to normal decimal numbers */
//	//TO DO:
//	//TASK 3
//	//Complete the BCD to decimal function
//
//	//YOUR CODE HERE
    uint8_t units = 0;		//
    uint8_t tens = 0;
    uint8_t ans = 0;
 
    units = val << 4;		// shifts the units 4 units to the left to drop the tens.
    units = units >> 4;		// after shifts the units 4 units to the right, which is the correct position for the units
 
    tens = (val >> 4) * 10 ;	// shifts tens to units and multiply by 10 to get tens in decimal
    ans = tens + units;			// add both tens and units to the value to be returned
    return ans;
}
 
void setTime (uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year)
{
//    /* Write the time to the RTC using I2C */
//	//TO DO:
//	//TASK 4
//
	uint8_t set_time[7];
 
	set_time[0] = decToBcd(sec);
	set_time[1] = decToBcd(min);
	set_time[2] = decToBcd(hour);
	set_time[3] = decToBcd(dow);
	set_time[4] = decToBcd(dom);
	set_time[5] = decToBcd(month);
	set_time[6] = decToBcd(year);
//	//fill in the address of the RTC, the address of the first register to write anmd the size of each register
//	//The function and RTC supports multiwrite. That means we can give the function a buffer and first address
//	//and it will write 1 byte of data, increment the register address, write another byte and so on
	HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDRESS, 0x00, 1, set_time, 7, 1000);
}
//
void getTime (void)
{
//    /* Get the time from the RTC using I2C */
//	//TO DO:
//	//TASK 4
//	//Update the global TIME time structure
//
	uint8_t get_time[7];
//
//	//fill in the address of the RTC, the address of the first register to write anmd the size of each register
//	//The function and RTC supports multiread. That means we can give the function a buffer and first address
//	//and it will read 1 byte of data, increment the register address, write another byte and so on
	HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDRESS, 0x00, 1, get_time, 7, 1000);
	time.seconds = bcdToDec(get_time[0]);
	time.minutes = bcdToDec(get_time[1]);
	time.hour = bcdToDec(get_time[2]);
	time.dayofweek = bcdToDec(get_time[3]);
	time.dayofmonth = bcdToDec(get_time[4]);
	time.month = bcdToDec(get_time[5]);
	time.year = bcdToDec(get_time[6]);
//
//
//	//YOUR CODE HERE
//
}
 
int epochFromTime(TIME time){
    /* Convert time to UNIX epoch time */
	//TO DO:
	//TASK 5
	//You have been given the epoch time for Saturday, January 1, 2022 12:00:00 AM GMT+02:00
	//It is define above as EPOCH_2022. You can work from that and ignore the effects of leap years/seconds
 
	//YOUR CODE HERE
 
	int day = time.dayofmonth - 1;
 
	switch(time.month){
	case 2:
		day += 31;
		break;
	case 3:
			day += 59;
			break;
	case 4:
			day += 90;
			break;
	case 5:
			day += 120;
			break;
	case 6:
			day += 151;
			break;
	case 7:
			day += 181;
			break;
	case 8:
			day += 212;
			break;
	case 9:
			day += 243;
			break;
	case 10:
			day += 273;
			break;
	case 11:
			day += 304;
			break;
	case 12:
			day += 334;
			break;
	/*
	 *COMPLETE THE SWITCH CASE OR INSERT YOUR OWN LOGIC
	 */
	default:
		day = day;
	}
	int add_time;
		add_time = time.seconds;
		add_time += time.minutes*60;
		add_time += time.hour*3600;
		add_time += day*86400;
	if (time.year != 2022)
	{
		int year = time.year - 22;
		add_time += year*3153600;
	}
 
	return EPOCH_2022 + add_time + 50000000;
}
 
void BCDtoDecimalTesting(uint8_t val){
	/**
	* funtion to test the function BCDToDecimal function, and print the output to the serial port
	*/
	sprintf(buffer, " BCD To Dec : %d\r\n", bcdToDec(val));
//	HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), 1000);
	HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), 1000);
}
 
void decimalToBCDTesting(int val){
	/**
	 * funtion to test the function decimalToBCD function, and print the output to the serial port
	 */
	sprintf(buffer, " Dec To BCD : %d\r\n", decToBcd(val));
	HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), 1000);
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
