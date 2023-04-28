/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
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
UART_HandleTypeDef huart1;

/* Definitions for UART_Com */
osThreadId_t UART_ComHandle;
const osThreadAttr_t UART_Com_attributes = {
  .name = "UART_Com",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LED_Blink */
osThreadId_t LED_BlinkHandle;
const osThreadAttr_t LED_Blink_attributes = {
  .name = "LED_Blink",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IRSensor */
osThreadId_t IRSensorHandle;
const osThreadAttr_t IRSensor_attributes = {
  .name = "IRSensor",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for FlashMemCheck */
osThreadId_t FlashMemCheckHandle;
const osThreadAttr_t FlashMemCheck_attributes = {
  .name = "FlashMemCheck",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */


HAL_StatusTypeDef uartState;
uint8_t uartBuffer[BUFFER_SIZE] = "Hi!\r\n";
uint32_t flashBuffer[4];
uint32_t peopleInCount = 0;
uint32_t peopleOutCount = 0;
GPIO_PinState sensorState = GPIO_PIN_SET;
GPIO_PinState sensor1State = GPIO_PIN_SET;
GPIO_PinState sensor2State = GPIO_PIN_SET;
uint8_t sensorMessage[SENSOR_MESSAGE_SIZE] = "Object detected!\r\n";
uint8_t sensorMessageIn[SENSOR_MESSAGE_SIZE] = "Object IN!\r\n";
uint8_t sensorMessageOut[SENSOR_MESSAGE_SIZE] = "Object OUT!\r\n";
uint32_t sensor1Timestamp;
uint32_t sensor2Timestamp;
FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t PageError;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void StartUART(void *argument);
void StartLEDBlink(void *argument);
void StartIRSensor(void *argument);
void StartMemoryCheck(void *argument);

/* USER CODE BEGIN PFP */
uint32_t FlashEraseData(FLASH_EraseInitTypeDef EraseInitStruct, uint32_t PageError);
uint32_t FlashWriteData(uint32_t startPageAddress, uint32_t *data, uint16_t wordsNumber);
void FlashReadData(uint32_t startPageAddress, uint32_t *rxBuffer, uint16_t wordsNumber);
void ConvertToStr(uint32_t num1, uint32_t num2, uint8_t* buf);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);

  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = USER_DATA_START_PAGE_ADDRESS;
  EraseInitStruct.NbPages = 1;
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UART_Com */
  UART_ComHandle = osThreadNew(StartUART, NULL, &UART_Com_attributes);

  /* creation of LED_Blink */
  LED_BlinkHandle = osThreadNew(StartLEDBlink, NULL, &LED_Blink_attributes);

  /* creation of IRSensor */
  IRSensorHandle = osThreadNew(StartIRSensor, NULL, &IRSensor_attributes);

  /* creation of FlashMemCheck */
  FlashMemCheckHandle = osThreadNew(StartMemoryCheck, NULL, &FlashMemCheck_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IR_Sensor_1_Pin IR_Sensor_2_Pin */
  GPIO_InitStruct.Pin = IR_Sensor_1_Pin|IR_Sensor_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Blue_PushButton_Pin */
  GPIO_InitStruct.Pin = Blue_PushButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Blue_PushButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD4_Pin */
  GPIO_InitStruct.Pin = LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

///=========================================================================
/// @brief Function for transmiting echo message via UART
///=========================================================================
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART1)
	{
		HAL_UART_Transmit_IT(&huart1, uartBuffer, Size);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, uartBuffer, sizeof(uartBuffer));
		uartState = HAL_OK;
	}
}

///=========================================================================
/// @brief Function for tracking UART errors
///=========================================================================
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		uartState = HAL_ERROR;
	}
}

///=========================================================================
/// @brief Function for sensor interrupt callback
///=========================================================================
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET)
	{
      sensor1State = GPIO_PIN_RESET;
      sensor1Timestamp = HAL_GetTick();
	}
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_RESET)
	{
      sensor2State = GPIO_PIN_RESET;
      sensor2Timestamp = HAL_GetTick();
	}
}

///=========================================================================
/// @brief Function for erasing specific data page in flash memory
/// @return result (success == 0)
///=========================================================================
uint32_t FlashEraseData(FLASH_EraseInitTypeDef EraseInitStruct, uint32_t PageError)
{
	HAL_FLASH_Unlock();
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
	{
		return HAL_FLASH_GetError();
	}
	HAL_FLASH_Lock();
	return 0;
}

///=========================================================================
/// @brief Function for writing data to specific data page in flash memory
/// @return result (success == 0)
///=========================================================================
uint32_t FlashWriteData(uint32_t startPageAddress, uint32_t *data, uint16_t wordsNumber)
{
	HAL_FLASH_Unlock();
	FlashEraseData(EraseInitStruct, PageError);
    for (uint16_t count = 0; count < wordsNumber; count++)
    {
    	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, startPageAddress, data[count]) == HAL_OK)
    	{
    		startPageAddress += FLASH_MEMORY_WORD_SIZE;
    	}
    	else
    	{
    		return HAL_FLASH_GetError();
      }
    }
    HAL_FLASH_Lock();
    return 0;
}

///=========================================================================
/// @brief Function for reading data from specific data page in flash memory
///=========================================================================
void FlashReadData(uint32_t startPageAddress, uint32_t *rxBuffer, uint16_t wordsNumber)
{
	for (; wordsNumber > 0; wordsNumber--)
	{
		*rxBuffer = *(__IO uint32_t *)startPageAddress;
		startPageAddress += FLASH_MEMORY_WORD_SIZE;
		rxBuffer++;
	}
}

///=========================================================================
/// @brief Function for converting 2 integer values to string 
///        and writing the string in buffer
///=========================================================================
void ConvertToStr(uint32_t num1, uint32_t num2, uint8_t* buf)
{
    uint32_t i = 2;
    uint32_t len = strlen(buf);
    uint32_t temp, rem;

    buf[0] = '\n';
    buf[1] = '\r';
    while (num2 != 0)
    {
        rem = num2 % 10;
        buf[i++] = rem + '0';
        num2 /= 10;
    }
    while (num1 != 0)
	  {
		  rem = num1 % 10;
		  buf[i++] = rem + '0';
		  num1 /= 10;
	  }
    buf[i] = '\0';

    for (int j = 0; j < len/2; j++)
    {
        temp = buf[j];
        buf[j] = buf[len - j - 1];
        buf[len - j - 1] = temp;
    }
}

///=========================================================================
/// @brief Function for updating buffer with key words and data from sensor
///=========================================================================
void updateBuffer(uint32_t *buffer)
{
	buffer[0] = 1;
	buffer[1] = peopleInCount;
	buffer[2] = peopleOutCount;
	buffer[3] = 2;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartUART */
/**
  * @brief  Function implementing the UART_Com thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUART */
void StartUART(void *argument)
{
  /* USER CODE BEGIN 5 */
  HAL_UART_Transmit(&huart1, uartBuffer, sizeof(uartBuffer), TRANSMISSION_TIMEOUT);
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, uartBuffer, sizeof(uartBuffer));
  /* Infinite loop */
  for(;;)
  {
	if (uartState != HAL_OK)	// Check if UART is working
	{
		osThreadSuspend(LED_BlinkHandle);	// Suspend led blinking if UART communication fails
		HAL_GPIO_WritePin (GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	}
	else
	{
		if (osThreadGetState(LED_BlinkHandle) != osThreadRunning)
		{
			osThreadResume(LED_BlinkHandle);	// Resume led blinking if UART is working
		}
	}
    osDelay(LED_BLINK_DELAY);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLEDBlink */
/**
* @brief Function implementing the LED_Blink thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLEDBlink */
void StartLEDBlink(void *argument)
{
  /* USER CODE BEGIN StartLEDBlink */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);	// Blink blue led
	osDelay(LED_BLINK_DELAY);
  }
  /* USER CODE END StartLEDBlink */
}

/* USER CODE BEGIN Header_StartIRSensor */
/**
* @brief Function implementing the IRSensor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIRSensor */
void StartIRSensor(void *argument)
{
  /* USER CODE BEGIN StartIRSensor */
  /* Infinite loop */
  for(;;)
  {
    if (sensor1State == GPIO_PIN_RESET && sensor2State == GPIO_PIN_RESET)
    {
      if (sensor1Timestamp > sensor2Timestamp)
      {
        sensor1Timestamp = sensor1Timestamp - sensor2Timestamp;
        if (sensor1Timestamp <= WALKTHROUGH_INTERVAL)
        {
          HAL_UART_Transmit_IT(&huart1, sensorMessageIn, SENSOR_MESSAGE_SIZE);
          peopleInCount++;
        }
      }
      if (sensor1Timestamp < sensor2Timestamp)
      {
        sensor1Timestamp = sensor2Timestamp - sensor1Timestamp;
        if (sensor1Timestamp <= WALKTHROUGH_INTERVAL)
        {
          HAL_UART_Transmit_IT(&huart1, sensorMessageOut, SENSOR_MESSAGE_SIZE);
          peopleOutCount++;
        }
      }
      sensor1State = GPIO_PIN_SET;
      sensor2State = GPIO_PIN_SET;
      
      if (flashBuffer[0] == USER_DATA_START_MAGIC_WORD &&
          flashBuffer[3] == USER_DATA_END_MAGIC_WORD)
      {
        if (flashBuffer[1] % 10 == 0 || flashBuffer[2] % 10 == 0)
        {
          HAL_UART_Transmit_IT(&huart1, uartBuffer, sizeof(uartBuffer));
        }
      }

    }
	  osDelay(SENSOR_STATE_CHECK);
  }
  /* USER CODE END StartIRSensor */
}

/* USER CODE BEGIN Header_StartMemoryCheck */
/**
* @brief Function implementing the FlashMemCheck thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMemoryCheck */
void StartMemoryCheck(void *argument)
{
  /* USER CODE BEGIN StartMemoryCheck */
  FlashReadData(USER_DATA_START_PAGE_ADDRESS, (uint32_t *)flashBuffer, 4);
  if (flashBuffer[0] != USER_DATA_START_MAGIC_WORD ||
      flashBuffer[3] != USER_DATA_END_MAGIC_WORD)
  {
	  FlashEraseData(EraseInitStruct, PageError);
	  updateBuffer(flashBuffer);
	  FlashWriteData(USER_DATA_START_PAGE_ADDRESS, flashBuffer, 4);
  }
  else
  {
    peopleInCount = flashBuffer[1];
    peopleOutCount = flashBuffer[2];
  }
  /* Infinite loop */
  for(;;)
  {
    updateBuffer(flashBuffer);
	  FlashWriteData(USER_DATA_START_PAGE_ADDRESS, flashBuffer, 4);
    memset(flashBuffer, '\0', sizeof(flashBuffer));

	  if (peopleInCount != 0 && peopleInCount != 0)
    {
      FlashReadData(USER_DATA_START_PAGE_ADDRESS, flashBuffer, 4);
     	ConvertToStr(flashBuffer[1], flashBuffer[2], uartBuffer);
    }
    osDelay(5000);
  }
  /* USER CODE END StartMemoryCheck */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
