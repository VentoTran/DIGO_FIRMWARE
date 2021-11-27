/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//  4x6 button matrix

#define RowPort GPIOD
#define ColPort GPIOA

#define Row1Pin GPIO_PIN_0
#define Row2Pin GPIO_PIN_1
#define Row3Pin GPIO_PIN_2
#define Row4Pin GPIO_PIN_3
#define Row5Pin GPIO_PIN_4
#define Row6Pin GPIO_PIN_5

#define Col1Pin GPIO_PIN_0
#define Col2Pin GPIO_PIN_1
#define Col3Pin GPIO_PIN_2
#define Col4Pin GPIO_PIN_3

#define Pressing    1
#define Releasing   0


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
  


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */


uint32_t readButton(void);
void scanRow(uint8_t Col, uint16_t ColPin, uint32_t* mask);
buttonPushEvent pushEventHandler(uint32_t* mask, uint16_t GPIO_Pin);
buttonReleaseEvent releaseEventHandler(uint32_t* mask, uint16_t GPIO_Pin);
buttonPushEvent p_pushFunc = pushEventHandler;
buttonReleaseEvent p_releaseFunc = releaseEventHandler;

void clearGPIO_EXTI(void);
void setGPIO_EXTI(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t gButtonMatrixMask = 0;

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
  /* USER CODE BEGIN 2 */

	uint32_t buttonMatrixMask = 0;

  buttonMatrixMask = readButton();

  
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    HAL_Delay(2000);
		
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15
                          PD0 PD1 PD2 PD3
                          PD4 PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_GPIO_WritePin(RowPort, Row1Pin|Row2Pin|Row3Pin|Row4Pin|Row5Pin|Row6Pin, 0);

}

/* USER CODE BEGIN 4 */

void clearGPIO_EXTI(void)
{
  EXTI->FTSR &= ~(15UL << 0);   //disable all falling & raising edge EXTI enable bit
  EXTI->RTSR &= ~(15UL << 0);
}

void setGPIO_EXTI(void)
{
  EXTI->FTSR |= (15UL << 0);
  EXTI->RTSR |= (15UL << 0);
}

uint32_t readButton(void)
{
  clearGPIO_EXTI();
  uint32_t MaskBit = 0;
  
  HAL_GPIO_WritePin(RowPort, Row1Pin|Row2Pin|Row3Pin|Row4Pin|Row5Pin|Row6Pin, 0);

  uint8_t CheckCol = 0;
  CheckCol =    ((uint8_t)!HAL_GPIO_ReadPin(ColPort, Col1Pin) << 0) | 
                ((uint8_t)!HAL_GPIO_ReadPin(ColPort, Col2Pin) << 1) | 
                ((uint8_t)!HAL_GPIO_ReadPin(ColPort, Col3Pin) << 2) | 
                ((uint8_t)!HAL_GPIO_ReadPin(ColPort, Col4Pin) << 3);

  if(CheckCol == 0)
  {
    setGPIO_EXTI();
    return MaskBit;    //No button press
  }
  else    //something happen, figgure it out!
  {
    if(CheckCol & 0b0001) //col1
    {
      scanRow(1, Col1Pin, &MaskBit);
    }
    if(CheckCol & 0b0010) //col2
    {
      scanRow(2, Col2Pin, &MaskBit);
    }
    if(CheckCol & 0b0100) //col3
    {
      scanRow(3, Col3Pin, &MaskBit);
    }
    if(CheckCol & 0b1000) //col4
    {
      scanRow(4, Col4Pin, &MaskBit);
    }
  }
  setGPIO_EXTI();
  return MaskBit;
}

void scanRow(uint8_t Col, uint16_t ColPin, uint32_t* mask)
{
  HAL_GPIO_WritePin(RowPort, Row1Pin, 0);
  HAL_GPIO_WritePin(RowPort, Row2Pin|Row3Pin|Row4Pin|Row5Pin|Row6Pin, 1);
  if(!HAL_GPIO_ReadPin(ColPort, ColPin))
  {
    *mask |= (1UL << (Col-1));
  }
  HAL_GPIO_WritePin(RowPort, Row2Pin, 0);
  HAL_GPIO_WritePin(RowPort, Row1Pin|Row3Pin|Row4Pin|Row5Pin|Row6Pin, 1);
  if(!HAL_GPIO_ReadPin(ColPort, ColPin))
  {
    *mask |= (1UL << (Col + 3));
  }
  HAL_GPIO_WritePin(RowPort, Row3Pin, 0);
  HAL_GPIO_WritePin(RowPort, Row1Pin|Row2Pin|Row4Pin|Row5Pin|Row6Pin, 1);
  if(!HAL_GPIO_ReadPin(ColPort, ColPin))
  {
    *mask |= (1UL << (Col + 7));
  }
  HAL_GPIO_WritePin(RowPort, Row4Pin, 0);
  HAL_GPIO_WritePin(RowPort, Row1Pin|Row2Pin|Row3Pin|Row5Pin|Row6Pin, 1);
  if(!HAL_GPIO_ReadPin(ColPort, ColPin))
  {
    *mask |= (1UL << (Col + 11));
  }
  HAL_GPIO_WritePin(RowPort, Row5Pin, 0);
  HAL_GPIO_WritePin(RowPort, Row1Pin|Row2Pin|Row3Pin|Row4Pin|Row6Pin, 1);
  if(!HAL_GPIO_ReadPin(ColPort, ColPin))
  {
    *mask |= (1UL << (Col + 15));
  }
  HAL_GPIO_WritePin(RowPort, Row6Pin, 0);
  HAL_GPIO_WritePin(RowPort, Row1Pin|Row2Pin|Row3Pin|Row4Pin|Row5Pin, 1);
  if(!HAL_GPIO_ReadPin(ColPort, ColPin))
  {
    *mask |= (1UL << (Col + 19));
  }

  HAL_GPIO_WritePin(RowPort, Row1Pin|Row2Pin|Row3Pin|Row4Pin|Row5Pin|Row6Pin, 0);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{ 
  for(int i = 0; i < 10000000; i++) {}
  clearGPIO_EXTI();
  EXTI->PR &= ~(uint32_t)0;
  if(HAL_GPIO_ReadPin(ColPort, GPIO_Pin) == SET)  
  {
    p_releaseFunc(&gButtonMatrixMask, GPIO_Pin);
  }
  else 
  {
    p_pushFunc(&gButtonMatrixMask, GPIO_Pin);
  }
}

buttonPushEvent pushEventHandler(uint32_t* mask, uint16_t GPIO_Pin)
{
  uint8_t nCol = 0;
  if (GPIO_Pin == Col1Pin)       {nCol = 1;}
  else if (GPIO_Pin == Col2Pin)  {nCol = 2;}
  else if (GPIO_Pin == Col3Pin)  {nCol = 3;}
  else if (GPIO_Pin == Col4Pin)  {nCol = 4;}

  scanRow(nCol, GPIO_Pin, &gButtonMatrixMask);
  setGPIO_EXTI();
  
}

buttonReleaseEvent releaseEventHandler(uint32_t* mask, uint16_t GPIO_Pin)
{
  uint8_t nCol = 0;
  if (GPIO_Pin == Col1Pin)       {nCol = 1;}
  else if (GPIO_Pin == Col2Pin)  {nCol = 2;}
  else if (GPIO_Pin == Col3Pin)  {nCol = 3;}
  else if (GPIO_Pin == Col4Pin)  {nCol = 4;}

  for(int i = 0; i < 24; i += 4)
  {
    if(*mask & (1UL << (nCol - 1 + i)))
    {
      *mask &= ~(1UL << (nCol - 1 + i));
    }
  }
  setGPIO_EXTI();
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
