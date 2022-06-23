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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

//THE FOLLOWING IS THE CALLBACK CODE FOR THE INTERUPT
uint8_t CASE_VALUE = 0; //was uint16_t
uint8_t CASE_SEL_VALUE = 0;
//EMOTIONS
char h[8] = "h";//104
char s[8] = "j";//s 115
char m[8] = "m";//109
char a[8] = "a";//97
//NEED
char r[8] = "r";//114
char f[8] = "f";//102
char z[8] = "z";//122
char n[8] = "n";//110
//PAIN
char p[8] = "t";//116
char pp[8] = "k";//107
char ppp[8] = "w";//119
char pppp[8] = "y";//121
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//DATA ENETERING THE MATRIX IS 16 BITS
//15-12, 11-8 7-0 BITS [XXXX, ADDRESS, (MSB DATA LSB)] RESPECTIVELY


//FUNCTION FOR OPERATING THE MATRIX DISPLAY

//HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

//HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);


//FOLLOWING METHODS FOR THE VARIOUS EMOTION PRESENTED ON THE MATRIX



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);

  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
  // The idea here is that the pin when equal to the pin pressed will run a while loop that will track or count while
  // The gpio pin is equal to the pin pressed. This will then check the track value and see if it is equal to the count limit
  // If it is equal to the count limit then the mood/emotion place holder will be assigned a value of one and then run the while code.
  // Thinking about making this a method to shorten the code


  // This interrupt will only trigger when the reset button is pressed
  // This will cause the case value to become 0
  if(GPIO_Pin == Reset_Pin)
    {

	  //Matrix_Init();
	  CASE_SEL_VALUE = 0;
  	  //void Matrix_Data_Write(uint8_t Data);
	  char h[8] = "u";//117
	  HAL_UART_Transmit(&huart6,(uint8_t *)h,strlen(h),1000);
    }
// I BELIVE THERE CURRENTLY IS NO USE FOR THIS
  if(GPIO_Pin == GPIO_PIN_15)
  {
	  /*TESTING THAT THE SWITCH WORKS
	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
	  HAL_Delay(1000);
	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);

	  Matrix_test();
	  Matrix_scan();
	  Matrix_intensity();
	  Matrix_shutdown();
	  Matrix_decodemode();
	  */


  }

  if(GPIO_Pin == CASE_SEL_Pin)
  {
	  CASE_SEL_VALUE++;
	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
	  HAL_Delay(50);
	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);
	  if(CASE_SEL_VALUE == 4)
	  {
		  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
		  			  		  	  HAL_Delay(2000);
		  			  		  	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);
		  CASE_SEL_VALUE = 0;
	  }
	  else
	  {
		  CASE_SEL_VALUE = CASE_SEL_VALUE;
	  }

  }


}




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
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


//===========================================================

//How the code segment in the switch works is:
	  //when the case select interrupt button is pressed the button will add a value to the case select value variable
	  //when the case value is then assigned a value a if statement will check if the value is equal to 4 if it is the value will reset to zero
	  //the zero case value allows us to operate in the case statement where our main 4 button are used to select feelings
	  //changing the case value we change the case we run which changes the selection purpose of our buttons
	  //0-emotions 1- 2- 3- @4- we go back to 0
	  //we use the case select as an interrupt as we will always want to change the operation of the board instantly

switch(CASE_SEL_VALUE)
{


//How the code segment in each case works is:
	  //while the button is being pressed there will be a delay of 300ms
	  //after the delay if the button is still being pressed then the code inside the if statement will run
	  //if the the the button is not read then the code will not run/ will not be considered as a button press

//FEELINGS
case 0:
	while(HAL_GPIO_ReadPin(GPIOA, Happy_Pin))
		  	  {
			  	  HAL_Delay(300);
			  	  if(HAL_GPIO_ReadPin(GPIOA, Happy_Pin))
			  	  	  {
			  		  	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
			  		  	  HAL_Delay(1000);
			  		  	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);

			  		  	HAL_UART_Transmit(&huart6,(uint8_t *)h,strlen(h),1000);
			  	  	  }
		  	  }

		  while(HAL_GPIO_ReadPin(GPIOA, Sad_Pin))
		  	  {
			  	  HAL_Delay(300);
			  	  if(HAL_GPIO_ReadPin(GPIOA, Sad_Pin))
			  	  	  {
			  		  	 HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
			  		  	 HAL_Delay(2000);
			  		  	 HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);

			  		  	HAL_UART_Transmit(&huart6,(uint8_t *)s,strlen(s),1000);
			  	  	  }
		  	  }
		  while(HAL_GPIO_ReadPin(GPIOA, Mad_Pin))
		  	  {
			  	  HAL_Delay(300);
			  	  if(HAL_GPIO_ReadPin(GPIOA, Mad_Pin))
			  	  	  {
			  		  	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
			  		  	  HAL_Delay(3000);
			  		  	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);

			  		  	HAL_UART_Transmit(&huart6,(uint8_t *)m,strlen(m),1000);
			  	  	  }
		  	  }

		  while(HAL_GPIO_ReadPin(GPIOD, Anxious_Pin))
		  	  {
			  	  HAL_Delay(300);
			  	  if(HAL_GPIO_ReadPin(GPIOD, Anxious_Pin))
			  	  	  {
			  		  	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
			  		  	  HAL_Delay(4000);
			  		  	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);

			  		  	HAL_UART_Transmit(&huart6,(uint8_t *)a,strlen(a),1000);
			  	  	  }
		  	  }
break;
//This asks about needs
case 1:

	while(HAL_GPIO_ReadPin(GPIOA, Happy_Pin))
		  	  {
			  	  HAL_Delay(300);
			  	  if(HAL_GPIO_ReadPin(GPIOA, Happy_Pin))
			  	  	  {
			  		  	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
			  		  	  HAL_Delay(5000);
			  		  	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);

			  		  	HAL_UART_Transmit(&huart6,(uint8_t *)r,strlen(r),1000);
			  	  	  }
		  	  }

		  while(HAL_GPIO_ReadPin(GPIOA, Sad_Pin))
		  	  {
			  	  HAL_Delay(300);
			  	  if(HAL_GPIO_ReadPin(GPIOA, Sad_Pin))
			  	  	  {
			  		  	 HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
			  		  	 HAL_Delay(6000);
			  		  	 HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);

			  		  	HAL_UART_Transmit(&huart6,(uint8_t *)f,strlen(f),1000);
			  	  	  }
		  	  }
		  while(HAL_GPIO_ReadPin(GPIOA, Mad_Pin))
		  	  {
			  	  HAL_Delay(300);
			  	  if(HAL_GPIO_ReadPin(GPIOA, Mad_Pin))
			  	  	  {
			  		  	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
			  		  	  HAL_Delay(7000);
			  		  	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);

			  		  	HAL_UART_Transmit(&huart6,(uint8_t *)z,strlen(z),1000);
			  	  	  }
		  	  }

		  while(HAL_GPIO_ReadPin(GPIOD, Anxious_Pin))
		  	  {
			  	  HAL_Delay(300);
			  	  if(HAL_GPIO_ReadPin(GPIOD, Anxious_Pin))
			  	  	  {
			  		  	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
			  		  	  HAL_Delay(7000);
			  		  	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);

			  		  	HAL_UART_Transmit(&huart6,(uint8_t *)n,strlen(n),1000);
			  	  	  }
		  	  }

break;
// This asks about pain scale
case 2:
	while(HAL_GPIO_ReadPin(GPIOA, Happy_Pin))
		  	  {
			  	  HAL_Delay(300);
			  	  if(HAL_GPIO_ReadPin(GPIOA, Happy_Pin))
			  	  	  {
			  		  	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
			  		  	  HAL_Delay(8000);
			  		  	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);

			  		  	HAL_UART_Transmit(&huart6,(uint8_t *)p,strlen(p),1000);
			  	  	  }
		  	  }

		  while(HAL_GPIO_ReadPin(GPIOA, Sad_Pin))
		  	  {
			  	  HAL_Delay(300);
			  	  if(HAL_GPIO_ReadPin(GPIOA, Sad_Pin))
			  	  	  {
			  		  	 HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
			  		  	 HAL_Delay(9000);
			  		  	 HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);

			  		  	HAL_UART_Transmit(&huart6,(uint8_t *)pp,strlen(pp),1000);
			  	  	  }
		  	  }
		  while(HAL_GPIO_ReadPin(GPIOA, Mad_Pin))
		  	  {
			  	  HAL_Delay(300);
			  	  if(HAL_GPIO_ReadPin(GPIOA, Mad_Pin))
			  	  	  {
			  		  	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
			  		  	  HAL_Delay(10000);
			  		  	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);

			  		  	HAL_UART_Transmit(&huart6,(uint8_t *)ppp,strlen(ppp),1000);
			  	  	  }
		  	  }

		  while(HAL_GPIO_ReadPin(GPIOD, Anxious_Pin))
		  	  {
			  	  HAL_Delay(300);
			  	  if(HAL_GPIO_ReadPin(GPIOD, Anxious_Pin))
			  	  	  {
			  		  	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
			  		  	  HAL_Delay(11000);
			  		  	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);

			  		  	HAL_UART_Transmit(&huart6,(uint8_t *)pppp,strlen(pppp),1000);
			  	  	  }
		  	  }
break;
//This asks about call button
case 3:
	while(HAL_GPIO_ReadPin(GPIOA, Happy_Pin))
		  	  {
			  	  HAL_Delay(300);
			  	  if(HAL_GPIO_ReadPin(GPIOA, Happy_Pin))
			  	  	  {
			  		  	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
			  		  	  HAL_Delay(12000);
			  		  	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);

			  		  	HAL_UART_Transmit(&huart6,(uint8_t *)h,strlen(h),1000);
			  	  	  }
		  	  }

		  while(HAL_GPIO_ReadPin(GPIOA, Sad_Pin))
		  	  {
			  	  HAL_Delay(300);
			  	  if(HAL_GPIO_ReadPin(GPIOA, Sad_Pin))
			  	  	  {
			  		  	 HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
			  		  	 HAL_Delay(13000);
			  		  	 HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);

			  		  	HAL_UART_Transmit(&huart6,(uint8_t *)h,strlen(h),1000);
			  	  	  }
		  	  }
		  while(HAL_GPIO_ReadPin(GPIOA, Mad_Pin))
		  	  {
			  	  HAL_Delay(300);
			  	  if(HAL_GPIO_ReadPin(GPIOA, Mad_Pin))
			  	  	  {
			  		  	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
			  		  	  HAL_Delay(14000);
			  		  	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);

			  		  	HAL_UART_Transmit(&huart6,(uint8_t *)h,strlen(h),1000);
			  	  	  }
		  	  }

		  while(HAL_GPIO_ReadPin(GPIOD, Anxious_Pin))
		  	  {
			  	  HAL_Delay(300);
			  	  if(HAL_GPIO_ReadPin(GPIOD, Anxious_Pin))
			  	  	  {
			  		  	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
			  		  	  HAL_Delay(15000);
			  		  	  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);

			  		  	HAL_UART_Transmit(&huart6,(uint8_t *)h,strlen(h),1000);
			  	  	  }
		  	  }
break;
//This is the default case
default:
	HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);

}//END OF SWITCH

}//END OF WHILE(1)
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Reset_Pin */
  GPIO_InitStruct.Pin = Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Reset_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Happy_Pin Sad_Pin Mad_Pin */
  GPIO_InitStruct.Pin = Happy_Pin|Sad_Pin|Mad_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CASE_SEL_Pin */
  GPIO_InitStruct.Pin = CASE_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(CASE_SEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Anxious_Pin */
  GPIO_InitStruct.Pin = Anxious_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Anxious_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : BLUE_Pin */
  GPIO_InitStruct.Pin = BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLUE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
