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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "servo.h"
#include "commands.h"
#include "recipes.h"

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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx_buffer[256];
uint8_t command_buffer[2];
uint8_t newline[3] = "\r\n>";
uint8_t rx_index = 0;
uint8_t command_index = 0;
_Bool is_command_ready = 0;
uint8_t status[100];

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
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MFS_init();
  /* USER CODE BEGIN 2 */
  char Message[] = "Write anything on Serial Terminal\r\n";

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Transmit(&huart2, (uint8_t *)Message, strlen(Message), 10);
  HAL_UART_Transmit(&huart2, newline, sizeof(newline), HAL_MAX_DELAY);
  HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_index], 1);
  Servo servo_1 = initServo(1);
  Servo servo_2 = initServo(2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Start of loop
	  uint32_t startLoop = HAL_GetTick();


	      // Check if a command is ready
	      if (is_command_ready)
	      {
	    	  // execute command for servo 1
//	    	  if (startLoop > (servo_1.last_instr_time + (servo_1.delay + DELAY))){
	    		  servo_1.delay = 0;
	    		  execute_input(&servo_1, command_buffer[0] );
//		    	  sprintf(status, "Servo %d is at position %d, running recipe %d at index %d\r\n", servo_1.servo_num, servo_1.servo_pos, servo_1.recipe_num, servo_1.recipe_index);
//	    		  HAL_UART_Transmit(&huart2, status, strlen(status), HAL_MAX_DELAY);
//	    	  }

	    	  // execute command for servo 2
//	    	  if (startLoop > (servo_2.last_instr_time + (servo_2.delay + DELAY))){
	    		  servo_2.delay = 0;
				  execute_input(&servo_2, command_buffer[1] );
//		    	  sprintf(status, "Servo %d is at position %d, running recipe %d at index %d\r\n", servo_2.servo_num, servo_2.servo_pos, servo_2.recipe_num, servo_2.recipe_index);
//				  HAL_UART_Transmit(&huart2, status, strlen(status), HAL_MAX_DELAY);
//			  }


	    	  is_command_ready = 0;
	    	  command_index = 0;
	      }

	  // execute next recipe step
	      if (servo_1.is_running){
	    	  if (startLoop > (servo_1.last_instr_time + (servo_1.delay + DELAY))){
	    		  servo_1.delay = 0;
	    		  if (execute_recipe_step(&servo_1)) {
					  sprintf((char*)status, "Servo %d is at position %d, running recipe %d at index %d\r\n", servo_1.servo_num, servo_1.servo_pos, servo_1.recipe_num, servo_1.recipe_index);
					  HAL_UART_Transmit(&huart2, status, strlen((char*)status), HAL_MAX_DELAY);
					  HAL_UART_Transmit(&huart2, newline, sizeof(newline), HAL_MAX_DELAY);
				  }
				  servo_1.last_instr_time = HAL_GetTick();
//				  sprintf(status, "Servo %d is at position %d, running recipe %d at index %d\r\n", servo_1.servo_num, servo_1.servo_pos, servo_1.recipe_num, servo_1.recipe_index);
//				  HAL_UART_Transmit(&huart2, status, strlen(status), HAL_MAX_DELAY);
	    	  }

	      }

	      if (servo_2.is_running){
	    	  if (startLoop > (servo_2.last_instr_time + (servo_2.delay + DELAY))){
	    		  servo_2.delay = 0;
				  if (execute_recipe_step(&servo_2)) {
					  sprintf((char*)status, "Servo %d is at position %d, running recipe %d at index %d\r\n", servo_2.servo_num, servo_2.servo_pos, servo_2.recipe_num, servo_2.recipe_index);
					  HAL_UART_Transmit(&huart2, status, strlen((char*)status), HAL_MAX_DELAY);
					  HAL_UART_Transmit(&huart2, newline, sizeof(newline), HAL_MAX_DELAY);
				  }
				  servo_2.last_instr_time = HAL_GetTick();
//
	    	  }


		  }

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// convert to uppercase
	uint8_t data = toupper(rx_buffer[rx_index]);
	_Bool reset = 0;

	// if X
	if (data == 'X') {
		// echo character
		HAL_UART_Transmit(huart, &data, 1, HAL_MAX_DELAY);
		// reset buffers
		command_index = 0;
		rx_index = 0;
		reset = 1;
		uint8_t response[] = "\r\nBuffer reset\r\n";
		HAL_UART_Transmit(&huart2, response, sizeof(response), HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2, newline, sizeof(newline), HAL_MAX_DELAY);
	}

	if (!reset) {

		// if command index < 2
		if (command_index < 2) {
			// echo character
			HAL_UART_Transmit(huart, &data, 1, HAL_MAX_DELAY);
			// add current rx position to command buffer
			command_buffer[command_index] = data;
			command_index++;
		}
		// else
		else {
			// if char is carriage return
			if (data == '\r') {
				// pass command
				is_command_ready = 1;
				// echo newline
				HAL_UART_Transmit(&huart2, newline, sizeof(newline), HAL_MAX_DELAY);
			}
			// else
					// don't add to command buffer, don't echo
		}

	}
	// increment rx_index
	if ((++rx_index) == 255) rx_index = 0;
	// if rx_index == 255 reset to 0
	// re-enable UART receieve
	HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_index], 1);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
