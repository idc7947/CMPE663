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
#include <ctype.h>
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */
typedef struct {
    char command[20];      // Name of command to be executed
    int channel;           // Either 1 or 2
    char type;             // Type of waveform
    double freq;           // Number of cycles per second, frequency of waveform
    float minv;
    float maxv;
    uint16_t noise;
} commandStruct;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.1415926
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;
DMA_HandleTypeDef hdma_dac_ch2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* Definitions for commandTask */
osThreadId_t commandTaskHandle;
const osThreadAttr_t commandTask_attributes = {
  .name = "commandTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DACTask01 */
osThreadId_t DACTask01Handle;
const osThreadAttr_t DACTask01_attributes = {
  .name = "DACTask01",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for DACTask02 */
osThreadId_t DACTask02Handle;
const osThreadAttr_t DACTask02_attributes = {
  .name = "DACTask02",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ADCTask */
osThreadId_t ADCTaskHandle;
const osThreadAttr_t ADCTask_attributes = {
  .name = "ADCTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for commandQueue */
osMessageQueueId_t commandQueueHandle;
uint8_t myQueue01Buffer[ 40 * sizeof( commandStruct ) ];
osStaticMessageQDef_t myQueue01ControlBlock;
const osMessageQueueAttr_t commandQueue_attributes = {
  .name = "commandQueue",
  .cb_mem = &myQueue01ControlBlock,
  .cb_size = sizeof(myQueue01ControlBlock),
  .mq_mem = &myQueue01Buffer,
  .mq_size = sizeof(myQueue01Buffer)
};
/* USER CODE BEGIN PV */
char command[20];     // name of command to be executed
int channel;	      // either 1 or 2
char type;			  // type of waveform
double freq;		  // num cycles per second, frequency of waveform
float minv;
float maxv;
uint16_t noise;
uint8_t rx_buffer[256];
uint8_t command_buffer[256];
uint8_t newline[3] = "\r\n>";
uint8_t rx_index = 0;
uint8_t command_index = 0;
_Bool is_command_ready = 0;
uint8_t status[100];



uint32_t dmaBuffer_1[256];
uint32_t dmaBuffer_2[256];
#define SAMPLES_BUFFER_SIZE 20000 // 2 seconds * 10 kHz
uint16_t samples_buffer[SAMPLES_BUFFER_SIZE];
uint32_t ekg1[] = {
	1690, 1680, 1680, 1669, 1648, 1648, 1648, 1680, 1680, 1690, 1680, 1680, 1680, 1680, 1680, 1658,
	1690, 1690, 1712, 1690, 1690, 1680, 1669, 1669, 1680, 1690, 1690, 1680, 1669, 1669, 1669, 1680,
	1680, 1680, 1669, 1669, 1680, 1690, 1690, 1680, 1690, 1690, 1680, 1690, 1690, 1712, 1680, 1680,
	1658, 1648, 1648, 1648, 1669, 1669, 1680, 1690, 1690, 1701, 1680, 1680, 1669, 1680, 1680, 1680,
	1701, 1701, 1701, 1690, 1690, 1701, 1712, 1712, 1722, 1712, 1712, 1690, 1669, 1669, 1680, 1690,
	1690, 1690, 1733, 1733, 1765, 1776, 1861, 1882, 1936, 1936, 1968, 1989, 1989, 2032, 2053, 2053,
	2085, 2149, 2069, 2080, 2058, 2058, 1930, 1930, 1845, 1824, 1792, 1872, 1840, 1754, 1754, 1722,
	1680, 1680, 1680, 1637, 1637, 1637, 1637, 1637, 1626, 1648, 1648, 1637, 1605, 1605, 1616, 1680,
	1680, 1765, 1776, 1861, 2042, 2106, 2021, 1776, 2480, 2400, 2176, 1632, 1637, 1360, 933, 928,
	1962, 1962, 2042, 2149, 3141, 3141, 2320, 1200, 1200, 1392, 1669, 1669, 1658, 1701, 1701, 1701,
	1701, 1701, 1722, 1690, 1690, 1690, 1680, 1680, 1690, 1690, 1690, 1669, 1669, 1669, 1701, 1733,
	1733, 1754, 1744, 1744, 1733, 1733, 1733, 1722, 1765, 1765, 1765, 1733, 1733, 1733, 1722, 1722,
	1701, 1690, 1690, 1701, 1690, 1690, 1701, 1701, 1701, 1701, 1722, 1722, 1712, 1722, 1722, 1733,
	1733, 1733, 1733, 1712, 1712, 1712, 1733, 1733, 1733, 1733, 1733, 1733, 1744, 1744, 1744, 1744,
	1744, 1744, 1733, 1733, 1722, 1722, 1722, 1722, 1722, 1722, 1733, 1722, 1722, 1722, 1722, 1722,
	1701, 1669, 1669, 1680, 1690, 1690, 1690, 1701, 1701, 1712, 1712, 1712, 1690, 1669, 1669, 1680,
};

uint32_t ekg2[] = {
	1690, 1680, 1680, 1669, 1648, 1648, 1648, 1680, 1680, 1690, 1680, 1680, 1680, 1680, 1680, 1658,
	1690, 1690, 1712, 1690, 1690, 1680, 1669, 1669, 1680, 1690, 1690, 1680, 1669, 1669, 1669, 1680,
	1680, 1680, 1669, 1669, 1680, 1690, 1690, 1680, 1690, 1690, 1680, 1690, 1690, 1712, 1680, 1680,
	1658, 1648, 1648, 1648, 1669, 1669, 1680, 1690, 1690, 1701, 1680, 1680, 1669, 1680, 1680, 1680,
	1701, 1701, 1701, 1690, 1690, 1701, 1712, 1712, 1722, 1712, 1712, 1690, 1669, 1669, 1680, 1690,
	1690, 1690, 1733, 1733, 1765, 1776, 1861, 1882, 1936, 1936, 1968, 1989, 1989, 2032, 2053, 2053,
	2085, 2149, 2069, 2080, 2058, 2058, 1930, 1930, 1845, 1824, 1792, 1872, 1840, 1754, 1754, 1722,
	1680, 1680, 1680, 1637, 1637, 1637, 1637, 1637, 1626, 1648, 1648, 1637, 1605, 1605, 1616, 1680,
	1680, 1765, 1776, 1861, 2042, 2106, 2021, 1776, 2480, 2400, 2176, 1632, 1637, 1360, 933, 928,
	1962, 1962, 2042, 2149, 3141, 3141, 2320, 1200, 1200, 1392, 1669, 1669, 1658, 1701, 1701, 1701,
	1701, 1701, 1722, 1690, 1690, 1690, 1680, 1680, 1690, 1690, 1690, 1669, 1669, 1669, 1701, 1733,
	1733, 1754, 1744, 1744, 1733, 1733, 1733, 1722, 1765, 1765, 1765, 1733, 1733, 1733, 1722, 1722,
	1701, 1690, 1690, 1701, 1690, 1690, 1701, 1701, 1701, 1701, 1722, 1722, 1712, 1722, 1722, 1733,
	1733, 1733, 1733, 1712, 1712, 1712, 1733, 1733, 1733, 1733, 1733, 1733, 1744, 1744, 1744, 1744,
	1744, 1744, 1733, 1733, 1722, 1722, 1722, 1722, 1722, 1722, 1733, 1722, 1722, 1722, 1722, 1722,
	1701, 1669, 1669, 1680, 1690, 1690, 1690, 1701, 1701, 1712, 1712, 1712, 1690, 1669, 1669, 1680,
};

_Bool DAC1ready;
_Bool DAC2ready;
_Bool ADCready;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
void commandLoop(void *argument);
void StartDAC01(void *argument);
void StartDAC02(void *argument);
void StartADC(void *argument);

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim5);
  HAL_TIM_Base_Start(&htim2);
//  TIM3->EGR |= TIM_EGR_UG;
//  HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, (uint32_t *)ekg1, 256, DAC_ALIGN_12B_R);
//  HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_2, (uint32_t *)ekg2, 256, DAC_ALIGN_12B_R);

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

  /* Create the queue(s) */
  /* creation of commandQueue */
  commandQueueHandle = osMessageQueueNew (40, sizeof(commandStruct), &commandQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of commandTask */
  commandTaskHandle = osThreadNew(commandLoop, NULL, &commandTask_attributes);

  /* creation of DACTask01 */
  DACTask01Handle = osThreadNew(StartDAC01, NULL, &DACTask01_attributes);

  /* creation of DACTask02 */
  DACTask02Handle = osThreadNew(StartDAC02, NULL, &DACTask02_attributes);

  /* creation of ADCTask */
  ADCTaskHandle = osThreadNew(StartADC, NULL, &ADCTask_attributes);

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
  RCC_OscInitStruct.PLL.PLLN = 10;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T4_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T5_TRGO;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 159;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 49;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void uart_transmit(char* string) {
    HAL_UART_Transmit(&huart2, (uint8_t*)string, strlen(string), HAL_MAX_DELAY);
}

void uart_transmit_line(char* string) {
    char newline[] = "\r\n";
    uart_transmit(string);
    uart_transmit(newline);
}

void clear_screen() {
    char clear[] = "\033[2J\033[;H";
    uart_transmit(clear);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	char u_command[20];     // name of command to be executed
	int u_channel;	      // either 1 or 2
	char u_type;			  // type of waveform
	double u_freq;		  // num cycles per second, frequency of waveform
	float u_minv;
	float u_maxv;
	uint16_t u_noise;
	// convert to uppercase
	uint8_t data = rx_buffer[rx_index];
	HAL_UART_Transmit(huart, &data, 1, HAL_MAX_DELAY);
	command_buffer[command_index] = data;
	command_index++;
	if (data == '\r') {
		// pass command
		command_index = 0;
		   if (
			sscanf( command_buffer, "%s %d %c %lf %f %f %hu",
					u_command,
					&u_channel,
					&u_type,
					&u_freq,
					&u_minv,
					&u_maxv,
					&u_noise ) > 0
				) {
			   is_command_ready = 1;
			   commandStruct myCommand = {
			       .command = u_command,
			       .channel = u_channel,
			       .type = u_type, // For example, 'S' could represent a sine wave
			       .freq = u_freq,
			       .minv = u_minv,
			       .maxv = u_maxv,
			       .noise = u_noise
			   };
			   strcpy(myCommand.command, u_command);
			   osMessageQueuePut(commandQueueHandle, &myCommand, 0, 0);
		   }
		memset(command_buffer,0,sizeof(command_buffer)); // clear buffer
		// echo newline
		HAL_UART_Transmit(&huart2, newline, sizeof(newline), HAL_MAX_DELAY);
	}
	// increment rx_index
	if ((++rx_index) == 255) rx_index = 0;
	// if rx_index == 255 reset to 0
	// re-enable UART receieve
	HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_index], 1);
}

void process_commands(void) {


	if (strcmp(command, "gen") == 0) { // do same for other commands
		switch(channel) {
		case 1:
			DAC1ready = 1;
			break;
		case 2:
			DAC2ready = 1;
			break;
		default:
			break;
		}


	}
	if (strcmp(command, "cap") == 0) { // do same for other commands
		ADCready = 1;
	}
}

void sin_gen(uint16_t chan, float minv, float maxv) {
	float digital_conv = 4096 / 3.3 * (maxv - minv);
	for (int i = 0; i < 256; i++) {
		if (chan == 1){
			dmaBuffer_1[i] = round(((sin(i*2*PI/256) + 1)*(digital_conv/2)) + 4096 / 3.3 * minv);
		}
		if (chan == 2) {
			dmaBuffer_2[i] = round(((sin(i*2*PI/256) + 1)*(digital_conv/2)) + 4096 / 3.3 * minv);
		}
	}

}

void ekg_gen(uint16_t chan, float minv, float maxv) {
	for (int i = 0; i < 256; i++) {
		if (chan == 1) dmaBuffer_1[i] = ekg1[i];
		if (chan == 2) dmaBuffer_2[i] = ekg1[i];
	}

}

void square_gen(uint16_t chan, float minv, float maxv) {
	for (int i = 0; i < 128; i++) {
		if (chan == 1){
			dmaBuffer_1[i] = ((float)(4096/3.3) * (float) minv);
			dmaBuffer_1[i + 127] = ((float)(4096/3.3) * (float) maxv);
		}
		if (chan == 2){
			dmaBuffer_2[i] = ((float)(4096/3.3) * (float) minv);
			dmaBuffer_2[i + 127] = ((float)(4096/3.3) * (float) maxv);
		}
	}

}

void triangle_gen(uint16_t chan, float minv, float maxv) {
	uint32_t digMax = ((float)(4096/3.3) * (float) maxv);
	uint32_t digMin = ((float)(4096/3.3) * (float) minv);
	double incr = ((double)digMax - (double)digMin) / 128;
	for (int i = 0; i < 128; i++) {
		if (chan == 1){
			dmaBuffer_1[i] = (uint32_t) ((double)digMin + (double)(incr * i));
			dmaBuffer_1[i + 127] = (uint32_t) ((double)digMax - (double)(incr * i));
			dmaBuffer_1[255] = (uint32_t) (digMin);
		}
		if (chan == 2){
			dmaBuffer_2[i] = (uint32_t) ((double)digMin + (double)(incr * i));
			dmaBuffer_2[i + 127] = (uint32_t) ((double)digMax - (double)(incr * i));
			dmaBuffer_2[255] = (uint32_t) (digMin);
		}
	}

}

void sawtooth_gen(uint16_t chan, float minv, float maxv) {
	uint32_t digMax = ((float)(4096/3.3) * (float) maxv);
	uint32_t digMin = ((float)(4096/3.3) * (float) minv);
	double incr = (digMax - digMin) / 512;
	for (int i = 0; i < 256; i++) {
		if (chan == 1){
			dmaBuffer_1[i] = (uint32_t) ((double)digMin + (double)(incr * i));
			dmaBuffer_1[i + 127] = (uint32_t) ((double)digMax - (double)(incr * i));
		}
		if (chan == 2){
			dmaBuffer_2[i] = (uint32_t) ((double)digMin + (double)(incr * i));
			dmaBuffer_2[i + 127] = (uint32_t) ((double)digMax - (double)(incr * i));
		}
	}

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_commandLoop */
/**
  * @brief  Function implementing the commandTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_commandLoop */
void commandLoop(void *argument)
{
  /* USER CODE BEGIN 5 */
	HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_index], 1);
  /* Infinite loop */
  for(;;)
  {
	  if (is_command_ready) {
		  commandStruct retrieved;
		  osMessageQueueGet(commandQueueHandle, &retrieved, NULL, 0);
		  strcpy(command, retrieved.command);
//		  command = retrieved.command;     // name of command to be executed
		  channel = retrieved.channel;	      // either 1 or 2
		  type = retrieved.type;			  // type of waveform
		  freq = retrieved.freq;		  // num cycles per second, frequency of waveform
		  minv = retrieved.minv;
		  maxv = retrieved.maxv;
		  noise = retrieved.noise;
		  	 process_commands();
		  	 is_command_ready = 0;
	  }
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartDAC01 */
/**
* @brief Function implementing the DACTask01 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDAC01 */
void StartDAC01(void *argument)
{
  /* USER CODE BEGIN StartDAC01 */
	DAC1ready = 0;
  /* Infinite loop */
  for(;;)
  {
	  if (DAC1ready) {
		  if (freq < 0.001) {
			  for (int i = 0; i < 255; i++) {
				  dmaBuffer_1[i] = (uint32_t) ((float)(4096/3.3) * (float) minv);
			  }
		  }
		  else {
			  uint32_t amplitude_noise;
			  switch(noise) {
			  case 1:
				  amplitude_noise = DAC_LFSRUNMASK_BITS1_0;
				  break;
			  case 2:
				  amplitude_noise = DAC_LFSRUNMASK_BITS2_0;
				  break;
			  case 3:
				  amplitude_noise = DAC_LFSRUNMASK_BITS3_0;
				  break;
			  case 4:
				  amplitude_noise = DAC_LFSRUNMASK_BITS4_0;
				  break;
			  case 5:
				  amplitude_noise = DAC_LFSRUNMASK_BITS5_0;
				  break;
			  case 6:
				  amplitude_noise = DAC_LFSRUNMASK_BITS6_0;
				  break;
			  case 7:
				  amplitude_noise = DAC_LFSRUNMASK_BITS7_0;
				  break;
			  case 8:
				  amplitude_noise = DAC_LFSRUNMASK_BITS8_0;
				  break;
			  case 9:
				  amplitude_noise = DAC_LFSRUNMASK_BITS9_0;
				  break;
			  case 10:
				  amplitude_noise = DAC_LFSRUNMASK_BITS10_0;
				  break;
			  case 11:
				  amplitude_noise = DAC_LFSRUNMASK_BITS11_0;
				  break;
			  default:
				  amplitude_noise = DAC_LFSRUNMASK_BIT0;
				  break;
			  }
			  HAL_DAC_Stop_DMA(&hdac1, DAC1_CHANNEL_1);
			  HAL_DACEx_NoiseWaveGenerate(&hdac1, DAC1_CHANNEL_1, amplitude_noise);
			  TIM2->ARR = round(312500/ freq);
			  TIM2->EGR |= TIM_EGR_UG;
			  switch(type) {
			  case 's':
				sin_gen(channel, minv, maxv);
				HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, dmaBuffer_1, 256, DAC_ALIGN_12B_R);
				  break;
			  case 'r':
				square_gen(channel, minv, maxv);
				HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, dmaBuffer_1, 256, DAC_ALIGN_12B_R);
				  break;
			  case 't':
				triangle_gen(channel, minv, maxv);
				HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, dmaBuffer_1, 256, DAC_ALIGN_12B_R);
				  break;
			  case 'a':
				  ekg_gen(channel, minv, maxv);
				  HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, dmaBuffer_1, 256, DAC_ALIGN_12B_R);
				  break;
			  case 'c':
				  TIM2->ARR = round((312500 / freq) * 12.5);
				  TIM2->EGR |= TIM_EGR_UG;
				  HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, (uint32_t *)samples_buffer, SAMPLES_BUFFER_SIZE / 2, DAC_ALIGN_12B_R);
				  break;
			  default:
				  break;
			  }
		  }


		  clear_screen();

		  char buffer[50];

		  sprintf(buffer, "Shape: %c", type);
		  uart_transmit_line(buffer);
		  // Display raw frequency and dac frequency
		  sprintf(buffer, "Raw frequency: %.2f Hz", freq);
		  uart_transmit_line(buffer);
		  if (freq != 0) {
			  sprintf(buffer, "DAC frequency: %.2f", 312500.0f / freq);
			  uart_transmit_line(buffer);
		  }
		  uart_transmit_line("----------------------");

		  // Display number of samples
		  sprintf(buffer, "Number of samples: %d", 256);
		  uart_transmit_line(buffer);
		  uart_transmit_line("----------------------");

		  // Display voltage ranges
		  sprintf(buffer, "Voltage Range: %.2f V - %.2f V", minv, maxv);
		  uart_transmit_line(buffer);
		  sprintf(buffer, "DAC Voltage Range: %.2f V - %.2f", ((4096.0f/3.3f) * minv), ((4096.0f/3.3f) * maxv));
		  uart_transmit_line(buffer);
		  uart_transmit_line("----------------------");
		  DAC1ready = 0;


	  }
    osDelay(1);
  }
  /* USER CODE END StartDAC01 */
}

/* USER CODE BEGIN Header_StartDAC02 */
/**
* @brief Function implementing the DACTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDAC02 */
void StartDAC02(void *argument)
{
  /* USER CODE BEGIN StartDAC02 */
	DAC2ready = 0;
  /* Infinite loop */
  for(;;)
  {
	  if (DAC2ready) {
		  commandStruct retrieved;
		  osMessageQueueGet(commandQueueHandle, &retrieved, NULL, 0);
		  if (freq < 0.001) {
			  for (int i = 0; i < 255; i++) {
				  dmaBuffer_2[i] = (uint32_t) ((float)(4096/3.3) * (float) minv);
			  }
		  }
		  else {
			  uint32_t amplitude_noise;
			  switch(noise) {
			  case 1:
				  amplitude_noise = DAC_LFSRUNMASK_BITS1_0;
				  break;
			  case 2:
				  amplitude_noise = DAC_LFSRUNMASK_BITS2_0;
				  break;
			  case 3:
				  amplitude_noise = DAC_LFSRUNMASK_BITS3_0;
				  break;
			  case 4:
				  amplitude_noise = DAC_LFSRUNMASK_BITS4_0;
				  break;
			  case 5:
				  amplitude_noise = DAC_LFSRUNMASK_BITS5_0;
				  break;
			  case 6:
				  amplitude_noise = DAC_LFSRUNMASK_BITS6_0;
				  break;
			  case 7:
				  amplitude_noise = DAC_LFSRUNMASK_BITS7_0;
				  break;
			  case 8:
				  amplitude_noise = DAC_LFSRUNMASK_BITS8_0;
				  break;
			  case 9:
				  amplitude_noise = DAC_LFSRUNMASK_BITS9_0;
				  break;
			  case 10:
				  amplitude_noise = DAC_LFSRUNMASK_BITS10_0;
				  break;
			  case 11:
				  amplitude_noise = DAC_LFSRUNMASK_BITS11_0;
				  break;
			  default:
				  amplitude_noise = DAC_LFSRUNMASK_BIT0;
				  break;
			  }
			  HAL_DACEx_NoiseWaveGenerate(&hdac1, DAC1_CHANNEL_2, amplitude_noise);
		  	  TIM5->ARR = round(312500 / freq);
			  TIM5->EGR |= TIM_EGR_UG;
	  		  HAL_DAC_Stop_DMA(&hdac1, DAC1_CHANNEL_2);
	  		  switch(type) {
	  		  case 's':
	  			sin_gen(channel, minv, maxv);
	  			HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_2, dmaBuffer_2, 256, DAC_ALIGN_12B_R);
	  			  break;
	  		  case 'r':
	  			square_gen(channel, minv, maxv);
	  			HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_2, dmaBuffer_2, 256, DAC_ALIGN_12B_R);
	  			  break;
	  		  case 't':
	  			triangle_gen(channel, minv, maxv);
	  			HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_2, dmaBuffer_2, 256, DAC_ALIGN_12B_R);
	  			  break;
	  		  case 'a':
	  			  ekg_gen(channel, minv, maxv);
	  			HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_2, dmaBuffer_2, 256, DAC_ALIGN_12B_R);
	  			  break;
	  		  case 'c':
	  			  TIM5->ARR = round((312500 / freq) * 12.5);
	  			  TIM5->EGR |= TIM_EGR_UG;
				  HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_2, (uint32_t *)samples_buffer, SAMPLES_BUFFER_SIZE / 2, DAC_ALIGN_12B_R);
				  break;
	  		  default:
	  			  break;
	  		  }
		  }


	  		clear_screen();

			char buffer[50];
			sprintf(buffer, "Shape: %c", type);
			uart_transmit_line(buffer);
			// Display raw frequency and dac frequency
			sprintf(buffer, "Raw frequency: %.2f Hz", freq);
			uart_transmit_line(buffer);
			if (freq != 0) {
			  sprintf(buffer, "DAC frequency: %.2f", 312500.0f / freq);
			  uart_transmit_line(buffer);
		  }
			uart_transmit_line("----------------------");

			// Display number of samples
			sprintf(buffer, "Number of samples: %d", 256);
			uart_transmit_line(buffer);
			uart_transmit_line("----------------------");

			// Display voltage ranges
			sprintf(buffer, "Voltage Range: %.2f V - %.2f V", minv, maxv);
			uart_transmit_line(buffer);
			sprintf(buffer, "DAC Voltage Range: %.2f V - %.2f", ((4096.0f/3.3f) * minv), ((4096.0f/3.3f) * maxv));
			uart_transmit_line(buffer);
			uart_transmit_line("----------------------");
	  		  DAC2ready = 0;


	  	  }
    osDelay(1);
  }
  /* USER CODE END StartDAC02 */
}

/* USER CODE BEGIN Header_StartADC */
/**
* @brief Function implementing the ADCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartADC */
void StartADC(void *argument)
{
  /* USER CODE BEGIN StartADC */
	uint16_t min_voltage, max_voltage;
	char buffer[50];
  /* Infinite loop */
  for(;;)
  {
	if (ADCready) {
		// Start the timer to trigger the ADC
		HAL_TIM_Base_Start(&htim4);

		// Start ADC in DMA mode
		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)samples_buffer, SAMPLES_BUFFER_SIZE);

		// Wait for 2 seconds to collect samples
		osDelay(2000);

		// Stop the timer and ADC
		HAL_TIM_Base_Stop(&htim4);
		HAL_ADC_Stop_DMA(&hadc1);
		min_voltage = 4096;
		max_voltage = 0;

		for (int i = 1; i < SAMPLES_BUFFER_SIZE; i++) {
		  if (samples_buffer[i] < min_voltage) {
			  if ((i < 19900) && (i > 100)) min_voltage = samples_buffer[i];
		  } else if (samples_buffer[i] > max_voltage) {
			max_voltage = samples_buffer[i];
		  }
		}

		// Convert ADC values to voltage and print
		float min_voltage_volts = (min_voltage * 3.3) / 4096;
		float max_voltage_volts = (max_voltage * 3.3) / 4096;
//		clear_screen();
		sprintf(buffer, "DAC Voltage Range: %d - %d", min_voltage, max_voltage);
		uart_transmit_line(buffer);
		sprintf(buffer, "Voltage Range: %.2f V - %.2f V", min_voltage_volts, max_voltage_volts);
		uart_transmit_line(buffer);
		printf("Min voltage: %.2f V\n", min_voltage_volts);
		printf("Max voltage: %.2f V\n", max_voltage_volts);
		ADCready = 0;
	}
    osDelay(1);
  }
  /* USER CODE END StartADC */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM8 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM8) {
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
