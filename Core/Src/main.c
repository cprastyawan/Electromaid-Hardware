/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "dwt_delay.h"

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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
ADC_HandleTypeDef hadc4;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;
DMA_HandleTypeDef hdma_adc4;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* Definitions for ADC1Task */
osThreadId_t ADC1TaskHandle;
const osThreadAttr_t ADC1Task_attributes = {
  .name = "ADC1Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for sendPowerTask */
osThreadId_t sendPowerTaskHandle;
const osThreadAttr_t sendPowerTask_attributes = {
  .name = "sendPowerTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for processDataTask */
osThreadId_t processDataTaskHandle;
const osThreadAttr_t processDataTask_attributes = {
  .name = "processDataTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for ADC2Task */
osThreadId_t ADC2TaskHandle;
const osThreadAttr_t ADC2Task_attributes = {
  .name = "ADC2Task",
  .priority = (osPriority_t) osPriorityNormal1,
  .stack_size = 128 * 4
};
/* Definitions for ADC3Task */
osThreadId_t ADC3TaskHandle;
const osThreadAttr_t ADC3Task_attributes = {
  .name = "ADC3Task",
  .priority = (osPriority_t) osPriorityNormal2,
  .stack_size = 128 * 4
};
/* Definitions for ADC4Task */
osThreadId_t ADC4TaskHandle;
const osThreadAttr_t ADC4Task_attributes = {
  .name = "ADC4Task",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};
/* Definitions for ADC1Event */
osEventFlagsId_t ADC1EventHandle;
const osEventFlagsAttr_t ADC1Event_attributes = {
  .name = "ADC1Event"
};
/* Definitions for ADC2Event */
osEventFlagsId_t ADC2EventHandle;
const osEventFlagsAttr_t ADC2Event_attributes = {
  .name = "ADC2Event"
};
/* Definitions for ADC3Event */
osEventFlagsId_t ADC3EventHandle;
const osEventFlagsAttr_t ADC3Event_attributes = {
  .name = "ADC3Event"
};
/* Definitions for ADC4Event */
osEventFlagsId_t ADC4EventHandle;
const osEventFlagsAttr_t ADC4Event_attributes = {
  .name = "ADC4Event"
};
/* Definitions for UARTReceiveEvent */
osEventFlagsId_t UARTReceiveEventHandle;
const osEventFlagsAttr_t UARTReceiveEvent_attributes = {
  .name = "UARTReceiveEvent"
};
/* USER CODE BEGIN PV */
module_t module[10];
uint8_t buffer[200];
uint16_t strSize;

//KONEKSI ESP32
uint8_t dma_rx_buf[DMA_BUF_SIZE];
uint8_t data[DMA_BUF_SIZE] = {'\0'};
bool OKToStart = false;
DMA_Event_t dma_uart_rx = {0, 0, DMA_BUF_SIZE};

//ADC Buffer
volatile uint16_t adc1Buffer[FILTER_BLOCK_SIZE * 2 * 4];
volatile uint16_t adc2Buffer[FILTER_BLOCK_SIZE * 2 * 4];
volatile uint16_t adc3Buffer[FILTER_BLOCK_SIZE * 2 * 4];
volatile uint16_t adc4Buffer[FILTER_BLOCK_SIZE * 2];

uint8_t adc1State = 0;
uint8_t adc2State = 0;
uint8_t adc3State = 0;
uint8_t adc4State = 0;

//Filter
const float filter_coeff[5] = { 0.00005808217093870561,
		0.00011616434187741122,
		0.00005808217093870561,
		1.9783280315766534,
		-0.9785603602604083};

float filter_input_adc1[FILTER_BLOCK_SIZE * 2];
float filter_input_adc2[FILTER_BLOCK_SIZE * 2];
float filter_input_adc3[FILTER_BLOCK_SIZE * 2];

float filter_output_adc1[FILTER_BLOCK_SIZE * 2];
float filter_output_adc2[FILTER_BLOCK_SIZE * 2];
float filter_output_adc3[FILTER_BLOCK_SIZE * 2];

//Testing
uint32_t test;
uint32_t prevTest;
uint32_t deltaTest;

//Volt Meter
arm_biquad_casd_df1_inst_f32 filter_voltMeter;
float pStateVoltMeter[4];
float filter_input_adc4[FILTER_BLOCK_SIZE * 2];
float voltMeterFilterOutput[FILTER_BLOCK_SIZE * 2];
uint8_t voltMeterSend[5] = {'#','V','\0','\0','S'};
uint8_t voltRMSSend[7] = {'#','R','\0','\0','\0','\0','S'};
Floating voltMeterValue;
Floating voltRMS;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
void StartADC1Task(void *argument);
void StartSendPowerTask(void *argument);
void StartProcessDataTask(void *argument);
void StartADC2Task(void *argument);
void StartADC3Task(void *argument);
void StartADC4Task(void *argument);

/* USER CODE BEGIN PFP */
void module_init(){
	module[0].GPIOPin = D1_Pin;
	module[1].GPIOPin = D2_Pin;
	module[2].GPIOPin = D3_Pin;
	module[3].GPIOPin = D4_Pin;
	module[4].GPIOPin = D5_Pin;
	module[5].GPIOPin = D6_Pin;
	module[6].GPIOPin = D7_Pin;
	module[7].GPIOPin = D8_Pin;
	module[8].GPIOPin = D9_Pin;
	module[9].GPIOPin = D10_Pin;

	module[0].GPIOPort = D1_GPIO_Port;
	module[1].GPIOPort = D2_GPIO_Port;
	module[2].GPIOPort = D3_GPIO_Port;
	module[3].GPIOPort = D4_GPIO_Port;
	module[4].GPIOPort = D5_GPIO_Port;
	module[5].GPIOPort = D6_GPIO_Port;
	module[6].GPIOPort = D7_GPIO_Port;
	module[7].GPIOPort = D8_GPIO_Port;
	module[8].GPIOPort = D9_GPIO_Port;
	module[9].GPIOPort = D10_GPIO_Port;


	for(int i = 0; i < 10; i++){
		sprintf((char*)module[i].data_send, "#A%d", i);
		module[i].data_send[7] = 'S';

		module[i].status = false;
		module[i].aktif = true;
		module[i].midPoint = 0;
		module[i].midPointSet = false;

		arm_biquad_cascade_df1_init_f32(&module[i].filter_inst, 1, filter_coeff, module[i].pState);

	}
}


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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_ADC4_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Start_IT(&htim6);
  HAL_Delay(2000);

  module_init();
  DWT_Init();
  HAL_TIM_Base_Start_IT(&htim2);

  arm_biquad_cascade_df1_init_f32(&filter_voltMeter, 1, filter_coeff, pStateVoltMeter);

  strSize = sprintf((char*)buffer, "Test123\r\n");
  HAL_UART_Transmit(&huart1, buffer, strSize, 100);
  HAL_UART_Transmit(&huart3, buffer, strSize, 100);

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
  /* creation of ADC1Task */
  ADC1TaskHandle = osThreadNew(StartADC1Task, NULL, &ADC1Task_attributes);

  /* creation of sendPowerTask */
  sendPowerTaskHandle = osThreadNew(StartSendPowerTask, NULL, &sendPowerTask_attributes);

  /* creation of processDataTask */
  processDataTaskHandle = osThreadNew(StartProcessDataTask, NULL, &processDataTask_attributes);

  /* creation of ADC2Task */
  ADC2TaskHandle = osThreadNew(StartADC2Task, NULL, &ADC2Task_attributes);

  /* creation of ADC3Task */
  ADC3TaskHandle = osThreadNew(StartADC3Task, NULL, &ADC3Task_attributes);

  /* creation of ADC4Task */
  ADC4TaskHandle = osThreadNew(StartADC4Task, NULL, &ADC4Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of ADC1Event */
  ADC1EventHandle = osEventFlagsNew(&ADC1Event_attributes);

  /* creation of ADC2Event */
  ADC2EventHandle = osEventFlagsNew(&ADC2Event_attributes);

  /* creation of ADC3Event */
  ADC3EventHandle = osEventFlagsNew(&ADC3Event_attributes);

  /* creation of ADC4Event */
  ADC4EventHandle = osEventFlagsNew(&ADC4Event_attributes);

  /* creation of UARTReceiveEvent */
  UARTReceiveEventHandle = osEventFlagsNew(&UARTReceiveEvent_attributes);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
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
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 4;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 4;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief ADC4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC4_Init(void)
{

  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */
  /** Common config
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc4.Init.ContinuousConvMode = ENABLE;
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.NbrOfConversion = 1;
  hadc4.Init.DMAContinuousRequests = ENABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC4_Init 2 */

  /* USER CODE END ADC4_Init 2 */

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
  htim2.Init.Prescaler = 72 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 57600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  /* DMA2_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D10_Pin|D9_Pin|D8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D7_Pin|D6_Pin|D5_Pin|D4_Pin
                          |D3_Pin|D2_Pin|D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : D10_Pin D9_Pin D8_Pin */
  GPIO_InitStruct.Pin = D10_Pin|D9_Pin|D8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D7_Pin D6_Pin D5_Pin D4_Pin
                           D3_Pin D2_Pin D1_Pin */
  GPIO_InitStruct.Pin = D7_Pin|D6_Pin|D5_Pin|D4_Pin
                          |D3_Pin|D2_Pin|D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	if(hadc->Instance == ADC1){
		osEventFlagsSet(ADC1EventHandle, 1);
	} else if(hadc->Instance == ADC2){
		osEventFlagsSet(ADC2EventHandle, 1);
	} else if(hadc->Instance == ADC3){
		osEventFlagsSet(ADC3EventHandle, 1);
	} else if(hadc->Instance == ADC4){
		osEventFlagsSet(ADC4EventHandle, 1);
	}
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc){
		/*if(hadc->Instance == ADC1){
			adc1State = 1;
		} else if(hadc->Instance == ADC2){
			adc2State = 1;
		} else if(hadc->Instance == ADC3){
			adc3State = 1;
		} else if(hadc->Instance == ADC4){
			adc4State = 1;
		}*/
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART3){
		static int length;
		static bool captureSerial;
		static bool stringComplete;
		static uint16_t rxChar;

		rxChar = USART3->RDR;

		if(rxChar == '#' && !captureSerial){
			captureSerial = true;
		} else if(rxChar == 'S' && captureSerial){
			stringComplete = true;
		}

		if(captureSerial){
			data[length] = rxChar;
			length++;
		}

		if(stringComplete){
			captureSerial = false;
		    stringComplete = false;
		    length = 0;
		    osEventFlagsSet(UARTReceiveEventHandle, 1);
		}
		HAL_UART_Receive_IT(&huart3, dma_rx_buf, 1);
	    /*uint16_t i, pos, start, length;
	    uint16_t currCNDTR = __HAL_DMA_GET_COUNTER(huart3.hdmarx);

	    // Ignore IDLE Timeout when the received characters exactly filled up the DMA buffer and DMA Rx Complete IT is generated, but there is no new character during timeout
	    if(dma_uart_rx.flag && currCNDTR == DMA_BUF_SIZE)
	    {
	        dma_uart_rx.flag = 0;
	        return;
	    }

	    // Determine start position in DMA buffer based on previous CNDTR value
	    start = (dma_uart_rx.prevCNDTR < DMA_BUF_SIZE) ? (DMA_BUF_SIZE - dma_uart_rx.prevCNDTR) : 0;

	    if(dma_uart_rx.flag)    //TimeOutEvent
	    {
	        /* Determine new data length based on previous DMA_CNDTR value:
	         *  If previous CNDTR is less than DMA buffer size: there is old data in DMA buffer (from previous timeout) that has to be ignored.
	         *  If CNDTR == DMA buffer size: entire buffer content is new and has to be processed.

	        length = (dma_uart_rx.prevCNDTR < DMA_BUF_SIZE) ? (dma_uart_rx.prevCNDTR - currCNDTR) : (DMA_BUF_SIZE - currCNDTR);
	        dma_uart_rx.prevCNDTR = currCNDTR;
	        dma_uart_rx.flag = 0;
	    }
	    else                /* DMA Rx Complete event
	    {
	        length = DMA_BUF_SIZE - start;
	        dma_uart_rx.prevCNDTR = DMA_BUF_SIZE;
	    }

	    /* Copy and Process new data
	    for(i=0,pos=start; i<length; ++i,++pos)
	    {
	        data[i] = dma_rx_buf[pos];
	    }*/
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartADC1Task */
/**
  * @brief  Function implementing the ADC1Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartADC1Task */
void StartADC1Task(void *argument)
{
  /* USER CODE BEGIN 5 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1Buffer, FILTER_BLOCK_SIZE * 2 * 4);
  /* Infinite loop */
  for(;;)
  {
	  osEventFlagsWait(ADC1EventHandle, 1, osFlagsWaitAny, osWaitForever);
	  for(int i = 0; i < 4; i++){
		for(int j = 0; j < FILTER_BLOCK_SIZE * 2; j++){
			filter_input_adc1[j] = (float)(((float)adc1Buffer[i + (4 * j)] - 2080) / (float)4095) * 3.30;
		}

		arm_biquad_cascade_df1_f32(&module[i].filter_inst, filter_input_adc1, filter_output_adc1, FILTER_BLOCK_SIZE * 2);
		arm_rms_f32(filter_output_adc1, FILTER_BLOCK_SIZE * 2, &module[i].rms.f);
		module[i].totalAmp += module[i].rms.f;
		module[i].samples++;
		module[i].Amp.f = module[i].totalAmp / module[i].samples;
		if(module[i].status == false || module[i].aktif == false){
			module[i].Power.f = 0;
		}else{
			module[i].Power.f = (module[i].Amp.f * 220.0);
		}
	  }
	  adc1State = 0;
	  osEventFlagsClear(ADC1EventHandle, 1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSendPowerTask */
/**
* @brief Function implementing the sendPowerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSendPowerTask */
void StartSendPowerTask(void *argument)
{
  /* USER CODE BEGIN StartSendPowerTask */
  /* Infinite loop */
  for(;;)
  {
	  for(int i = 0; i < 10; i++){
		memcpy(module[i].data_send + 3, module[i].Power.ui8, 4);
		memcpy(buffer + (i * 8), module[i].data_send, 8);
		module[i].totalAmp = 0;
		module[i].samples = 0;
		//HAL_UART_Transmit_IT(&huart3, module[i].data_send, 8);
	  }
	  HAL_UART_Transmit_IT(&huart3, buffer, 80);
	  osDelay(1000);
  }
  /* USER CODE END StartSendPowerTask */
}

/* USER CODE BEGIN Header_StartProcessDataTask */
/**
* @brief Function implementing the processDataTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartProcessDataTask */
void StartProcessDataTask(void *argument)
{
  /* USER CODE BEGIN StartProcessDataTask */
	//HAL_UART_Receive_DMA(&huart3, dma_rx_buf, DMA_BUF_SIZE);
	//__HAL_UART_CLEAR_IDLEFLAG(&huart3);
	//__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

	//__HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
	HAL_UART_Receive_IT(&huart3, dma_rx_buf, 1);
  /* Infinite loop */
  for(;;)
  {
	osEventFlagsWait(UARTReceiveEventHandle, 1, osFlagsWaitAny, osWaitForever);
	if(data[0] == '#' && data[4] == 'S'){
		if(data[1] == 'D'){
			module[data[2] - 48].status = data[3];
			HAL_GPIO_WritePin(module[data[2] - 48].GPIOPort, module[data[2] - 48].GPIOPin, module[data[2] - 48].status);
			//HAL_UART_Transmit_IT(&huart3, (uint8_t*)"#RECVS", 6);
		}
		else if(data[1] == 'Q'){
			//module[data[2] - 48].aktif = data[3];
			//HAL_UART_Transmit_IT(&huart3, (uint8_t*)"#RECVS", 6);
		}

	}
    osEventFlagsClear(UARTReceiveEventHandle, 1);
  }
  /* USER CODE END StartProcessDataTask */
}

/* USER CODE BEGIN Header_StartADC2Task */
/**
* @brief Function implementing the ADC2Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartADC2Task */
void StartADC2Task(void *argument)
{
  /* USER CODE BEGIN StartADC2Task */
	  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2Buffer, FILTER_BLOCK_SIZE * 2 * 4);
  /* Infinite loop */
  for(;;)
  {
	 osEventFlagsWait(ADC2EventHandle, 1, osFlagsWaitAny, osWaitForever);
	 for(int i = 0; i < 4; i++){
		for(int j = 0; j < FILTER_BLOCK_SIZE * 2; j++){
		//adcBuffer[j] = adc1Buffer[i + (4 * j)];
		filter_input_adc2[j] = (float)(((float)adc2Buffer[i + (4 * j)] - (float)2080) / (float)4095) * 3.30;
		}

		arm_biquad_cascade_df1_f32(&module[i + 4].filter_inst, filter_input_adc2, filter_output_adc2, FILTER_BLOCK_SIZE * 2);
		arm_rms_f32(filter_output_adc2, FILTER_BLOCK_SIZE * 2, &module[i + 4].rms.f);
		module[i + 4].totalAmp += module[i + 4].rms.f;
		module[i + 4].samples++;
		module[i + 4].Amp.f = module[i + 4].totalAmp / module[i + 4].samples;
		if(module[i + 4].status == false || module[i + 4].aktif == false){
			module[i + 4].Power.f = 0;
		} else{
			module[i + 4].Power.f = (module[i + 4].Amp.f * 220.0);
		}
	}
	osEventFlagsClear(ADC2EventHandle, 1);
  }
  /* USER CODE END StartADC2Task */
}

/* USER CODE BEGIN Header_StartADC3Task */
/**
* @brief Function implementing the ADC3Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartADC3Task */
void StartADC3Task(void *argument)
{
  /* USER CODE BEGIN StartADC3Task */
	  HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3Buffer, FILTER_BLOCK_SIZE * 2 * 4);
  /* Infinite loop */
  for(;;)
  {
	 osEventFlagsWait(ADC3EventHandle, 1, osFlagsWaitAny, osWaitForever);
	 for(int i = 0; i < 2; i++){
		for(int j = 0; j < FILTER_BLOCK_SIZE * 2; j++){
			//adcBuffer[j] = adc1Buffer[i + (4 * j)];
			filter_input_adc3[j] = (float)(((float)adc3Buffer[i + (4 * j)] - (float)2080) / (float)4095) * 3.30;
		}

		arm_biquad_cascade_df1_f32(&module[i + 8].filter_inst, filter_input_adc3, filter_output_adc3, FILTER_BLOCK_SIZE * 2);
		arm_rms_f32(filter_output_adc3, FILTER_BLOCK_SIZE * 2, &module[i + 8].rms.f);
		module[i + 8].totalAmp += module[i + 8].rms.f;
		module[i + 8].samples++;
		module[i + 8].Amp.f = module[i + 8].totalAmp / module[i + 8].samples;
		if(module[i + 8].status == false || module[i + 8].aktif == false){
			module[i + 8].Power.f = 0;
		} else{
			module[i + 8].Power.f = (module[i + 8].Amp.f * 220.0);
		}
	}
	osEventFlagsClear(ADC3EventHandle, 1);
  }
  /* USER CODE END StartADC3Task */
}

/* USER CODE BEGIN Header_StartADC4Task */
/**
* @brief Function implementing the ADC4Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartADC4Task */
void StartADC4Task(void *argument)
{
  /* USER CODE BEGIN StartADC4Task */
	HAL_ADC_Start_DMA(&hadc4, (uint32_t*)adc4Buffer, FILTER_BLOCK_SIZE * 2);
  /* Infinite loop */
  for(;;)
  {
    osDelay(osWaitForever);
  }
  /* USER CODE END StartADC4Task */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /*if(htim->Instance == TIM2){
	if(dma_uart_rx.timer == 1){
		dma_uart_rx.flag = 1;
		hdma_usart3_rx.XferCpltCallback(&hdma_usart3_rx);
	}
	if(dma_uart_rx.timer) --dma_uart_rx.timer;
  }*/
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
