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
#include <math.h>
#include "myprintf.h"
//#include "MPU9250.h"
#include "mpu9250.h"
#include "doublyLinkedList.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#define MPU_SPI hspi3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

// Global variables
float AccOffset[3] = {0.0f, 0.0f, 0.0f};
float GyroOffset[3] = {0.0f, 0.0f, 0.0f};
float MagOffset[3] = {0.0f, 0.0f, 0.0f};
struct doubleLinkedList gyro_list[3];
struct doubleLinkedListCord cord_list;
int n_window = 10;

union bytes_to_float{
	uint8_t val_arr[sizeof(float)];
	float val;
};
union bytes_to_float *float_bytes = (union bytes_to_float *)0x30000000;

uint16_t *const x = (uint16_t *)0x30000030;
uint16_t *const y = (uint16_t *)0x30000040;
uint16_t *const z = (uint16_t *)0x30000050;
bool *const flag = (bool *)0x30000060;

bool cord_flag = false;

float robot_angle = 0.0;

float imu_pos_x = 0.0;
float imu_pos_y = 0.0;

float imu_vel_x = 0.0;
float imu_vel_y = 0.0;

float distance_to_wp = 0.0;

uint8_t ak8963_WhoAmI = 0;
uint8_t mpu9250_WhoAmI = 0;
MPU9250 mpu;

// State variables
float psi = 0; // Global yaw angle

// Stanley variables
float x_desired = 0; // Desired x coordinate of the start of the ref line
float y_desired = 0; // Desired y coordinate of the start of the ref line
static float traction_setpoint; // Desired traction
static float traction_current; // Current level of traction
static float steering_delta;    // Desired steering angle
float steering_angle = 0.0f; // Global steering angle
float steering_max = 0.85f;     // Maximum steering angle
float steering_min = 0.15f;     // Minimum steering angle
uint8_t blinker_mode = 0;


osThreadId_t Handle_Task_Traction;
osThreadId_t Handle_Task_Steering;
osThreadId_t Handle_Task_Navigation;
osThreadId_t Handle_Task_UART;
osThreadId_t Handle_Task_MPU9250;
osThreadId_t Handle_Task_LateralP;
osThreadId_t Handle_Task_Stanley;
osThreadId_t Handle_Task_Blinkers;

const osThreadAttr_t Attributes_Task_Traction = {
  .name = "Task_Traction",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

const osThreadAttr_t Attributes_Task_Steering = {
  .name = "Task_Steering",
  .stack_size = 128 * 8*2,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

const osThreadAttr_t Attributes_Task_Navigation = {
  .name = "Task_StateMachine",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t Attributes_Task_UART = {
  .name = "Task_UART",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

const osThreadAttr_t Attributes_Task_MPU9250 = {
  .name = "Task_MPU9250",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityHigh,
};

const osThreadAttr_t Attributes_Task_Blinkers = {
  .name = "Task_Traction",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityNormal,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM13_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void Function_Task_Traction(void *argument);
void Function_Task_Steering(void *argument);
void Function_Task_Navigation(void *argument);
void Function_Task_UART(void *argument);
void Function_Task_MPU9250(void *argument);
void Function_Task_LateralP(void *argument);
void Function_Task_Stanley(void *argument);
void Function_Task_Blinkers(void *argument);
void calibrate_MPU9250(SPI_HandleTypeDef *spi);
void initDoubleLinkedList(struct doubleLinkedList* list[], int n);
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
	float_bytes->val_arr[0] = 0;
	float_bytes->val_arr[1] = 0;
	float_bytes->val_arr[2] = 0;
	float_bytes->val_arr[3] = 0;

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
	int32_t timeout;


/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_TIM14_Init();
  MX_TIM13_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(2000);
  TIM13->CCR1 = (uint32_t)(63999*0.075);
  TIM14->CCR1 = (uint32_t)(63999*0.075);
  HAL_Delay(2000);
  TIM13->CCR1 = (uint32_t)(63999*0.1);
  TIM14->CCR1 = (uint32_t)(63999*0.1);
  HAL_Delay(500);
  //TIM13->CCR1 = (uint32_t)(63999*0.075);
  //TIM14->CCR1 = (uint32_t)(63999*0.075);
  //HAL_Delay(500);
  TIM13->CCR1 = (uint32_t)(63999*0.05);
  TIM14->CCR1 = (uint32_t)(63999*0.05);
  HAL_Delay(500);
  TIM13->CCR1 = (uint32_t)(63999*0.075);
  TIM14->CCR1 = (uint32_t)(63999*0.075);


  printf("Initializing MPU...\r\n");

  for (int i = 0; i < 3; i++) {
    DBLL_init(&gyro_list[i], n_window);
    //DBLL_init(&acce_list[i], n_window);
    //DBLL_init(&mag_list[i], n_window);
  }

  c_DBLL_init(&cord_list);

  c_push_back(&cord_list, 115, 15);
    c_push_back(&cord_list, 160, 75);
    c_push_back(&cord_list, 120, 145);
    c_push_back(&cord_list, 46, 100);

  MPU9250_Init(&mpu);
  calibrate_MPU9250(&MPU_SPI);
  HAL_Delay(2000);
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
  for(int i = 0; i < 4; i++){HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin); HAL_Delay(20);}

  printf("PreeRTOS\r\n");

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  Handle_Task_Steering     = osThreadNew(Function_Task_Steering,   NULL, &Attributes_Task_Steering);
  Handle_Task_Traction     = osThreadNew(Function_Task_Traction,   NULL, &Attributes_Task_Traction);
  Handle_Task_Navigation   = osThreadNew(Function_Task_Navigation, NULL, &Attributes_Task_Navigation);
  Handle_Task_UART         = osThreadNew(Function_Task_UART,       NULL, &Attributes_Task_UART);
  Handle_Task_MPU9250      = osThreadNew(Function_Task_MPU9250,    NULL, &Attributes_Task_MPU9250);
  Handle_Task_Blinkers     = osThreadNew(Function_Task_Blinkers,   NULL, &Attributes_Task_Blinkers);
  //Handle_Task_Stanley       = osThreadNew(Function_Task_Stanley, NULL, &Attributes_Task_Stanley);
  
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 12;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 0;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 65535;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */


  TIM13->ARR = 63999;
  TIM13->PSC = 74;
  TIM13->CCR1 = (uint32_t)(63999*0.075);
  traction_setpoint = 0.5f;
  traction_current = 0.5f;
  HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  TIM14->ARR = 63999;
  TIM14->PSC = 74;
  TIM14->CCR1 = (uint32_t)(63999*0.075);
  steering_delta = 0.5f;
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
  HAL_Delay(40);


  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Left_Blinker_GPIO_Port, Left_Blinker_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Right_Blinker_GPIO_Port, Right_Blinker_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B2_Pin */
  GPIO_InitStruct.Pin = B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI3_CS_Pin */
  GPIO_InitStruct.Pin = SPI3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI3_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Left_Blinker_Pin */
  GPIO_InitStruct.Pin = Left_Blinker_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Left_Blinker_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Right_Blinker_Pin */
  GPIO_InitStruct.Pin = Right_Blinker_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Right_Blinker_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void Function_Task_Blinkers(void *argument){
  for(;;){
    if(blinker_mode == 0){
    	HAL_GPIO_WritePin(Left_Blinker_GPIO_Port, Left_Blinker_Pin, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(Right_Blinker_GPIO_Port, Right_Blinker_Pin, GPIO_PIN_RESET);
    }else if(blinker_mode == 1){
    	HAL_GPIO_TogglePin(Left_Blinker_GPIO_Port, Left_Blinker_Pin);
    	HAL_GPIO_TogglePin(Right_Blinker_GPIO_Port, Right_Blinker_Pin);
    }else if(blinker_mode == 2){
    	HAL_GPIO_TogglePin(Left_Blinker_GPIO_Port, Left_Blinker_Pin);
    	HAL_GPIO_WritePin(Right_Blinker_GPIO_Port, Right_Blinker_Pin, GPIO_PIN_RESET);
    }else if(blinker_mode == 3){
    	HAL_GPIO_WritePin(Left_Blinker_GPIO_Port, Left_Blinker_Pin, GPIO_PIN_RESET);
        HAL_GPIO_TogglePin(Right_Blinker_GPIO_Port, Right_Blinker_Pin);
    }else{
    	HAL_GPIO_WritePin(Left_Blinker_GPIO_Port, Left_Blinker_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(Right_Blinker_GPIO_Port, Right_Blinker_Pin, GPIO_PIN_SET);
    }
    osDelay(500);
  }
}

void Function_Task_Traction(void *argument){
  float kp = 10, velocity = 0, delta_movement = 0, local_x = 0.0f,  local_y = 0.0f;
  uint32_t prev_time = 0, elapsed_time = 0;
  for(;;){
	// x y encoder estimation
	velocity = float_bytes->val;
	elapsed_time = HAL_GetTick() - prev_time;
	delta_movement = velocity * (elapsed_time/1000.0f);
	local_x += delta_movement * cosf(psi);
	local_y += delta_movement * sinf(psi);
    // P control
	//distance_to_wp = sqrt( pow((x_desired - (local_x*100)),2) + pow((y_desired - (local_y*100)),2) ); // Estimacion
	distance_to_wp = sqrt( pow((x_desired - (*x)),2) + pow((y_desired - (*y)),2) ); // Camara
	traction_setpoint = (kp * (distance_to_wp) / 400) + 0.5;
	//printf("%f\r\n", distance_to_wp);
    // Saturation
	traction_setpoint = 0.679;
    traction_setpoint = (traction_setpoint > 1) ? 1 : traction_setpoint;
	  if(traction_current < traction_setpoint){
		  traction_current += 0.01;
	  }else if(traction_current > traction_setpoint){
		  traction_current -= 0.01;
	  }
    // Update PWM
    TIM14->CCR1 = (uint32_t)((63999*0.05)+(63999*0.05*traction_current));

	cord_flag = (distance_to_wp < 25) ? false : true;
    prev_time = HAL_GetTick();
    osDelay(10);
  }
}

void Function_Task_Steering(void *argument){
  float errorIzq = 0, errorDer = 0, error = 0, kp = 2;
    for(;;){ 
      // Blinkers
      if (steering_delta < 0.4){
        blinker_mode = 2;
      } else if (steering_delta > 0.6){
        blinker_mode = 3;
      } else {
        blinker_mode = 1;
      }
      // P control
      /*if (y_desired < (*y)){
    	  error = psi - ((180/M_PI) * atan2f(x_desired - (*x), y_desired - (*y)));
      }else{
    	  error = psi - ((180/M_PI) * atan2f(y_desired - (*y), x_desired - (*x)));
      }*/
      /*if(x_desired - (*x) < 0){
    	  errorIzq = 180 + (((180/M_PI) * atan2f(y_desired - (*y), x_desired - (*x))));
      }else{
    	  if(y_desired - (*y) < 0){
    	  		  errorIzq = psi - ( ((180/M_PI) * atan2f(y_desired - (*y), x_desired - (*x))) + 360);
    	  	  }else{
    	  		  errorIzq = psi - ((180/M_PI) * atan2f(y_desired - (*y), x_desired - (*x)));
    	  	  }
      }*/


	 /*if(y_desired - (*y) > 0){
		  errorDer =  psi - ( ((180/M_PI) * atan2f(y_desired - (*y), x_desired - (*x))) - 360)  ;
	  }else{
		  errorDer = psi - ((180/M_PI) * atan2f(y_desired - (*y), x_desired - (*x))) ;
	  }*/

	/* if(abs(errorIzq) < abs(errorDer)) error = errorIzq;
	 else error = errorDer;*/
      float objective_angle = (180/M_PI) * atan2f(y_desired - (*y), x_desired - (*x));\
      float robot_angle_calc = psi;
      if(objective_angle < 0){
    	  objective_angle += 360;
      }
      if (psi < 0){
          	  objective_angle += 360;
      }

      error = robot_angle_calc - objective_angle;



	 //printf("Ang: %f, errIzq: %f, errorDer: %f\r\n", psi, errorIzq, errorDer, atan2f(y_desired - (*y), x_desired - (*x)));

      steering_delta = (abs(error) < 2) ? 0.5 : ((error*kp+60)/120);
      // Saturation
      if(steering_delta > steering_max){
        steering_delta = steering_max;
      }else if(steering_delta < steering_min){
        steering_delta = steering_min;
      }
      // Update PWM
      TIM13->CCR1 = (uint32_t)((63999*0.05)+(63999*0.05*steering_delta));
      osDelay(100);
    }
}

void Function_Task_Navigation(void *argument){
	struct NodeCord* curr_node = (struct NodeCord*)malloc(sizeof(struct NodeCord));
  // Se asume que la linked list ya esta generada
  curr_node = cord_list.head;

  for(;;){
    if (cord_flag == false){
		  x_desired = curr_node->x;
		  y_desired = curr_node->y;
    	
      if (curr_node == cord_list.tail){
		    curr_node = cord_list.head;
		  }

		  curr_node = curr_node->next;
		  cord_flag = true;
    }
    
    osDelay(100);
  }
}

void Function_Task_UART(void *argument){  
	float errorIzq = 0, errorDer = 0;
	char bt_msg[300];
  uint8_t nbytes;
  for(;;){
    HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    nbytes = sprintf(bt_msg, "Psi: %.2f Delta: %.3f P: %u, %u WP: %.1f, %.1f D2WP: %.3f Traction: %.1f \r\n", psi, steering_delta, *x, *y, x_desired, y_desired, distance_to_wp, traction_setpoint);
    HAL_UART_Transmit(&huart1, (uint8_t*)bt_msg, nbytes, HAL_MAX_DELAY);

    osDelay(50);
  }
}

void Function_Task_MPU9250(void *argument){
  float time_sample_s = 0.05; uint32_t prevtime = 0; uint32_t elapsed_time;
  for(;;){
    // print elapsed time
    elapsed_time = HAL_GetTick() - prevtime;
    // update robot angle with gyro
    if (abs(gyro_list[2].mean) > 1){
      psi += (gyro_list[2].mean * elapsed_time/1000) / 4;
      if(psi < -360 || psi > 360){
    	  psi = 0;
      }
    }

    prevtime = HAL_GetTick();

		ak8963_WhoAmI = mpu_r_ak8963_WhoAmI(&mpu);
		mpu9250_WhoAmI = mpu_r_WhoAmI(&mpu);
		MPU9250_ReadAccel(&mpu);
		MPU9250_ReadGyro(&mpu);
		MPU9250_ReadMag(&mpu);

		push_back(&gyro_list[0], mpu.mpu_data.Gyro[0] - GyroOffset[0]);
		push_back(&gyro_list[1], mpu.mpu_data.Gyro[1] - GyroOffset[1]);
		push_back(&gyro_list[2], mpu.mpu_data.Gyro[2] - GyroOffset[2]);

		uint32_t os_delay = time_sample_s*1000 - (HAL_GetTick() - prevtime);

		osDelay(os_delay);
    
  }
}

void calibrate_MPU9250(SPI_HandleTypeDef *spi){
	float AccAccum[3] = {0.0f, 0.0f, 0.0f};
  float GyroAccum[3] = {0.0f, 0.0f, 0.0f};
  float MagAccum[3] = {0.0f, 0.0f, 0.0f};

  uint8_t num_samples = 50;
  for (uint8_t i = 0; i < num_samples; i++){
    MPU9250_ReadAccel(&mpu);
    MPU9250_ReadGyro(&mpu);
    MPU9250_ReadMag(&mpu);

    AccAccum[0] += mpu.mpu_data.Accel[0];
    AccAccum[1] += mpu.mpu_data.Accel[1];
    AccAccum[2] += mpu.mpu_data.Accel[2];
    GyroAccum[0] += mpu.mpu_data.Gyro[0];
    GyroAccum[1] += mpu.mpu_data.Gyro[1];
    GyroAccum[2] += mpu.mpu_data.Gyro[2];
    MagAccum[0] += mpu.mpu_data.Magn[0];
    MagAccum[1] += mpu.mpu_data.Magn[1];
    MagAccum[2] += mpu.mpu_data.Magn[2];

    HAL_Delay(50);
  }

  AccOffset[0] = AccAccum[0] / num_samples;
  AccOffset[1] = AccAccum[1] / num_samples;
  AccOffset[2] = AccAccum[2] / num_samples;
  GyroOffset[0] = GyroAccum[0] / num_samples;
  GyroOffset[1] = GyroAccum[1] / num_samples;
  GyroOffset[2] = GyroAccum[2] / num_samples;
  MagOffset[0] = MagAccum[0] / num_samples;
  MagOffset[1] = MagAccum[1] / num_samples;
  MagOffset[2] = MagAccum[2] / num_samples;

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /*float setpoint = 0.8f;
  float delta = 0.8f;
  for(;;){
	  osDelay(3000);
	  *traction_setpoint = setpoint;
    steering_delta = delta;
	  setpoint -= 0.1f;
	  delta -= 0.1f;
  
	  Handle_Task_Traction = osThreadNew(Function_Task_Traction, (void *)traction_setpoint, &Attributes_Task_Traction);
  }*/
  for(;;){
    osDelay(10000);
  }
  /* USER CODE END 5 */
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
