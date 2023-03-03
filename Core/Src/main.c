/* USER CODE BEGIN Header */
//!!!! COMMENT MQTTConnectTaskHandle, PINGTaskHandle, myTaskGetFirmHandle
//AND RESET ALL SEMAPHORES !!!!!!!
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include <stdlib.h>
#include "micro_delay.h"
#include "w25qxx.h"
#include "private_data.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PING_TIME 50000  //period of sending ping package
#define BUFF_SIM_SIZE 76 //size of RX SIM buffer
#define MESSAGE_TYPE_BUFF_SIZE 70
#define GET_DATA_PERIOD 30000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef struct {
	char str[MESSAGE_TYPE_BUFF_SIZE];
} message_type;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GetDataTask */
osThreadId_t GetDataTaskHandle;
const osThreadAttr_t GetDataTask_attributes = {
  .name = "GetDataTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for DebugTask */
osThreadId_t DebugTaskHandle;
const osThreadAttr_t DebugTask_attributes = {
  .name = "DebugTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SIM800SendTask */
osThreadId_t SIM800SendTaskHandle;
const osThreadAttr_t SIM800SendTask_attributes = {
  .name = "SIM800SendTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PINGTask */
osThreadId_t PINGTaskHandle;
const osThreadAttr_t PINGTask_attributes = {
  .name = "PINGTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SIM800StartTask */
osThreadId_t SIM800StartTaskHandle;
const osThreadAttr_t SIM800StartTask_attributes = {
  .name = "SIM800StartTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for MQTTConnectTask */
osThreadId_t MQTTConnectTaskHandle;
const osThreadAttr_t MQTTConnectTask_attributes = {
  .name = "MQTTConnectTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MessHandlerTask */
osThreadId_t MessHandlerTaskHandle;
const osThreadAttr_t MessHandlerTask_attributes = {
  .name = "MessHandlerTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTaskGetFirm */
osThreadId_t myTaskGetFirmHandle;
const osThreadAttr_t myTaskGetFirm_attributes = {
  .name = "myTaskGetFirm",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for debugQueue */
osMessageQueueId_t debugQueueHandle;
const osMessageQueueAttr_t debugQueue_attributes = {
  .name = "debugQueue"
};
/* Definitions for SIM800SendQueue */
osMessageQueueId_t SIM800SendQueueHandle;
const osMessageQueueAttr_t SIM800SendQueue_attributes = {
  .name = "SIM800SendQueue"
};
/* Definitions for RxSIM800Queue */
osMessageQueueId_t RxSIM800QueueHandle;
const osMessageQueueAttr_t RxSIM800Queue_attributes = {
  .name = "RxSIM800Queue"
};
/* Definitions for PingTimer */
osTimerId_t PingTimerHandle;
const osTimerAttr_t PingTimer_attributes = {
  .name = "PingTimer"
};
/* Definitions for DataTimer */
osTimerId_t DataTimerHandle;
const osTimerAttr_t DataTimer_attributes = {
  .name = "DataTimer"
};
/* Definitions for UART1Mutex */
osMutexId_t UART1MutexHandle;
const osMutexAttr_t UART1Mutex_attributes = {
  .name = "UART1Mutex"
};
/* Definitions for PINGSem */
osSemaphoreId_t PINGSemHandle;
const osSemaphoreAttr_t PINGSem_attributes = {
  .name = "PINGSem"
};
/* Definitions for SemGetData */
osSemaphoreId_t SemGetDataHandle;
const osSemaphoreAttr_t SemGetData_attributes = {
  .name = "SemGetData"
};
/* USER CODE BEGIN PV */

//flags
uint8_t Error_init = 0; //show that we have error on working SIM800
uint8_t Error_ping = 0; //show that we have error on send ping package
uint8_t Error_broker_connect = 0; //show that we have error on connect to broker

uint8_t Start_SIM800 = 0; //show that SIM800 started well
uint8_t RX = 0;  //show that SIM800 transmitted some data
uint8_t Broker_connect = 0; //show that SIM800 connect to broker
uint8_t Tech_ans_wait = 0; //show that we waiting special answer from module
uint8_t Get_data = 0; //show that we have ask from server to get data

uint8_t str_SIM800[BUFF_SIM_SIZE] = {};
uint8_t SIM800BuffRx[BUFF_SIM_SIZE] = {}; //buffer for RX data from SIM800
uint8_t firmware_buf[400] = {};
message_type Data_Queue;

uint32_t tmp4;
uint32_t tmp5;
uint32_t tmp6;
uint8_t Size_of_firmwware[5] = {};

/* STRINGS */
uint8_t PING[] = "\xC0\0\0";
uint8_t PING_ASK[] = "\xD0\0\0";
uint8_t MQTT_CONNECT_ASK[] = "\0x20\x02\0";
uint8_t TOPIC[] = DEF_TOPIC;
uint8_t TOPIC_ASK[] = "\x90\x03\0";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void StartGetDataTask(void *argument);
void StartDebugTask(void *argument);
void StartSIM800SendTask(void *argument);
void PINGStartTask(void *argument);
void StartSIM800Task(void *argument);
void StartMQTTConnectTask(void *argument);
void StartMessHandlerTask(void *argument);
void StartGetFirmware(void *argument);
void CallbackPingTimer(void *argument);
void CallbackDataTimer(void *argument);

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
	/*__disable_irq();
	SCB->VTOR = 0x0800C000;
	__enable_irq();*/
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of UART1Mutex */
  UART1MutexHandle = osMutexNew(&UART1Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of PINGSem */
  PINGSemHandle = osSemaphoreNew(1, 0, &PINGSem_attributes);

  /* creation of SemGetData */
  SemGetDataHandle = osSemaphoreNew(1, 0, &SemGetData_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of PingTimer */
  PingTimerHandle = osTimerNew(CallbackPingTimer, osTimerPeriodic, NULL, &PingTimer_attributes);

  /* creation of DataTimer */
  DataTimerHandle = osTimerNew(CallbackDataTimer, osTimerPeriodic, NULL, &DataTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of debugQueue */
  debugQueueHandle = osMessageQueueNew (10, sizeof(message_type), &debugQueue_attributes);

  /* creation of SIM800SendQueue */
  SIM800SendQueueHandle = osMessageQueueNew (10, sizeof(message_type), &SIM800SendQueue_attributes);

  /* creation of RxSIM800Queue */
  RxSIM800QueueHandle = osMessageQueueNew (10, sizeof(message_type), &RxSIM800Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of GetDataTask */
  GetDataTaskHandle = osThreadNew(StartGetDataTask, NULL, &GetDataTask_attributes);

  /* creation of DebugTask */
  DebugTaskHandle = osThreadNew(StartDebugTask, NULL, &DebugTask_attributes);

  /* creation of SIM800SendTask */
  SIM800SendTaskHandle = osThreadNew(StartSIM800SendTask, NULL, &SIM800SendTask_attributes);

  /* creation of PINGTask */
  //PINGTaskHandle = osThreadNew(PINGStartTask, NULL, &PINGTask_attributes);

  /* creation of SIM800StartTask */
  SIM800StartTaskHandle = osThreadNew(StartSIM800Task, NULL, &SIM800StartTask_attributes);

  /* creation of MQTTConnectTask */
  //MQTTConnectTaskHandle = osThreadNew(StartMQTTConnectTask, NULL, &MQTTConnectTask_attributes);

  /* creation of MessHandlerTask */
  MessHandlerTaskHandle = osThreadNew(StartMessHandlerTask, NULL, &MessHandlerTask_attributes);

  /* creation of myTaskGetFirm */
  //myTaskGetFirmHandle = osThreadNew(StartGetFirmware, NULL, &myTaskGetFirm_attributes);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(temp_GPIO_Port, temp_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SIM_START_Pin|SIM_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PC1 PC3 SIM_START_Pin SIM_RESET_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|SIM_START_Pin|SIM_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 temp_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_6|temp_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : FLASH_CS_Pin */
  GPIO_InitStruct.Pin = FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FLASH_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void SPI2_Init_Master(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : FLASH_CS_Pin */
	GPIO_InitStruct.Pin = FLASH_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(FLASH_CS_GPIO_Port, &GPIO_InitStruct);

	hspi2.Init.Mode = SPI_MODE_MASTER;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief  make string in MQTT protocol with message msg and topic TOPIC.
  * @param  send_MQTT_message[] String, where will be result.
  * @param  msg[] struct with message for sending with MQTT.
 */

void Make_MQTT_send_string(uint8_t send_MQTT_message[], message_type msg)
{
	send_MQTT_message[0] = 0x30;
    send_MQTT_message[1] = strlen(msg.str) + strlen(TOPIC) + 2;
    send_MQTT_message[2] = 0;
    send_MQTT_message[3] = strlen(TOPIC);
    sprintf(send_MQTT_message + 4,"%s",TOPIC);
    sprintf(send_MQTT_message + 4 + strlen(TOPIC),"%s",msg.str);
}

/**
  * @brief  Find string in Receive buffer SIM800.
  * @param  exm[] String, that we search in Receive buffer SIM800.
  * @retval Is string in Receive buffer SIM800.
 */

uint8_t String_in_SIM800BuffRx(uint8_t exm[])
{
	uint8_t len = strlen(exm);
	uint8_t ans = 0;
	for (uint8_t i = 0; i < BUFF_SIM_SIZE - len; i++)
	{
		if (SIM800BuffRx[i] == exm[0])
		{
			ans = 1;
			for (uint8_t k = 0; k < len; k++)
			{
				if (SIM800BuffRx[i+k] != exm[k])
				{
					ans = 0;
				}
			}
			if (ans)
				break;
		}
	}
	return ans;
}

uint8_t String_in_String(uint8_t str[], uint8_t size, uint8_t exm[])
{
	uint8_t len = strlen(exm);
	uint8_t ans = 0;
	for (uint8_t i = 0; i < size - len; i++)
	{
		if (str[i] == exm[0])
		{
			ans = 1;
			for (uint8_t k = 0; k < len; k++)
			{
				if (str[i+k] != exm[k])
				{
					ans = 0;
				}
			}
			if (ans)
				break;
		}
	}
	return ans;
}

void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	// off half_interupt
	// file stm32f1xx_hal_uart.c
	// comment
	// Set the UART DMA Half transfer complete callback
	// huart->hdmatx->XferHalfCpltCallback = UART_DMATxHalfCplt

	if(huart == &huart1) //Determine whether it is serial port 1
	{
		if(RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))   //Judging whether it is idle interruption
		{
			HAL_UART_DMAStop(&huart1);
			__HAL_UART_CLEAR_IDLEFLAG(&huart1); //Clear idle interrupt sign (otherwise it will continue to enter interrupt)
			RX = 1;
			if (Broker_connect)
			{

				if (String_in_SIM800BuffRx(";;"))
				{
					memcpy(&(Data_Queue.str), SIM800BuffRx, MESSAGE_TYPE_BUFF_SIZE);
					memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
					osMessageQueuePut(RxSIM800QueueHandle, &Data_Queue, 0, NULL);
					HAL_UART_Receive_DMA(&huart1, SIM800BuffRx, BUFF_SIM_SIZE);
				}

				if (!Tech_ans_wait)
				{
					memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
					HAL_UART_Receive_DMA(&huart1, SIM800BuffRx, BUFF_SIM_SIZE);
				}

			}
			//HAL_UART_DMAStop(&huart1);
		}
	}
}

/**
  * @brief  Waiting special answer from SIM800.
  * @param  exm[] String, that we expect.
  * @retval Is special answer in Receive buffer SIM800.
  */
uint8_t SIM800_Ans(uint8_t exm[])
{
	RX = 0;
	Tech_ans_wait = 1;
	HAL_UART_Receive_DMA(&huart1, SIM800BuffRx, BUFF_SIM_SIZE);

	SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // permission counter
	DWT_CONTROL |= DWT_CTRL_CYCCNTENA_Msk;   // start counter

	uint32_t tacts = HAL_RCC_GetSysClockFreq() / 1000000;
	tacts *= 6000000; //timeout - 6s
	DWT->CYCCNT = 0; // clear counter
	while (RX != 1 && DWT->CYCCNT < tacts) {};
	Tech_ans_wait = 0;
	if (!String_in_SIM800BuffRx(exm))
	{
		return 0;
	}
	else
	{
		return 1;
	}
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
  /* Infinite loop */
  for(;;)
  {
	  if (!Start_SIM800 && Error_init) //if we have problems with SIM800, restart it
	  {
		  Error_init = 0;
		  SIM800StartTaskHandle = osThreadNew(StartSIM800Task, NULL, &SIM800StartTask_attributes);
	  }

	  //show that program works
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
	  osDelay(500);

	  //if we have flag, start getting data
	  if (Get_data && Broker_connect)
	  {
		osSemaphoreRelease(SemGetDataHandle);
		Get_data = 0;
	  }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartGetDataTask */
/**
* @brief Function implementing the GetDataTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGetDataTask */
void StartGetDataTask(void *argument)
{
  /* USER CODE BEGIN StartGetDataTask */
	message_type msg; //string for queue
	uint8_t power;
	uint8_t temp = 23;
	uint8_t humidity = 50;
  /* Infinite loop */
  for(;;)
  {
	  // wait semaphore for measuring temperature
	  if (osSemaphoreAcquire(SemGetDataHandle, osWaitForever) == osOK)
	  {

		  // get temp and humidity
		  // Paste code for RS485

		  // get smoke and move


		  // get power
		  power = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);

		  // put string with data to queue
		  sprintf(&(msg.str), "{\"temp\":%d,\"humidity\":%d,\"power\":%d}\r\n\0", temp, humidity, power);
		  osMessageQueuePut(SIM800SendQueueHandle, &msg, 0, osWaitForever);

	  }
  }
  /* USER CODE END StartGetDataTask */
}

/* USER CODE BEGIN Header_StartDebugTask */
/**
* @brief Function implementing the DebugTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDebugTask */
void StartDebugTask(void *argument)
{
  /* USER CODE BEGIN StartDebugTask */
	message_type msg;
  /* Infinite loop */
  for(;;)
  {
	  if (osMessageQueueGet(debugQueueHandle, &msg, 0, osWaitForever) == osOK)
	  {
		  //HAL_UART_Transmit_DMA(&huart2, msg.str, MESSAGE_TYPE_BUFF_SIZE);
	  }
  }
  /* USER CODE END StartDebugTask */
}

/* USER CODE BEGIN Header_StartSIM800SendTask */
/**
* @brief Function implementing the SIM800SendTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSIM800SendTask */
void StartSIM800SendTask(void *argument)
{
  /* USER CODE BEGIN StartSIM800SendTask */
	message_type msg;

	//+ message_len(1 byte) + topic_len(2 bytes) + topic + message
    uint8_t send_MQTT_message[BUFF_SIM_SIZE] = {};
    uint8_t ans;
  /* Infinite loop */
  for(;;)
  {
	  if (osMessageQueueGet(SIM800SendQueueHandle, &msg, 0, osWaitForever) == osOK)
	  {
		  if (osMutexAcquire(UART1MutexHandle, osWaitForever) == osOK)
		  {
			  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

			  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
			  sprintf(send_MQTT_message,"AT+CIPSEND=%d\r\n\0",strlen(msg.str) + strlen(TOPIC) + 4);
			  HAL_UART_Transmit_DMA(&huart1, send_MQTT_message, strlen(send_MQTT_message));
			  ans = SIM800_Ans(">");
			  if (!ans)
			  {

			  }
			  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
			  osDelay(100);

			  //make string for transmit
			  Make_MQTT_send_string(send_MQTT_message, msg);
			  HAL_UART_Transmit_DMA(&huart1, send_MQTT_message, strlen(msg.str) + strlen(TOPIC) + 4);

			  ans = SIM800_Ans("\x30"); //our transmited message
			  if (!ans)
			  {
				  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
				  ans = SIM800_Ans("\x30"); //our transmited message
				  if (!ans)
				  {

				  }
			  }

			  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
			  HAL_UART_Receive_DMA(&huart1, SIM800BuffRx, BUFF_SIM_SIZE);
			  osMutexRelease(UART1MutexHandle); //release UART for SIM800
		  }
	  }
  }
  /* USER CODE END StartSIM800SendTask */
}

/* USER CODE BEGIN Header_PINGStartTask */
/**
* @brief Function implementing the PINGTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PINGStartTask */
void PINGStartTask(void *argument)
{
  /* USER CODE BEGIN PINGStartTask */
	message_type msg;
	uint8_t send_MQTT_message[BUFF_SIM_SIZE] = {};
	uint8_t ans = 0;
  /* Infinite loop */
  for(;;)
  {
	  if (osSemaphoreAcquire(PINGSemHandle, osWaitForever) == osOK)
	  {
		  if (osMutexAcquire(UART1MutexHandle, osWaitForever) == osOK)
		  {

			  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
			  sprintf(send_MQTT_message,"AT+CIPSEND=2\r\n\0");
			  HAL_UART_Transmit_DMA(&huart1, send_MQTT_message, strlen(send_MQTT_message));
			  ans = SIM800_Ans(">");
			  if (!ans)
			  {
				  strcpy(&(msg.str), "FAILE_send_PING1\r\n\0");
				  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
				  Error_ping = 1;
				  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1); //signal for error
				  osMutexRelease(UART1MutexHandle); //release UART for SIM800
				  //start task to try reconnect
				  MQTTConnectTaskHandle = osThreadNew(StartMQTTConnectTask, NULL, &MQTTConnectTask_attributes);
				  //delete this thread
				  osThreadTerminate(PINGTaskHandle);
			  }
			  osDelay(200);
			  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
			  HAL_UART_Transmit_DMA(&huart1, PING, 2);
			  ans = SIM800_Ans(PING);
			  if (!ans)
			  {
				  strcpy(&(msg.str), "FAILE_send_PING2\r\n\0");
				  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
				  Error_ping = 1;
				  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
				  osMutexRelease(UART1MutexHandle);
				  MQTTConnectTaskHandle = osThreadNew(StartMQTTConnectTask, NULL, &MQTTConnectTask_attributes);
				  osThreadTerminate(PINGTaskHandle);
			  }
			  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
			  ans = SIM800_Ans(PING_ASK);
			  if (!ans)
			  {
				  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
				  ans = SIM800_Ans(PING_ASK);
				  if (!ans)
				  {
					  strcpy(&(msg.str), "FAILE_send_PING3\r\n\0");
					  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
					  Error_ping = 1;
					  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
					  osMutexRelease(UART1MutexHandle);
					  MQTTConnectTaskHandle = osThreadNew(StartMQTTConnectTask, NULL, &MQTTConnectTask_attributes);
					  osThreadTerminate(PINGTaskHandle);
				  }
			  }

			  strcpy(&(msg.str), "PING_SEND_OK\r\n\0");
			  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);

			  HAL_UART_Receive_DMA(&huart1, SIM800BuffRx, BUFF_SIM_SIZE);
			  osMutexRelease(UART1MutexHandle);
		  }
	  }
  }
  /* USER CODE END PINGStartTask */
}

/* USER CODE BEGIN Header_StartSIM800Task */
/**
* @brief Function implementing the SIM800StartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSIM800Task */
void StartSIM800Task(void *argument)
{
  /* USER CODE BEGIN StartSIM800Task */

	message_type msg; // string for debug message
	uint8_t ans = 0; //is one string in another

  /* Infinite loop */
  for(;;)
  {
	  Error_init = 0;
	  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

start:

		//Hardware reset of module

		HAL_GPIO_WritePin(SIM_START_GPIO_Port, SIM_START_Pin, RESET);
		osDelay(500);
		HAL_GPIO_WritePin(SIM_START_GPIO_Port, SIM_START_Pin, SET);
		osDelay(8000);

	  	//send initializing data

	  	sprintf(str_SIM800,"ATZ\r\n\0");
	  	HAL_UART_Transmit_IT(&huart1, str_SIM800, strlen(str_SIM800));
	  	osDelay(6000);

	  	memset(SIM800BuffRx, 0, BUFF_SIM_SIZE); //clear receive buffer
	  	sprintf(str_SIM800,"AT+CIPMODE=0\r\n\0"); //make string for send
	  	HAL_UART_Transmit_IT(&huart1, str_SIM800, strlen(str_SIM800)); //send string
	  	ans = SIM800_Ans("OK"); //waiting answer, that we transmit in function
	  	if (!ans)
	  	{
	  		strcpy(&(msg.str), "FAILE_CIPMODE\r\n\0");
	  		osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
	  		goto start;
	  	}
	  	osDelay(300);

	  	memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
	  	sprintf(str_SIM800,"AT+CIPMUX=0\r\n\0");
	  	HAL_UART_Transmit_IT(&huart1, str_SIM800, strlen(str_SIM800));
	  	ans = SIM800_Ans("OK");
	  	if (!ans)
		{
			strcpy(&(msg.str), "FAILE_CIPMUX\r\n\0");
			osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
			goto start;
		}
		osDelay(300);

	  	memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
	  	sprintf(str_SIM800,"AT+CIPSTATUS\r\n\0");
	  	HAL_UART_Transmit_IT(&huart1, str_SIM800, strlen(str_SIM800));
	  	ans = SIM800_Ans("OK");
	  	if (!ans)
		{
			strcpy(&(msg.str), "FAILE_CIPSTATUS\r\n\0");
			osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
			goto start;
		}
		osDelay(300);

	  	memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
	  	sprintf(str_SIM800,"AT+CIPRXGET=0\r\n\0");
	  	HAL_UART_Transmit_IT(&huart1, str_SIM800, strlen(str_SIM800));
	  	ans = SIM800_Ans("OK");
	  	if (!ans)
		{
			strcpy(&(msg.str), "FAILE_CIPRXGET\r\n\0");
			osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
			goto start;
		}
		osDelay(300);

	  	memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
	  	sprintf(str_SIM800,"AT+CSTT=\"internet.beeline.ru\",\"beeline\",\"beeline\"\r\n\0");
	  	HAL_UART_Transmit_IT(&huart1, str_SIM800, strlen(str_SIM800));
	  	ans = SIM800_Ans("OK");
	  	if (!ans)
		{
			strcpy(&(msg.str), "FAILE_CSTT\r\n\0");
			osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
			goto start;
		}
		osDelay(300);

	  	memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
	  	sprintf(str_SIM800,"AT+CIPSTATUS\r\n\0");
	  	HAL_UART_Transmit_IT(&huart1, str_SIM800, strlen(str_SIM800));
	  	ans = SIM800_Ans("OK");
	  	if (!ans)
		{
			strcpy(&(msg.str), "FAILE_CIPSTATUS\r\n\0");
			osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
			goto start;
		}
		osDelay(300);

	  	memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
	  	sprintf(str_SIM800,"AT+CIICR\r\n\0");
	  	HAL_UART_Transmit_IT(&huart1, str_SIM800, strlen(str_SIM800));
	  	ans = SIM800_Ans("AT+CIICR");
	  	memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
	  	ans = SIM800_Ans("OK");
	  	if (!ans)
		{
			strcpy(&(msg.str), "FAILE_CIICR\r\n\0");
			osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
			goto start;
		}
		osDelay(500);

	  	memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
	  	sprintf(str_SIM800,"AT+CIPSTATUS\r\n\0");
	  	HAL_UART_Transmit_IT(&huart1, str_SIM800, strlen(str_SIM800));
	  	ans = SIM800_Ans("OK");
	  	if (!ans)
		{
			strcpy(&(msg.str), "FAILE_CIPSTATUS\r\n\0");
			osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
			goto start;
		}
		osDelay(300);

	  	memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
	  	sprintf(str_SIM800,"AT+CIFSR\r\n\0");
	  	HAL_UART_Transmit_IT(&huart1, str_SIM800, strlen(str_SIM800));
	  	RX = 0;
	  	HAL_UART_Receive_DMA(&huart1, SIM800BuffRx, BUFF_SIM_SIZE);
	  	while (RX < 1) {};
	  	osDelay(300);

	  	memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
	  	sprintf(str_SIM800,"AT+CIPSTATUS\r\n\0");
	  	HAL_UART_Transmit_IT(&huart1, str_SIM800, strlen(str_SIM800));
	  	ans = SIM800_Ans("OK");
	  	if (!ans)
		{
			strcpy(&(msg.str), "FAILE_CIPSTATUS\r\n\0");
			osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
			goto start;
		}
		osDelay(300);

		strcpy(&(msg.str), "START_SIM800_OK\r\n\0");
		osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);

	  	Start_SIM800 = 1; //module starting good
	  	osMutexRelease(UART1MutexHandle); //release UART1
	  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
	  	//start thread to connect to broker
	  	MQTTConnectTaskHandle = osThreadNew(StartMQTTConnectTask, NULL, &MQTTConnectTask_attributes);
	  	osThreadTerminate(SIM800StartTaskHandle); //execute this thread

  }
  /* USER CODE END StartSIM800Task */
}

/* USER CODE BEGIN Header_StartMQTTConnectTask */
/**
* @brief Function implementing the MQTTConnectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMQTTConnectTask */
void StartMQTTConnectTask(void *argument)
{
  /* USER CODE BEGIN StartMQTTConnectTask */

	message_type msg; // string for debug message
	uint8_t ans; //is one string in another
	uint8_t send_MQTT_message[BUFF_SIM_SIZE] = {}; //message to SIM800
	uint8_t counter = 0; //counter of bad trying to connect to broker

  /* Infinite loop */
  for(;;)
  {

	  // waiting mutex
	  if (osMutexAcquire(UART1MutexHandle, osWaitForever) == osOK)
	  {
	  start:
		  if (counter > 5)
		  {
			  Error_init = 1;
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);
			  Start_SIM800 = 0;
			  osMutexRelease(UART1MutexHandle);
			  osThreadTerminate(MQTTConnectTaskHandle);
		  }

		  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
		  //sprintf(str_SIM800,"AT+CIPSTART=\"TCP\",\"broker.hivemq.com\",1883\r\0");
		  sprintf(str_SIM800,"AT+CIPSTART=\"TCP\",\"broker.emqx.io\",1883\r\n\0");
		  HAL_UART_Transmit_DMA(&huart1, str_SIM800, strlen(str_SIM800));
		  ans = SIM800_Ans("OK");
		  if (!ans)
		  {
			  strcpy(&(msg.str), "FAILE_connect1\r\n\0");
			  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
			  counter++;
			  goto start;
		  }
		  osDelay(300);

		  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
		  ans = SIM800_Ans("OK");
		  if (!ans)
		  {
			  strcpy(&(msg.str), "FAILE_connect2\r\n\0");
			  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
			  counter++;
			  goto start;
		  }
		  osDelay(300);

		  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
		  sprintf(str_SIM800,"AT+CIPSEND=14\r\n\0");
		  //sprintf(str_SIM800,"AT+CIPSEND=24\r\0");
		  HAL_UART_Transmit_DMA(&huart1, str_SIM800, strlen(str_SIM800));
		  ans = SIM800_Ans(">");
		  if (!ans)
		  {
			  strcpy(&(msg.str), "FAILE_send\r\n\0");
			  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
			  counter++;
			  goto start;
		  }
		  osDelay(300);

		  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
		  uint8_t send_MQTT_init[50] = "\x10\x0C\0\x04MQTT\x04\x02\0\x3C\0\0\0";
		  HAL_UART_Transmit_DMA(&huart1, send_MQTT_init, 14);
		  //uint8_t send_MQTT_init[50] = "\x10\x16\0\x04MQTT\x04\x02\0\x3C\0\x0A\x34\x62\x58\x34\x56\x66\x47\x42\x51\x75\0";
		  //HAL_UART_Transmit_DMA(&huart1, send_MQTT_init, 24);
		  //osDelay(2000);

		  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
		  ans = SIM800_Ans("\x10\x0C\0\x04MQTT\x04\x02\0\x3C\0\0\0");
		  if (!ans)
		  {
			  strcpy(&(msg.str), "FAILE_init_mqtt1\r\n\0");
			  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
			  counter++;
			  goto start;
		  }

		  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
		  ans = SIM800_Ans(MQTT_CONNECT_ASK);
		  if (!ans)
		  {
			  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
			  ans = SIM800_Ans(MQTT_CONNECT_ASK);
			  if (!ans)
			  {
				  strcpy(&(msg.str), "FAILE_init_mqtt2\r\n\0");
				  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
				  counter++;
				  goto start;
			  }
		  }
		  osDelay(100);

		  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
		  sprintf(send_MQTT_message,"AT+CIPSEND=%d\r\n\0", strlen(TOPIC) + 7);
		  HAL_UART_Transmit_DMA(&huart1, send_MQTT_message, strlen(send_MQTT_message));
		  ans = SIM800_Ans(">");
		  if (!ans)
		  {
			  strcpy(&(msg.str), "FAILE_send1\r\n\0");
			  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
			  counter++;
			  goto start;
		  }
		  osDelay(100);
		  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);

		  //create packet to subscribe on topic
		  send_MQTT_message[0] = 0x82;
		  send_MQTT_message[1] = strlen(TOPIC) + 5;
		  send_MQTT_message[2] = 0;
		  send_MQTT_message[3] = 1;
		  send_MQTT_message[4] = 0;
		  send_MQTT_message[5] = strlen(TOPIC);
		  sprintf(send_MQTT_message + 6,"%s\0",TOPIC);
		  HAL_UART_Transmit_DMA(&huart1, send_MQTT_message, strlen(TOPIC) + 7);
		  ans = SIM800_Ans(send_MQTT_message);
		  if (!ans)
		  {
			  strcpy(&(msg.str), "FAILE_send2\r\n\0");
			  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
			  counter++;
			  goto start;
		  }
		  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
		  ans = SIM800_Ans(TOPIC_ASK);
		  if (!ans)
		  {
			  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
			  ans = SIM800_Ans(TOPIC_ASK);
			  if (!ans)
			  {
				  strcpy(&(msg.str), "FAILE_send3\r\n\0");
				  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
				  counter++;
				  goto start;
			  }
		  }


		  strcpy(&(msg.str), "MQTT_connect_OK\r\n\0");
		  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);

		  Broker_connect = 1;
		  osTimerStart(PingTimerHandle, PING_TIME); //start ping timer
		  osTimerStart(DataTimerHandle, GET_DATA_PERIOD); //start ping timer
		  HAL_UART_Receive_DMA(&huart1, SIM800BuffRx, BUFF_SIM_SIZE); //start receive messages
		  Tech_ans_wait = 0;

		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
		  //start receiving data from broker
		  osMutexRelease(UART1MutexHandle);
		  PINGTaskHandle = osThreadNew(PINGStartTask, NULL, &PINGTask_attributes);
		  sprintf(&(msg.str), "On the line, Firmware V0.92\r\n\0");
		  osMessageQueuePut(SIM800SendQueueHandle, &msg, 0, osWaitForever);
		  //HAL_UART_Receive_DMA(&huart1, SIM800BuffRx, BUFF_SIM_SIZE); //start receive messages
		  osThreadTerminate(MQTTConnectTaskHandle);
	  }


  }
  /* USER CODE END StartMQTTConnectTask */
}

/* USER CODE BEGIN Header_StartMessHandlerTask */
/**
* @brief Function implementing the MessHandlerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMessHandlerTask */
void StartMessHandlerTask(void *argument)
{
  /* USER CODE BEGIN StartMessHandlerTask */
	message_type msg;
	message_type msg2;
  /* Infinite loop */
  for(;;)
  {
	  if (osMessageQueueGet(RxSIM800QueueHandle, &msg, 0, osWaitForever) == osOK)
	  {

		sprintf(&(msg2.str), "RX\r\n\0");
		osMessageQueuePut(SIM800SendQueueHandle, &msg2, 0, osWaitForever);
		if (String_in_String(msg.str,MESSAGE_TYPE_BUFF_SIZE,";;on"))
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
		}
		else if (String_in_String(msg.str,MESSAGE_TYPE_BUFF_SIZE,";;off"))
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
		}
		else if (String_in_String(msg.str,MESSAGE_TYPE_BUFF_SIZE,";;data"))
		{
			Get_data = 1;
		}
		else if (String_in_String(msg.str,MESSAGE_TYPE_BUFF_SIZE,";;updt"))
		{
			if (osMutexAcquire(UART1MutexHandle, osWaitForever) == osOK)
		    {
				memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
				sprintf(str_SIM800,"AT+CIPCLOSE\r\n\0");
				HAL_UART_Transmit_DMA(&huart1, str_SIM800, strlen(str_SIM800));
				osDelay(1000);
				osThreadTerminate(PINGTaskHandle);
				osThreadTerminate(SIM800SendTaskHandle);
				osThreadTerminate(defaultTaskHandle);
				osMutexRelease(UART1MutexHandle); //release UART for SIM800
				myTaskGetFirmHandle = osThreadNew(StartGetFirmware, NULL, &myTaskGetFirm_attributes);

		    }
		}
	  }
  }
  /* USER CODE END StartMessHandlerTask */
}

/* USER CODE BEGIN Header_StartGetFirmware */
/**
* @brief Function implementing the myTaskGetFirm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGetFirmware */
void StartGetFirmware(void *argument)
{
  /* USER CODE BEGIN StartGetFirmware */
	message_type msg; // string for debug message
	uint8_t ans; //is one string in another
	uint8_t send_MQTT_message[BUFF_SIM_SIZE] = {}; //message to SIM800
	uint32_t sector_of_firmware = 30;
	uint32_t firmware_count_bytes;
	uint8_t tmp2[30];
	uint8_t tmp3[30];
  /* Infinite loop */
  for(;;)
  {
	  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
	  sprintf(str_SIM800,"AT+FTPMODE=1\r\n\0");
	  HAL_UART_Transmit_DMA(&huart1, str_SIM800, strlen(str_SIM800));
	  ans = SIM800_Ans("OK");
	  if (!ans)
	  {
		  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
		  ans = SIM800_Ans("OK");
		  if (!ans)
		  {
			  strcpy(&(msg.str), "FTPMODE\r\n\0");
			  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
		  }
	  }
	  osDelay(300);

	  /*memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
	  sprintf(str_SIM800,"AT+FTPSSL=2\r\n\0");
	  HAL_UART_Transmit_DMA(&huart1, str_SIM800, strlen(str_SIM800));
	  ans = SIM800_Ans("OK");
	  if (!ans)
	  {
		  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
		  ans = SIM800_Ans("OK");
		  if (!ans)
		  {
			  strcpy(&(msg.str), "FTPFTPSSL\r\n\0");
			  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
		  }
	  }
	  osDelay(300);*/

	  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
	  sprintf(str_SIM800,"AT+FTPTYPE=I\r\n\0");
	  HAL_UART_Transmit_DMA(&huart1, str_SIM800, strlen(str_SIM800));
	  ans = SIM800_Ans("OK");
	  if (!ans)
	  {
		  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
		  ans = SIM800_Ans("OK");
		  if (!ans)
		  {
			  strcpy(&(msg.str), "FTPTYPE\r\n\0");
			  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
		  }
	  }
	  osDelay(300);

	  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
	  sprintf(str_SIM800,"AT+SAPBR=3,1,CONTYPE,GPRS\r\n\0");
	  HAL_UART_Transmit_DMA(&huart1, str_SIM800, strlen(str_SIM800));
	  ans = SIM800_Ans("OK");
	  if (!ans)
	  {
		  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
		  ans = SIM800_Ans("OK");
		  if (!ans)
		  {
			  strcpy(&(msg.str), "SAPBR\r\n\0");
			  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
		  }
	  }
	  osDelay(300);

	  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
	  sprintf(str_SIM800,"AT+SAPBR=3,1,APN,internet.beeline.ru\r\n\0");
	  HAL_UART_Transmit_DMA(&huart1, str_SIM800, strlen(str_SIM800));
	  ans = SIM800_Ans("OK");
	  if (!ans)
	  {
		  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
		  ans = SIM800_Ans("OK");
		  if (!ans)
		  {
			  strcpy(&(msg.str), "SAPBR\r\n\0");
			  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
		  }
	  }
	  osDelay(300);

	  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
	  sprintf(str_SIM800,"AT+SAPBR=1,1\r\n\0");
	  HAL_UART_Transmit_DMA(&huart1, str_SIM800, strlen(str_SIM800));
	  ans = SIM800_Ans("OK");
	  if (!ans)
	  {
		  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
		  ans = SIM800_Ans("OK");
		  if (!ans)
		  {
			  strcpy(&(msg.str), "SAPBR\r\n\0");
			  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
		  }
	  }
	  osDelay(300);

	  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
	  sprintf(str_SIM800,"AT+FTPCID=1\r\n\0");
	  HAL_UART_Transmit_DMA(&huart1, str_SIM800, strlen(str_SIM800));
	  ans = SIM800_Ans("OK");
	  if (!ans)
	  {
		  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
		  ans = SIM800_Ans("OK");
		  if (!ans)
		  {
			  strcpy(&(msg.str), "FTPCID\r\n\0");
			  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
		  }
	  }
	  osDelay(300);

	  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
	  sprintf(str_SIM800,"AT+FTPSERV=%s\r\n\0", DEF_IP_FTP);
	  HAL_UART_Transmit_DMA(&huart1, str_SIM800, strlen(str_SIM800));
	  ans = SIM800_Ans("OK");
	  if (!ans)
	  {
		  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
		  ans = SIM800_Ans("OK");
		  if (!ans)
		  {
			  strcpy(&(msg.str), "FTPSERV\r\n\0");
			  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
		  }
	  }
	  osDelay(300);

	  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
	  sprintf(str_SIM800,"AT+FTPUN=%s\r\n\0", DEF_USER_FTP);
	  HAL_UART_Transmit_DMA(&huart1, str_SIM800, strlen(str_SIM800));
	  ans = SIM800_Ans("OK");
	  if (!ans)
	  {
		  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
		  ans = SIM800_Ans("OK");
		  if (!ans)
		  {
			  strcpy(&(msg.str), "FTPUN\r\n\0");
			  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
		  }
	  }
	  osDelay(300);

	  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
	  sprintf(str_SIM800,"AT+FTPPW=%s\r\n\0", DEF_PASSWORD_FTP);
	  HAL_UART_Transmit_DMA(&huart1, str_SIM800, strlen(str_SIM800));
	  ans = SIM800_Ans("OK");
	  if (!ans)
	  {
		  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
		  ans = SIM800_Ans("OK");
		  if (!ans)
		  {
			  strcpy(&(msg.str), "FTPPW\r\n\0");
			  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
		  }
	  }
	  osDelay(300);

	  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
	  sprintf(str_SIM800,"AT+FTPGETPATH=/firmware/\r\n\0");
	  HAL_UART_Transmit_DMA(&huart1, str_SIM800, strlen(str_SIM800));
	  ans = SIM800_Ans("OK");
	  if (!ans)
	  {
		  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
		  ans = SIM800_Ans("OK");
		  if (!ans)
		  {
			  strcpy(&(msg.str), "FTPGETPATH\r\n\0");
			  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
		  }
	  }
	  osDelay(300);

	  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
	  sprintf(str_SIM800,"AT+FTPGETNAME=Firmware.txt\r\n\0");
	  HAL_UART_Transmit_DMA(&huart1, str_SIM800, strlen(str_SIM800));
	  ans = SIM800_Ans("OK");
	  if (!ans)
	  {
		  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
		  ans = SIM800_Ans("OK");
		  if (!ans)
		  {
			  strcpy(&(msg.str), "FTPGETNAME\r\n\0");
			  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
		  }
	  }
	  osDelay(300);

	  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
	  sprintf(str_SIM800,"AT+FTPGETTOFS=0,Firmware.txt\r\n\0");
	  HAL_UART_Transmit_DMA(&huart1, str_SIM800, strlen(str_SIM800));
	  ans = SIM800_Ans("OK");
	  if (!ans)
	  {
		  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
		  ans = SIM800_Ans("OK");
		  if (!ans)
		  {
			  strcpy(&(msg.str), "FTPGETTOFS\r\n\0");
			  osMessageQueuePut(debugQueueHandle, &msg, 0, osWaitForever);
		  }
	  }
	  osDelay(300);

	  memset(SIM800BuffRx, 0, BUFF_SIM_SIZE);
	  RX = 0;
	  Tech_ans_wait = 1;
	  HAL_UART_Receive_DMA(&huart1, SIM800BuffRx, BUFF_SIM_SIZE);
	  while (RX != 1) {};
	  Tech_ans_wait = 0;



	  //good download
	  if (String_in_SIM800BuffRx("+FTPGETTOFS: "))
	  {
		  sscanf(SIM800BuffRx, "%s%2s%d", tmp2, tmp3, &firmware_count_bytes);

		  SPI2_Init_Master();
		  HAL_Delay(200);

		  W25qxx_Init();

		  uint32_t count_of_pages = (firmware_count_bytes / 256) + ((firmware_count_bytes % 256) ? 1 : 0);
		  uint32_t count_of_sectors = (firmware_count_bytes / 4096) + ((firmware_count_bytes % 4096) ? 1 : 0);
		  count_of_sectors++;

		  //start write from 30 sector
		  for (uint32_t i = 0; i <= count_of_sectors; i++)
		  {
			  // sectors for clear
			  uint32_t tmp = i + sector_of_firmware - 1;
			  W25qxx_EraseSector(tmp);
		  }

		  //writes size of firmware
		  tmp4 = (firmware_count_bytes & 0xFF0000) >> 16;
		  tmp5 = (firmware_count_bytes & 0xFF00) >> 8;
		  tmp6 = (firmware_count_bytes & 0xFF);
		  Size_of_firmwware[0] = tmp4;
		  Size_of_firmwware[1] = tmp5;
		  Size_of_firmwware[2] = tmp6;

		  int32_t page_start1 = 16 * (sector_of_firmware - 1);

		  W25qxx_WritePage(Size_of_firmwware, page_start1, 0, 3);

		  for (uint32_t i = 0; i < count_of_pages; i++)
		  {
			  int32_t start = i * 256;
			  //start += (i > 0) ? 1 : 0;

			  memset(firmware_buf, 0, 399);
			  sprintf(str_SIM800,"AT+FSREAD=C:\\User\\FTP\\Firmware.txt,1,256,%d\r\n\0", start);
			  HAL_UART_Transmit_DMA(&huart1, str_SIM800, strlen(str_SIM800));
			  RX = 0;
			  Tech_ans_wait = 1;
			  HAL_UART_Receive_DMA(&huart1, firmware_buf, 390);
			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
			  while (RX != 1) {};
			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
			  Tech_ans_wait = 0;

			  int32_t page_start =  i + 16 * sector_of_firmware; // number of start page
			  W25qxx_WritePage(firmware_buf + strlen(str_SIM800) + 1, page_start, 0, 256);
			  osDelay(10); //importent!!!
		  }
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
		  osDelay(2000);
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);

		  HAL_Delay(200);
		  //";start"
		  strcpy(&(msg.str), ";start");
		  //HAL_UART_Transmit_DMA(&huart2, msg.str, MESSAGE_TYPE_BUFF_SIZE);
		  HAL_Delay(1000);
		  HAL_NVIC_SystemReset();
		  //osThreadTerminate(myTaskGetFirmHandle);
	  }

  }
  /* USER CODE END StartGetFirmware */
}

/* CallbackPingTimer function */
void CallbackPingTimer(void *argument)
{
  /* USER CODE BEGIN CallbackPingTimer */
	osSemaphoreRelease(PINGSemHandle);
  /* USER CODE END CallbackPingTimer */
}

/* CallbackDataTimer function */
void CallbackDataTimer(void *argument)
{
  /* USER CODE BEGIN CallbackDataTimer */
	Get_data = 1;
  /* USER CODE END CallbackDataTimer */
}

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
  if (htim->Instance == TIM1) {
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
