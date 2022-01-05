/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "string.h"
#include "stdio.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */
typedef enum eTestStatus
{
	RET_FAIL=0,
	RET_OK,
	RET_TIMEOUT
}eTestStatus;
eTestStatus GPS_test;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
eTestStatus GPS_FWTest(void);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* Definitions for Running_Actor */
osThreadId_t Running_ActorHandle;
uint32_t Running_ActorBuffer[ 128 ];
osStaticThreadDef_t Running_ActorControlBlock;
const osThreadAttr_t Running_Actor_attributes = {
  .name = "Running_Actor",
  .stack_mem = &Running_ActorBuffer[0],
  .stack_size = sizeof(Running_ActorBuffer),
  .cb_mem = &Running_ActorControlBlock,
  .cb_size = sizeof(Running_ActorControlBlock),
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void Active_EventLoop(void *argument);

/* USER CODE BEGIN PFP */
#define GPS_TEST 1
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
  HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_SET);
  GPS_FWTest();
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
  /* creation of Running_Actor */
  Running_ActorHandle = osThreadNew(Active_EventLoop, NULL, &Running_Actor_attributes);

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_HIGH);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GPS_EN_Pin */
  GPIO_InitStruct.Pin = GPS_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPS_EN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**************************************************************************************/
#define PMTK_SET_NMEA_UPDATE_1HZ		"$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ		"$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ		"$PMTK220,100*2F"


#define PMTK_SET_NMEA_OUTPUT_RMCONLY	"$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"		// Turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_GGAONLY	"$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"		// Turn on only the second sentence (GPGGA)
#define PMTK_SET_NMEA_OUTPUT_ALLDATA	"$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"		// Turn on ALL THE DATA


#define PMTK_CMD_COLD_START				"$PMTK103*30\r\n"		// Turn GPS to "cold start"
#define PMTK_CMD_WARM_START				"$PMTK102*31\r\n"		// Turn GPS to "warm start"
#define PMTK_CMD_HOT_START				"$PMTK101*32\r\n"		// Turn GPS to "hot start"

#define PMTK_SET_FLP_MODE				"$PMTK262,1*29\r\n"		// Tracking mode




/* GPS sensor FW TEST - Read data */




#define UART_TIMEOUT		100 //in ms
#define MAX_LEN_GPS			100

uint8_t g_gps_datarecv[MAX_LEN_GPS]={0};

bool GPS_SendCMD(char* p_cmd, uint8_t cmd_len)
{
	return false;
}

bool GPS_Settings(void)
{
	uint8_t data_recv=0;
	uint8_t gps_dataindex=0;
	uint8_t max_retry = 3;
	uint8_t expected_respond[]="PMTK001,314";
	uint8_t respond_len=sizeof(expected_respond)-1;
	for (int count = 0; count < max_retry; ++count)		//Retry in case unsuccessful communication
	{
		uint32_t cur_time = HAL_GetTick();
		printf("Data sent to module GPS: \n");
		if(HAL_UART_Transmit(&huart1, (uint8_t*)PMTK_SET_NMEA_OUTPUT_GGAONLY, strlen( PMTK_SET_NMEA_OUTPUT_GGAONLY), UART_TIMEOUT) == HAL_OK)
		{
			gps_dataindex = 0;
			do
			{
				if (HAL_UART_Receive_IT(&huart1, &data_recv, 1) == HAL_OK)
				{
					if(data_recv == '$') 	gps_dataindex = 0;
					else
					{
						g_gps_datarecv[gps_dataindex++] = data_recv;
					}
				}
			}
			while( (g_gps_datarecv[gps_dataindex-1] != '\r') && (g_gps_datarecv[gps_dataindex] != '\n')
					&& (HAL_GetTick() <= cur_time + 3000) );

			if ( memcmp(g_gps_datarecv, expected_respond, respond_len) == 0)
			{
				#if DEBUG_CONSOLE
				printf("Data recv GPS:  %s \n", g_gps_datarecv);
				#else
				HAL_UART_Transmit(&huart1, g_gps_datarecv, gps_dataindex, 100);
				#endif  /* End of DEBUG_CONSOLE */
				return true;
			}
		}
		HAL_UART_DeInit(&huart1);
		MX_USART1_UART_Init();
	}
	return false;
}


eTestStatus GPS_FWTest(void)
{
	GPS_test = false;
#if GPS_TEST
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);												//Wait for GPS supply power stable
	if (GPS_Settings() == true) GPS_test = true;
	else						GPS_test = false;
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
#else
	printf("----- Skipped test ----- \n");
#endif /*End GPS_TEST*/
	return GPS_test;
}

/**************************************************************************************/
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Active_EventLoop */
/**
  * @brief  Function implementing the Running_Actor thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Active_EventLoop */
void Active_EventLoop(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
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

