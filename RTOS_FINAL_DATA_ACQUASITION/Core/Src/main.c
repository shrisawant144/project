/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <string.h>
#include "bmp280.h"
#include "stdio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FIXED_POINT_FRACTIONAL_BITS 16
#define FIXED_POINT_ONE (1 << FIXED_POINT_FRACTIONAL_BITS)
#define FIXED_POINT_CONVERT(val) ((val) * FIXED_POINT_ONE)
#define FIXED_POINT_TO_FLOAT(val) ((float)(val) / FIXED_POINT_ONE)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
typedef struct sensordata{int32_t temp;uint32_t pres;uint32_t humd;uint32_t mq135;uint32_t mq2;uint32_t d;int l;}s_data;
/* USER CODE END PV */
s_data s;
char buffer[120];
volatile uint32_t IC_Val1 = 0;
volatile uint32_t IC_Val2 = 0;
volatile uint32_t Difference = 0;
volatile uint8_t Is_First_Captured = 0;  // is the first value captured ?
volatile uint32_t Distance  = 0;
volatile int distance1,distance2;
static int32_t Distance_Q16 = 0;
char msg[100];
xSemaphoreHandle semaphore;
#define TRIG_PIN GPIO_PIN_7
#define TRIG_PORT GPIOA
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void const * argument);
void input_capture_handeler(TIM_HandleTypeDef *htim);
/* USER CODE BEGIN PFP */
int _write(int file, char *data, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)data, len, 100);
    return len;
}
void send_uart(const char *message) {
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}
/**
 * Compensation algorithm is taken from BMP280 datasheet.
 *
 * Return value is in Pa, 24 integer bits and 8 fractional bits.
 */

/* USER CODE END PFP */
uint32_t period;

/*Variable to hold the measured distance*/
float distance;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void vHandlerTask(void *pvParam)
{
	char *pcMessage = (char *)pvParam;
	xSemaphoreTake(semaphore, 0);
	while(1)
	{
		HAL_UART_Transmit(&huart2, (uint8_t *)pcMessage, strlen(pcMessage), HAL_MAX_DELAY);
		xSemaphoreTake(semaphore, portMAX_DELAY);
		 UART_Transmit("semaphore taken\r\n");
		portENTER_CRITICAL();
		input_capture_handeler(&htim3);
		portEXIT_CRITICAL();
	}
	vTaskDelete(NULL);
}

void vUARTSendingTask(void *pvParam)
{
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(500);  /* 1 second period */
    xLastWakeTime = xTaskGetTickCount();
	char *pcMessage = (char *)pvParam;

	while(1)
	{
		portENTER_CRITICAL();
		HAL_UART_Transmit(&huart2, (uint8_t *)pcMessage, strlen(pcMessage), HAL_MAX_DELAY);
        snprintf(buffer,sizeof(buffer),"T:%ld.%02ld,P:%ld.%02ld,H:%ld.%02ld,mq135:%lu,mq2:%lu,L:%lu,L:%d\r\n",
     		   s.temp / 100, s.temp % 100,s.pres / 256, (s.pres % 256) * 100 / 256,
				   s.humd / 1024, (s.humd % 1024) * 100 / 1024,s.mq135,s.mq2,s.d,s.l);
        send_uart(buffer);
        HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
		portEXIT_CRITICAL();
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
	}
	vTaskDelete(NULL);
}

void vBMP280Task(void*pvParam)
{
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(1000);  /* 1 second period */
    xLastWakeTime = xTaskGetTickCount();
    char *pcMessage = (char *)pvParam;
    bmp280_params_t params;
    bmp280_init_default_params(&params);

    BMP280_HandleTypedef bmp280;
    bmp280.addr = BMP280_I2C_ADDRESS_0;
    if (!bmp280_init(&bmp280, &params)) {
        strcpy(pcMessage,"\r\nBMP280 initialization failed");
		HAL_UART_Transmit(&huart2, (uint8_t *)pcMessage, strlen(pcMessage), HAL_MAX_DELAY);

        while (1);
    }

    int32_t temperature;
    uint32_t pressure, humidity;

   while(1)
   {
	   bmp280_force_measurement(&bmp280);
	   portENTER_CRITICAL();
		HAL_UART_Transmit(&huart2, (uint8_t *)pcMessage, strlen(pcMessage), HAL_MAX_DELAY);
		vTaskDelay(pdMS_TO_TICKS(100));
		   if (!bmp280_is_measuring(&bmp280)) {
			   if (bmp280_read_fixed(&bmp280, &temperature, &pressure, &humidity)) {
						s.temp = temperature;
						s.pres = pressure;
						s.humd = humidity;
			   } else {
				   send_uart("Failed to read values\r\n");
			   }
		   }
		   vTaskDelay(pdMS_TO_TICKS(1000));
       portEXIT_CRITICAL();
	   vTaskDelayUntil(&xLastWakeTime, xPeriod);
   }
}

void vADCTask(void *pvParam)
{
    char *pcMessage = (char *)pvParam;
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(500);  /* 5 second period */
    xLastWakeTime = xTaskGetTickCount();

    ADC_ChannelConfTypeDef sConfig = {0};

    while (1)
    {
        portENTER_CRITICAL();
        HAL_UART_Transmit(&huart2, (uint8_t *)pcMessage, strlen(pcMessage), HAL_MAX_DELAY);

        // Configure ADC to read from channel 0
        sConfig.Channel = ADC_CHANNEL_0;
        sConfig.Rank = 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }

        // Start ADC Conversion
        HAL_ADC_Start(&hadc1);
        // Poll ADC1 Peripheral & TimeOut = 10mSec
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
        {
            // Read the ADC converted values
            s.mq135 = HAL_ADC_GetValue(&hadc1);
        }
        HAL_ADC_Stop(&hadc1);

        // Configure ADC to read from channel 1
        sConfig.Channel = ADC_CHANNEL_1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }

        // Start ADC Conversion
        HAL_ADC_Start(&hadc1);
        // Poll ADC1 Peripheral & TimeOut = 10mSec
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
        {
            // Read the ADC converted values
            s.mq2 = HAL_ADC_GetValue(&hadc1);
        }
        HAL_ADC_Stop(&hadc1);

        portEXIT_CRITICAL();
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
    vTaskDelete(NULL);
}

int FixedPointToString(int32_t fixedPointValue, char *buffer, int bufferSize)
{
    // Extract integer and fractional parts
    int32_t integerPart = fixedPointValue >> FIXED_POINT_FRACTIONAL_BITS;
    int32_t fractionalPart = fixedPointValue & (FIXED_POINT_ONE - 1);

    // Format as a string with 2 decimal places
    snprintf(buffer, bufferSize, "%d.%02d", integerPart, (fractionalPart * 100) >> FIXED_POINT_FRACTIONAL_BITS);
    return integerPart;
}

/* USER CODE BEGIN 0 */
void delay(uint16_t time)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    while (__HAL_TIM_GET_COUNTER(&htim3) < time);
}

void UART_Transmit(const char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void HCSR04_Read(void)
{
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
    delay(10);  // wait for 10 us
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);
}

void vUltrasonicTask(void *argument)
{
    char *pcMessage = (char *)argument;
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(1000);  /* 5 second period */
    xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)pcMessage, strlen(pcMessage), HAL_MAX_DELAY);
    	portENTER_CRITICAL();
        HCSR04_Read(); // Trigger sensor reading
        osDelay(200); // Wait for 2 seconds
        portEXIT_CRITICAL();
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
    vTaskDelete(NULL);
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
	BaseType_t xRet;
	char * pcMessage = "This is UART Demo\r\n";
  /* USER CODE END Init */
  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */
  vSemaphoreCreateBinary(semaphore);
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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xRet = xTaskCreate(vUARTSendingTask, "UART1 Task", configMINIMAL_STACK_SIZE, "UART Task\r\n", 1, NULL);
       if(xRet != pdTRUE)
  	      Error_Handler();
  xRet = xTaskCreate(vBMP280Task, "bmp280 Task", 1024, "bmp280 task\r\n", 3, NULL);
       if(xRet != pdTRUE)
     	  Error_Handler();
  xRet = xTaskCreate(vADCTask, "adc_task", 128, "adc task\r\n", 2, NULL);
       if(xRet != pdTRUE)
       	  Error_Handler();
  xRet = xTaskCreate( vUltrasonicTask, "ultrasonic_task", 128, "ultrasonic task\r\n", 4, NULL);
       if(xRet != pdTRUE)
       	  Error_Handler();
  xRet = xTaskCreate( vHandlerTask, "handler_task", 128, "handler task\r\n", 5, NULL);
        if(xRet != pdTRUE)
          Error_Handler();
  /* USER CODE END RTOS_THREADS */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 24;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
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
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  /* USER CODE BEGIN Callback 0 */
//
//  /* USER CODE END Callback 0 */
//  if (htim->Instance == TIM1) {
//    HAL_IncTick();
//  }
//  /* USER CODE BEGIN Callback 1 */
//
//  /* USER CODE END Callback 1 */
//}

void input_capture_handeler(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	    {
	        if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC1) != RESET)
	        {
	            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);

	            if (Is_First_Captured == 0) // If the first value is not captured
	            {
	                IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // Read the first value
	                Is_First_Captured = 1;  // Set the first captured as true
	                // Now change the polarity to falling edge
	                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
	                snprintf(msg, sizeof(msg), "First value captured: %lu\r\n", IC_Val1);
	                UART_Transmit(msg);
	                memset(msg, 0, sizeof(msg));
	            }
	            else if (Is_First_Captured == 1)   // If the first value is already captured
	            {
	                IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // Read the second value
	                __HAL_TIM_SET_COUNTER(htim, 0);  // Reset the counter
	                // Set polarity to rising edge
	                if (IC_Val2 > IC_Val1)
	                {
	                    Difference = IC_Val2 - IC_Val1;
	                }
	                else if (IC_Val1 > IC_Val2)
	                {
	                    Difference = (0xffff - IC_Val1) + IC_Val2;
	                }
	                else if(IC_Val1 == IC_Val2)
	                {
	                	Difference = 0;
	                }
	                Distance = Difference * 0.034 / 2;
	                distance1 = Distance/10;
	                distance2 = Distance%10;
	                distance1 = distance1*10+distance2;
//	                Distance_Q16 = (Difference * FIXED_POINT_CONVERT(34)) / 2; // 0.034 * 2 in Q16.16
//
//	                // Convert fixed-point distance to string
//	                char distanceStr[20];
//	                s.l = FixedPointToString(Distance_Q16, distanceStr, sizeof(distanceStr));
                    s.l = distance1;
	                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
	                __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
	                Is_First_Captured = 0; // set it back to false
	            }
	        }
	    }
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	static signed portBASE_TYPE xHigherPriorityTaskWoken;
	  xHigherPriorityTaskWoken = pdFALSE;
	  /* 'Give' the semaphore to unblock the task. */
	  xSemaphoreGiveFromISR(semaphore, (signed portBASE_TYPE*)&xHigherPriorityTaskWoken );
	  portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
	  UART_Transmit("semarphore given\r\n");
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
