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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
typedef struct sensordata{int32_t temp;uint32_t pres;uint32_t humd;uint32_t mq135;uint32_t mq2;uint32_t d;}s_data;
/* USER CODE END PV */
s_data s;
char buffer[120];
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void const * argument);
/* USER CODE BEGIN PFP */
/**
 * Compensation algorithm is taken from BMP280 datasheet.
 *
 * Return value is in degrees Celsius.
 */

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
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void vUltrasonicTask(void *pvParam)
//{
//    char *pcMessage = (char *)pvParam;
//    TickType_t xLastWakeTime;
//    const TickType_t xPeriod = pdMS_TO_TICKS(5000);  /* 5-second period */
//    xLastWakeTime = xTaskGetTickCount();
//    /* Enable float point hardware */
//    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));
//    /* Set the clock to 100MHz */
//
//
//    /* Enable clock access to GPIOA and GPIOB */
//    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
//
//    /* Set PA5 for alternate function */
//    GPIOA->MODER |= GPIO_MODER_MODE5_1;
//    GPIOA->MODER &= ~GPIO_MODER_MODE5_0;
//
//    /* Set PB3 for alternate function */
//    GPIOB->MODER |= GPIO_MODER_MODE3_1;
//    GPIOB->MODER &= ~GPIO_MODER_MODE3_0;
//
//    /* Select AF01 for TIM2 */
//    #define AF01 0x01
//    GPIOA->AFR[0] |= (AF01 << GPIO_AFRL_AFSEL5_Pos);
//    GPIOB->AFR[0] |= (AF01 << GPIO_AFRL_AFSEL3_Pos);
//
//    /* Enable clock access to TIM2 */
//    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
//
//    /* Set the prescaler to 99 to get 1MHz */
//    TIM2->PSC = 99;
//
//    /* Set the maximum to 199999 to get 5 Hz update rate */
//    TIM2->ARR = 1000000 - 1;
//
//    /* Reset current counter */
//    TIM2->CNT = 0;
//
//    /* Set PA5 as PWM (Channel 1) */
//    TIM2->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
//
//    /* Set PB3 as Input capture (Channel 2) */
//    TIM2->CCMR1 |= TIM_CCMR1_CC2S_0;
//
//    /* Enable CH1 and CH2 */
//    TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
//
//    /* Enable capture/compare 2 DMA */
//    TIM2->DIER |= TIM_DIER_CC2DE;
//
//    /* Set CH2 to be on both edge (falling/rising edge) */
//    TIM2->CCER |= TIM_CCER_CC2P | TIM_CCER_CC2NP;
//
//    /* DMA configuration */
//
//    /* Enable clock access to DMA1 */
//    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
//
//    /* Disable the stream and wait until it is disabled */
//    DMA1_Stream6->CR &= ~DMA_SxCR_EN;
//    while ((DMA1_Stream6->CR & DMA_SxCR_EN) == DMA_SxCR_EN);
//
//    /* Configure the DMA with the following parameters:
//     * Channel is channel 3
//     * Memory and peripheral size is word (32-bit)
//     * Circular mode
//     */
//    #define CH3 0x03
//    DMA1_Stream6->CR |= (CH3 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PSIZE_1 | DMA_SxCR_MSIZE_1 | DMA_SxCR_CIRC;
//
//    /* Set number of transfers to be 1
//     * Peripheral address to TIM2->CCR2
//     * Memory address to be the period variable
//     */
//    DMA1_Stream6->NDTR = 1;
//    DMA1_Stream6->PAR = (uint32_t)&TIM2->CCR2;
//    DMA1_Stream6->M0AR = (uint32_t)&period;
//
//    /* Delay to ensure the ultrasonic sensor is up */
//    vTaskDelay(pdMS_TO_TICKS(30));
//
//    /* Enable DMA */
//    DMA1_Stream6->CR |= DMA_SxCR_EN;
//
//    /* Enable timer */
//    TIM2->CR1 = TIM_CR1_CEN;
//
//    /* Give 12 microsecond pulse to trigger pin (Channel 1) */
//    TIM2->CCR1 = 12;
//
//    while (1)
//    {
//		HAL_UART_Transmit(&huart2, (uint8_t *)pcMessage, strlen(pcMessage), HAL_MAX_DELAY);
//
//        /* Calculate the distance */
//        distance = (float)(period - 538.0) * 0.034 / 2.0;
//
//        /* Convert the distance to integer */
//        s.d = (int)distance;
//
//        /* Print the distance as integer */
//
//
//        /* Delay between each print */
//        vTaskDelayUntil(&xLastWakeTime, xPeriod);
//    }
//    vTaskDelete(NULL);
//}

void vUARTSendingTask(void *pvParam)
{
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(1000);  /* 1 second period */
    xLastWakeTime = xTaskGetTickCount();
	char *pcMessage = (char *)pvParam;

	while(1)
	{
		portENTER_CRITICAL();
		HAL_UART_Transmit(&huart2, (uint8_t *)pcMessage, strlen(pcMessage), HAL_MAX_DELAY);
        snprintf(buffer,sizeof(buffer),"T:%ld.%02ld,P:%ld.%02ld,H:%ld.%02ld,mq135:%lu,mq2:%lu,L:%lu\r\n",
     		   s.temp / 100, s.temp % 100,s.pres / 256, (s.pres % 256) * 100 / 256,
				   s.humd / 1024, (s.humd % 1024) * 100 / 1024,s.mq135,s.mq2,s.d);
        send_uart(buffer);
		portEXIT_CRITICAL();
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
	}
	vTaskDelete(NULL);
}

void vBMP280Task(void*pvParam)
{
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(5000);  /* 1 second period */
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
/* USER CODE END 0 */
void vADCTask(void *pvParam)
{
    char *pcMessage = (char *)pvParam;
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(5000);  /* 5 second period */
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

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	  /* USER CODE BEGIN 1 */
		BaseType_t xRet;
		char * pcMessage = "This is UART Demo\r\n";
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
	  MX_ADC1_Init();
	  MX_USART2_UART_Init();
	  MX_I2C1_Init();
	  /* USER CODE BEGIN 2 */
	  HAL_UART_Transmit(&huart2, (uint8_t *)pcMessage, strlen(pcMessage), HAL_MAX_DELAY);
	  /* USER CODE END 2 */

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
	  /* definition and creation of defaultTask */
	  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
	  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	  /* USER CODE BEGIN RTOS_THREADS */
	  /* add threads, ... */


	  xRet = xTaskCreate(vUARTSendingTask, "UART1 Task", configMINIMAL_STACK_SIZE, "UART Task1\r\n", 2, NULL);
	    if(xRet != pdTRUE)
	  	  Error_Handler();

	    xRet = xTaskCreate(vBMP280Task, "bmp280 Task", 1024, "bmp280\r\n", 4, NULL);
	       if(xRet != pdTRUE)
	     	  Error_Handler();
	       xRet = xTaskCreate(vADCTask, "adc_task", 128, "adc\r\n", 3, NULL);
	       if(xRet != pdTRUE)
	       	  Error_Handler();
//	       xRet = xTaskCreate( vUltrasonicTask, "ultrasonic_task", 128, "ultrasonic task\r\n", 2, NULL);
//	       if(xRet != pdTRUE)
//	       	  Error_Handler();
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
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
