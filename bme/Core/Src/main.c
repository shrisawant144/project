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
#include<stdio.h>
#include<string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * BMP280 or BME280 address is 0x77 if SDO pin is high, and is 0x76 if
 * SDO pin is low.
 */
#define BMP280_REG_TEMP_XLSB   0xFC /* bits: 7-4 */
#define BMP280_REG_TEMP_LSB    0xFB
#define BMP280_REG_TEMP_MSB    0xFA
#define BMP280_REG_TEMP        (BMP280_REG_TEMP_MSB)
#define BMP280_REG_PRESS_XLSB  0xF9 /* bits: 7-4 */
#define BMP280_REG_PRESS_LSB   0xF8
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_PRESSURE    (BMP280_REG_PRESS_MSB)
#define BMP280_REG_CONFIG      0xF5 /* bits: 7-5 t_sb; 4-2 filter; 0 spi3w_en */
#define BMP280_REG_CTRL        0xF4 /* bits: 7-5 osrs_t; 4-2 osrs_p; 1-0 mode */
#define BMP280_REG_STATUS      0xF3 /* bits: 3 measuring; 0 im_update */
#define BMP280_REG_CTRL_HUM    0xF2 /* bits: 2-0 osrs_h; */
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_ID          0xD0
#define BMP280_REG_CALIB       0x88
#define BMP280_REG_HUM_CALIB   0x88

#define BMP280_RESET_VALUE     0xB6
#define BMP280_I2C_ADDRESS_0  0x76
#define BMP280_I2C_ADDRESS_1  0x77

#define BMP280_CHIP_ID  0x58 /* BMP280 has chip-id 0x58 */
#define BME280_CHIP_ID  0x60 /* BME280 has chip-id 0x60 */

/**
 * Mode of BMP280 module operation.
 * Forced - Measurement is initiated by user.
 * Normal - Continues measurement.
 */
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
typedef enum {
    BMP280_MODE_SLEEP = 0,
    BMP280_MODE_FORCED = 1,
    BMP280_MODE_NORMAL = 3
} BMP280_Mode;

typedef enum {
    BMP280_FILTER_OFF = 0,
    BMP280_FILTER_2 = 1,
    BMP280_FILTER_4 = 2,
    BMP280_FILTER_8 = 3,
    BMP280_FILTER_16 = 4
} BMP280_Filter;

/**
 * Pressure oversampling settings
 */
typedef enum {
    BMP280_SKIPPED = 0,          /* no measurement  */
    BMP280_ULTRA_LOW_POWER = 1,  /* oversampling x1 */
    BMP280_LOW_POWER = 2,        /* oversampling x2 */
    BMP280_STANDARD = 3,         /* oversampling x4 */
    BMP280_HIGH_RES = 4,         /* oversampling x8 */
    BMP280_ULTRA_HIGH_RES = 5    /* oversampling x16 */
} BMP280_Oversampling;

/**
 * Stand by time between measurements in normal mode
 */
typedef enum {
    BMP280_STANDBY_05 = 0,      /* stand by time 0.5ms */
    BMP280_STANDBY_62 = 1,      /* stand by time 62.5ms */
    BMP280_STANDBY_125 = 2,     /* stand by time 125ms */
    BMP280_STANDBY_250 = 3,     /* stand by time 250ms */
    BMP280_STANDBY_500 = 4,     /* stand by time 500ms */
    BMP280_STANDBY_1000 = 5,    /* stand by time 1s */
    BMP280_STANDBY_2000 = 6,    /* stand by time 2s BMP280, 10ms BME280 */
    BMP280_STANDBY_4000 = 7,    /* stand by time 4s BMP280, 20ms BME280 */
} BMP280_StandbyTime;

/**
 * Configuration parameters for BMP280 module.
 * Use function bmp280_init_default_params to use default configuration.
 */
typedef struct {
    BMP280_Mode mode;
    BMP280_Filter filter;
    BMP280_Oversampling oversampling_pressure;
    BMP280_Oversampling oversampling_temperature;
    BMP280_Oversampling oversampling_humidity;
    BMP280_StandbyTime standby;
} bmp280_params_t;


typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    /* Humidity compensation for BME280 */
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;

    uint16_t addr;



    bmp280_params_t params;

    uint8_t  id;        /* Chip ID */

} BMP280_HandleTypedef;

/**
 * Initialize default parameters.
 * Default configuration:
 *      mode: NORAML
 *      filter: OFF
 *      oversampling: x4
 *      standby time: 250ms
 */
void bmp280_init_default_params(bmp280_params_t *params);

/**
 * Initialize BMP280 module, probes for the device, soft resets the device,
 * reads the calibration constants, and configures the device using the supplied
 * parameters. Returns true on success otherwise false.
 *
 * The I2C address is assumed to have been initialized in the dev, and
 * may be either BMP280_I2C_ADDRESS_0 or BMP280_I2C_ADDRESS_1. If the I2C
 * address is unknown then try initializing each in turn.
 *
 * This may be called again to soft reset the device and initialize it again.
 */
bool bmp280_init(BMP280_HandleTypedef *dev, bmp280_params_t *params);

/**
 * Start measurement in forced mode.
 * The module remains in forced mode after this call.
 * Do not call this method in normal mode.
 */
bool bmp280_force_measurement(BMP280_HandleTypedef *dev);

/**
 * Check if BMP280 is busy with measuring temperature/pressure.
 * Return true if BMP280 is busy.
 */
bool bmp280_is_measuring(BMP280_HandleTypedef *dev);

/**
 * Read compensated temperature and pressure data:
 *
 *  Temperature in degrees Celsius times 100.
 *
 *  Pressure in Pascals in fixed point 24 bit integer 8 bit fraction format.
 *
 *  Humidity is optional and only read for the BME280, in percent relative
 *  humidity as a fixed point 22 bit interger and 10 bit fraction format.
 */
bool bmp280_read_fixed(BMP280_HandleTypedef *dev, int32_t *temperature,
                       uint32_t *pressure, uint32_t *humidity);

/**
 * Read compensated temperature and pressure data:
 *  Temperature in degrees Celsius.
 *  Pressure in Pascals.
 *  Humidity is optional and only read for the BME280, in percent relative
 *  humidity.
 */
bool bmp280_read_float(BMP280_HandleTypedef *dev, float *temperature,
                       float *pressure, float *humidity);


/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

void bmp280_init_default_params(bmp280_params_t *params) {
	params->mode = BMP280_MODE_NORMAL;
	params->filter = BMP280_FILTER_OFF;
	params->oversampling_pressure = BMP280_STANDARD;
	params->oversampling_temperature = BMP280_STANDARD;
	params->oversampling_humidity = BMP280_STANDARD;
	params->standby = BMP280_STANDBY_250;
}

static bool read_register16(BMP280_HandleTypedef *dev, uint8_t addr, uint16_t *value) {
	uint16_t tx_buff;
	uint8_t rx_buff[2];
	tx_buff = (dev->addr << 1);

	if (HAL_I2C_Mem_Read(&hi2c1, tx_buff, addr, 1, rx_buff, 2, 5000)
			== HAL_OK) {
		*value = (uint16_t) ((rx_buff[1] << 8) | rx_buff[0]);
		return true;
	} else
		return false;

}

static inline int read_data(BMP280_HandleTypedef *dev, uint8_t addr, uint8_t *value,
		uint8_t len) {
	uint16_t tx_buff;
	tx_buff = (dev->addr << 1);
	if (HAL_I2C_Mem_Read(&hi2c1, tx_buff, addr, 1, value, len, 5000) == HAL_OK)
		return 0;
	else
		return 1;

}

static bool read_calibration_data(BMP280_HandleTypedef *dev) {

	if (read_register16(dev, 0x88, &dev->dig_T1)
			&& read_register16(dev, 0x8a, (uint16_t *) &dev->dig_T2)
			&& read_register16(dev, 0x8c, (uint16_t *) &dev->dig_T3)
			&& read_register16(dev, 0x8e, &dev->dig_P1)
			&& read_register16(dev, 0x90, (uint16_t *) &dev->dig_P2)
			&& read_register16(dev, 0x92, (uint16_t *) &dev->dig_P3)
			&& read_register16(dev, 0x94, (uint16_t *) &dev->dig_P4)
			&& read_register16(dev, 0x96, (uint16_t *) &dev->dig_P5)
			&& read_register16(dev, 0x98, (uint16_t *) &dev->dig_P6)
			&& read_register16(dev, 0x9a, (uint16_t *) &dev->dig_P7)
			&& read_register16(dev, 0x9c, (uint16_t *) &dev->dig_P8)
			&& read_register16(dev, 0x9e,
					(uint16_t *) &dev->dig_P9)) {

		return true;
	}

	return false;
}

static bool read_hum_calibration_data(BMP280_HandleTypedef *dev) {
	uint16_t h4, h5;

	if (!read_data(dev, 0xa1, &dev->dig_H1, 1)
			&& read_register16(dev, 0xe1, (uint16_t *) &dev->dig_H2)
			&& !read_data(dev, 0xe3, &dev->dig_H3, 1)
			&& read_register16(dev, 0xe4, &h4)
			&& read_register16(dev, 0xe5, &h5)
			&& !read_data(dev, 0xe7, (uint8_t *) &dev->dig_H6, 1)) {
		dev->dig_H4 = (h4 & 0x00ff) << 4 | (h4 & 0x0f00) >> 8;
		dev->dig_H5 = h5 >> 4;

		return true;
	}

	return false;
}

static int write_register8(BMP280_HandleTypedef *dev, uint8_t addr, uint8_t value) {
	uint16_t tx_buff;

	tx_buff = (dev->addr << 1);

	if (HAL_I2C_Mem_Write(&hi2c1, tx_buff, addr, 1, &value, 1, 10000) == HAL_OK)
		return false;
	else
		return true;
}

bool bmp280_init(BMP280_HandleTypedef *dev, bmp280_params_t *params) {

	if (dev->addr != BMP280_I2C_ADDRESS_0
			&& dev->addr != BMP280_I2C_ADDRESS_1) {

		return false;
	}

	if (read_data(dev, BMP280_REG_ID, &dev->id, 1)) {
		return false;
	}

	if (dev->id != BMP280_CHIP_ID && dev->id != BME280_CHIP_ID) {

		return false;
	}

	// Soft reset.
	if (write_register8(dev, BMP280_REG_RESET, BMP280_RESET_VALUE)) {
		return false;
	}

	// Wait until finished copying over the NVP data.
	while (1) {
		uint8_t status;
		if (!read_data(dev, BMP280_REG_STATUS, &status, 1)
				&& (status & 1) == 0)
			break;
	}

	if (!read_calibration_data(dev)) {
		return false;
	}

	if (dev->id == BME280_CHIP_ID && !read_hum_calibration_data(dev)) {
		return false;
	}

	uint8_t config = (params->standby << 5) | (params->filter << 2);
	if (write_register8(dev, BMP280_REG_CONFIG, config)) {
		return false;
	}

	if (params->mode == BMP280_MODE_FORCED) {
		params->mode = BMP280_MODE_SLEEP;  // initial mode for forced is sleep
	}

	uint8_t ctrl = (params->oversampling_temperature << 5)
			| (params->oversampling_pressure << 2) | (params->mode);

	if (dev->id == BME280_CHIP_ID) {
		// Write crtl hum reg first, only active after write to BMP280_REG_CTRL.
		uint8_t ctrl_hum = params->oversampling_humidity;
		if (write_register8(dev, BMP280_REG_CTRL_HUM, ctrl_hum)) {
			return false;
		}
	}

	if (write_register8(dev, BMP280_REG_CTRL, ctrl)) {
		return false;
	}

	return true;
}

bool bmp280_force_measurement(BMP280_HandleTypedef *dev) {
	uint8_t ctrl;
	if (read_data(dev, BMP280_REG_CTRL, &ctrl, 1))
		return false;
	ctrl &= ~0b11;  // clear two lower bits
	ctrl |= BMP280_MODE_FORCED;
	if (write_register8(dev, BMP280_REG_CTRL, ctrl)) {
		return false;
	}
	return true;
}

bool bmp280_is_measuring(BMP280_HandleTypedef *dev) {
	uint8_t status;
	if (read_data(dev, BMP280_REG_STATUS, &status, 1))
		return false;
	if (status & (1 << 3)) {
		return true;
	}
	return false;
}

/**
 * Compensation algorithm is taken from BMP280 datasheet.
 *
 * Return value is in degrees Celsius.
 */
static inline int32_t compensate_temperature(BMP280_HandleTypedef *dev, int32_t adc_temp,
		int32_t *fine_temp) {
	int32_t var1, var2;

	var1 = ((((adc_temp >> 3) - ((int32_t) dev->dig_T1 << 1)))
			* (int32_t) dev->dig_T2) >> 11;
	var2 = (((((adc_temp >> 4) - (int32_t) dev->dig_T1)
			* ((adc_temp >> 4) - (int32_t) dev->dig_T1)) >> 12)
			* (int32_t) dev->dig_T3) >> 14;

	*fine_temp = var1 + var2;
	return (*fine_temp * 5 + 128) >> 8;
}

/**
 * Compensation algorithm is taken from BMP280 datasheet.
 *
 * Return value is in Pa, 24 integer bits and 8 fractional bits.
 */
static inline uint32_t compensate_pressure(BMP280_HandleTypedef *dev, int32_t adc_press,
		int32_t fine_temp) {
	int64_t var1, var2, p;

	var1 = (int64_t) fine_temp - 128000;
	var2 = var1 * var1 * (int64_t) dev->dig_P6;
	var2 = var2 + ((var1 * (int64_t) dev->dig_P5) << 17);
	var2 = var2 + (((int64_t) dev->dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t) dev->dig_P3) >> 8)
			+ ((var1 * (int64_t) dev->dig_P2) << 12);
	var1 = (((int64_t) 1 << 47) + var1) * ((int64_t) dev->dig_P1) >> 33;

	if (var1 == 0) {
		return 0;  // avoid exception caused by division by zero
	}

	p = 1048576 - adc_press;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = ((int64_t) dev->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
	var2 = ((int64_t) dev->dig_P8 * p) >> 19;

	p = ((p + var1 + var2) >> 8) + ((int64_t) dev->dig_P7 << 4);
	return p;
}

/**
 * Compensation algorithm is taken from BME280 datasheet.
 *
 * Return value is in Pa, 24 integer bits and 8 fractional bits.
 */
static inline uint32_t compensate_humidity(BMP280_HandleTypedef *dev, int32_t adc_hum,
		int32_t fine_temp) {
	int32_t v_x1_u32r;

	v_x1_u32r = fine_temp - (int32_t) 76800;
	v_x1_u32r = ((((adc_hum << 14) - ((int32_t) dev->dig_H4 << 20)
			- ((int32_t) dev->dig_H5 * v_x1_u32r)) + (int32_t) 16384) >> 15)
			* (((((((v_x1_u32r * (int32_t) dev->dig_H6) >> 10)
					* (((v_x1_u32r * (int32_t) dev->dig_H3) >> 11)
							+ (int32_t) 32768)) >> 10) + (int32_t) 2097152)
					* (int32_t) dev->dig_H2 + 8192) >> 14);
	v_x1_u32r = v_x1_u32r
			- (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7)
					* (int32_t) dev->dig_H1) >> 4);
	v_x1_u32r = v_x1_u32r < 0 ? 0 : v_x1_u32r;
	v_x1_u32r = v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r;
	return v_x1_u32r >> 12;
}

bool bmp280_read_fixed(BMP280_HandleTypedef *dev, int32_t *temperature, uint32_t *pressure,
		uint32_t *humidity) {
	int32_t adc_pressure;
	int32_t adc_temp;
	uint8_t data[8];

	// Only the BME280 supports reading the humidity.
	if (dev->id != BME280_CHIP_ID) {
		if (humidity)
			*humidity = 0;
		humidity = NULL;
	}

	// Need to read in one sequence to ensure they match.
	size_t size = humidity ? 8 : 6;
	if (read_data(dev, 0xf7, data, size)) {
		return false;
	}

	adc_pressure = data[0] << 12 | data[1] << 4 | data[2] >> 4;
	adc_temp = data[3] << 12 | data[4] << 4 | data[5] >> 4;

	int32_t fine_temp;
	*temperature = compensate_temperature(dev, adc_temp, &fine_temp);
	*pressure = compensate_pressure(dev, adc_pressure, fine_temp);

	if (humidity) {
		int32_t adc_humidity = data[6] << 8 | data[7];
		*humidity = compensate_humidity(dev, adc_humidity, fine_temp);
	}

	return true;
}

bool bmp280_read_float(BMP280_HandleTypedef *dev, float *temperature, float *pressure,
		float *humidity) {
	int32_t fixed_temperature;
	uint32_t fixed_pressure;
	uint32_t fixed_humidity;
	if (bmp280_read_fixed(dev, &fixed_temperature, &fixed_pressure,
			humidity ? &fixed_humidity : NULL)) {
		*temperature = (float) fixed_temperature / 100;
		*pressure = (float) fixed_pressure / 256;
		if (humidity)
			*humidity = (float) fixed_humidity / 1024;
		return true;
	}

	return false;
}

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
BMP280_HandleTypedef bmp280;

float pressure, temperature, humidity;

uint16_t size;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *data, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)data, len, 100);
    return len;
}
void send_uart(const char *message) {
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	 HAL_Init();
	    SystemClock_Config();
	    MX_GPIO_Init();
	    MX_USART2_UART_Init();
	    MX_I2C1_Init();

	    bmp280_params_t params;
	    bmp280_init_default_params(&params);

	    BMP280_HandleTypedef bmp280;
	    bmp280.addr = BMP280_I2C_ADDRESS_0;
	    if (!bmp280_init(&bmp280, &params)) {
	        send_uart("BMP280 initialization failed\r\n");
	        while (1);
	    }

	    int32_t temperature;
	    uint32_t pressure, humidity;
	    char buffer[50];

	    while (1) {
	        bmp280_force_measurement(&bmp280);
	        HAL_Delay(100);

	        if (!bmp280_is_measuring(&bmp280)) {
	            if (bmp280_read_fixed(&bmp280, &temperature, &pressure, &humidity)) {
	                snprintf(buffer, sizeof(buffer), "Temperature: %ld.%02ld C\r\n", temperature / 100, temperature % 100);
	                send_uart(buffer);
	                snprintf(buffer, sizeof(buffer), "Pressure: %ld.%02ld Pa\r\n", pressure / 256, (pressure % 256) * 100 / 256);
	                send_uart(buffer);
	                snprintf(buffer, sizeof(buffer), "Humidity: %ld.%02ld %%\r\n", humidity / 1024, (humidity % 1024) * 100 / 1024);
	                send_uart(buffer);
	            } else {
	                send_uart("Failed to read values\r\n");
	            }
	        }
	        HAL_Delay(1000);
	    }

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
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
