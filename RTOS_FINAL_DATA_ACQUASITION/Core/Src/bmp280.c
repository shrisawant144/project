/*
 * bmp280.c
 *
 *  Created on: Jul 30, 2024
 *      Author: shrikrishna
 */

#include "bmp280.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
/* Declare external I2C handle */
extern I2C_HandleTypeDef hi2c1;

/* Declare external UART handle for debugging */
extern UART_HandleTypeDef huart2;
static inline int read_data(BMP280_HandleTypedef *dev, uint8_t addr, uint8_t *value,uint8_t len);
static inline int32_t compensate_temperature(BMP280_HandleTypedef *dev, int32_t adc_temp,int32_t *fine_temp);
static inline uint32_t compensate_pressure(BMP280_HandleTypedef *dev, int32_t adc_press,int32_t fine_temp);
static inline uint32_t compensate_humidity(BMP280_HandleTypedef *dev, int32_t adc_hum,int32_t fine_temp);
void uart_debug_printf(const char* fmt, ...) {
    char buffer[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void bmp280_init_default_params(bmp280_params_t *params) {
    params->mode = BMP280_MODE_NORMAL;
    params->filter = BMP280_FILTER_OFF;
    params->oversampling_pressure = BMP280_STANDARD;
    params->oversampling_temperature = BMP280_STANDARD;
    params->oversampling_humidity = BMP280_STANDARD;
    params->standby = BMP280_STANDBY_250;
}

bool read_register16(BMP280_HandleTypedef *dev, uint8_t addr, uint16_t *value) {
    uint16_t tx_buff;
    uint8_t rx_buff[2];
    tx_buff = (dev->addr << 1);

    uart_debug_printf("Reading register 0x%02X\r\n", addr);

    if (HAL_I2C_Mem_Read(&hi2c1, tx_buff, addr, 1, rx_buff, 2, 5000) == HAL_OK) {
        *value = (uint16_t) ((rx_buff[1] << 8) | rx_buff[0]);
        uart_debug_printf("Read 0x%04X from register 0x%02X\r\n", *value, addr);
        return true;
    } else {
        uart_debug_printf("Failed to read from register 0x%02X\r\n", addr);
        return false;
    }
}

bool read_calibration_data(BMP280_HandleTypedef *dev) {
    uart_debug_printf("Reading calibration data\n");

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
        && read_register16(dev, 0x9e, (uint16_t *) &dev->dig_P9)) {
        uart_debug_printf("Successfully read calibration data\r\n");
        return true;
    }

    uart_debug_printf("Failed to read calibration data\r\n");
    return false;
}

bool read_hum_calibration_data(BMP280_HandleTypedef *dev) {
    uint16_t h4, h5;

    uart_debug_printf("Reading humidity calibration data\r\n");

    if (!read_data(dev, 0xa1, &dev->dig_H1, 1)
        && read_register16(dev, 0xe1, (uint16_t *) &dev->dig_H2)
        && !read_data(dev, 0xe3, &dev->dig_H3, 1)
        && read_register16(dev, 0xe4, &h4)
        && read_register16(dev, 0xe5, &h5)
        && !read_data(dev, 0xe7, (uint8_t *) &dev->dig_H6, 1)) {
        dev->dig_H4 = (h4 & 0x00ff) << 4 | (h4 & 0x0f00) >> 8;
        dev->dig_H5 = h5 >> 4;

        uart_debug_printf("Successfully read humidity calibration data\r\n");
        return true;
    }

    uart_debug_printf("Failed to read humidity calibration data\r\n");
    return false;
}

int write_register8(BMP280_HandleTypedef *dev, uint8_t addr, uint8_t value) {
    uint16_t tx_buff;
    tx_buff = (dev->addr << 1);

    uart_debug_printf("Writing 0x%02X to register 0x%02X\r\n", value, addr);

    if (HAL_I2C_Mem_Write(&hi2c1, tx_buff, addr, 1, &value, 1, 10000) == HAL_OK) {
        uart_debug_printf("Successfully wrote to register 0x%02X\r\n", addr);
        return false;
    } else {
        uart_debug_printf("Failed to write to register 0x%02X\r\n", addr);
        return true;
    }
}

bool bmp280_init(BMP280_HandleTypedef *dev, bmp280_params_t *params) {
    uart_debug_printf("Initializing BMP280\r\n");

    if (dev->addr != BMP280_I2C_ADDRESS_0 && dev->addr != BMP280_I2C_ADDRESS_1) {
        uart_debug_printf("Invalid BMP280 I2C address\r\n");
        return false;
    }

    if (read_data(dev, BMP280_REG_ID, &dev->id, 1)) {
        uart_debug_printf("Failed to read BMP280 ID\r\n");
        return false;
    }

    uart_debug_printf("BMP280 ID: 0x%02X\n", dev->id);

    if (dev->id != BMP280_CHIP_ID && dev->id != BME280_CHIP_ID) {
        uart_debug_printf("Invalid BMP280 ID\r\n");
        return false;
    }

    // Soft reset.
    if (write_register8(dev, BMP280_REG_RESET, BMP280_RESET_VALUE)) {
        uart_debug_printf("Failed to reset BMP280\r\n");
        return false;
    }

    // Wait until finished copying over the NVP data.
    while (1) {
        uint8_t status;
        if (!read_data(dev, BMP280_REG_STATUS, &status, 1) && (status & 1) == 0)
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

    uart_debug_printf("BMP280 initialized successfully\n");
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
static inline int read_data(BMP280_HandleTypedef *dev, uint8_t addr, uint8_t *value,uint8_t len) {
	uint16_t tx_buff;
	tx_buff = (dev->addr << 1);
	if (HAL_I2C_Mem_Read(&hi2c1, tx_buff, addr, 1, value, len, 5000) == HAL_OK)
		return 0;
	else
		return 1;

}

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
