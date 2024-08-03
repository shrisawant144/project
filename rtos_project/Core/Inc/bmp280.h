/*
 * bmp280.h
 *
 *  Created on: Jul 30, 2024
 *      Author: shrikrishna
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#ifndef BMP280_H
#define BMP280_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* BMP280 or BME280 address */
#define BMP280_I2C_ADDRESS_0  0x76
#define BMP280_I2C_ADDRESS_1  0x77

#define BMP280_CHIP_ID  0x58 /* BMP280 has chip-id 0x58 */
#define BME280_CHIP_ID  0x60 /* BME280 has chip-id 0x60 */

/* BMP280 Registers */
#define BMP280_REG_TEMP_XLSB   0xFC
#define BMP280_REG_TEMP_LSB    0xFB
#define BMP280_REG_TEMP_MSB    0xFA
#define BMP280_REG_PRESS_XLSB  0xF9
#define BMP280_REG_PRESS_LSB   0xF8
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_CONFIG      0xF5
#define BMP280_REG_CTRL        0xF4
#define BMP280_REG_STATUS      0xF3
#define BMP280_REG_CTRL_HUM    0xF2
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_ID          0xD0
#define BMP280_REG_CALIB       0x88
#define BMP280_REG_HUM_CALIB   0x88

#define BMP280_RESET_VALUE     0xB6

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

typedef enum {
    BMP280_SKIPPED = 0,
    BMP280_ULTRA_LOW_POWER = 1,
    BMP280_LOW_POWER = 2,
    BMP280_STANDARD = 3,
    BMP280_HIGH_RES = 4,
    BMP280_ULTRA_HIGH_RES = 5
} BMP280_Oversampling;

typedef enum {
    BMP280_STANDBY_05 = 0,
    BMP280_STANDBY_62 = 1,
    BMP280_STANDBY_125 = 2,
    BMP280_STANDBY_250 = 3,
    BMP280_STANDBY_500 = 4,
    BMP280_STANDBY_1000 = 5,
    BMP280_STANDBY_2000 = 6,
    BMP280_STANDBY_4000 = 7
} BMP280_StandbyTime;

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

    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;

    uint16_t addr;
    bmp280_params_t params;
    uint8_t  id;
} BMP280_HandleTypedef;


void bmp280_init_default_params(bmp280_params_t *params);
bool read_register16(BMP280_HandleTypedef *dev, uint8_t addr, uint16_t *value);
bool read_calibration_data(BMP280_HandleTypedef *dev);
bool read_hum_calibration_data(BMP280_HandleTypedef *dev);
int write_register8(BMP280_HandleTypedef *dev, uint8_t addr, uint8_t value);
bool bmp280_init(BMP280_HandleTypedef *dev, bmp280_params_t *params);
bool bmp280_force_measurement(BMP280_HandleTypedef *dev);
bool bmp280_is_measuring(BMP280_HandleTypedef *dev);
bool bmp280_read_fixed(BMP280_HandleTypedef *dev, int32_t *temperature, uint32_t *pressure, uint32_t *humidity);
bool bmp280_read_float(BMP280_HandleTypedef *dev, float *temperature, float *pressure, float *humidity);



#endif /* BMP280_H */


#endif /* INC_BMP280_H_ */
