/**
 *
 * ESP-Drone Firmware
 *
 * @file bmp280.h
 * Driver for the BMP280 pressure sensor porting for ESP-Drone
 *
 */
#ifndef BMP280_H
#define BMP280_H

#include <stdbool.h>
#include "i2cdev.h"

// --- I2C Addresses ---
#define BMP280_I2C_ADDR_LOW  0x76
#define BMP280_I2C_ADDR_HIGH 0x77
#define BMP280_I2C_ADDR      BMP280_I2C_ADDR_LOW

// --- Chip ID & Reset Value ---
#define BMP280_CHIP_ID       0x58
#define BMP280_RESET_VALUE   0xB6
#define BMP280_D1D2_SIZE 3

// --- Memory Registers ---
#define BMP280_REG_CALIB_00  0x88
#define BMP280_REG_CHIPID    0xD0
#define BMP280_REG_SOFTRESET 0xE0
#define BMP280_REG_STATUS    0xF3
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG    0xF5
#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_REG_TEMP_MSB  0xFA

// --- Oversampling (CTRL_MEAS) ---
#define BMP280_SAMPLING_NONE 0x00
#define BMP280_SAMPLING_X1   0x01
#define BMP280_SAMPLING_X2   0x02
#define BMP280_SAMPLING_X4   0x03
#define BMP280_SAMPLING_X8   0x04
#define BMP280_SAMPLING_X16  0x05

// --- Power Modes (CTRL_MEAS) ---
#define BMP280_MODE_SLEEP    0x00
#define BMP280_MODE_FORCED   0x01
#define BMP280_MODE_NORMAL   0x03

// --- Standby Times (CONFIG) ---
#define BMP280_STANDBY_MS_0_5   0x00
#define BMP280_STANDBY_MS_62_5  0x01
#define BMP280_STANDBY_MS_125   0x02
#define BMP280_STANDBY_MS_250   0x03
#define BMP280_STANDBY_MS_500   0x04
#define BMP280_STANDBY_MS_1000  0x05
#define BMP280_STANDBY_MS_10    0x06
#define BMP280_STANDBY_MS_20    0x07

// --- IIR Filter (CONFIG) ---
#define BMP280_FILTER_OFF 0x00
#define BMP280_FILTER_X2  0x01
#define BMP280_FILTER_X4  0x02
#define BMP280_FILTER_X8  0x03
#define BMP280_FILTER_X16 0x04

// --- Self Test Parameters ---
#define BMP280_ST_PRESS_MAX   (1100.0) //mbar
#define BMP280_ST_PRESS_MIN   (450.0)  //mbar
#define BMP280_ST_TEMP_MAX    (60.0)   //degree celcius
#define BMP280_ST_TEMP_MIN    (-20.0)  //degree celcius

// --- Constants ---
#define CONST_SEA_PRESSURE 102610.f 
#define CONST_PF 0.1902630958 
#define CONST_PF2 44330.0f

// --- Functions Declaration (Mirrored from bmp280) ---
bool bmp280Init(I2C_Dev *i2cPort);
bool bmp280SelfTest(void);
bool bmp280EvaluateSelfTest(float min, float max, float value, char *string);
float bmp280GetPressure(uint8_t osr);
float bmp280CalcPressure(int32_t rawPress, int32_t dT);
float bmp280GetTemperature(uint8_t osr);
float bmp280CalcTemp(int32_t deltaT);
int32_t bmp280GetDeltaTemp(uint8_t osr);
int32_t bmp280CalcDeltaTemp(int32_t rawTemp);
int32_t bmp280RawPressure(uint8_t osr);
int32_t bmp280RawTemperature(uint8_t osr);
bool bmp280ReadPROM();
void bmp280Reset();
void bmp280StartConversion(uint8_t command);
int32_t bmp280GetConversion(uint8_t command);

void bmp280GetData(float *pressure, float *temperature, float *asl);
float bmp280PressureToAltitude(float *pressure);

#endif // BMP280_H