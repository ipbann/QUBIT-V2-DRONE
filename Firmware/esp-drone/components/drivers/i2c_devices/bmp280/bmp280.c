/**
 *
 * ESP-Drone Firmware
 *
 * @file bmp280.c
 * Driver for the BMP280 pressure sensor porting for ESP-Drone
 *
 */
#define DEBUG_MODULE "BMP280"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bmp280.h"
#include "i2cdev.h"
#include "debug_cf.h"
#include "eprintf.h"
#include "math.h"

#define CONVERSION_TIME_MS   10 
#define FIX_TEMP 25

static uint8_t devAddr;
static I2C_Dev *I2Cx;
static bool isInit = false;

// Variabel Kalibrasi Factory BMP280
static uint16_t _dig_T1;
static int16_t  _dig_T2, _dig_T3;
static uint16_t _dig_P1;
static int16_t  _dig_P2, _dig_P3, _dig_P4, _dig_P5, _dig_P6, _dig_P7, _dig_P8, _dig_P9;
static int32_t  _t_fine;

static uint32_t lastConv = 0;
static float savedPress = 0;
static float savedTemp = 0;

// Keyword static dihilangkan agar sesuai dengan bmp280.h
float bmp280PressureToAltitude(float *pressure) {
    if (*pressure > 0) {
        return ((powf((1015.7f / *pressure), CONST_PF) - 1.0f) * (FIX_TEMP + 273.15f)) / 0.0065f;
    }
    return 0;
}

bool bmp280Init(I2C_Dev *i2cPort)
{
    if (isInit) return true;

    I2Cx = i2cPort;
    devAddr = BMP280_I2C_ADDR;

    uint8_t chip_ID;
    // Diperbaiki: penambahan argumen '1' untuk parameter 'len'
    i2cdevReadReg8(I2Cx, devAddr, BMP280_REG_CHIPID, 1, &chip_ID);
    if (chip_ID != BMP280_CHIP_ID) {
        DEBUG_PRINTI("BMP280 [FAIL] ID: 0x%X\n", chip_ID);
        return false;
    }
    DEBUG_PRINTI("BMP280 [OK]\n");

    // Soft reset
    i2cdevWriteByte(I2Cx, devAddr, BMP280_REG_SOFTRESET, BMP280_RESET_VALUE);
    vTaskDelay(M2T(10));

    // Tunggu NVM selesai
    uint8_t stat_Reg = 1;
    while ((stat_Reg & 0x01) == 1) {
        // Diperbaiki: penambahan argumen '1'
        i2cdevReadReg8(I2Cx, devAddr, BMP280_REG_STATUS, 1, &stat_Reg);
        vTaskDelay(M2T(1));
    }

    // Ambil Data Kalibrasi
    uint8_t calib[24];
    i2cdevWriteByte(I2Cx, devAddr, I2CDEV_NO_MEM_ADDR, BMP280_REG_CALIB_00);
    i2cdevRead(I2Cx, devAddr, 24, calib);

    _dig_T1 = (calib[1] << 8) | calib[0];
    _dig_T2 = (int16_t)((calib[3] << 8) | calib[2]);
    _dig_T3 = (int16_t)((calib[5] << 8) | calib[4]);
    _dig_P1 = (calib[7] << 8) | calib[6];
    _dig_P2 = (int16_t)((calib[9] << 8) | calib[8]);
    _dig_P3 = (int16_t)((calib[11] << 8) | calib[10]);
    _dig_P4 = (int16_t)((calib[13] << 8) | calib[12]);
    _dig_P5 = (int16_t)((calib[15] << 8) | calib[14]);
    _dig_P6 = (int16_t)((calib[17] << 8) | calib[16]);
    _dig_P7 = (int16_t)((calib[19] << 8) | calib[18]);
    _dig_P8 = (int16_t)((calib[21] << 8) | calib[20]);
    _dig_P9 = (int16_t)((calib[23] << 8) | calib[22]);

    // Masuk mode sleep sebelum setting
    i2cdevWriteByte(I2Cx, devAddr, BMP280_REG_CTRL_MEAS, BMP280_MODE_SLEEP);

    // Filter x16, Standby 0.5ms
    uint8_t configValues = (BMP280_STANDBY_MS_0_5 << 5) | (BMP280_FILTER_X16 << 2); 
    i2cdevWriteByte(I2Cx, devAddr, BMP280_REG_CONFIG, configValues);

    // Temp x1, Press x4, Normal Mode
    uint8_t ctrlMeas = (BMP280_SAMPLING_X1 << 5) | (BMP280_SAMPLING_X4 << 2) | BMP280_MODE_NORMAL;
    i2cdevWriteByte(I2Cx, devAddr, BMP280_REG_CTRL_MEAS, ctrlMeas);

    vTaskDelay(M2T(100)); // Tunggu pengukuran pertama

    isInit = true;
    return true;
}

bool bmp280SelfTest(void)
{
    return isInit; 
}

void bmp280GetData(float *pressure, float *temperature, float *asl)
{
    uint32_t now = xTaskGetTickCount();

    // Hindari pembacaan melebihi I2C limit
    if ((now - lastConv) < CONVERSION_TIME_MS) {
        *pressure = savedPress;
        *temperature = savedTemp;
        *asl = bmp280PressureToAltitude(pressure);
        return;
    }
    lastConv = now;

    // Baca data mentah 6 Byte dari register 0xF7
    uint8_t data[6];
    i2cdevWriteByte(I2Cx, devAddr, I2CDEV_NO_MEM_ADDR, BMP280_REG_PRESS_MSB);
    i2cdevRead(I2Cx, devAddr, 6, data);

    int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);

    // Kalkulasi Suhu
    int32_t var1_T = ((((adc_T >> 3) - ((int32_t)_dig_T1 << 1))) * ((int32_t)_dig_T2)) >> 11;
    int32_t var2_T = (((((adc_T >> 4) - ((int32_t)_dig_T1)) * ((adc_T >> 4) - ((int32_t)_dig_T1))) >> 12) * ((int32_t)_dig_T3)) >> 14;
    _t_fine = var1_T + var2_T;
    float T = (((_t_fine * 5) + 128) >> 8);
    *temperature = T / 100.0f;

    // Kalkulasi Tekanan
    int64_t var1_P, var2_P, p;
    var1_P = ((int64_t)_t_fine) - 128000;
    var2_P = var1_P * var1_P * (int64_t)_dig_P6;
    var2_P = var2_P + ((var1_P * (int64_t)_dig_P5) << 17);
    var2_P = var2_P + (((int64_t)_dig_P4) << 35);
    var1_P = ((var1_P * var1_P * (int64_t)_dig_P3) >> 8) + ((var1_P * (int64_t)_dig_P2) << 12);
    var1_P = (((((int64_t)1) << 47) + var1_P)) * ((int64_t)_dig_P1) >> 33;

    if (var1_P == 0) {
        *pressure = savedPress;
    } else {
        p = 1048576 - adc_P;
        p = (((p << 31) - var2_P) * 3125) / var1_P;
        var1_P = (((int64_t)_dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        var2_P = (((int64_t)_dig_P8) * p) >> 19;
        p = ((p + var1_P + var2_P) >> 8) + (((int64_t)_dig_P7) << 4);
        
        // Final hPa
        *pressure = (float)p / 25600.0f;
    }

    savedTemp = *temperature;
    savedPress = *pressure;
    *asl = bmp280PressureToAltitude(pressure);
}

// --- DUMMY FUNCTIONS (Pencegah Linker Error) ---
bool bmp280EvaluateSelfTest(float min, float max, float value, char *string) { return true; }
float bmp280GetPressure(uint8_t osr) { return 0; }
float bmp280CalcPressure(int32_t rawPress, int32_t dT) { return 0; }
float bmp280GetTemperature(uint8_t osr) { return 0; }
float bmp280CalcTemp(int32_t deltaT) { return 0; }
int32_t bmp280GetDeltaTemp(uint8_t osr) { return 0; }
int32_t bmp280CalcDeltaTemp(int32_t rawTemp) { return 0; }
int32_t bmp280RawPressure(uint8_t osr) { return 0; }
int32_t bmp280RawTemperature(uint8_t osr) { return 0; }
bool bmp280ReadPROM() { return true; }
void bmp280Reset() {}
void bmp280StartConversion(uint8_t command) {}
int32_t bmp280GetConversion(uint8_t command) { return 0; }