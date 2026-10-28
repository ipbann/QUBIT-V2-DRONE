/**
 *
 * ESP-Drone Firmware
 * 
 * Copyright 2019-2020  Espressif Systems (Shanghai) 
 * Copyright (C) 2012 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * vl53l0x.c: Time-of-flight distance sensor driver
 */

#define DEBUG_MODULE "VLX"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "system.h"
#include "debug_cf.h"
#include "log.h"
#include "param.h"
#include "range.h"
#include "config.h"
#include "i2cdev.h"
#include "zranger.h"
#include "vl53l0x.h"
#include "stabilizer_types.h"
#include "estimator.h"
#include "cf_math.h"

// Measurement noise model
static const float expPointA = 1.0f;
static const float expStdA = 0.0025f; // STD at elevation expPointA [m]
static const float expPointB = 1.3f;
static const float expStdB = 0.2f;    // STD at elevation expPointB [m]
static float expCoeff;

#define RANGE_OUTLIER_LIMIT 3000 // the measured range is in [mm]

static uint16_t range_last = 0;
static uint16_t last_update_tick = 0;
static bool isInit;
static VL53L0xDev dev;

void zRangerInit(void)
{
  if (isInit)
    return;

  dev.I2Cx = 0;
  dev.devAddr = VL53L0X_DEFAULT_ADDRESS;
  dev.I2Cx = I2C0_DEV;
  i2cdevInit(dev.I2Cx);

  dev.io_timeout = 2000; // Timeout 2 detik yang aman
  dev.did_timeout = 0;
  dev.timeout_start_ms = 0;
  dev.stop_variable = 0;
  dev.measurement_timing_budget_us = 0;
  dev.measurement_timing_budget_ms = 0;

  vTaskDelay(M2T(20));

  // 1. HARD RESET SOFTWARE: Memastikan memori sensor bersih
  DEBUG_PRINTI("VL53L0X Soft Reset...\n");
  i2cdevWriteByte(dev.I2Cx, dev.devAddr, 0xBF, 0x00);
  vTaskDelay(M2T(20));
  i2cdevWriteByte(dev.I2Cx, dev.devAddr, 0xBF, 0x01);
  vTaskDelay(M2T(50));

  uint16_t wordData;
  wordData = vl53l0xGetModelID(&dev);
  DEBUG_PRINTI("VL53L0X ID: %02X\n", wordData);

  // 2. INISIALISASI: Harus dipanggil dengan parameter io_2V8 = true
  if (!vl53l0xInit(&dev, I2C1_DEV, true)) {
     DEBUG_PRINTW(">>> ERROR: Kalibrasi VL53L0X GAGAL! <<<\n");
  } else {
     DEBUG_PRINTI(">>> SUKSES: Kalibrasi VL53L0X BERHASIL! <<<\n");
  }

  // Buat Task untuk pembacaan jarak (ditidurkan hingga sistem siap)
  xTaskCreate(zRangerTask, ZRANGER_TASK_NAME, ZRANGER_TASK_STACKSIZE, NULL, ZRANGER_TASK_PRI, NULL);

  // Pre-compute constant in the measurement noise model for kalman
  expCoeff = logf(expStdB / expStdA) / (expPointB - expPointA);

  isInit = true;
}

bool zRangerTest(void)
{
  bool testStatus;

  if (!isInit)
    return false;

  testStatus  = vl53l0xTestConnection(&dev);

  return testStatus;
}

void zRangerTask(void* arg)
{
  // Tunggu semua kalibrasi selesai (MPU6050 dll)
  systemWaitStart();
  TickType_t xLastWakeTime;

  // 3. Mulai ambil data beruntun (Continuous) tanpa modifikasi pulse period
  vl53l0xStartContinuous(&dev, 0);
  
  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    // Jeda 30ms (Sesuai timing budget sensor Default)
    vTaskDelayUntil(&xLastWakeTime, M2T(30));

    uint8_t val = 0;
    
    // 4. BACA INTERUPSI (Non-Blocking Polling)
    i2cdevReadByte(dev.I2Cx, dev.devAddr, VL53L0X_RA_RESULT_INTERRUPT_STATUS, &val);

    if ((val & 0x07) != 0) { 
      
      // Ambil jarak dari register
      uint16_t range = vl53l0xReadReg16Bit(&dev, VL53L0X_RA_RESULT_RANGE_STATUS + 10);
      
      // Bersihkan interupsi agar sensor mengukur data berikutnya
      i2cdevWriteByte(dev.I2Cx, dev.devAddr, VL53L0X_RA_SYSTEM_INTERRUPT_CLEAR, 0x01);

      // Saring error 65535 dan batas rasional
      // Di dalam if (range > 0 && range < 8000)
      if (range > 0 && range < 8000) {
        range_last = range;
        last_update_tick = xTaskGetTickCount(); // <--- TAMBAHKAN BARIS INI
        float distance = (float)range_last * 0.001f; 
        
        // DEBUG_PRINTI("ZRANGE = %f\n", distance); // (Sebaiknya di-comment agar log terminal tidak berat)
        
        if (range_last < RANGE_OUTLIER_LIMIT) {
          float stdDev = expStdA * (1.0f  + expf( expCoeff * (distance - expPointA)));
          rangeEnqueueDownRangeInEstimator(distance, stdDev, last_update_tick); // Gunakan last_update_tick
        }
      }
    }
  }
}

bool zRangerReadRange(zDistance_t* zrange, const uint32_t tick)
{
  bool updated = false;

  if (isInit) {
    // Cek: Apakah datanya ada, masuk akal, DAN belum kedaluwarsa (umur < 100ms)?
    if (range_last != 0 && range_last < RANGE_OUTLIER_LIMIT && (tick - last_update_tick) < M2T(100)) {
      zrange->distance = (float)range_last * 0.001f; // Skala ke meter
      zrange->timestamp = last_update_tick; // Gunakan stempel waktu asli data, bukan tick baru
      updated = true;
    }
  }
  return updated;
}

// -------------------------------------------------------------
// DECK DRIVER DIBIARKAN TER-COMMENT KARENA CUSTOM BOARD
// -------------------------------------------------------------
// static const DeckDriver zranger_deck = {
//   .vid = 0xBC,
//   .pid = 0x09,
//   .name = "bcZRanger",
//   .usedGpio = 0x0C,

//   .init = zRangerInit,
//   .test = zRangerTest,
// };

// DECK_DRIVER(zranger_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcZRanger, &isInit)
PARAM_GROUP_STOP(deck)