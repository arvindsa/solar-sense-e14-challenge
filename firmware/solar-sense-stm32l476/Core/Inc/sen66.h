/**
 ******************************************************************************
 * @file    sen66.h
 * @brief   Driver for the Sensirion SEN66 environmental sensor.
 *
 * The SEN66 combines a laser particle counter (PM1.0 / PM2.5 / PM4.0 / PM10),
 * a capacitive humidity+temperature sensor, a VOC index engine, a NOx index
 * engine, and a CO2 sensor (NDIR) in one module.  It communicates over I2C at
 * a fixed 7-bit address of 0x6B with CRC-8/SENSIRON (poly 0x31, init 0xFF)
 * appended to every word in the response.
 *
 * SolarSense connects the SEN66 to I2C1 (the shared sensor bus on PB8/PB9).
 * The part draws ~60–70 mA while measuring; a Sleep command reduces this to
 * < 0.4 mA for battery-conserving stop intervals.
 ******************************************************************************
 */
#ifndef SEN66_H
#define SEN66_H

#include "stm32l4xx_hal.h"

#define SEN66_ADDR  0x6BU   /* 7-bit I2C address (fixed) */

typedef struct {
  I2C_HandleTypeDef *hi2c;
  uint8_t present;    /* 1 once SEN66_Init() confirms the part    */
  uint8_t measuring;  /* 1 when continuous measurement is running */
} SEN66_t;

/**
 * Probe the device, issue a soft-reset, and start continuous measurement.
 * Returns HAL_OK and sets present=1 on success.
 *
 * Output scaling (applied inside this driver before returning):
 *   pm*   tenths µg/m³  (123 = 12.3 µg/m³)
 *   rh    tenths %RH    (554 = 55.4 %)
 *   t     centi-°C      (2340 = 23.40 °C)
 *   voc   VOC index     (100 = nominal clean air; range 1–500)
 *   nox   NOx index     (1 = clean air; range 1–500)
 *   co2   ppm           (420 = 420 ppm; range 400–40000)
 *
 * Values are 0 / invalid if the sensor reports 0xFFFF (not yet ready after
 * startup warm-up); SEN66_Read() returns HAL_ERROR in that case.
 */
HAL_StatusTypeDef SEN66_Init(SEN66_t *dev, I2C_HandleTypeDef *hi2c);

/**
 * Fetch the latest completed measurement from the sensor.  Any pointer may be
 * NULL.  Returns HAL_ERROR if the sensor is absent, not measuring, or data is
 * not yet valid (first ~1 s after StartMeasurement).
 */
HAL_StatusTypeDef SEN66_Read(SEN66_t *dev,
                              int32_t *pm10,  int32_t *pm25,
                              int32_t *pm40,  int32_t *pm100,
                              int32_t *rh,    int32_t *t,
                              int32_t *voc,   int32_t *nox,
                              int32_t *co2);

/** Put the SEN66 into sleep mode (< 0.4 mA). */
HAL_StatusTypeDef SEN66_Sleep(SEN66_t *dev);

/** Wake from sleep and restart continuous measurement. */
HAL_StatusTypeDef SEN66_Wake(SEN66_t *dev);

/** Stop continuous measurement without entering sleep. */
HAL_StatusTypeDef SEN66_StopMeasurement(SEN66_t *dev);

/** (Re-)start continuous measurement. */
HAL_StatusTypeDef SEN66_StartMeasurement(SEN66_t *dev);

#endif /* SEN66_H */
