/**
 ******************************************************************************
 * @file    max31855.h
 * @brief   Driver for the Maxim MAX31855 SPI thermocouple amplifier.
 *
 * SolarSense populates three MAX31855KASA+ (K-type) on SPI1:
 *   TC1 → PA4, TC2 → PB0, TC3 → PB1  (all active-low, software CS)
 *
 * The device is read-only; no configuration registers exist.  One 32-bit
 * SPI frame (two 16-bit HAL transactions) yields the thermocouple temp
 * (0.25 °C/LSB) and the on-chip cold-junction temp (0.0625 °C/LSB), plus
 * three open/short fault bits.
 ******************************************************************************
 */
#ifndef MAX31855_H
#define MAX31855_H

#include "stm32l4xx_hal.h"

/* Fault bits returned in the faults byte (mirrors hardware bits [2:0]). */
#define MAX31855_FAULT_OC   0x01U  /* thermocouple open circuit       */
#define MAX31855_FAULT_SCG  0x02U  /* thermocouple shorted to GND     */
#define MAX31855_FAULT_SCV  0x04U  /* thermocouple shorted to VCC     */

typedef struct {
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef      *cs_port;
  uint16_t           cs_pin;
  uint8_t            present;   /* 1 after a successful Init() SPI probe */
} MAX31855_t;

/* Bind the driver to an SPI bus and a software CS pin.  Performs one SPI
   read to confirm the device is wired.  Sets present = 1 on success. */
HAL_StatusTypeDef MAX31855_Init(MAX31855_t *dev, SPI_HandleTypeDef *hspi,
                                GPIO_TypeDef *cs_port, uint16_t cs_pin);

/* Read thermocouple and cold-junction temperatures (tenths of °C, signed)
   and the fault byte.  Any output pointer may be NULL.
   Returns HAL_OK on a clean read; HAL_ERROR if the SPI transaction fails.
   A non-zero *faults value means the thermocouple wire has a wiring fault
   and *tc10 is unreliable, but *cj10 (board ambient) remains valid. */
HAL_StatusTypeDef MAX31855_Read(MAX31855_t *dev,
                                int32_t *tc10, int32_t *cj10,
                                uint8_t *faults);

#endif /* MAX31855_H */
