/**
 ******************************************************************************
 * @file    bmp280.h
 * @brief   Driver for the Bosch BMP280 barometric pressure + temperature sensor.
 *
 * SolarSense populates one BMP280 (U14) on I2C1 with SDO tied low, giving the
 * 7-bit address 0x76. The part measures atmospheric pressure and a board-local
 * temperature (biased by PCB self-heating, so it is logged as board/PCB temp
 * rather than ambient -- see DEVLOG). No humidity channel (that is the BME280).
 *
 * The factory trimming coefficients are read once at init and the fixed-point
 * compensation formulas from the datasheet (section 3.11.3) are evaluated on
 * the MCU: temperature in centi-degrees Celsius and pressure in pascals.
 ******************************************************************************
 */
#ifndef BMP280_H
#define BMP280_H

#include "stm32l4xx_hal.h"

/* 7-bit I2C address with SDO -> GND (0x77 when SDO -> VDDIO). Many breakout
   modules float SDO high, so BMP280_Init() probes both. */
#define BMP280_ADDR        0x76U
#define BMP280_ADDR_ALT    0x77U

/* Chip-id register (0xD0). 0x58 = genuine BMP280; 0x60 = BME280, which many
   "BMP280" modules actually carry -- its temp/pressure map and compensation
   are identical, so we accept it (humidity channel simply goes unused). */
#define BMP280_CHIP_ID     0x58U
#define BME280_CHIP_ID     0x60U

typedef struct {
  I2C_HandleTypeDef *hi2c;
  uint8_t  addr7;        /* 7-bit device address actually used            */
  uint8_t  present;      /* 1 once BMP280_Init() has confirmed the part    */
  uint8_t  chip_id;      /* last chip-id byte read (0x58/0x60, or 0xFF)    */

  /* Factory calibration (read from 0x88..0x9F at init). */
  uint16_t dig_T1;
  int16_t  dig_T2, dig_T3;
  uint16_t dig_P1;
  int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

  int32_t  t_fine;       /* carries temperature term into pressure comp.   */
} BMP280_t;

/* Probe the chip id, soft-reset, read the calibration, and configure for
   indoor-navigation-style sampling (x2 temp, x16 pressure, IIR x16, normal
   mode @ ~0.5 s standby). addr7 is tried first, then the alternate address;
   the one that ACKs with a known chip id is adopted (dev->addr7/chip_id record
   what was found). Sets present=1 on success; returns HAL_ERROR (and leaves
   present=0, chip_id = last byte seen) if neither address yields a known part. */
HAL_StatusTypeDef BMP280_Init(BMP280_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7);

/* Read and compensate both channels. temp_c100 is centi-degrees Celsius
   (2345 = 23.45 C); press_pa is absolute pressure in pascals. Either output
   pointer may be NULL. */
HAL_StatusTypeDef BMP280_Read(BMP280_t *dev, int32_t *temp_c100, uint32_t *press_pa);

#endif /* BMP280_H */
