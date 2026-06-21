/**
 ******************************************************************************
 * @file    ina219.h
 * @brief   Driver for the TI INA219(B) high-side current/voltage monitor.
 *
 * SolarSense uses three INA219BIDCNR, one per solar panel branch, all on I2C1:
 *   Panel 1 -> 0x40, Panel 2 -> 0x41, Panel 3 -> 0x44
 * (Panel 4 @0x45 / Panel 5 @0x41-on-I2C3 are DNP provisions.)
 *
 * Shunt = 100 mOhm, configured for the +/-40 mV PGA range (0-400 mA FSR).
 * Rather than program the calibration register, this driver reads the raw
 * shunt- and bus-voltage registers and computes current/power on the MCU,
 * which is exact for the fixed 100 mOhm shunt and avoids CAL rounding.
 ******************************************************************************
 */
#ifndef INA219_H
#define INA219_H

#include "stm32l4xx_hal.h"

/* 7-bit I2C addresses of the three populated panel monitors. */
#define INA219_ADDR_PANEL1   0x40U
#define INA219_ADDR_PANEL2   0x41U
#define INA219_ADDR_PANEL3   0x44U

/* Shunt resistor value, milliohms (100 mOhm, 1%). */
#define INA219_RSHUNT_MOHM   100

typedef struct {
  I2C_HandleTypeDef *hi2c;
  uint8_t  addr7;      /* 7-bit device address                       */
  uint8_t  present;    /* 1 once INA219_Init() has confirmed the part */
} INA219_t;

/* Configure the device: 16 V bus range, +/-40 mV PGA, 12-bit ADC with 8x
   averaging on both channels, continuous shunt+bus mode. Sets present=1 on
   success. Returns HAL_ERROR (and leaves present=0) if the part does not ACK. */
HAL_StatusTypeDef INA219_Init(INA219_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7);

/* Bus (panel terminal) voltage in millivolts. */
HAL_StatusTypeDef INA219_ReadBus_mV(INA219_t *dev, int32_t *mv);

/* Shunt voltage in microvolts (signed; +ve = current into the load). */
HAL_StatusTypeDef INA219_ReadShunt_uV(INA219_t *dev, int32_t *uv);

/* Branch current in microamps, derived from the shunt voltage and
   INA219_RSHUNT_MOHM (signed). */
HAL_StatusTypeDef INA219_ReadCurrent_uA(INA219_t *dev, int32_t *ua);

/* Panel power in milliwatts (bus_mV * current / shunt), >= 0 for normal flow. */
HAL_StatusTypeDef INA219_ReadPower_mW(INA219_t *dev, int32_t *mw);

#endif /* INA219_H */
