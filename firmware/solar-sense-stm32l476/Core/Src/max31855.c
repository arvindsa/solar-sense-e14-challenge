/**
 ******************************************************************************
 * @file    max31855.c
 * @brief   Driver for the Maxim MAX31855 SPI thermocouple amplifier.
 ******************************************************************************
 */
#include "max31855.h"

#define SPI_TIMEOUT_MS 10U

/* Pull CS low, clock out 32 bits as two 16-bit HAL frames, release CS. */
static HAL_StatusTypeDef read_raw(MAX31855_t *dev, uint32_t *raw)
{
  uint16_t w[2];
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef st = HAL_SPI_Receive(dev->hspi, (uint8_t *)w, 2, SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
  if (st == HAL_OK)
  {
    /* MAX31855 sends MSB first; SPI1 is configured MSB-first 16-bit mode,
       so w[0] = bits[31:16], w[1] = bits[15:0]. */
    *raw = ((uint32_t)w[0] << 16) | w[1];
  }
  return st;
}

/* --------------------------------------------------------------------------*/
HAL_StatusTypeDef MAX31855_Init(MAX31855_t *dev, SPI_HandleTypeDef *hspi,
                                GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
  dev->hspi    = hspi;
  dev->cs_port = cs_port;
  dev->cs_pin  = cs_pin;
  dev->present = 0;

  /* No config registers to write; probe by doing one SPI read. */
  uint32_t raw;
  if (read_raw(dev, &raw) != HAL_OK)
  {
    return HAL_ERROR;
  }
  dev->present = 1;
  return HAL_OK;
}

HAL_StatusTypeDef MAX31855_Read(MAX31855_t *dev,
                                int32_t *tc10, int32_t *cj10,
                                uint8_t *faults)
{
  uint32_t raw;
  HAL_StatusTypeDef st = read_raw(dev, &raw);
  if (st != HAL_OK)
  {
    return st;
  }

  if (faults)
  {
    *faults = (uint8_t)(raw & 0x07U);
  }

  /* Thermocouple temp: bits[31:18], 14-bit two's complement, 0.25 °C/LSB.
     Cast the top half to int16_t so the sign bit (bit 31 → bit 15) is
     preserved, then arithmetic-right-shift by 2 to drop the reserved bit
     and FAULT bit, yielding a sign-extended 14-bit value.
     Tenths: raw14 × 0.25 × 10 = raw14 × 5 / 2. */
  if (tc10)
  {
    int32_t t = (int32_t)(int16_t)(raw >> 16) >> 2;
    *tc10 = (t * 5) / 2;
  }

  /* Cold-junction temp: bits[15:4], 12-bit two's complement, 0.0625 °C/LSB.
     Same trick with the bottom half; shift right 4 drops the reserved bit
     and the three fault bits, sign-extending from bit 15.
     Tenths: raw12 × 0.0625 × 10 = raw12 × 10 / 16 = raw12 × 5 / 8. */
  if (cj10)
  {
    int32_t c = (int32_t)(int16_t)(raw & 0xFFFFU) >> 4;
    *cj10 = (c * 10) / 16;
  }

  return HAL_OK;
}
