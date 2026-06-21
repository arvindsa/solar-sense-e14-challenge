/**
 ******************************************************************************
 * @file    ina219.c
 * @brief   Driver for the TI INA219(B) high-side current/voltage monitor.
 ******************************************************************************
 */
#include "ina219.h"

/* Register map (16-bit, MSB first). */
#define REG_CONFIG     0x00U
#define REG_SHUNT      0x01U   /* shunt voltage, 10 uV/LSB, signed          */
#define REG_BUS        0x02U   /* bus voltage in bits[15:3], 4 mV/LSB       */
#define REG_POWER      0x03U
#define REG_CURRENT    0x04U
#define REG_CALIB      0x05U

/* CONFIG fields */
#define CFG_RST        0x8000U
#define CFG_BRNG_16V   0x0000U   /* bit13 = 0 -> 16 V bus range             */
#define CFG_PG_40MV    0x0000U   /* bits[12:11] = 00 -> +/-40 mV PGA        */
#define CFG_BADC_12B8S 0x0580U   /* bits[10:7]  = 1011 -> 12-bit, 8 samples */
#define CFG_SADC_12B8S 0x0058U   /* bits[6:3]   = 1011 -> 12-bit, 8 samples */
#define CFG_MODE_CONT  0x0007U   /* shunt + bus, continuous                 */

/* Composite operating configuration written at init. */
#define INA219_CONFIG (CFG_BRNG_16V | CFG_PG_40MV | CFG_BADC_12B8S | \
                       CFG_SADC_12B8S | CFG_MODE_CONT)   /* = 0x05DF */

/* Bus-voltage register: data is bits[15:3], LSB = 4 mV. */
#define BUS_LSB_MV     4
/* Shunt-voltage register LSB = 10 uV. */
#define SHUNT_LSB_UV   10

#define I2C_TIMEOUT_MS 50U

/* --- low-level 16-bit register access (big-endian on the wire) ------------*/
static HAL_StatusTypeDef reg_read(INA219_t *dev, uint8_t reg, uint16_t *val)
{
  uint8_t buf[2];
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(dev->hi2c, (uint16_t)(dev->addr7 << 1),
                                          reg, I2C_MEMADD_SIZE_8BIT, buf, 2,
                                          I2C_TIMEOUT_MS);
  if (st == HAL_OK)
  {
    *val = ((uint16_t)buf[0] << 8) | buf[1];
  }
  return st;
}

static HAL_StatusTypeDef reg_write(INA219_t *dev, uint8_t reg, uint16_t val)
{
  uint8_t buf[2] = { (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
  return HAL_I2C_Mem_Write(dev->hi2c, (uint16_t)(dev->addr7 << 1), reg,
                           I2C_MEMADD_SIZE_8BIT, buf, 2, I2C_TIMEOUT_MS);
}

/* --------------------------------------------------------------------------*/
HAL_StatusTypeDef INA219_Init(INA219_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
  dev->hi2c    = hi2c;
  dev->addr7   = addr7;
  dev->present = 0;

  if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr7 << 1), 3, I2C_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* Soft-reset, then apply our operating configuration. */
  if (reg_write(dev, REG_CONFIG, CFG_RST) != HAL_OK) return HAL_ERROR;
  if (reg_write(dev, REG_CONFIG, INA219_CONFIG) != HAL_OK) return HAL_ERROR;

  dev->present = 1;
  return HAL_OK;
}

HAL_StatusTypeDef INA219_ReadBus_mV(INA219_t *dev, int32_t *mv)
{
  uint16_t raw;
  HAL_StatusTypeDef st = reg_read(dev, REG_BUS, &raw);
  if (st == HAL_OK)
  {
    /* Voltage data is the top 13 bits; bits 1:0 are CNVR/OVF flags. */
    *mv = (int32_t)(raw >> 3) * BUS_LSB_MV;
  }
  return st;
}

HAL_StatusTypeDef INA219_ReadShunt_uV(INA219_t *dev, int32_t *uv)
{
  uint16_t raw;
  HAL_StatusTypeDef st = reg_read(dev, REG_SHUNT, &raw);
  if (st == HAL_OK)
  {
    *uv = (int32_t)(int16_t)raw * SHUNT_LSB_UV;
  }
  return st;
}

HAL_StatusTypeDef INA219_ReadCurrent_uA(INA219_t *dev, int32_t *ua)
{
  int32_t uv;
  HAL_StatusTypeDef st = INA219_ReadShunt_uV(dev, &uv);
  if (st == HAL_OK)
  {
    /* I(uA) = Vshunt(uV) / R(Ohm) = Vshunt(uV) * 1000 / R(mOhm). */
    *ua = (uv * 1000) / INA219_RSHUNT_MOHM;
  }
  return st;
}

HAL_StatusTypeDef INA219_ReadPower_mW(INA219_t *dev, int32_t *mw)
{
  int32_t bus_mv, ua;
  HAL_StatusTypeDef st = INA219_ReadBus_mV(dev, &bus_mv);
  if (st != HAL_OK) return st;
  st = INA219_ReadCurrent_uA(dev, &ua);
  if (st != HAL_OK) return st;
  /* P(mW) = V(mV) * I(mA) / 1000 = bus_mv * (ua/1000) / 1000. */
  *mw = (int32_t)(((int64_t)bus_mv * ua) / 1000000);
  return st;
}
