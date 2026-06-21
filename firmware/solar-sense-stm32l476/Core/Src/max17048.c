/**
 ******************************************************************************
 * @file    max17048.c
 * @brief   Driver for the MAX17048 ModelGauge Li+ fuel gauge (I2C).
 ******************************************************************************
 */
#include "max17048.h"

/* Register map (16-bit, MSB first) */
#define REG_VCELL     0x02U
#define REG_SOC       0x04U
#define REG_MODE      0x06U
#define REG_VERSION   0x08U
#define REG_HIBRT     0x0AU
#define REG_CONFIG    0x0CU
#define REG_VALRT     0x14U
#define REG_CRATE     0x16U
#define REG_VRESET_ID 0x18U
#define REG_STATUS    0x1AU
#define REG_CMD       0xFEU

/* CONFIG register bit fields */
#define CONFIG_ALSC   0x0040U   /* enable SOC-change alert       */
#define CONFIG_ALRT   0x0020U   /* alert flag (write 0 to clear) */
#define CONFIG_ATHD   0x001FU   /* empty alert threshold mask    */

/* Scale factors */
#define VCELL_LSB_V   (78.125e-6f)   /* 78.125 uV/LSB           */
#define SOC_LSB       (1.0f / 256.0f)/* 1%/256 per LSB          */
#define CRATE_LSB     (0.208f)       /* 0.208 %/hr per LSB      */
#define VALRT_LSB_V   (0.02f)        /* 20 mV/LSB               */

#define I2C_TIMEOUT_MS  100U

static I2C_HandleTypeDef *s_hi2c = NULL;

/* --- low-level 16-bit register access (big-endian on the wire) ------------*/
static HAL_StatusTypeDef reg_read(uint8_t reg, uint16_t *val)
{
  uint8_t buf[2];
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(s_hi2c, MAX17048_I2C_ADDR, reg,
                                          I2C_MEMADD_SIZE_8BIT, buf, 2,
                                          I2C_TIMEOUT_MS);
  if (st == HAL_OK)
  {
    *val = ((uint16_t)buf[0] << 8) | buf[1];
  }
  return st;
}

static HAL_StatusTypeDef reg_write(uint8_t reg, uint16_t val)
{
  uint8_t buf[2] = { (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
  return HAL_I2C_Mem_Write(s_hi2c, MAX17048_I2C_ADDR, reg,
                           I2C_MEMADD_SIZE_8BIT, buf, 2, I2C_TIMEOUT_MS);
}

/* Read-modify-write helper: new = (old & ~clear) | set */
static HAL_StatusTypeDef reg_update(uint8_t reg, uint16_t clear, uint16_t set)
{
  uint16_t v;
  HAL_StatusTypeDef st = reg_read(reg, &v);
  if (st != HAL_OK) return st;
  v = (uint16_t)((v & ~clear) | set);
  return reg_write(reg, v);
}

/* --------------------------------------------------------------------------*/
HAL_StatusTypeDef MAX17048_Init(I2C_HandleTypeDef *hi2c)
{
  s_hi2c = hi2c;
  /* Confirm the device is present/addressable. */
  if (HAL_I2C_IsDeviceReady(s_hi2c, MAX17048_I2C_ADDR, 3, I2C_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* Clear any power-on Reset Indicator left in STATUS. */
  return MAX17048_ClearStatusFlags(MAX17048_STATUS_RI);
}

HAL_StatusTypeDef MAX17048_ReadVoltage(float *volts)
{
  uint16_t raw;
  HAL_StatusTypeDef st = reg_read(REG_VCELL, &raw);
  if (st == HAL_OK) *volts = (float)raw * VCELL_LSB_V;
  return st;
}

HAL_StatusTypeDef MAX17048_ReadSOC(float *percent)
{
  uint16_t raw;
  HAL_StatusTypeDef st = reg_read(REG_SOC, &raw);
  if (st == HAL_OK) *percent = (float)raw * SOC_LSB;
  return st;
}

HAL_StatusTypeDef MAX17048_ReadCRate(float *pct_per_hr)
{
  uint16_t raw;
  HAL_StatusTypeDef st = reg_read(REG_CRATE, &raw);
  if (st == HAL_OK) *pct_per_hr = (float)(int16_t)raw * CRATE_LSB;
  return st;
}

HAL_StatusTypeDef MAX17048_ReadVersion(uint16_t *ver)
{
  return reg_read(REG_VERSION, ver);
}

HAL_StatusTypeDef MAX17048_QuickStart(void)
{
  return reg_write(REG_MODE, 0x4000U);
}

HAL_StatusTypeDef MAX17048_SetEmptyAlertThreshold(uint8_t percent)
{
  if (percent < 1U)  percent = 1U;
  if (percent > 32U) percent = 32U;
  /* ATHD = 32 - threshold% (5-bit field) */
  uint16_t athd = (uint16_t)(32U - percent) & CONFIG_ATHD;
  return reg_update(REG_CONFIG, CONFIG_ATHD, athd);
}

HAL_StatusTypeDef MAX17048_SetVoltageAlerts(float vmin, float vmax)
{
  int mn = (int)(vmin / VALRT_LSB_V + 0.5f);
  int mx = (int)(vmax / VALRT_LSB_V + 0.5f);
  if (mn < 0)   mn = 0;
  if (mn > 255) mn = 255;
  if (mx < 0)   mx = 0;
  if (mx > 255) mx = 255;   /* 255 * 20 mV = 5.10 V -> effectively disabled */
  return reg_write(REG_VALRT, (uint16_t)(((uint16_t)mx << 8) | (uint16_t)mn));
}

HAL_StatusTypeDef MAX17048_EnableSOCChangeAlert(uint8_t enable)
{
  return reg_update(REG_CONFIG, CONFIG_ALSC, enable ? CONFIG_ALSC : 0U);
}

HAL_StatusTypeDef MAX17048_GetStatus(uint16_t *status)
{
  return reg_read(REG_STATUS, status);
}

HAL_StatusTypeDef MAX17048_ClearStatusFlags(uint16_t mask)
{
  return reg_update(REG_STATUS, mask & MAX17048_STATUS_ALL, 0U);
}

HAL_StatusTypeDef MAX17048_ClearAlert(void)
{
  HAL_StatusTypeDef st = MAX17048_ClearStatusFlags(MAX17048_STATUS_ALL);
  if (st != HAL_OK) return st;
  /* Clear CONFIG.ALRT to release the open-drain ALRT pin. */
  return reg_update(REG_CONFIG, CONFIG_ALRT, 0U);
}
