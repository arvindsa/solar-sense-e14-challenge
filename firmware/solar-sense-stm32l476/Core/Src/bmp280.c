/**
 ******************************************************************************
 * @file    bmp280.c
 * @brief   Driver for the Bosch BMP280 barometric pressure + temperature sensor.
 ******************************************************************************
 */
#include "bmp280.h"

/* Register map. */
#define REG_CALIB_00   0x88U   /* dig_T1..dig_P9: 24 bytes, little-endian   */
#define REG_ID         0xD0U
#define REG_RESET      0xE0U
#define REG_STATUS     0xF3U
#define REG_CTRL_MEAS  0xF4U
#define REG_CONFIG     0xF5U
#define REG_PRESS_MSB  0xF7U   /* press[2] temp[2] data, 6 bytes, big-endian */

#define RESET_WORD     0xB6U

/* ctrl_meas: osrs_t[7:5] osrs_p[4:2] mode[1:0]. x2 temp, x16 pressure, normal. */
#define OSRS_T_X2      (0x2U << 5)
#define OSRS_P_X16     (0x5U << 2)
#define MODE_NORMAL    0x3U
#define CTRL_MEAS_CFG  (OSRS_T_X2 | OSRS_P_X16 | MODE_NORMAL)

/* config: t_sb[7:5] filter[4:2] spi3w_en[0]. 0.5 ms standby, IIR x16. */
#define T_SB_0P5MS     (0x0U << 5)
#define FILTER_X16     (0x4U << 2)
#define CONFIG_CFG     (T_SB_0P5MS | FILTER_X16)

#define STATUS_MEASURING 0x08U

#define I2C_TIMEOUT_MS 50U

/* --- low-level register access --------------------------------------------*/
static HAL_StatusTypeDef reg_write(BMP280_t *dev, uint8_t reg, uint8_t val)
{
  return HAL_I2C_Mem_Write(dev->hi2c, (uint16_t)(dev->addr7 << 1), reg,
                           I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef reg_read(BMP280_t *dev, uint8_t reg, uint8_t *buf, uint16_t n)
{
  return HAL_I2C_Mem_Read(dev->hi2c, (uint16_t)(dev->addr7 << 1), reg,
                          I2C_MEMADD_SIZE_8BIT, buf, n, I2C_TIMEOUT_MS);
}

/* --- compensation (datasheet 3.11.3, 32/64-bit fixed point) ---------------*/
static int32_t compensate_T(BMP280_t *dev, int32_t adc_T)
{
  int32_t var1, var2;
  var1 = ((((adc_T >> 3) - ((int32_t)dev->dig_T1 << 1))) * ((int32_t)dev->dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)dev->dig_T1)) *
            ((adc_T >> 4) - ((int32_t)dev->dig_T1))) >> 12) *
          ((int32_t)dev->dig_T3)) >> 14;
  dev->t_fine = var1 + var2;
  /* (t_fine*5 + 128) >> 8 yields temperature in 0.01 C. */
  return (dev->t_fine * 5 + 128) >> 8;
}

/* Returns pressure in Q24.8 pascals (value / 256 = Pa). Needs t_fine set. */
static uint32_t compensate_P(BMP280_t *dev, int32_t adc_P)
{
  int64_t var1, var2, p;
  var1 = ((int64_t)dev->t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)dev->dig_P6;
  var2 = var2 + ((var1 * (int64_t)dev->dig_P5) << 17);
  var2 = var2 + (((int64_t)dev->dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dev->dig_P3) >> 8) +
         ((var1 * (int64_t)dev->dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dev->dig_P1) >> 33;
  if (var1 == 0)
    return 0;   /* avoid division by zero */
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dev->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)dev->dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dev->dig_P7) << 4);
  return (uint32_t)p;
}

/* --------------------------------------------------------------------------*/
HAL_StatusTypeDef BMP280_Init(BMP280_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
  dev->hi2c    = hi2c;
  dev->present = 0;
  dev->chip_id = 0xFF;

  /* Try the requested address first, then the other one (SDO high vs low). */
  const uint8_t cand[2] = { addr7,
                            (addr7 == BMP280_ADDR) ? BMP280_ADDR_ALT : BMP280_ADDR };
  uint8_t id = 0xFF;
  uint8_t found = 0;
  for (int i = 0; i < 2 && !found; i++)
  {
    dev->addr7 = cand[i];
    if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(cand[i] << 1), 3, I2C_TIMEOUT_MS) != HAL_OK)
      continue;
    if (reg_read(dev, REG_ID, &id, 1) != HAL_OK)
      continue;
    dev->chip_id = id;
    if (id == BMP280_CHIP_ID || id == BME280_CHIP_ID)
      found = 1;
  }
  if (!found)
    return HAL_ERROR;

  /* Soft reset and give the device its ~2 ms start-up time. */
  if (reg_write(dev, REG_RESET, RESET_WORD) != HAL_OK) return HAL_ERROR;
  HAL_Delay(5);

  /* Read the 24-byte calibration block (little-endian words). */
  uint8_t c[24];
  if (reg_read(dev, REG_CALIB_00, c, sizeof(c)) != HAL_OK) return HAL_ERROR;
  dev->dig_T1 = (uint16_t)(c[1]  << 8 | c[0]);
  dev->dig_T2 = (int16_t) (c[3]  << 8 | c[2]);
  dev->dig_T3 = (int16_t) (c[5]  << 8 | c[4]);
  dev->dig_P1 = (uint16_t)(c[7]  << 8 | c[6]);
  dev->dig_P2 = (int16_t) (c[9]  << 8 | c[8]);
  dev->dig_P3 = (int16_t) (c[11] << 8 | c[10]);
  dev->dig_P4 = (int16_t) (c[13] << 8 | c[12]);
  dev->dig_P5 = (int16_t) (c[15] << 8 | c[14]);
  dev->dig_P6 = (int16_t) (c[17] << 8 | c[16]);
  dev->dig_P7 = (int16_t) (c[19] << 8 | c[18]);
  dev->dig_P8 = (int16_t) (c[21] << 8 | c[20]);
  dev->dig_P9 = (int16_t) (c[23] << 8 | c[22]);

  /* config must be written before ctrl_meas latches normal mode. */
  if (reg_write(dev, REG_CONFIG, CONFIG_CFG) != HAL_OK) return HAL_ERROR;
  if (reg_write(dev, REG_CTRL_MEAS, CTRL_MEAS_CFG) != HAL_OK) return HAL_ERROR;

  dev->present = 1;
  return HAL_OK;
}

HAL_StatusTypeDef BMP280_Read(BMP280_t *dev, int32_t *temp_c100, uint32_t *press_pa)
{
  uint8_t d[6];
  HAL_StatusTypeDef st = reg_read(dev, REG_PRESS_MSB, d, sizeof(d));
  if (st != HAL_OK) return st;

  /* 20-bit raw samples, MSB first: press in d[0..2], temp in d[3..5]. */
  int32_t adc_P = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
  int32_t adc_T = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);

  int32_t t = compensate_T(dev, adc_T);   /* sets t_fine, used by pressure */
  if (temp_c100) *temp_c100 = t;
  if (press_pa)  *press_pa  = compensate_P(dev, adc_P) >> 8;  /* Q24.8 -> Pa */
  return HAL_OK;
}
