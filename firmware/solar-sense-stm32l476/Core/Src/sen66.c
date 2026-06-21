/**
 ******************************************************************************
 * @file    sen66.c
 * @brief   Driver for the Sensirion SEN66 environmental sensor (I2C).
 ******************************************************************************
 */
#include "sen66.h"
#include <string.h>

/* ---- I2C command codes (2-byte, MSB first) -------------------------------- */
#define CMD_START_MEAS  0x0021u
#define CMD_STOP_MEAS   0x0104u
#define CMD_READ_MEAS   0x0300u
#define CMD_SLEEP       0x1001u
#define CMD_WAKE        0x1103u
#define CMD_RESET       0xD304u

#define SEN66_TIMEOUT   50U   /* ms per I2C operation */

/* ---- CRC-8/SENSIRON: poly 0x31, init 0xFF, no final XOR ------------------ */
static uint8_t sen66_crc8(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0xFFu;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x31u) : (uint8_t)(crc << 1);
  }
  return crc;
}

/* Write a 2-byte command; ignores NAK (expected when device is asleep). */
static HAL_StatusTypeDef sen66_cmd(SEN66_t *dev, uint16_t cmd)
{
  uint8_t buf[2] = { (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFFu) };
  return HAL_I2C_Master_Transmit(dev->hi2c,
                                 (uint16_t)(SEN66_ADDR << 1),
                                 buf, 2, SEN66_TIMEOUT);
}

/* Read nwords × (2-byte word + 1-byte CRC).  Copies validated word bytes into
   dst[] (2 × nwords bytes, big-endian).  Returns HAL_ERROR on CRC mismatch. */
static HAL_StatusTypeDef sen66_read_words(SEN66_t *dev,
                                          uint8_t *dst, uint8_t nwords)
{
  /* Maximum response is 9 words × 3 bytes = 27 bytes for ReadMeasuredValues. */
  uint8_t raw[27];
  uint8_t total = (uint8_t)(nwords * 3u);

  if (total > sizeof(raw))
    return HAL_ERROR;

  if (HAL_I2C_Master_Receive(dev->hi2c,
                              (uint16_t)(SEN66_ADDR << 1),
                              raw, total, SEN66_TIMEOUT) != HAL_OK)
    return HAL_ERROR;

  for (uint8_t i = 0; i < nwords; i++) {
    if (sen66_crc8(raw + i * 3u, 2) != raw[i * 3u + 2u])
      return HAL_ERROR;   /* CRC mismatch — drop the whole frame */
    dst[i * 2u + 0u] = raw[i * 3u + 0u];
    dst[i * 2u + 1u] = raw[i * 3u + 1u];
  }
  return HAL_OK;
}

/* ---- Public API ----------------------------------------------------------- */

HAL_StatusTypeDef SEN66_Init(SEN66_t *dev, I2C_HandleTypeDef *hi2c)
{
  dev->hi2c      = hi2c;
  dev->present   = 0;
  dev->measuring = 0;

  if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(SEN66_ADDR << 1), 2, 10) != HAL_OK)
    return HAL_ERROR;

  /* Soft-reset: clears prior state.  The SEN66 needs ~500 ms to boot. */
  sen66_cmd(dev, CMD_RESET);
  HAL_Delay(500);

  if (sen66_cmd(dev, CMD_START_MEAS) != HAL_OK)
    return HAL_ERROR;

  dev->present   = 1;
  dev->measuring = 1;
  return HAL_OK;
}

HAL_StatusTypeDef SEN66_Read(SEN66_t *dev,
                              int32_t *pm10,  int32_t *pm25,
                              int32_t *pm40,  int32_t *pm100,
                              int32_t *rh,    int32_t *t,
                              int32_t *voc,   int32_t *nox,
                              int32_t *co2)
{
  if (!dev->present || !dev->measuring)
    return HAL_ERROR;

  /* Send the read command then wait before clocking out the response. */
  if (sen66_cmd(dev, CMD_READ_MEAS) != HAL_OK)
    return HAL_ERROR;
  HAL_Delay(20);

  /* 9 output words: PM1.0, PM2.5, PM4.0, PM10, RH, T, VOC, NOx, CO2. */
  uint8_t d[18]; /* 9 words × 2 bytes */
  if (sen66_read_words(dev, d, 9) != HAL_OK)
    return HAL_ERROR;

#define WU(i)  ((uint16_t)(((uint16_t)(d[(i)*2u]) << 8) | d[(i)*2u+1u]))
#define WS(i)  ((int16_t)WU(i))

  uint16_t r_pm10  = WU(0);
  uint16_t r_pm25  = WU(1);
  uint16_t r_pm40  = WU(2);
  uint16_t r_pm100 = WU(3);
  int16_t  r_rh    = WS(4);
  int16_t  r_t     = WS(5);
  int16_t  r_voc   = WS(6);
  int16_t  r_nox   = WS(7);
  uint16_t r_co2   = WU(8);

#undef WU
#undef WS

  /* 0xFFFF / 0x7FFF → sensor data not yet valid (warm-up). */
  if (r_pm25 == 0xFFFFu)
    return HAL_ERROR;

  /* Scale to the protocol units defined in sen66.h:
     PM    raw / 10  µg/m³  → store raw (tenths µg/m³)
     RH    raw / 100 %RH   → tenths %RH  = raw / 10
     T     raw / 200 °C    → centi-°C    = raw / 2
     VOC   raw / 10  index → actual index = raw / 10
     NOx   raw / 10  index → actual index = raw / 10
     CO2   ppm directly (no scaling needed)                     */
  if (pm10)  *pm10  = (int32_t)r_pm10;
  if (pm25)  *pm25  = (int32_t)r_pm25;
  if (pm40)  *pm40  = (int32_t)r_pm40;
  if (pm100) *pm100 = (int32_t)r_pm100;
  if (rh)    *rh    = (int32_t)(r_rh  / 10);
  if (t)     *t     = (int32_t)(r_t   / 2);
  if (voc)   *voc   = (int32_t)(r_voc / 10);
  if (nox)   *nox   = (int32_t)(r_nox / 10);
  if (co2)   *co2   = (int32_t)r_co2;

  return HAL_OK;
}

HAL_StatusTypeDef SEN66_Sleep(SEN66_t *dev)
{
  if (!dev->present)
    return HAL_ERROR;
  if (dev->measuring) {
    sen66_cmd(dev, CMD_STOP_MEAS);
    HAL_Delay(1);
    dev->measuring = 0;
  }
  /* Sleep command — sensor may NAK if already sleeping; ignore. */
  sen66_cmd(dev, CMD_SLEEP);
  return HAL_OK;
}

HAL_StatusTypeDef SEN66_Wake(SEN66_t *dev)
{
  if (!dev->present)
    return HAL_ERROR;
  /* The SEN66 NAKs the wake command while asleep; the STOP condition itself
     triggers the wake-up.  Ignore the return value. */
  sen66_cmd(dev, CMD_WAKE);
  HAL_Delay(20);
  return SEN66_StartMeasurement(dev);
}

HAL_StatusTypeDef SEN66_StopMeasurement(SEN66_t *dev)
{
  if (!dev->present)
    return HAL_ERROR;
  HAL_StatusTypeDef st = sen66_cmd(dev, CMD_STOP_MEAS);
  HAL_Delay(1);
  if (st == HAL_OK)
    dev->measuring = 0;
  return st;
}

HAL_StatusTypeDef SEN66_StartMeasurement(SEN66_t *dev)
{
  if (!dev->present)
    return HAL_ERROR;
  HAL_StatusTypeDef st = sen66_cmd(dev, CMD_START_MEAS);
  if (st == HAL_OK)
    dev->measuring = 1;
  return st;
}
