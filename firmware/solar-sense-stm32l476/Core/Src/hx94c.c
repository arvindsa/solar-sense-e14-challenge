/**
 ******************************************************************************
 * @file    hx94c.c
 * @brief   Driver for the Omega HX94C 4-20 mA humidity/temperature probe.
 ******************************************************************************
 */
#include "hx94c.h"

#define ADC_FULL_SCALE   4095U   /* 12-bit right-aligned conversions */
#define ADC_TIMEOUT_MS   10U

/* --- helpers --------------------------------------------------------------*/

/* Configure `channel` on regular rank 1, run one conversion, and return the
   sense voltage in millivolts. Mirrors the borrow-rank-1 pattern used for the
   internal VBAT read in main.c so the shared 4-channel scan keeps working. */
static HAL_StatusTypeDef adc_read_mv(HX94C_t *dev, uint32_t channel, uint32_t *mv)
{
  ADC_ChannelConfTypeDef c = {0};
  c.Channel      = channel;
  c.Rank         = ADC_REGULAR_RANK_1;
  c.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;   /* high source impedance */
  c.SingleDiff   = ADC_SINGLE_ENDED;
  c.OffsetNumber = ADC_OFFSET_NONE;
  c.Offset       = 0;

  HAL_StatusTypeDef st = HAL_ERROR;
  if (HAL_ADC_ConfigChannel(dev->hadc, &c) == HAL_OK &&
      HAL_ADC_Start(dev->hadc) == HAL_OK)
  {
    if (HAL_ADC_PollForConversion(dev->hadc, ADC_TIMEOUT_MS) == HAL_OK)
    {
      uint32_t raw = HAL_ADC_GetValue(dev->hadc);
      *mv = (raw * HX94C_VDDA_MV) / ADC_FULL_SCALE;
      st = HAL_OK;
    }
    HAL_ADC_Stop(dev->hadc);
  }
  return st;
}

/* Linearly map a 4-20 mA loop current (microamps) onto [lo,hi]. The input is
   clamped to 4-20 mA by the caller, so the result stays within [lo,hi]. */
static int32_t map_loop(int32_t i_ua, int32_t lo, int32_t hi)
{
  return lo + (int32_t)(((int64_t)(i_ua - 4000) * (hi - lo)) / 16000);
}

static int32_t clamp_i(int32_t v, int32_t lo, int32_t hi)
{
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

/* --------------------------------------------------------------------------*/
void HX94C_Init(HX94C_t *dev, ADC_HandleTypeDef *hadc,
                uint32_t rh_channel, uint32_t t_channel, uint32_t idle_channel)
{
  dev->hadc         = hadc;
  dev->rh_channel   = rh_channel;
  dev->t_channel    = t_channel;
  dev->idle_channel = idle_channel;
  dev->present      = 0;
}

HAL_StatusTypeDef HX94C_Read(HX94C_t *dev, int32_t *rh_pct10, int32_t *temp_c100,
                             int32_t *i_rh_ua, int32_t *i_t_ua, uint8_t *flags)
{
  uint32_t rh_mv = 0, t_mv = 0;
  HAL_StatusTypeDef st_rh = adc_read_mv(dev, dev->rh_channel, &rh_mv);
  HAL_StatusTypeDef st_t  = adc_read_mv(dev, dev->t_channel,  &t_mv);

  /* Always put rank 1 back to its idle channel, even on a conversion error. */
  {
    ADC_ChannelConfTypeDef c = {0};
    c.Channel      = dev->idle_channel;
    c.Rank         = ADC_REGULAR_RANK_1;
    c.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    c.SingleDiff   = ADC_SINGLE_ENDED;
    c.OffsetNumber = ADC_OFFSET_NONE;
    c.Offset       = 0;
    HAL_ADC_ConfigChannel(dev->hadc, &c);
  }

  if (st_rh != HAL_OK || st_t != HAL_OK)
  {
    dev->present = 0;
    return HAL_ERROR;
  }

  /* I(uA) = V(mV) / Rsense(ohm) * 1000. Max term 3300*1000 fits int32. */
  int32_t i_rh = (int32_t)(rh_mv * 1000U / HX94C_RSENSE_OHMS);
  int32_t i_t  = (int32_t)(t_mv  * 1000U / HX94C_RSENSE_OHMS);

  uint8_t f = 0;
  if (i_rh < HX94C_LOOP_LO_UA)      f |= HX94C_FLT_RH_OPEN;
  else if (i_rh > HX94C_LOOP_HI_UA) f |= HX94C_FLT_RH_OVER;
  if (i_t < HX94C_LOOP_LO_UA)       f |= HX94C_FLT_T_OPEN;
  else if (i_t > HX94C_LOOP_HI_UA)  f |= HX94C_FLT_T_OVER;

  /* Pin the current to the 4-20 mA band before scaling so a faulted/over-range
     loop still produces a bounded reading next to its fault flag. */
  int32_t i_rh_c = clamp_i(i_rh, 4000, 20000);
  int32_t i_t_c  = clamp_i(i_t,  4000, 20000);

  if (rh_pct10)
  {
    int32_t rh = map_loop(i_rh_c, HX94C_RH_AT_4MA_PCT10, HX94C_RH_AT_20MA_PCT10);
    *rh_pct10 = clamp_i(rh, 0, 1000);   /* RH is physically 0..100 % */
  }
  if (temp_c100)
    *temp_c100 = map_loop(i_t_c, HX94C_T_AT_4MA_C100, HX94C_T_AT_20MA_C100);
  if (i_rh_ua) *i_rh_ua = i_rh;
  if (i_t_ua)  *i_t_ua  = i_t;
  if (flags)   *flags   = f;

  dev->present = (f == 0);
  return HAL_OK;
}
