/**
 ******************************************************************************
 * @file    hx94c.h
 * @brief   Driver for the Omega HX94C relative-humidity / temperature probe.
 *
 * The HX94C is a loop-powered transmitter with two independent 4-20 mA current
 * loops -- one for relative humidity, one for temperature. SolarSense excites
 * both loops from the switched +12 V rail (EN_12V / TPS61040); each loop sinks
 * its 4-20 mA through a board sense resistor to GND, and the MCU reads the
 * voltage across that resistor on two ADC1 channels:
 *
 *     RH   loop -> ADC_HX94C_RH   (PA1, ADC1_IN6)
 *     temp loop -> ADC_HX94C_TEMP (PC4, ADC1_IN13)
 *
 * There is no digital bus to probe, so the part is not auto-detected: a loop
 * current inside the valid 4-20 mA band is taken as "present and healthy", and
 * a current below ~3.5 mA means the loop is open or the +12 V rail is off.
 *
 * Loop current is recovered from the sense voltage (I = V / Rsense) and then
 * linearly mapped 4 mA -> low end, 20 mA -> high end of each measurand. The
 * full-scale endpoints below are the HX94C defaults (0-100 %RH, 0-100 C);
 * confirm them against your probe's datasheet/label and the fitted Rsense and
 * adjust the macros here if they differ.
 ******************************************************************************
 */
#ifndef HX94C_H
#define HX94C_H

#include "stm32l4xx_hal.h"

/* Loop sense resistor (ohms). The 4-20 mA loop develops V = I * Rsense across
   it; 150 ohm gives 0.60 V @ 4 mA and 3.00 V @ 20 mA, safely under VDDA. */
#ifndef HX94C_RSENSE_OHMS
#define HX94C_RSENSE_OHMS   150U
#endif

/* ADC reference / VDDA in millivolts (the regulated 3V3 rail). */
#ifndef HX94C_VDDA_MV
#define HX94C_VDDA_MV       3300U
#endif

/* Sensor full-scale endpoints: value reported at the 4 mA and 20 mA ends.
   RH in tenths of a percent, temperature in centi-degrees Celsius. */
#define HX94C_RH_AT_4MA_PCT10    0      /*   0.0 %RH at  4 mA */
#define HX94C_RH_AT_20MA_PCT10   1000   /* 100.0 %RH at 20 mA */
#define HX94C_T_AT_4MA_C100      0      /*   0.00 C  at  4 mA */
#define HX94C_T_AT_20MA_C100     10000  /* 100.00 C  at 20 mA */

/* A healthy 4-20 mA loop sits in [4,20] mA; allow a little slop for tolerance.
   Below LO the loop reads as open (sensor unpowered / wire break); above HI it
   is over-range (short or wrong Rsense). Thresholds in microamps. */
#define HX94C_LOOP_LO_UA    3500
#define HX94C_LOOP_HI_UA    21000

/* Per-channel fault bits returned in *flags (0 = both loops healthy). */
#define HX94C_FLT_RH_OPEN   0x01U   /* RH loop   < LO  (open / no +12 V)       */
#define HX94C_FLT_RH_OVER   0x02U   /* RH loop   > HI  (short / over-range)     */
#define HX94C_FLT_T_OPEN    0x04U   /* temp loop < LO                          */
#define HX94C_FLT_T_OVER    0x08U   /* temp loop > HI                          */

typedef struct {
  ADC_HandleTypeDef *hadc;
  uint32_t rh_channel;     /* ADC_CHANNEL_x feeding the RH loop sense resistor   */
  uint32_t t_channel;      /* ADC_CHANNEL_x feeding the temp loop sense resistor */
  uint32_t idle_channel;   /* channel restored on rank 1 after a read           */
  uint8_t  present;        /* 1 after a read where both loops were in-band       */
} HX94C_t;

/* Record the ADC handle and the two loop channels. idle_channel is what the
   shared ADC's regular rank 1 is configured for elsewhere (ADC_CHANNEL_5 / the
   UV input on this board); HX94C_Read() borrows rank 1 and restores it. No bus
   traffic happens here, so this never fails and cannot detect the probe -- that
   is reported per-read via the fault flags. */
void HX94C_Init(HX94C_t *dev, ADC_HandleTypeDef *hadc,
                uint32_t rh_channel, uint32_t t_channel, uint32_t idle_channel);

/* Sample both loops. Outputs (any pointer may be NULL):
     rh_pct10   relative humidity, tenths of a percent  (clamped 0..1000)
     temp_c100  temperature, centi-degrees Celsius        (clamped to F.S. span)
     i_rh_ua    RH   loop current, microamps  (raw, unclamped -- for diagnostics)
     i_t_ua     temp loop current, microamps  (raw, unclamped)
     flags      OR of HX94C_FLT_* bits; 0 means both loops healthy
   Reported RH/temp are computed from the current clamped to 4-20 mA, so a
   faulted loop still yields a sane (pinned) reading alongside the flag.
   Returns HAL_OK if both ADC conversions ran, HAL_ERROR otherwise. */
HAL_StatusTypeDef HX94C_Read(HX94C_t *dev, int32_t *rh_pct10, int32_t *temp_c100,
                             int32_t *i_rh_ua, int32_t *i_t_ua, uint8_t *flags);

#endif /* HX94C_H */
