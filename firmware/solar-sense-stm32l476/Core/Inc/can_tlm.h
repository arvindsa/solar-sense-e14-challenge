/* can_tlm.h — CAN telemetry transmit for SolarSense v1
 *
 * Protocol: 10 standard 8-byte frames at 500 kbps, IDs 0x100–0x109.
 * All multi-byte fields are big-endian (MSB first).
 *
 * Frame layout:
 *  0x100 SYS   [0:1]=bat_mv [2:3]=bat_soc [4]=chg [5]=flags(b0=r12v,b1=sd) [6:7]=uv_mv
 *  0x101 P1    [0]=ok [1]=pad [2:3]=p_mv [4:5]=p_ma [6:7]=p_mw
 *  0x102 P2    same layout as P1
 *  0x103 P3    same layout as P1
 *  0x104 TC    [0]=ok [1]=fault [2:3]=tc_c [4:5]=tc_cj [6:7]=pad
 *  0x105 BMP   [0]=ok [1]=pad [2:3]=bmp_c [4:7]=bmp_pa
 *  0x106 HX    [0]=ok [1]=flt [2:3]=hx_rh [4:5]=hx_t [6]=hx_rhi [7]=hx_ti
 *  0x107 AQPM  [0]=ok [1]=pad [2:3]=pm10 [4:5]=pm25 [6:7]=pm40
 *  0x108 AQG1  [0:1]=pm100 [2:3]=aq_rh [4:5]=aq_t [6:7]=voc
 *  0x109 AQG2  [0:1]=nox [2:3]=co2 [4:7]=pad
 *
 * Units match the TLM line: *_mv in mV, bat_soc in tenths-%, p_ma in tenths-mA,
 * p_mw in mW, tc_c/bmp_c/hx_t in centi-°C, hx_rh/aq_rh in tenths-%RH,
 * pm* in tenths-µg/m³, co2 in ppm, voc/nox are dimensionless index.
 */
#ifndef CAN_TLM_H
#define CAN_TLM_H

#include "stm32l4xx_hal.h"
#include <stdint.h>

#define CAN_TLM_ID_SYS   0x100U
#define CAN_TLM_ID_P1    0x101U
#define CAN_TLM_ID_P2    0x102U
#define CAN_TLM_ID_P3    0x103U
#define CAN_TLM_ID_TC    0x104U
#define CAN_TLM_ID_BMP   0x105U
#define CAN_TLM_ID_HX    0x106U
#define CAN_TLM_ID_AQPM  0x107U
#define CAN_TLM_ID_AQG1  0x108U
#define CAN_TLM_ID_AQG2  0x109U

typedef struct {
    /* power */
    uint16_t bat_mv;
    uint16_t bat_soc;    /* tenths % */
    uint8_t  chg;        /* 0=idle 1=charging 2=done */
    uint8_t  r12v;
    uint8_t  sd;
    uint16_t uv_mv;

    /* solar panels (indices 0–2 = P1–P3) */
    uint8_t  p_ok[3];
    int16_t  p_mv[3];   /* bus voltage mV */
    int16_t  p_ma[3];   /* current tenths-mA */
    int16_t  p_mw[3];   /* power mW */

    /* MAX31855 thermocouple */
    uint8_t  tc3_ok;
    uint8_t  tc3_fault;
    int16_t  tc3_c;     /* centi-°C */
    int16_t  tc3_cj;    /* centi-°C */

    /* BMP280 barometer + board temp */
    uint8_t  bmp_ok;
    int16_t  bmp_c;     /* centi-°C */
    uint32_t bmp_pa;

    /* HX94C humidity/temp probe */
    uint8_t  hx_ok;
    uint8_t  hx_flt;
    int16_t  hx_rh;     /* tenths %RH */
    int16_t  hx_t;      /* centi-°C */
    uint8_t  hx_rhi;    /* loop current tenths-mA (0–255) */
    uint8_t  hx_ti;

    /* SEN66 air quality */
    uint8_t  aq_ok;
    int16_t  aq_pm10;   /* tenths µg/m³ */
    int16_t  aq_pm25;
    int16_t  aq_pm40;
    int16_t  aq_pm100;
    int16_t  aq_rh;     /* tenths %RH */
    int16_t  aq_t;      /* centi-°C */
    int16_t  aq_voc;
    int16_t  aq_nox;
    int16_t  aq_co2;    /* ppm */
} SolarSenseTLM_t;

/* Configure a pass-all CAN filter and start the peripheral.
   Call once in USER CODE BEGIN 2, after MX_CAN1_Init(). */
HAL_StatusTypeDef CAN_TLM_Start(CAN_HandleTypeDef *hcan);

/* Pack and transmit all 10 TLM frames.  Non-blocking: each frame waits
   at most 10 ms for a free TX mailbox. */
HAL_StatusTypeDef CAN_TLM_Send(CAN_HandleTypeDef *hcan, const SolarSenseTLM_t *t);

#endif /* CAN_TLM_H */
