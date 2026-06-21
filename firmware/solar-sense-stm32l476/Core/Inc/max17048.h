/**
 ******************************************************************************
 * @file    max17048.h
 * @brief   Driver for the MAX17048 ModelGauge Li+ fuel gauge (I2C).
 *
 * Pure I2C driver - no GPIO/EXTI here. The ALRT pin (open-drain, active-low)
 * is wired to the MCU at the application layer; on an alert, call
 * MAX17048_GetStatus() to find the cause, then MAX17048_ClearAlert() to
 * release the ALRT pin.
 *
 * Device: MAX17048G+T, 7-bit I2C address 0x36. Registers are 16-bit, MSB first.
 ******************************************************************************
 */
#ifndef MAX17048_H
#define MAX17048_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

/* 7-bit address 0x36, shifted for the HAL 8-bit address argument */
#define MAX17048_I2C_ADDR        (0x36U << 1)

/* STATUS register (0x1A) alert-cause bit masks */
#define MAX17048_STATUS_RI       (0x0100U)  /* Reset Indicator            */
#define MAX17048_STATUS_VH       (0x0200U)  /* Voltage High               */
#define MAX17048_STATUS_VL       (0x0400U)  /* Voltage Low                */
#define MAX17048_STATUS_VR       (0x0800U)  /* Voltage Reset              */
#define MAX17048_STATUS_HD       (0x1000U)  /* SOC Low (below empty thr.) */
#define MAX17048_STATUS_SC       (0x2000U)  /* SOC Change (>= 1%)         */
#define MAX17048_STATUS_ALL      (0x3F00U)

/**
 * @brief  Bind the driver to an I2C bus and verify the device responds.
 * @param  hi2c  initialised I2C handle the gauge sits on (I2C1 here).
 * @retval HAL_OK if the device ACKs, otherwise HAL_ERROR/HAL_TIMEOUT.
 */
HAL_StatusTypeDef MAX17048_Init(I2C_HandleTypeDef *hi2c);

/* --- Measurements ---------------------------------------------------------*/
HAL_StatusTypeDef MAX17048_ReadVoltage(float *volts);   /* cell voltage, V   */
HAL_StatusTypeDef MAX17048_ReadSOC(float *percent);     /* state of charge,% */
HAL_StatusTypeDef MAX17048_ReadCRate(float *pct_per_hr);/* +charge/-discharge*/
HAL_StatusTypeDef MAX17048_ReadVersion(uint16_t *ver);

/* --- Control --------------------------------------------------------------*/
HAL_StatusTypeDef MAX17048_QuickStart(void);            /* force a re-gauge  */

/**
 * @brief  Set the empty-alert (HD) threshold, 1..32 %.
 *         ALRT asserts when SOC falls to/below this value.
 */
HAL_StatusTypeDef MAX17048_SetEmptyAlertThreshold(uint8_t percent);

/**
 * @brief  Set the under/over-voltage alert window (VALRT), 20 mV resolution.
 *         Pass 0.0f for vmin to disable the low limit, or > 5.1f for vmax
 *         to disable the high limit.
 */
HAL_StatusTypeDef MAX17048_SetVoltageAlerts(float vmin, float vmax);

/** @brief Enable/disable the "SOC changed by 1%" alert (CONFIG.ALSC). */
HAL_StatusTypeDef MAX17048_EnableSOCChangeAlert(uint8_t enable);

/* --- Alert handling -------------------------------------------------------*/
/** @brief Read the STATUS register (use MAX17048_STATUS_* masks). */
HAL_StatusTypeDef MAX17048_GetStatus(uint16_t *status);

/** @brief Clear the given STATUS flags (write 0 to those bits). */
HAL_StatusTypeDef MAX17048_ClearStatusFlags(uint16_t mask);

/**
 * @brief  Acknowledge an alert: clears all STATUS cause bits and the
 *         CONFIG.ALRT flag, which releases the (open-drain) ALRT pin high.
 */
HAL_StatusTypeDef MAX17048_ClearAlert(void);

#ifdef __cplusplus
}
#endif

#endif /* MAX17048_H */
