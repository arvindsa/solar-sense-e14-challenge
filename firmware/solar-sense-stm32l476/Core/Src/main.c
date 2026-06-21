/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "max17048.h"
#include "ina219.h"
#include "max31855.h"
#include "bmp280.h"
#include "hx94c.h"
#include "ssd1306.h"
#include "sen66.h"
#include "can_tlm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static volatile uint8_t bat_alert_flag = 0;  /* set by BAT_ALRT (PB13) EXTI  */
static uint8_t bat_alert_masked = 0;         /* 1 = EXTI off until condition clears */
/* Alert-condition bits that, while present, would re-assert ALRT forever. */
#define BAT_ALERT_CONDITIONS (MAX17048_STATUS_VH | MAX17048_STATUS_VL | \
                              MAX17048_STATUS_VR | MAX17048_STATUS_HD)

/* Per-panel current/voltage monitors. Only the three populated parts are
   instantiated; Panel 4/5 footprints are DNP. */
#define INA219_PANEL_COUNT 3
static INA219_t panel_ina[INA219_PANEL_COUNT];
static const uint8_t panel_ina_addr[INA219_PANEL_COUNT] = {
  INA219_ADDR_PANEL1, INA219_ADDR_PANEL2, INA219_ADDR_PANEL3
};

static MAX31855_t tc3;

/* BMP280 barometric pressure + board temperature (U14, I2C1 @0x76). */
static BMP280_t bmp;

/* HX94C humidity/temperature probe: two 4-20 mA loops on ADC1 (RH=IN6/PA1,
   temp=IN13/PC4), excited from the +12 V rail. Reads are only meaningful while
   r12v is on; with the rail off both loops read as open (HX94C_FLT_*_OPEN). */
static HX94C_t hx;

/* 128x64 SSD1306 status OLED on the shared I2C1 bus (@0x3C). Optional: if it
   isn't fitted, oled.present stays 0 and all display calls become no-ops. */
static SSD1306_t oled;

/* SEN66 multi-sensor: PM1.0/PM2.5/PM4.0/PM10, RH, T, VOC, NOx, CO2 (I2C1 @0x6B). */
static SEN66_t sen;

/* --- host serial command interface (USART2) --------------------------- */

static volatile uint8_t stream_enabled = 1;  /* 1 = emit periodic TLM lines     */
static volatile uint8_t maint_mode = 0;      /* 1 = maintenance mode, stream off */
static volatile uint8_t force_report   = 0;  /* one-shot TLM (from "status" cmd) */
static uint8_t          rx_byte;             /* single-byte IT receive slot      */
static char             rx_buf[40];          /* incoming command-line accumulator*/
static volatile uint8_t rx_len   = 0;
static char             rx_cmd[40];          /* completed command line for main  */
static volatile uint8_t rx_ready = 0;        /* 1 = rx_cmd holds a full line     */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
static void RTC_EmitTime(void);
static uint8_t RTC_SetFromString(const char *s);
static uint8_t RTC_ReadVbat_mV(uint32_t *vbat_mv);
static void OLED_ScreenAQ(void);
static void OLED_RenderScreen(uint8_t screen);
static void Power_EnterStop2(uint32_t seconds);
static void Power_EnterStandby(uint32_t wakeup_s);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* microSD card detect on PC6 (SDMMC1_CD): active-low with a 10k pull-up to
   +3V3, so the pin reads LOW when a card is seated. */
static uint8_t SD_CardPresent(void)
{
  return (HAL_GPIO_ReadPin(SDMMC1_CD_GPIO_Port, SDMMC1_CD_Pin) == GPIO_PIN_RESET);
}

/* Initialise the SD card in 4-bit mode. Returns 1 on success, 0 on failure.
   Non-fatal: never calls Error_Handler() so a bad card can't hang the node. */
static uint8_t SD_TryInit(void)
{
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    return 0;
  }
  if (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) != HAL_OK)
  {
    return 0;
  }
  return 1;
}

/* --- RTC time get/set over the host serial link ----------------------- */
/* The RTC runs from the LSE and is backed by VBAT, so it keeps time across
   resets (and across power loss while a coin cell is fitted). We stamp a
   magic value into backup register DR0 the first time the host sets the
   clock so the dashboard can tell "real time" from "unset since cold boot". */
#define RTC_SET_MAGIC 0x53C1u   /* "Set Clock 1" marker in RTC_BKP_DR0 */

/* Emit the current RTC date/time as a single parseable line:
     RTC set=1 2026-06-19 12:34:56
   set=1 means the host has set the clock at least once (DR0 magic present). */
static void RTC_EmitTime(void)
{
  RTC_TimeTypeDef t = {0};
  RTC_DateTypeDef d = {0};
  /* GetTime must be read before GetDate: reading time locks the shadow
     registers and reading date unlocks them again. */
  HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN);
  uint8_t valid = (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) == RTC_SET_MAGIC);

  char m[48];
  int n = snprintf(m, sizeof(m), "RTC set=%d %04d-%02d-%02d %02d:%02d:%02d\r\n",
                   valid, 2000 + d.Year, d.Month, d.Date,
                   t.Hours, t.Minutes, t.Seconds);
  HAL_UART_Transmit(&huart2, (uint8_t *)m, (uint16_t)n, HAL_MAX_DELAY);
}

/* Parse "YYYY-MM-DD HH:MM:SS" and load it into the RTC. The host sends this
   from "time set ...". Returns 1 on success, 0 on a malformed/out-of-range
   string (the RTC is left unchanged in that case). */
static uint8_t RTC_SetFromString(const char *s)
{
  int yr, mo, da, hh, mm, ss;
  if (sscanf(s, "%d-%d-%d %d:%d:%d", &yr, &mo, &da, &hh, &mm, &ss) != 6)
    return 0;
  if (yr < 2000 || yr > 2099 || mo < 1 || mo > 12 || da < 1 || da > 31 ||
      hh > 23 || mm > 59 || ss > 59)
    return 0;

  RTC_DateTypeDef d = {0};
  RTC_TimeTypeDef t = {0};
  d.Year    = (uint8_t)(yr - 2000);
  d.Month   = (uint8_t)mo;
  d.Date    = (uint8_t)da;
  d.WeekDay = RTC_WEEKDAY_MONDAY;   /* host doesn't track weekday; placeholder */
  t.Hours          = (uint8_t)hh;
  t.Minutes        = (uint8_t)mm;
  t.Seconds        = (uint8_t)ss;
  t.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  t.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &t, RTC_FORMAT_BIN) != HAL_OK) return 0;
  if (HAL_RTC_SetDate(&hrtc, &d, RTC_FORMAT_BIN) != HAL_OK) return 0;
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, RTC_SET_MAGIC);
  return 1;
}

/* --- RTC backup-cell (VBAT) voltage via the internal ADC channel ---------- */
/* The STM32L4 routes VBAT through an on-chip /3 bridge to an internal ADC
   channel, so we can read the coin cell with no extra parts. ADC1 is otherwise
   set up as an (idle) 4-channel scan for the planned analog sensors; we borrow
   rank 1 for a single VBAT conversion and put channel 5 back when done.
   VDDA is assumed = 3.300 V (regulated rail); good to a few % — plenty to tell
   a healthy CR2032 (~3.0 V) from a weak (<2.4 V) or missing (~0 V) cell.
   NOTE: only meaningful when the board's "VBAT Power Jumper" feeds the coin
   cell (BT1); if VBAT is strapped to 3V3 this just reports ~3.3 V. */
static uint8_t RTC_ReadVbat_mV(uint32_t *vbat_mv)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  uint8_t ok = 0;

  sConfig.Channel      = ADC_CHANNEL_VBAT;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;   /* bridge needs long Ts */
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) == HAL_OK &&
      HAL_ADC_Start(&hadc1) == HAL_OK)
  {
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
      uint32_t raw = HAL_ADC_GetValue(&hadc1);       /* 12-bit, VBAT/3 at pin */
      uint32_t at_pin_mv = (raw * 3300U) / 4095U;
      *vbat_mv = at_pin_mv * 3U;                     /* undo the /3 bridge */
      ok = 1;
    }
    HAL_ADC_Stop(&hadc1);
  }

  /* restore rank 1 to its original external channel (PA0 / ADC_CHANNEL_5) */
  sConfig.Channel = ADC_CHANNEL_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  return ok;
}

/* --- status OLED rendering -------------------------------------------------- */
/* A small multi-screen UI for the 128x64 SSD1306. The main loop cycles
   OLED_RenderScreen(0..OLED_SCREEN_COUNT-1) every 5 s. Each screen draws an
   inverted title bar (with a "n/N" page indicator) plus a few value rows, all
   read fresh from the sensors at draw time. Formatting is integer-only to match
   the nano-printf build (no %f). All draws no-op if the panel is absent. */
#define OLED_SCREEN_COUNT 5
#define OLED_HDR_H        11           /* title-bar height in pixels           */

/* Draw a string horizontally centred on the panel at row y. */
static void OLED_Center(int y, const char *s, uint8_t scale, SSD1306_Color color)
{
  int x = (SSD1306_WIDTH - SSD1306_TextWidth(s, scale)) / 2;
  SSD1306_WriteStringAt(&oled, x, y, s, scale, color);
}

/* Draw a little sun: a filled disc of radius r at (cx,cy) with eight short rays
   (4 cardinal + 4 diagonal) standing off the rim. Used on the boot splash. */
static void OLED_DrawSun(int cx, int cy, int r)
{
  for (int dy = -r; dy <= r; dy++)
    for (int dx = -r; dx <= r; dx++)
      if (dx * dx + dy * dy <= r * r)
        SSD1306_DrawPixel(&oled, cx + dx, cy + dy, SSD1306_WHITE);

  static const int8_t dir[8][2] = {
    { 0,-1}, { 0, 1}, {-1, 0}, { 1, 0},   /* N S W E         */
    {-1,-1}, { 1,-1}, {-1, 1}, { 1, 1}    /* NW NE SW SE     */
  };
  for (int i = 0; i < 8; i++)
    for (int d = r + 2; d <= r + 4; d++)   /* 3-px ray, offset off the rim */
      SSD1306_DrawPixel(&oled, cx + dir[i][0] * d, cy + dir[i][1] * d, SSD1306_WHITE);
}

/* Render the boot splash: rounded border, sun motif, bold title and a tagline.
   No-ops if the panel isn't present (all primitives guard on oled.present). */
static void OLED_Splash(void)
{
  if (!oled.present)
    return;
  SSD1306_Clear(&oled);

  /* Rounded outer frame (draw the rectangle, then knock out the 4 corners). */
  SSD1306_DrawRect(&oled, 0, 0, SSD1306_WIDTH, SSD1306_HEIGHT, SSD1306_WHITE);
  SSD1306_DrawPixel(&oled, 0, 0, SSD1306_BLACK);
  SSD1306_DrawPixel(&oled, SSD1306_WIDTH - 1, 0, SSD1306_BLACK);
  SSD1306_DrawPixel(&oled, 0, SSD1306_HEIGHT - 1, SSD1306_BLACK);
  SSD1306_DrawPixel(&oled, SSD1306_WIDTH - 1, SSD1306_HEIGHT - 1, SSD1306_BLACK);

  OLED_DrawSun(64, 11, 4);                       /* sun centred near the top   */
  OLED_Center(24, "SolarSense", 2, SSD1306_WHITE); /* bold title (12x16 cells) */
  SSD1306_DrawHLine(&oled, 39, 44, 50, SSD1306_WHITE); /* divider under title  */
  OLED_Center(50, "weather node  v1", 1, SSD1306_WHITE);

  SSD1306_Display(&oled);
}

/* Format a centi-unit fixed-point value (e.g. 2345 -> "23.45 C"), sign-aware. */
static void OLED_FmtCenti(char *o, size_t n, int32_t v, const char *unit)
{
  int neg = (v < 0);
  uint32_t a = (uint32_t)(neg ? -v : v);
  snprintf(o, n, "%s%lu.%02lu%s", neg ? "-" : "",
           (unsigned long)(a / 100U), (unsigned long)(a % 100U), unit);
}

/* Format a tenths fixed-point value (e.g. 872 -> "87.2 %"), sign-aware. */
static void OLED_FmtTenths(char *o, size_t n, int32_t v, const char *unit)
{
  int neg = (v < 0);
  uint32_t a = (uint32_t)(neg ? -v : v);
  snprintf(o, n, "%s%lu.%lu%s", neg ? "-" : "",
           (unsigned long)(a / 10U), (unsigned long)(a % 10U), unit);
}

/* Draw the inverted title bar with a right-aligned "screen+1/N" page chip. */
static void OLED_Header(const char *title, uint8_t screen)
{
  SSD1306_FillRect(&oled, 0, 0, SSD1306_WIDTH, OLED_HDR_H, SSD1306_WHITE);
  SSD1306_WriteStringAt(&oled, 2, 2, title, 1, SSD1306_BLACK);

  char page[8];
  snprintf(page, sizeof(page), "%u/%u", (unsigned)(screen + 1), OLED_SCREEN_COUNT);
  SSD1306_WriteStringAt(&oled, SSD1306_WIDTH - SSD1306_TextWidth(page, 1) - 2, 2,
                        page, 1, SSD1306_BLACK);
}

/* Right-aligned value at row y (keeps the label left, value flush-right). */
static void OLED_Row(int y, const char *label, const char *value)
{
  SSD1306_WriteStringAt(&oled, 2, y, label, 1, SSD1306_WHITE);
  SSD1306_WriteStringAt(&oled, SSD1306_WIDTH - SSD1306_TextWidth(value, 1) - 2, y,
                        value, 1, SSD1306_WHITE);
}

static void OLED_ScreenPower(void)
{
  OLED_Header("POWER", 0);

  float v = 0.0f, soc = 0.0f;
  MAX17048_ReadVoltage(&v);
  MAX17048_ReadSOC(&soc);
  int mv    = (int)(v * 1000.0f + 0.5f);
  int soc10 = (int)(soc * 10.0f + 0.5f);
  if (soc10 < 0) soc10 = 0;
  if (soc10 > 1000) soc10 = 1000;

  /* Big SOC readout on the left; battery voltage to its right. */
  char big[8];
  snprintf(big, sizeof(big), "%d%%", soc10 / 10);
  SSD1306_WriteStringAt(&oled, 2, 16, big, 2, SSD1306_WHITE);

  char volts[12];
  snprintf(volts, sizeof(volts), "%d.%02dV", mv / 1000, (mv % 1000) / 10);
  SSD1306_WriteStringAt(&oled, SSD1306_WIDTH - SSD1306_TextWidth(volts, 1) - 2, 20,
                        volts, 1, SSD1306_WHITE);

  /* CN3791 charge state (open-drain, active-low; DONE wins over CHRG). */
  uint8_t chrg_low = (HAL_GPIO_ReadPin(CN3791_CHRG_GPIO_Port, CN3791_CHRG_Pin) == GPIO_PIN_RESET);
  uint8_t done_low = (HAL_GPIO_ReadPin(CN3791_DONE_GPIO_Port, CN3791_DONE_Pin) == GPIO_PIN_RESET);
  const char *chg = done_low ? "DONE" : (chrg_low ? "CHARGING" : "IDLE");
  OLED_Row(40, "Charge", chg);

  uint32_t vbat_mv = 0;
  char bak[12];
  if (RTC_ReadVbat_mV(&vbat_mv))
    snprintf(bak, sizeof(bak), "%lu.%02luV",
             (unsigned long)(vbat_mv / 1000U), (unsigned long)((vbat_mv % 1000U) / 10U));
  else
    snprintf(bak, sizeof(bak), "--");
  OLED_Row(52, "Backup", bak);
}

static void OLED_ScreenPanels(void)
{
  OLED_Header("PANELS", 1);

  int32_t total_mw = 0;
  for (int i = 0; i < INA219_PANEL_COUNT; i++)
  {
    char label[6], value[20];
    snprintf(label, sizeof(label), "P%d", i + 1);

    int32_t bus_mv = 0, cur_ua = 0, pwr_mw = 0;
    if (panel_ina[i].present &&
        INA219_ReadBus_mV(&panel_ina[i], &bus_mv) == HAL_OK &&
        INA219_ReadCurrent_uA(&panel_ina[i], &cur_ua) == HAL_OK &&
        INA219_ReadPower_mW(&panel_ina[i], &pwr_mw) == HAL_OK)
    {
      int32_t ma = (cur_ua >= 0 ? cur_ua + 500 : cur_ua - 500) / 1000;
      total_mw += pwr_mw;
      snprintf(value, sizeof(value), "%ld.%02ldV %ldmA",
               (long)(bus_mv / 1000), (long)((bus_mv % 1000) / 10), (long)ma);
    }
    else
    {
      snprintf(value, sizeof(value), "--");
    }
    OLED_Row(14 + i * 11, label, value);
  }

  char total[16];
  snprintf(total, sizeof(total), "%ld.%02ldW",
           (long)(total_mw / 1000), (long)((total_mw % 1000) / 10));
  SSD1306_DrawHLine(&oled, 0, 47, SSD1306_WIDTH, SSD1306_WHITE);
  OLED_Row(52, "Total", total);
}

static void OLED_ScreenEnviron(void)
{
  OLED_Header("ENVIRONMENT", 2);

  char val[16];

  int32_t bmp_c100 = 0; uint32_t bmp_pa = 0;
  if (bmp.present && BMP280_Read(&bmp, &bmp_c100, &bmp_pa) == HAL_OK)
  {
    OLED_FmtCenti(val, sizeof(val), bmp_c100, "C");
    OLED_Row(14, "Board T", val);
    snprintf(val, sizeof(val), "%lu hPa", (unsigned long)((bmp_pa + 50U) / 100U));
    OLED_Row(25, "Press", val);
  }
  else
  {
    OLED_Row(14, "Board T", "--");
    OLED_Row(25, "Press", "--");
  }

  /* HX94C loops are only valid while the +12 V rail is on (else read open). */
  int32_t hx_rh10 = 0, hx_tc100 = 0, hx_irh = 0, hx_it = 0;
  uint8_t hx_flt = 0;
  int hx_ok = (HX94C_Read(&hx, &hx_rh10, &hx_tc100, &hx_irh, &hx_it, &hx_flt) == HAL_OK)
              && (hx_flt == 0);
  if (hx_ok)
  {
    OLED_FmtTenths(val, sizeof(val), hx_rh10, "%");
    OLED_Row(36, "Humidity", val);
    OLED_FmtCenti(val, sizeof(val), hx_tc100, "C");
    OLED_Row(47, "Air T", val);
  }
  else
  {
    OLED_Row(36, "Humidity", "12V off");
    OLED_Row(47, "Air T", "12V off");
  }
}

static void OLED_ScreenSystem(void)
{
  OLED_Header("SYSTEM", 3);

  char val[16];

  if (tc3.present)
  {
    int32_t tc_t10 = 0, tc_cj10 = 0; uint8_t tc_faults = 0;
    MAX31855_Read(&tc3, &tc_t10, &tc_cj10, &tc_faults);
    if (tc_faults == 0)
      OLED_FmtTenths(val, sizeof(val), tc_t10, "C");
    else
      snprintf(val, sizeof(val), "fault");
    OLED_Row(14, "Thermo", val);
  }
  else
  {
    OLED_Row(14, "Thermo", "--");
  }

  uint8_t rail12v = (HAL_GPIO_ReadPin(EN_12V_GPIO_Port, EN_12V_Pin) == GPIO_PIN_SET);
  OLED_Row(25, "12V rail", rail12v ? "ON" : "OFF");

  /* RAIN_DET is an active-low digital wet indicator (pulled high when dry). */
  uint8_t rain_wet = (HAL_GPIO_ReadPin(RAIN_DET_GPIO_Port, RAIN_DET_Pin) == GPIO_PIN_RESET);
  OLED_Row(36, "Rain", rain_wet ? "WET" : "dry");

  uint32_t up = HAL_GetTick() / 1000U;
  snprintf(val, sizeof(val), "%luh%02lum", (unsigned long)(up / 3600U),
           (unsigned long)((up % 3600U) / 60U));
  OLED_Row(47, "Uptime", val);
}

static void OLED_ScreenAQ(void)
{
  OLED_Header("AIR QUALITY", 4);

  int32_t pm25 = 0, pm100 = 0, co2 = 0, voc = 0, nox = 0;
  char val[16];
  int ok = (SEN66_Read(&sen, NULL, &pm25, NULL, &pm100,
                       NULL, NULL, &voc, &nox, &co2) == HAL_OK);

  if (ok)
  {
    snprintf(val, sizeof(val), "%ld.%ldug",
             (long)(pm25 / 10), (long)(pm25 % 10));
    OLED_Row(14, "PM2.5", val);

    snprintf(val, sizeof(val), "%ld.%ldug",
             (long)(pm100 / 10), (long)(pm100 % 10));
    OLED_Row(25, "PM10", val);

    snprintf(val, sizeof(val), "%ldppm", (long)co2);
    OLED_Row(36, "CO2", val);

    snprintf(val, sizeof(val), "%ld/%ld", (long)voc, (long)nox);
    OLED_Row(47, "VOC/NOx", val);
  }
  else
  {
    OLED_Row(14, "PM2.5",   "--");
    OLED_Row(25, "PM10",    "--");
    OLED_Row(36, "CO2",     "--");
    OLED_Row(47, "VOC/NOx", "--");
  }
}

/* --- Low power / maintenance helpers --------------------------------------- */

/* Send a raw SSD1306 command byte over I2C (control byte 0x00 = command). */
static void OLED_SendCmd(uint8_t cmd)
{
  if (!oled.present) return;
  uint8_t buf[2] = { 0x00u, cmd };
  HAL_I2C_Master_Transmit(oled.hi2c, oled.addr8, buf, 2, 10);
}

/* Blank the display frame buffer and push a black frame, then issue Display
   OFF so the panel draws ~0 mA during sleep. */
static void OLED_Blank(void)
{
  SSD1306_Clear(&oled);
  SSD1306_Display(&oled);
  OLED_SendCmd(0xAEu); /* Display OFF */
}

/* Turn the display back on (frame buffer already valid from last render). */
static void OLED_Unblank(void)
{
  OLED_SendCmd(0xAFu); /* Display ON */
}

/* Enter STM32L4 STOP2 mode for `seconds` seconds, then resume.
   Clocks, I2C, UART, and ADC are restored before returning.
   The SEN66 is put to sleep and woken around the stop interval. */
static void Power_EnterStop2(uint32_t seconds)
{
  /* Clamp to a reasonable range. */
  if (seconds < 5u)    seconds = 5u;
  if (seconds > 86400u) seconds = 86400u;

  char msg[56];
  int n = snprintf(msg, sizeof(msg), "SLEEP: STOP2 for %lu s\r\n",
                   (unsigned long)seconds);
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)n, HAL_MAX_DELAY);

  /* Sensor/display housekeeping before sleeping. */
  OLED_Blank();
  if (sen.present) SEN66_Sleep(&sen);
  HAL_GPIO_WritePin(EN_12V_GPIO_Port, EN_12V_Pin, GPIO_PIN_RESET);

  /* RTC periodic wake-up timer: CK_SPRE_16BITS = 1 Hz reference clock;
     counter (seconds - 1) gives exactly `seconds` ticks. */
  HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, seconds - 1u,
                              RTC_WAKEUPCLOCK_CK_SPRE_16BITS);

  /* Enter STOP2.  Execution resumes here after the RTC fires. */
  HAL_SuspendTick();
  HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

  /* --- Returned from STOP2 --- */

  /* Restore system and peripheral clocks (STOP2 switches to MSI @ 4 MHz). */
  SystemClock_Config();
  PeriphCommonClock_Config();
  HAL_ResumeTick();

  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

  /* Reinitialise clock-dependent peripherals. */
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  /* Re-arm the UART RX interrupt. */
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

  /* Wake sensors and display. */
  if (sen.present) SEN66_Wake(&sen);
  OLED_Unblank();

  stream_enabled = 1;
  force_report   = 1;

  const char *m2 = "SLEEP: awake\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)m2, strlen(m2), HAL_MAX_DELAY);
}

/* Enter STM32L4 Standby mode (deepest sleep, ~1 µA).
   SRAM is lost; the node performs a cold reboot on wake.
   Wake sources: RTC WakeUp timer (if `wakeup_s` > 0), or WKUP1 (PA0). */
static void Power_EnterStandby(uint32_t wakeup_s)
{
  char msg[72];
  int n;
  if (wakeup_s > 0u)
    n = snprintf(msg, sizeof(msg),
                 "SHUTDOWN: Standby for %lu s - will reboot on wake\r\n",
                 (unsigned long)wakeup_s);
  else
    n = snprintf(msg, sizeof(msg),
                 "SHUTDOWN: Standby - power cycle or WKUP1 to wake\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)n, HAL_MAX_DELAY);

  OLED_Blank();
  if (sen.present) SEN66_Sleep(&sen);
  HAL_GPIO_WritePin(EN_12V_GPIO_Port, EN_12V_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);

  if (wakeup_s > 0u) {
    HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
    HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, wakeup_s - 1u,
                                RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
  }

  /* Clear any pending wakeup flags, then enter Standby. */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF1);
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1_HIGH);
  HAL_PWR_EnterSTANDBYMode();
  /* Never returns — MCU resets on wake. */
}

static void OLED_RenderScreen(uint8_t screen)
{
  if (!oled.present)
    return;
  SSD1306_Clear(&oled);
  if (maint_mode)
  {
    OLED_Center(16, "MAINTENANCE", 1, SSD1306_WHITE);
    OLED_Center(30, "stream OFF", 1, SSD1306_WHITE);
    OLED_Center(44, "cmd: maintenance off", 1, SSD1306_WHITE);
  }
  else
  {
    switch (screen)
    {
      case 0:  OLED_ScreenPower();   break;
      case 1:  OLED_ScreenPanels();  break;
      case 2:  OLED_ScreenEnviron(); break;
      case 3:  OLED_ScreenSystem();  break;
      default: OLED_ScreenAQ();      break;
    }
  }
  SSD1306_Display(&oled);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  /* MX_SDMMC1_SD_Init();  -- replaced by non-fatal init in USER CODE BEGIN 2
     (the generated version calls Error_Handler() if no card is present, which
     would hang the whole node). Re-comment this if you regenerate from CubeMX. */
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  /* STM32L4 ADC requires calibration before first use; must be done while the
     ADC is enabled but not converting (i.e. right after MX_ADC1_Init). */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  uint32_t last_flash = 0;      /* LED_STATUS flash timer (250 ms pulse every 10 s) */
  uint8_t  led_on = 0;          /* current LED_STATUS pulse state */
  uint32_t last_heartbeat = 0;  /* serial heartbeat timer (every 5 s) */
  uint32_t last_alert_chk = 0;  /* battery-alert re-arm check timer (every 5 s) */
  uint32_t heartbeat_count = 0;
  uint32_t last_oled = 0;       /* OLED screen-cycle timer (advance every 5 s) */
  uint8_t  oled_screen = 0;     /* index of the screen currently shown         */

  const char *banner = "\r\nSolarSense v1 boot OK\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)banner, strlen(banner), HAL_MAX_DELAY);

  {
    HAL_StatusTypeDef st = CAN_TLM_Start(&hcan1);
    const char *m = (st == HAL_OK) ? "CAN TLM started @500kbps\r\n"
                                   : "CAN TLM start FAILED - CAN TX disabled\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)m, strlen(m), HAL_MAX_DELAY);
  }

  /* Card-detect-gated SD init. Only touch the SDMMC when a card is physically
     present; track card-present state so we can react to hot insert/remove. */
  uint8_t sd_ready = 0;                       /* 1 = SD initialised and usable  */
  uint8_t card_present = SD_CardPresent();    /* debounced state of PC6          */

  if (card_present)
  {
    sd_ready = SD_TryInit();
    const char *m = sd_ready ? "SD init OK\r\n"
                             : "SD present but init FAILED - continuing\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)m, strlen(m), HAL_MAX_DELAY);
  }
  else
  {
    const char *m = "No SD card detected - continuing\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)m, strlen(m), HAL_MAX_DELAY);
  }
  /* LED_ERR reflects "card present but unusable" */
  HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin,
                    (card_present && !sd_ready) ? GPIO_PIN_SET : GPIO_PIN_RESET);

  /* --- MAX17048 fuel gauge + BAT_ALRT (PB13) interrupt ------------------- */
  /* Reconfigure PB13 from plain input to a falling-edge EXTI. ALRT is
     open-drain/active-low, so enable a pull-up and trigger on the falling
     edge. (Done in code so it survives a CubeMX regenerate; if you instead
     set PB13 = GPIO_EXTI13 in CubeMX, remove this block and the handler in
     stm32l4xx_it.c to avoid a duplicate IRQ handler.) */
  {
    GPIO_InitTypeDef alrt = {0};
    alrt.Pin = BAT_ALRT_Pin;
    alrt.Mode = GPIO_MODE_IT_FALLING;
    alrt.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BAT_ALRT_GPIO_Port, &alrt);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  }

  if (MAX17048_Init(&hi2c1) == HAL_OK)
  {
    MAX17048_SetEmptyAlertThreshold(15);        /* alert at <= 15% SOC      */
    MAX17048_SetVoltageAlerts(3.0f, 4.25f);     /* under/over-voltage limits */
    MAX17048_EnableSOCChangeAlert(0);           /* keep SOC-change alert off */
    MAX17048_ClearAlert();

    uint16_t ver = 0; float v = 0.0f, soc = 0.0f;
    MAX17048_ReadVersion(&ver);
    MAX17048_ReadVoltage(&v);
    MAX17048_ReadSOC(&soc);
    /* integer formatting: nano printf has no %f unless -u _printf_float */
    int mv = (int)(v * 1000.0f + 0.5f);
    int soc10 = (int)(soc * 10.0f + 0.5f);
    char m[72];
    int n = snprintf(m, sizeof(m), "MAX17048 ok ver=0x%04X %dmV %d.%d%%\r\n",
                     ver, mv, soc10 / 10, soc10 % 10);
    HAL_UART_Transmit(&huart2, (uint8_t *)m, (uint16_t)n, HAL_MAX_DELAY);
  }
  else
  {
    const char *m = "MAX17048 not found\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)m, strlen(m), HAL_MAX_DELAY);
  }

  /* --- INA219 panel current/voltage monitors (I2C1) --------------------- */
  /* Three populated parts (0x40/0x41/0x44); each init is non-fatal so a
     missing/unpopulated panel monitor does not block the node. */
  for (int i = 0; i < INA219_PANEL_COUNT; i++)
  {
    HAL_StatusTypeDef st = INA219_Init(&panel_ina[i], &hi2c1, panel_ina_addr[i]);
    char m[56];
    int n = snprintf(m, sizeof(m), "INA219 P%d @0x%02X %s\r\n",
                     i + 1, panel_ina_addr[i],
                     (st == HAL_OK) ? "ok" : "not found");
    HAL_UART_Transmit(&huart2, (uint8_t *)m, (uint16_t)n, HAL_MAX_DELAY);
  }

  /* --- MAX31855 thermocouple (TC3, PB1) --------------------------------- */
  {
    HAL_StatusTypeDef st = MAX31855_Init(&tc3, &hspi1,
                                         TC3_CS_GPIO_Port, TC3_CS_Pin);
    const char *m = (st == HAL_OK) ? "MAX31855 TC3 ok\r\n"
                                   : "MAX31855 TC3 SPI FAIL\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)m, strlen(m), HAL_MAX_DELAY);
  }

  /* --- BMP280 pressure + board temperature (I2C1 @0x76) ----------------- */
  /* Non-fatal: a missing barometer just drops the bmp_* TLM keys. */
  {
    HAL_StatusTypeDef st = BMP280_Init(&bmp, &hi2c1, BMP280_ADDR);
    char m[56];
    int n;
    if (st == HAL_OK)
      n = snprintf(m, sizeof(m), "BMP280 ok @0x%02X id=0x%02X (%s)\r\n",
                   bmp.addr7, bmp.chip_id,
                   (bmp.chip_id == BME280_CHIP_ID) ? "BME280" : "BMP280");
    else
      n = snprintf(m, sizeof(m), "BMP280 not found (last id=0x%02X)\r\n",
                   bmp.chip_id);
    HAL_UART_Transmit(&huart2, (uint8_t *)m, (uint16_t)n, HAL_MAX_DELAY);
  }

  /* --- HX94C humidity/temp probe (4-20 mA loops on ADC1) ---------------- */
  /* Analog-only: nothing to probe, so just record the channels. The loops are
     fed by the +12 V rail, so a meaningful read needs "12v on" first. */
  HX94C_Init(&hx, &hadc1, ADC_CHANNEL_6, ADC_CHANNEL_13, ADC_CHANNEL_5);
  {
    const char *m = "HX94C on ADC RH=IN6 T=IN13 (needs +12 V rail)\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)m, strlen(m), HAL_MAX_DELAY);
  }

  /* --- I2C1 bus scan (debug aid) ---------------------------------------- */
  /* Walk the 7-bit address space and report every device that ACKs. Useful to
     confirm the OLED is wired/pulled-up and at what address it actually sits. */
  {
    char m[40];
    int n = snprintf(m, sizeof(m), "I2C1 scan:");
    HAL_UART_Transmit(&huart2, (uint8_t *)m, (uint16_t)n, HAL_MAX_DELAY);
    int found = 0;
    for (uint8_t a = 0x08; a <= 0x77; a++)
    {
      if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 2, 5) == HAL_OK)
      {
        n = snprintf(m, sizeof(m), " 0x%02X", a);
        HAL_UART_Transmit(&huart2, (uint8_t *)m, (uint16_t)n, HAL_MAX_DELAY);
        found++;
      }
    }
    n = snprintf(m, sizeof(m), found ? "\r\n" : " (none)\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)m, (uint16_t)n, HAL_MAX_DELAY);
  }

  /* --- SSD1306 status OLED (I2C1 @0x3C) --------------------------------- */
  /* Fully optional: the panel is probed at boot and, if it doesn't ACK,
     oled.present stays 0 and the firmware runs exactly as before -- the screen
     cycle is skipped and every draw call is an internal no-op. When present,
     show an elegant splash before the screens start cycling. */
  {
    HAL_StatusTypeDef st = SSD1306_Init(&oled, &hi2c1, SSD1306_ADDR);
    if (oled.present)
      OLED_Splash();
    const char *m = (st == HAL_OK) ? "SSD1306 OLED ok @0x3C\r\n"
                                   : "SSD1306 OLED not found - display disabled\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)m, strlen(m), HAL_MAX_DELAY);
  }

  /* --- SEN66 air quality sensor (I2C1 @0x6B) ---------------------------- */
  /* Probed only if the I2C scan found it.  Init issues a soft-reset and
     starts continuous measurement; first valid readings arrive ~1 s later. */
  {
    HAL_StatusTypeDef st = SEN66_Init(&sen, &hi2c1);
    char m[56];
    int n = snprintf(m, sizeof(m), "SEN66 %s @0x6B\r\n",
                     (st == HAL_OK) ? "ok" : "not found");
    HAL_UART_Transmit(&huart2, (uint8_t *)m, (uint16_t)n, HAL_MAX_DELAY);
  }

  /* --- host serial command interface ------------------------------------ */
  /* Enable the USART2 RX interrupt in code (so it survives a CubeMX
     regenerate; the IRQ handler lives in stm32l4xx_it.c USER CODE 1). */
  HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  {
    const char *m = "Commands: stream on|off, status, 12v on|off, "
                    "time [set YYYY-MM-DD HH:MM:SS], "
                    "sleep [N], shutdown [N], maintenance on|off, reboot, help\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)m, strlen(m), HAL_MAX_DELAY);
  }
  /* Report the RTC at boot so the host sees whether the clock is still set. */
  RTC_EmitTime();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* Flash LED_STATUS for 250 ms every 10 s (proof-of-life heartbeat). */
    if (!led_on && (now - last_flash >= 10000U))
    {
      last_flash = now;
      led_on = 1;
      HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
    }
    else if (led_on && (now - last_flash >= 250U))
    {
      led_on = 0;
      HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
    }

    /* Cycle the OLED through its screens, one every 5 s (after the boot splash
       holds for the first 5 s). Cheap to redraw, and reads sensors fresh. */
    if (oled.present && (now - last_oled >= 5000U))
    {
      last_oled = now;
      OLED_RenderScreen(oled_screen);
      oled_screen = (uint8_t)((oled_screen + 1) % OLED_SCREEN_COUNT);
    }

    /* Process a completed host command line (set by the USART2 RX callback). */
    if (rx_ready)
    {
      rx_ready = 0;
      char *c = rx_cmd;
      while (*c == ' ') c++;                 /* trim leading spaces */
      const char *resp = NULL;
      if      (!strcmp(c, "stream on"))  { stream_enabled = 1; resp = "# stream on\r\n"; }
      else if (!strcmp(c, "stream off")) { stream_enabled = 0; resp = "# stream off\r\n"; }
      else if (!strcmp(c, "status"))     { force_report = 1;   resp = "# status\r\n"; }
      else if (!strcmp(c, "12v on"))
        { HAL_GPIO_WritePin(EN_12V_GPIO_Port, EN_12V_Pin, GPIO_PIN_SET);  resp = "# 12v on\r\n"; }
      else if (!strcmp(c, "12v off"))
        { HAL_GPIO_WritePin(EN_12V_GPIO_Port, EN_12V_Pin, GPIO_PIN_RESET); resp = "# 12v off\r\n"; }
      else if (!strcmp(c, "time") || !strcmp(c, "time get"))
        { RTC_EmitTime(); }                  /* emits its own "RTC ..." line */
      else if (!strncmp(c, "time set ", 9))
        { if (RTC_SetFromString(c + 9)) { resp = "# time set\r\n"; RTC_EmitTime(); }
          else { resp = "# time set: bad format (YYYY-MM-DD HH:MM:SS)\r\n"; } }
      else if (!strncmp(c, "sleep", 5) && (c[5] == '\0' || c[5] == ' '))
      {
        uint32_t s = 60;
        if (c[5] == ' ')
        {
          long v = strtol(c + 6, NULL, 10);
          if (v >= 5 && v <= 86400) s = (uint32_t)v;
        }
        Power_EnterStop2(s); /* returns after wake */
      }
      else if (!strncmp(c, "shutdown", 8) && (c[8] == '\0' || c[8] == ' '))
      {
        uint32_t s = 0;
        if (c[8] == ' ')
        {
          long v = strtol(c + 9, NULL, 10);
          if (v >= 5 && v <= 86400) s = (uint32_t)v;
        }
        Power_EnterStandby(s); /* never returns */
      }
      else if (!strcmp(c, "maintenance on"))
      {
        maint_mode     = 1;
        stream_enabled = 0;
        HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
        resp = "# maintenance on\r\n";
      }
      else if (!strcmp(c, "maintenance off"))
      {
        maint_mode     = 0;
        stream_enabled = 1;
        HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
        resp = "# maintenance off\r\n";
      }
      else if (!strcmp(c, "reboot"))
      {
        const char *m = "# rebooting\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)m, strlen(m), HAL_MAX_DELAY);
        HAL_Delay(50);
        NVIC_SystemReset();
      }
      else if (!strcmp(c, "help") || !strcmp(c, "?"))
        resp = "# cmds: stream on|off, status, 12v on|off, "
               "time [set YYYY-MM-DD HH:MM:SS], "
               "sleep [N], shutdown [N], maintenance on|off, reboot, help\r\n";
      else
        resp = "# unknown cmd (try help)\r\n";
      if (resp)
        HAL_UART_Transmit(&huart2, (uint8_t *)resp, strlen(resp), HAL_MAX_DELAY);
    }

    /* Keep the command receiver armed: a blocking UART transmit can briefly
       hold the HAL lock and make the in-callback re-arm fail, so re-arm here
       whenever the RX path has gone idle. */
    if (huart2.RxState == HAL_UART_STATE_READY)
    {
      HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }

    /* Structured telemetry line every 5 s (or once on "status"). Emitted only
       while streaming is enabled; the host dashboard parses the key=value
       tokens. Units: *_mv = mV, bat_soc = tenths of %, *_ma = tenths of mA,
       *_mw = mW, chg 0=idle/1=charging/2=done, rtc_bv = RTC backup-cell mV
       (0 = read failed), bmp_pa = Pa, bmp_c = centi-C, hx_rh = tenths %RH,
       hx_t = centi-C, hx_rhi/hx_ti = loop current in tenths of mA, hx_flt =
       HX94C fault bits (0=ok; loops read open until the +12 V rail is on).
       Planned sensors (SEN66, UV, rain) will append their own keys. */
    if (force_report || (stream_enabled && (now - last_heartbeat >= 5000U)))
    {
      last_heartbeat = now;
      force_report = 0;

      SolarSenseTLM_t can_tlm = {0};

      uint8_t rail12v = (HAL_GPIO_ReadPin(EN_12V_GPIO_Port, EN_12V_Pin) == GPIO_PIN_SET);

      float v = 0.0f, soc = 0.0f;
      MAX17048_ReadVoltage(&v);
      MAX17048_ReadSOC(&soc);
      int mv = (int)(v * 1000.0f + 0.5f);
      int soc10 = (int)(soc * 10.0f + 0.5f);

      /* CN3791 status (open-drain, active-low). DONE takes priority. */
      uint8_t chrg_low = (HAL_GPIO_ReadPin(CN3791_CHRG_GPIO_Port, CN3791_CHRG_Pin) == GPIO_PIN_RESET);
      uint8_t done_low = (HAL_GPIO_ReadPin(CN3791_DONE_GPIO_Port, CN3791_DONE_Pin) == GPIO_PIN_RESET);
      int chg = done_low ? 2 : (chrg_low ? 1 : 0);

      can_tlm.bat_mv  = (uint16_t)(mv    < 0 ? 0 : mv);
      can_tlm.bat_soc = (uint16_t)(soc10 < 0 ? 0 : soc10);
      can_tlm.chg     = (uint8_t)chg;
      can_tlm.r12v    = rail12v;
      can_tlm.sd      = sd_ready ? 1u : 0u;

      uint32_t vbat_mv = 0;
      uint8_t  vbat_ok = RTC_ReadVbat_mV(&vbat_mv);

      char tlm[640];
      int o = snprintf(tlm, sizeof(tlm),
                       "TLM up=%lu hb=%lu sd=%d bat_mv=%d bat_soc=%d chg=%d r12v=%d rtc_bv=%lu",
                       (unsigned long)(now / 1000U),
                       (unsigned long)(++heartbeat_count),
                       sd_ready ? 1 : 0, mv, soc10, chg, rail12v ? 1 : 0,
                       (unsigned long)(vbat_ok ? vbat_mv : 0));

      for (int i = 0; i < INA219_PANEL_COUNT && o < (int)sizeof(tlm) - 56; i++)
      {
        int ok = 0;
        int32_t bus_mv = 0, cur_ua = 0, pwr_mw = 0;
        if (panel_ina[i].present &&
            INA219_ReadBus_mV(&panel_ina[i], &bus_mv) == HAL_OK &&
            INA219_ReadCurrent_uA(&panel_ina[i], &cur_ua) == HAL_OK &&
            INA219_ReadPower_mW(&panel_ina[i], &pwr_mw) == HAL_OK)
        {
          ok = 1;
        }
        int32_t ma10 = (cur_ua >= 0 ? cur_ua + 50 : cur_ua - 50) / 100;
        o += snprintf(tlm + o, sizeof(tlm) - o,
                      " p%d_ok=%d p%d_mv=%ld p%d_ma=%ld p%d_mw=%ld",
                      i + 1, ok, i + 1, (long)bus_mv,
                      i + 1, (long)ma10, i + 1, (long)pwr_mw);
        can_tlm.p_ok[i] = (uint8_t)ok;
        can_tlm.p_mv[i] = (int16_t)(bus_mv > 32767 ? 32767 : bus_mv < -32768 ? -32768 : bus_mv);
        can_tlm.p_ma[i] = (int16_t)(ma10   > 32767 ? 32767 : ma10   < -32768 ? -32768 : ma10);
        can_tlm.p_mw[i] = (int16_t)(pwr_mw > 32767 ? 32767 : pwr_mw < -32768 ? -32768 : pwr_mw);
      }
      if (tc3.present && o < (int)sizeof(tlm) - 48)
      {
        int32_t tc_t10 = 0, tc_cj10 = 0;
        uint8_t tc_faults = 0;
        MAX31855_Read(&tc3, &tc_t10, &tc_cj10, &tc_faults);
        /* Scale tenths→centi-°C (×10) to match the dashboard protocol spec. */
        o += snprintf(tlm + o, sizeof(tlm) - o,
                      " tc3_ok=%d tc3_c=%ld tc3_cj=%ld tc3_fault=0x%02X",
                      (tc_faults == 0) ? 1 : 0,
                      (long)(tc_t10 * 10), (long)(tc_cj10 * 10), tc_faults);
        can_tlm.tc3_ok    = (tc_faults == 0) ? 1u : 0u;
        can_tlm.tc3_fault = tc_faults;
        can_tlm.tc3_c     = (int16_t)(tc_t10 * 10);
        can_tlm.tc3_cj    = (int16_t)(tc_cj10 * 10);
      }
      if (bmp.present && o < (int)sizeof(tlm) - 40)
      {
        int32_t bmp_c100 = 0;
        uint32_t bmp_pa = 0;
        int ok = (BMP280_Read(&bmp, &bmp_c100, &bmp_pa) == HAL_OK);
        /* bmp_pa = absolute pressure in Pa, bmp_c = board temp in centi-C. */
        o += snprintf(tlm + o, sizeof(tlm) - o,
                      " bmp_ok=%d bmp_pa=%lu bmp_c=%ld",
                      ok, (unsigned long)bmp_pa, (long)bmp_c100);
        can_tlm.bmp_ok = (uint8_t)ok;
        can_tlm.bmp_c  = (int16_t)(bmp_c100 > 32767 ? 32767 : bmp_c100 < -32768 ? -32768 : bmp_c100);
        can_tlm.bmp_pa = bmp_pa;
      }
      if (o < (int)sizeof(tlm) - 72)
      {
        int32_t hx_rh10 = 0, hx_tc100 = 0, hx_irh = 0, hx_it = 0;
        uint8_t hx_flt = 0;
        int ok = (HX94C_Read(&hx, &hx_rh10, &hx_tc100, &hx_irh, &hx_it, &hx_flt)
                  == HAL_OK) && (hx_flt == 0);
        /* Loop currents reported in tenths of a mA, matching the panel scale. */
        int32_t irh10 = (hx_irh + 50) / 100;
        int32_t it10  = (hx_it  + 50) / 100;
        o += snprintf(tlm + o, sizeof(tlm) - o,
                      " hx_ok=%d hx_rh=%ld hx_t=%ld hx_rhi=%ld hx_ti=%ld hx_flt=0x%02X",
                      ok, (long)hx_rh10, (long)hx_tc100,
                      (long)irh10, (long)it10, hx_flt);
        can_tlm.hx_ok  = (uint8_t)ok;
        can_tlm.hx_flt = hx_flt;
        can_tlm.hx_rh  = (int16_t)hx_rh10;
        can_tlm.hx_t   = (int16_t)(hx_tc100 > 32767 ? 32767 : hx_tc100 < -32768 ? -32768 : hx_tc100);
        can_tlm.hx_rhi = (uint8_t)(irh10 > 255 ? 255 : irh10 < 0 ? 0 : irh10);
        can_tlm.hx_ti  = (uint8_t)(it10  > 255 ? 255 : it10  < 0 ? 0 : it10);
      }

      /* SEN66 air quality: PM, RH, T, VOC, NOx, CO2. */
      if (sen.present && o < (int)sizeof(tlm) - 128)
      {
        int32_t s_pm10 = 0, s_pm25 = 0, s_pm40 = 0, s_pm100 = 0;
        int32_t s_rh = 0, s_t = 0, s_voc = 0, s_nox = 0, s_co2 = 0;
        int ok = (SEN66_Read(&sen,
                             &s_pm10, &s_pm25, &s_pm40, &s_pm100,
                             &s_rh,   &s_t,    &s_voc,  &s_nox, &s_co2)
                  == HAL_OK);
        o += snprintf(tlm + o, sizeof(tlm) - o,
                      " aq_ok=%d"
                      " aq_pm10=%ld aq_pm25=%ld aq_pm40=%ld aq_pm100=%ld"
                      " aq_rh=%ld aq_t=%ld aq_voc=%ld aq_nox=%ld aq_co2=%ld",
                      ok,
                      (long)s_pm10, (long)s_pm25, (long)s_pm40, (long)s_pm100,
                      (long)s_rh, (long)s_t, (long)s_voc, (long)s_nox, (long)s_co2);
        can_tlm.aq_ok    = (uint8_t)ok;
        can_tlm.aq_pm10  = (int16_t)(s_pm10  > 32767 ? 32767 : s_pm10);
        can_tlm.aq_pm25  = (int16_t)(s_pm25  > 32767 ? 32767 : s_pm25);
        can_tlm.aq_pm40  = (int16_t)(s_pm40  > 32767 ? 32767 : s_pm40);
        can_tlm.aq_pm100 = (int16_t)(s_pm100 > 32767 ? 32767 : s_pm100);
        can_tlm.aq_rh    = (int16_t)s_rh;
        can_tlm.aq_t     = (int16_t)(s_t   > 32767 ? 32767 : s_t < -32768 ? -32768 : s_t);
        can_tlm.aq_voc   = (int16_t)(s_voc > 32767 ? 32767 : s_voc);
        can_tlm.aq_nox   = (int16_t)(s_nox > 32767 ? 32767 : s_nox);
        can_tlm.aq_co2   = (int16_t)(s_co2 > 32767 ? 32767 : s_co2);
      }

      /* UV sensor (GUVA analog) on ADC1 rank 1 (ADC_CHANNEL_5 / PA0).
         Scan group has 4 ranks; all must be drained on every conversion. */
      if (o < (int)sizeof(tlm) - 16)
      {
        uint32_t uv_raw = 0;
        if (HAL_ADC_Start(&hadc1) == HAL_OK)
        {
          if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
            uv_raw = HAL_ADC_GetValue(&hadc1);
          /* Drain ranks 2-4 (HX94C RH, HX94C TEMP, RAIN AO) */
          for (int r = 1; r < 4; r++)
          {
            if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
              (void)HAL_ADC_GetValue(&hadc1);
          }
          HAL_ADC_Stop(&hadc1);
        }
        uint32_t uv_mv = uv_raw * 3300U / 4095U;
        can_tlm.uv_mv = (uint16_t)(uv_mv > 65535U ? 65535U : uv_mv);
        o += snprintf(tlm + o, sizeof(tlm) - o, " uv_mv=%lu", (unsigned long)uv_mv);
      }

      o += snprintf(tlm + o, sizeof(tlm) - o, "\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t *)tlm, (uint16_t)o, HAL_MAX_DELAY);
      CAN_TLM_Send(&hcan1, &can_tlm);
    }

    /* Battery-alert re-arm check, every 5 s and independent of streaming: if
       the alert was masked, re-arm only once the condition is gone (prevents a
       storm from a persistent/bouncing condition). */
    if (bat_alert_masked && (now - last_alert_chk >= 5000U))
    {
      last_alert_chk = now;
      uint16_t status = 0;
      if (MAX17048_GetStatus(&status) == HAL_OK &&
          (status & BAT_ALERT_CONDITIONS) == 0)
      {
        MAX17048_ClearAlert();
        __HAL_GPIO_EXTI_CLEAR_IT(BAT_ALRT_Pin);   /* drop any stale edge */
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
        bat_alert_masked = 0;
        const char *m = "BAT alert re-armed\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)m, strlen(m), HAL_MAX_DELAY);
      }
    }

    /* Battery alert: BAT_ALRT (PB13) fired -> read cause, report once, then
       mask the EXTI. We must not re-clear while the condition is still true,
       or the gauge re-asserts ALRT immediately and storms us. */
    if (bat_alert_flag)
    {
      bat_alert_flag = 0;
      uint16_t status = 0;
      if (MAX17048_GetStatus(&status) == HAL_OK)
      {
        char msg[96];
        int len = snprintf(msg, sizeof(msg),
            "BAT ALERT status=0x%04X%s%s%s%s%s%s\r\n", status,
            (status & MAX17048_STATUS_VH) ? " VoltHigh"   : "",
            (status & MAX17048_STATUS_VL) ? " VoltLow"    : "",
            (status & MAX17048_STATUS_VR) ? " VoltReset"  : "",
            (status & MAX17048_STATUS_HD) ? " SOCLow"     : "",
            (status & MAX17048_STATUS_SC) ? " SOCChange"  : "",
            (status & MAX17048_STATUS_RI) ? " ResetInd"   : "");
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)len, HAL_MAX_DELAY);
      }
      MAX17048_ClearAlert();              /* release the ALRT pin             */
      HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);/* mask until the condition clears  */
      bat_alert_masked = 1;
    }

    /* While masked, periodically (re-uses the 5 s heartbeat tick below) check
       whether the alert condition has cleared, and only then re-arm. */

    /* Handle microSD hot insert/remove (poll PC6) */
    uint8_t cd_now = SD_CardPresent();
    if (cd_now != card_present)
    {
      HAL_Delay(50);                 /* debounce the mechanical detect switch */
      cd_now = SD_CardPresent();
      if (cd_now != card_present)
      {
        card_present = cd_now;
        if (card_present)
        {
          sd_ready = SD_TryInit();
          const char *m = sd_ready ? "SD inserted - init OK\r\n"
                                   : "SD inserted - init FAILED\r\n";
          HAL_UART_Transmit(&huart2, (uint8_t *)m, strlen(m), HAL_MAX_DELAY);
        }
        else
        {
          HAL_SD_DeInit(&hsd1);
          sd_ready = 0;
          const char *m = "SD removed\r\n";
          HAL_UART_Transmit(&huart2, (uint8_t *)m, strlen(m), HAL_MAX_DELAY);
        }
        HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin,
                          (card_present && !sd_ready) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 12;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 10;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TC1_CS_Pin|LORA_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TC2_CS_Pin|TC3_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_12V_Pin|LORA_RESET_Pin|LED_STATUS_Pin|LED_ERR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TC1_CS_Pin LORA_CS_Pin */
  GPIO_InitStruct.Pin = TC1_CS_Pin|LORA_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TC2_CS_Pin TC3_CS_Pin EN_12V_Pin LORA_RESET_Pin
                           LED_STATUS_Pin LED_ERR_Pin */
  GPIO_InitStruct.Pin = TC2_CS_Pin|TC3_CS_Pin|EN_12V_Pin|LORA_RESET_Pin
                          |LED_STATUS_Pin|LED_ERR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_DIO1_Pin LORA_BUSY_Pin BAT_ALRT_Pin CN3791_CHRG_Pin
                           CN3791_DONE_Pin RAIN_DET_Pin */
  GPIO_InitStruct.Pin = LORA_DIO1_Pin|LORA_BUSY_Pin|BAT_ALRT_Pin|CN3791_CHRG_Pin
                          |CN3791_DONE_Pin|RAIN_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SDMMC1_CD_Pin */
  GPIO_InitStruct.Pin = SDMMC1_CD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDMMC1_CD_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* EXTI line callback: BAT_ALRT (PB13) from the MAX17048 fuel gauge.
   Keep it minimal - just latch a flag and service it in the main loop
   (I2C reads must not run from the ISR). */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BAT_ALRT_Pin)
  {
    bat_alert_flag = 1;
  }
}

/* USART2 RX complete: build a command line one byte at a time and flag it for
   the main loop on CR/LF. Keep it minimal - parsing/I2C happen in the loop. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    char ch = (char)rx_byte;
    if (ch == '\r' || ch == '\n')
    {
      if (rx_len > 0 && !rx_ready)
      {
        rx_buf[rx_len] = '\0';
        memcpy(rx_cmd, rx_buf, (size_t)rx_len + 1);
        rx_ready = 1;
      }
      rx_len = 0;
    }
    else if (rx_len < sizeof(rx_buf) - 1)
    {
      rx_buf[rx_len++] = ch;
    }
    else
    {
      rx_len = 0;   /* line too long - drop it */
    }
    HAL_UART_Receive_IT(huart, &rx_byte, 1);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
