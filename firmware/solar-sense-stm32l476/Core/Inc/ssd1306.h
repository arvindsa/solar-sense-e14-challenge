/**
 ******************************************************************************
 * @file    ssd1306.h
 * @brief   Minimal driver for a 128x64 SSD1306 OLED on I2C1.
 *
 * SolarSense carries an optional 0.96" 128x64 OLED on the shared I2C1 bus
 * (PB8/PB9, same bus as the MAX17048 / INA219 / BMP280). The controller is an
 * SSD1306 in I2C mode; almost every breakout strapped this way answers at the
 * 7-bit address 0x3C (0x3D if SA0 is tied high), so SSD1306_Init() probes both.
 *
 * The driver keeps a full 1 KB frame buffer in RAM and pushes it to the panel
 * with SSD1306_Display(). Rendering is software: a 5x7 glyph font (drawn in a
 * 6x8 cell) with an integer scale factor, plus rectangle/line primitives for
 * UI chrome (header bars, separators). No external dependencies beyond the HAL
 * I2C API. Everything is non-fatal: a missing panel just leaves present=0 and
 * all draw calls become no-ops.
 ******************************************************************************
 */
#ifndef SSD1306_H
#define SSD1306_H

#include "stm32l4xx_hal.h"

/* 7-bit I2C address. 0x3C with SA0 low (the common breakout strap), 0x3D high. */
#define SSD1306_ADDR       0x3CU
#define SSD1306_ADDR_ALT   0x3DU

#define SSD1306_WIDTH      128
#define SSD1306_HEIGHT     64
#define SSD1306_PAGES      (SSD1306_HEIGHT / 8)          /* 8 pages of 8 rows  */
#define SSD1306_FB_SIZE    (SSD1306_WIDTH * SSD1306_PAGES)

/* Pixel colour for the draw primitives. */
typedef enum {
  SSD1306_BLACK = 0,   /* pixel off                                            */
  SSD1306_WHITE = 1    /* pixel on (lit)                                        */
} SSD1306_Color;

typedef struct {
  I2C_HandleTypeDef *hi2c;
  uint8_t  addr8;                    /* 8-bit (HAL-style) address actually used */
  uint8_t  present;                  /* 1 once SSD1306_Init() found the panel   */
  uint16_t cursor_x;                 /* text cursor, in pixels                  */
  uint16_t cursor_y;
  uint8_t  buf[SSD1306_FB_SIZE];     /* 128x64 1bpp frame buffer (page-major)   */
} SSD1306_t;

/* Probe the panel at addr7 (then the alternate), run the power-on init
   sequence and clear the screen. Returns HAL_OK with present=1 on success, or
   HAL_ERROR with present=0 if neither address ACKs (all later calls no-op). */
HAL_StatusTypeDef SSD1306_Init(SSD1306_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7);

/* Push the frame buffer to the panel (the only call that touches the bus after
   init). Cheap to call once per rendered frame. */
HAL_StatusTypeDef SSD1306_Display(SSD1306_t *dev);

/* Clear the frame buffer to all-black (does not touch the panel until the next
   SSD1306_Display()). */
void SSD1306_Clear(SSD1306_t *dev);

/* --- drawing primitives (operate on the frame buffer only) ----------------- */
void SSD1306_DrawPixel(SSD1306_t *dev, int x, int y, SSD1306_Color color);
void SSD1306_FillRect(SSD1306_t *dev, int x, int y, int w, int h, SSD1306_Color color);
void SSD1306_DrawHLine(SSD1306_t *dev, int x, int y, int w, SSD1306_Color color);
void SSD1306_DrawVLine(SSD1306_t *dev, int x, int y, int h, SSD1306_Color color);
/* Outline (unfilled) rectangle. */
void SSD1306_DrawRect(SSD1306_t *dev, int x, int y, int w, int h, SSD1306_Color color);

/* --- text ------------------------------------------------------------------ */
/* Move the text cursor (top-left of the next glyph), in pixels. */
void SSD1306_SetCursor(SSD1306_t *dev, int x, int y);

/* Draw one glyph at the cursor and advance it. The 5x7 font is rendered in a
   6x8 cell scaled by `scale` (1 = 6x8 px, 2 = 12x16 px ...). `color` is the lit
   colour; the cell background is left untouched so text composes over chrome.
   Use SSD1306_BLACK to "erase" (e.g. inverted header text drawn over a white
   bar). */
void SSD1306_WriteChar(SSD1306_t *dev, char c, uint8_t scale, SSD1306_Color color);
void SSD1306_WriteString(SSD1306_t *dev, const char *s, uint8_t scale, SSD1306_Color color);

/* Convenience: set the cursor then draw a string. */
void SSD1306_WriteStringAt(SSD1306_t *dev, int x, int y, const char *s,
                           uint8_t scale, SSD1306_Color color);

/* Pixel width a string would occupy at the given scale (6 px per char cell),
   handy for right-aligning values. */
int SSD1306_TextWidth(const char *s, uint8_t scale);

#endif /* SSD1306_H */
