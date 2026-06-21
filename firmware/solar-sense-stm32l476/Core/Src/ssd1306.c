/**
 ******************************************************************************
 * @file    ssd1306.c
 * @brief   128x64 SSD1306 OLED driver (I2C, software frame buffer). See header.
 ******************************************************************************
 */
#include "ssd1306.h"
#include <string.h>

/* Control byte sent as the first I2C data byte: bit6 (Co=0) means a stream
   follows; bit7 (D/C#) selects command (0x00) vs. data/GDDRAM (0x40). */
#define SSD1306_CTRL_CMD   0x00U
#define SSD1306_CTRL_DATA  0x40U

#define SSD1306_I2C_TIMEOUT 50U   /* ms; per-command probes/writes are tiny     */
/* A full 1024-byte frame at the bus's 100 kHz takes ~90 ms on the wire, so the
   bulk flush needs a much larger budget than the per-command timeout above. */
#define SSD1306_FLUSH_TIMEOUT 250U

/* 5x7 font, ASCII 0x20..0x7E. Each glyph is five column bytes (LSB = top row);
   the driver inserts a 1-px gap to fill the 6-px cell. Classic public-domain
   "font5x7" table as shipped with countless Arduino/Adafruit-style libraries. */
static const uint8_t FONT5X7[][5] = {
  {0x00,0x00,0x00,0x00,0x00}, /* 0x20 space */
  {0x00,0x00,0x5F,0x00,0x00}, /* ! */
  {0x00,0x07,0x00,0x07,0x00}, /* " */
  {0x14,0x7F,0x14,0x7F,0x14}, /* # */
  {0x24,0x2A,0x7F,0x2A,0x12}, /* $ */
  {0x23,0x13,0x08,0x64,0x62}, /* % */
  {0x36,0x49,0x55,0x22,0x50}, /* & */
  {0x00,0x05,0x03,0x00,0x00}, /* ' */
  {0x00,0x1C,0x22,0x41,0x00}, /* ( */
  {0x00,0x41,0x22,0x1C,0x00}, /* ) */
  {0x14,0x08,0x3E,0x08,0x14}, /* * */
  {0x08,0x08,0x3E,0x08,0x08}, /* + */
  {0x00,0x50,0x30,0x00,0x00}, /* , */
  {0x08,0x08,0x08,0x08,0x08}, /* - */
  {0x00,0x60,0x60,0x00,0x00}, /* . */
  {0x20,0x10,0x08,0x04,0x02}, /* / */
  {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
  {0x00,0x42,0x7F,0x40,0x00}, /* 1 */
  {0x42,0x61,0x51,0x49,0x46}, /* 2 */
  {0x21,0x41,0x45,0x4B,0x31}, /* 3 */
  {0x18,0x14,0x12,0x7F,0x10}, /* 4 */
  {0x27,0x45,0x45,0x45,0x39}, /* 5 */
  {0x3C,0x4A,0x49,0x49,0x30}, /* 6 */
  {0x01,0x71,0x09,0x05,0x03}, /* 7 */
  {0x36,0x49,0x49,0x49,0x36}, /* 8 */
  {0x06,0x49,0x49,0x29,0x1E}, /* 9 */
  {0x00,0x36,0x36,0x00,0x00}, /* : */
  {0x00,0x56,0x36,0x00,0x00}, /* ; */
  {0x08,0x14,0x22,0x41,0x00}, /* < */
  {0x14,0x14,0x14,0x14,0x14}, /* = */
  {0x00,0x41,0x22,0x14,0x08}, /* > */
  {0x02,0x01,0x51,0x09,0x06}, /* ? */
  {0x32,0x49,0x79,0x41,0x3E}, /* @ */
  {0x7E,0x11,0x11,0x11,0x7E}, /* A */
  {0x7F,0x49,0x49,0x49,0x36}, /* B */
  {0x3E,0x41,0x41,0x41,0x22}, /* C */
  {0x7F,0x41,0x41,0x22,0x1C}, /* D */
  {0x7F,0x49,0x49,0x49,0x41}, /* E */
  {0x7F,0x09,0x09,0x09,0x01}, /* F */
  {0x3E,0x41,0x49,0x49,0x7A}, /* G */
  {0x7F,0x08,0x08,0x08,0x7F}, /* H */
  {0x00,0x41,0x7F,0x41,0x00}, /* I */
  {0x20,0x40,0x41,0x3F,0x01}, /* J */
  {0x7F,0x08,0x14,0x22,0x41}, /* K */
  {0x7F,0x40,0x40,0x40,0x40}, /* L */
  {0x7F,0x02,0x0C,0x02,0x7F}, /* M */
  {0x7F,0x04,0x08,0x10,0x7F}, /* N */
  {0x3E,0x41,0x41,0x41,0x3E}, /* O */
  {0x7F,0x09,0x09,0x09,0x06}, /* P */
  {0x3E,0x41,0x51,0x21,0x5E}, /* Q */
  {0x7F,0x09,0x19,0x29,0x46}, /* R */
  {0x46,0x49,0x49,0x49,0x31}, /* S */
  {0x01,0x01,0x7F,0x01,0x01}, /* T */
  {0x3F,0x40,0x40,0x40,0x3F}, /* U */
  {0x1F,0x20,0x40,0x20,0x1F}, /* V */
  {0x3F,0x40,0x38,0x40,0x3F}, /* W */
  {0x63,0x14,0x08,0x14,0x63}, /* X */
  {0x07,0x08,0x70,0x08,0x07}, /* Y */
  {0x61,0x51,0x49,0x45,0x43}, /* Z */
  {0x00,0x7F,0x41,0x41,0x00}, /* [ */
  {0x02,0x04,0x08,0x10,0x20}, /* backslash */
  {0x00,0x41,0x41,0x7F,0x00}, /* ] */
  {0x04,0x02,0x01,0x02,0x04}, /* ^ */
  {0x40,0x40,0x40,0x40,0x40}, /* _ */
  {0x00,0x01,0x02,0x04,0x00}, /* ` */
  {0x20,0x54,0x54,0x54,0x78}, /* a */
  {0x7F,0x48,0x44,0x44,0x38}, /* b */
  {0x38,0x44,0x44,0x44,0x20}, /* c */
  {0x38,0x44,0x44,0x48,0x7F}, /* d */
  {0x38,0x54,0x54,0x54,0x18}, /* e */
  {0x08,0x7E,0x09,0x01,0x02}, /* f */
  {0x0C,0x52,0x52,0x52,0x3E}, /* g */
  {0x7F,0x08,0x04,0x04,0x78}, /* h */
  {0x00,0x44,0x7D,0x40,0x00}, /* i */
  {0x20,0x40,0x44,0x3D,0x00}, /* j */
  {0x7F,0x10,0x28,0x44,0x00}, /* k */
  {0x00,0x41,0x7F,0x40,0x00}, /* l */
  {0x7C,0x04,0x18,0x04,0x78}, /* m */
  {0x7C,0x08,0x04,0x04,0x78}, /* n */
  {0x38,0x44,0x44,0x44,0x38}, /* o */
  {0x7C,0x14,0x14,0x14,0x08}, /* p */
  {0x08,0x14,0x14,0x18,0x7C}, /* q */
  {0x7C,0x08,0x04,0x04,0x08}, /* r */
  {0x48,0x54,0x54,0x54,0x20}, /* s */
  {0x04,0x3F,0x44,0x40,0x20}, /* t */
  {0x3C,0x40,0x40,0x20,0x7C}, /* u */
  {0x1C,0x20,0x40,0x20,0x1C}, /* v */
  {0x3C,0x40,0x30,0x40,0x3C}, /* w */
  {0x44,0x28,0x10,0x28,0x44}, /* x */
  {0x0C,0x50,0x50,0x50,0x3C}, /* y */
  {0x44,0x64,0x54,0x4C,0x44}, /* z */
  {0x00,0x08,0x36,0x41,0x00}, /* { */
  {0x00,0x00,0x7F,0x00,0x00}, /* | */
  {0x00,0x41,0x36,0x08,0x00}, /* } */
  {0x08,0x04,0x08,0x10,0x08}, /* ~ */
};

/* --- low-level I2C helpers ------------------------------------------------- */

static HAL_StatusTypeDef ssd1306_cmd(SSD1306_t *dev, uint8_t cmd)
{
  uint8_t b[2] = { SSD1306_CTRL_CMD, cmd };
  return HAL_I2C_Master_Transmit(dev->hi2c, dev->addr8, b, 2, SSD1306_I2C_TIMEOUT);
}

/* The SSD1306 power-on init sequence for a 128x64 panel: charge pump on,
   horizontal addressing, full window, default contrast. */
static HAL_StatusTypeDef ssd1306_init_seq(SSD1306_t *dev)
{
  static const uint8_t seq[] = {
    0xAE,             /* display off                                          */
    0x20, 0x00,       /* memory addressing mode = horizontal                  */
    0xB0,             /* page start (overwritten each flush)                  */
    0xC8,             /* COM scan direction remapped (row 0 at top)           */
    0x00, 0x10,       /* low/high column start                                */
    0x40,             /* display start line 0                                 */
    0x81, 0x7F,       /* contrast                                             */
    0xA1,             /* segment remap (col 127 -> SEG0)                      */
    0xA6,             /* normal (non-inverted) display                        */
    0xA8, 0x3F,       /* multiplex ratio = 63 (64 rows)                       */
    0xA4,             /* output follows RAM                                   */
    0xD3, 0x00,       /* display offset 0                                     */
    0xD5, 0x80,       /* clock divide / osc freq                              */
    0xD9, 0x22,       /* pre-charge period                                    */
    0xDA, 0x12,       /* COM pins: alternate, no left/right remap             */
    0xDB, 0x20,       /* VCOMH deselect level                                 */
    0x8D, 0x14,       /* charge pump on                                       */
    0xAF              /* display on                                           */
  };
  for (size_t i = 0; i < sizeof(seq); i++)
  {
    if (ssd1306_cmd(dev, seq[i]) != HAL_OK)
      return HAL_ERROR;
  }
  return HAL_OK;
}

/* --- public API ------------------------------------------------------------ */

HAL_StatusTypeDef SSD1306_Init(SSD1306_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
  dev->hi2c     = hi2c;
  dev->present  = 0;
  dev->cursor_x = 0;
  dev->cursor_y = 0;
  memset(dev->buf, 0, sizeof(dev->buf));

  const uint8_t addrs[2] = { addr7, (addr7 == SSD1306_ADDR) ? SSD1306_ADDR_ALT
                                                            : SSD1306_ADDR };
  for (int i = 0; i < 2; i++)
  {
    dev->addr8 = (uint8_t)(addrs[i] << 1);
    if (HAL_I2C_IsDeviceReady(dev->hi2c, dev->addr8, 2, SSD1306_I2C_TIMEOUT) == HAL_OK)
    {
      if (ssd1306_init_seq(dev) == HAL_OK)
      {
        /* Panel acknowledged and accepted the init sequence: it's present.
           Don't gate this on the first frame flush -- that's a slow ~90 ms
           bulk write, and a one-off timeout there shouldn't read as "absent". */
        dev->present = 1;
        SSD1306_Clear(dev);
        SSD1306_Display(dev);
        return HAL_OK;
      }
    }
  }
  return HAL_ERROR;
}

HAL_StatusTypeDef SSD1306_Display(SSD1306_t *dev)
{
  if (!dev->present)
    return HAL_ERROR;

  /* Set the column/page window to the full panel, then stream all 1024 bytes. */
  static const uint8_t window[] = {
    0x21, 0x00, (uint8_t)(SSD1306_WIDTH - 1),  /* column address range */
    0x22, 0x00, (uint8_t)(SSD1306_PAGES - 1)   /* page address range   */
  };
  for (size_t i = 0; i < sizeof(window); i++)
  {
    if (ssd1306_cmd(dev, window[i]) != HAL_OK)
      return HAL_ERROR;
  }

  /* Stream the whole buffer in one transaction with the 0x40 data control byte
     as the "register address" -- no temporary copy, no large stack buffer. */
  return HAL_I2C_Mem_Write(dev->hi2c, dev->addr8, SSD1306_CTRL_DATA,
                           I2C_MEMADD_SIZE_8BIT, dev->buf, SSD1306_FB_SIZE,
                           SSD1306_FLUSH_TIMEOUT);
}

void SSD1306_Clear(SSD1306_t *dev)
{
  memset(dev->buf, 0, sizeof(dev->buf));
}

void SSD1306_DrawPixel(SSD1306_t *dev, int x, int y, SSD1306_Color color)
{
  if (!dev->present || x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT)
    return;
  uint8_t *cell = &dev->buf[x + (y / 8) * SSD1306_WIDTH];
  uint8_t  mask = (uint8_t)(1U << (y & 7));
  if (color == SSD1306_WHITE)
    *cell |= mask;
  else
    *cell &= (uint8_t)~mask;
}

void SSD1306_FillRect(SSD1306_t *dev, int x, int y, int w, int h, SSD1306_Color color)
{
  for (int yy = y; yy < y + h; yy++)
    for (int xx = x; xx < x + w; xx++)
      SSD1306_DrawPixel(dev, xx, yy, color);
}

void SSD1306_DrawHLine(SSD1306_t *dev, int x, int y, int w, SSD1306_Color color)
{
  for (int xx = x; xx < x + w; xx++)
    SSD1306_DrawPixel(dev, xx, y, color);
}

void SSD1306_DrawVLine(SSD1306_t *dev, int x, int y, int h, SSD1306_Color color)
{
  for (int yy = y; yy < y + h; yy++)
    SSD1306_DrawPixel(dev, x, yy, color);
}

void SSD1306_DrawRect(SSD1306_t *dev, int x, int y, int w, int h, SSD1306_Color color)
{
  SSD1306_DrawHLine(dev, x, y, w, color);
  SSD1306_DrawHLine(dev, x, y + h - 1, w, color);
  SSD1306_DrawVLine(dev, x, y, h, color);
  SSD1306_DrawVLine(dev, x + w - 1, y, h, color);
}

void SSD1306_SetCursor(SSD1306_t *dev, int x, int y)
{
  dev->cursor_x = (uint16_t)x;
  dev->cursor_y = (uint16_t)y;
}

void SSD1306_WriteChar(SSD1306_t *dev, char c, uint8_t scale, SSD1306_Color color)
{
  if (scale < 1) scale = 1;
  if ((unsigned char)c < 0x20 || (unsigned char)c > 0x7E)
    c = '?';
  const uint8_t *glyph = FONT5X7[(unsigned char)c - 0x20];

  /* 5 active columns + 1 spacing column; each column is 8 px tall (7 used). */
  for (int col = 0; col < 6; col++)
  {
    uint8_t bits = (col < 5) ? glyph[col] : 0x00;
    for (int row = 0; row < 8; row++)
    {
      if (bits & (1U << row))
      {
        if (scale == 1)
          SSD1306_DrawPixel(dev, dev->cursor_x + col, dev->cursor_y + row, color);
        else
          SSD1306_FillRect(dev, dev->cursor_x + col * scale,
                           dev->cursor_y + row * scale, scale, scale, color);
      }
    }
  }
  dev->cursor_x = (uint16_t)(dev->cursor_x + 6 * scale);
}

void SSD1306_WriteString(SSD1306_t *dev, const char *s, uint8_t scale, SSD1306_Color color)
{
  while (*s)
    SSD1306_WriteChar(dev, *s++, scale, color);
}

void SSD1306_WriteStringAt(SSD1306_t *dev, int x, int y, const char *s,
                           uint8_t scale, SSD1306_Color color)
{
  SSD1306_SetCursor(dev, x, y);
  SSD1306_WriteString(dev, s, scale, color);
}

int SSD1306_TextWidth(const char *s, uint8_t scale)
{
  if (scale < 1) scale = 1;
  return (int)strlen(s) * 6 * scale;
}
