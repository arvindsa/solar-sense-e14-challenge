/* can_bridge.c — ArduinoQ STM32 CAN → LPUSART bridge
 *
 * Receives SolarSense CAN frames (0x100–0x109), reassembles them into a
 * TLM key=value text line (same format as the SolarSense USART2 output),
 * and sends it over LPUSART to the Linux side.
 *
 * The Python serial_bridge.py on Linux parses this line identically to
 * the original TLM stream.
 */

#include "can_bridge.h"
#include <stdio.h>
#include <string.h>

/* -------------------------------------------------------------------------
 * TODO: replace these stubs with your ArduinoQ HAL peripheral handles.
 * -------------------------------------------------------------------------
 * Example for STM32H7 ArduinoQ with CAN1 and LPUART1:
 *
 *   extern CAN_HandleTypeDef  hcan1;
 *   extern UART_HandleTypeDef hlpuart1;
 *
 * Then implement the two functions at the bottom of this file.
 */

/* Big-endian decode helpers */
#define GET_U16(b, i)  ((uint16_t)(((uint16_t)(b)[i] << 8) | (b)[(i)+1]))
#define GET_I16(b, i)  ((int16_t)GET_U16(b, i))
#define GET_U32(b, i)  (((uint32_t)(b)[i] << 24) | ((uint32_t)(b)[(i)+1] << 16) \
                       | ((uint32_t)(b)[(i)+2] << 8) | (b)[(i)+3])

/* Shadow of all received TLM fields */
typedef struct {
    /* system */
    uint16_t bat_mv, bat_soc, uv_mv;
    uint8_t  chg, r12v, sd;
    /* panels */
    uint8_t  p_ok[3];
    int16_t  p_mv[3], p_ma[3], p_mw[3];
    /* thermocouple */
    uint8_t  tc3_ok, tc3_fault;
    int16_t  tc3_c, tc3_cj;
    /* BMP280 */
    uint8_t  bmp_ok;
    int16_t  bmp_c;
    uint32_t bmp_pa;
    /* HX94C */
    uint8_t  hx_ok, hx_flt, hx_rhi, hx_ti;
    int16_t  hx_rh, hx_t;
    /* SEN66 */
    uint8_t  aq_ok;
    int16_t  aq_pm10, aq_pm25, aq_pm40, aq_pm100;
    int16_t  aq_rh, aq_t, aq_voc, aq_nox, aq_co2;
} BridgeState_t;

static BridgeState_t s_state;

/* Bitmask: bit N = frame (0x100 + N) received in current burst */
#define FRAME_ALL_MASK 0x03FFU  /* bits 0–9 */
static uint16_t s_recv_mask;
static uint32_t s_burst_start_ms;  /* tick when first frame of burst arrived */
#define BURST_TIMEOUT_MS 500U       /* declare complete if no new frame for 500 ms */

static uint32_t s_seq;             /* sequence number for TLM hb field */

/* Forward declarations for the HAL hooks below */
static int  CAN_Bridge_ReceiveFrame(uint32_t *id, uint8_t *data8);
static void CAN_Bridge_TransmitLine(const char *line, uint16_t len);

/* -------------------------------------------------------------------------
 * Decode one received CAN frame into s_state
 * -------------------------------------------------------------------------*/
static void decode_frame(uint32_t id, const uint8_t *d)
{
    uint16_t bit = (uint16_t)(1U << (id - 0x100U));

    switch (id) {
    case 0x100:  /* SYS */
        s_state.bat_mv  = GET_U16(d, 0);
        s_state.bat_soc = GET_U16(d, 2);
        s_state.chg     = d[4];
        s_state.r12v    = d[5] & 0x01U;
        s_state.sd      = (d[5] >> 1) & 0x01U;
        s_state.uv_mv   = GET_U16(d, 6);
        break;
    case 0x101: case 0x102: case 0x103: {  /* P1–P3 */
        int i = (int)(id - 0x101U);
        s_state.p_ok[i] = d[0];
        s_state.p_mv[i] = GET_I16(d, 2);
        s_state.p_ma[i] = GET_I16(d, 4);
        s_state.p_mw[i] = GET_I16(d, 6);
        break;
    }
    case 0x104:  /* TC */
        s_state.tc3_ok    = d[0];
        s_state.tc3_fault = d[1];
        s_state.tc3_c     = GET_I16(d, 2);
        s_state.tc3_cj    = GET_I16(d, 4);
        break;
    case 0x105:  /* BMP */
        s_state.bmp_ok = d[0];
        s_state.bmp_c  = GET_I16(d, 2);
        s_state.bmp_pa = GET_U32(d, 4);
        break;
    case 0x106:  /* HX */
        s_state.hx_ok  = d[0];
        s_state.hx_flt = d[1];
        s_state.hx_rh  = GET_I16(d, 2);
        s_state.hx_t   = GET_I16(d, 4);
        s_state.hx_rhi = d[6];
        s_state.hx_ti  = d[7];
        break;
    case 0x107:  /* AQ-PM */
        s_state.aq_ok    = d[0];
        s_state.aq_pm10  = GET_I16(d, 2);
        s_state.aq_pm25  = GET_I16(d, 4);
        s_state.aq_pm40  = GET_I16(d, 6);
        break;
    case 0x108:  /* AQ-Gas1 */
        s_state.aq_pm100 = GET_I16(d, 0);
        s_state.aq_rh    = GET_I16(d, 2);
        s_state.aq_t     = GET_I16(d, 4);
        s_state.aq_voc   = GET_I16(d, 6);
        break;
    case 0x109:  /* AQ-Gas2 */
        s_state.aq_nox = GET_I16(d, 0);
        s_state.aq_co2 = GET_I16(d, 2);
        break;
    default:
        return;  /* unknown ID — do not set bit */
    }

    if (s_recv_mask == 0U)
        s_burst_start_ms = 0U;  /* first frame of new burst: reset timer below */
    s_recv_mask |= bit;
}

/* -------------------------------------------------------------------------
 * Assemble and transmit TLM line (matches SolarSense USART2 format)
 * -------------------------------------------------------------------------*/
static void emit_tlm(void)
{
    const BridgeState_t *b = &s_state;
    char line[700];
    int o = 0;

    o += snprintf(line + o, sizeof(line) - o,
                  "TLM up=0 hb=%lu sd=%d bat_mv=%u bat_soc=%u chg=%d r12v=%d rtc_bv=0",
                  (unsigned long)(++s_seq),
                  b->sd, b->bat_mv, b->bat_soc, b->chg, b->r12v);

    for (int i = 0; i < 3 && o < (int)sizeof(line) - 60; i++)
        o += snprintf(line + o, sizeof(line) - o,
                      " p%d_ok=%d p%d_mv=%d p%d_ma=%d p%d_mw=%d",
                      i+1, b->p_ok[i], i+1, (int)b->p_mv[i],
                      i+1, (int)b->p_ma[i], i+1, (int)b->p_mw[i]);

    if (b->tc3_ok && o < (int)sizeof(line) - 48)
        o += snprintf(line + o, sizeof(line) - o,
                      " tc3_ok=%d tc3_c=%d tc3_cj=%d tc3_fault=0x%02X",
                      b->tc3_ok, (int)b->tc3_c, (int)b->tc3_cj, b->tc3_fault);

    if (b->bmp_ok && o < (int)sizeof(line) - 40)
        o += snprintf(line + o, sizeof(line) - o,
                      " bmp_ok=%d bmp_pa=%lu bmp_c=%d",
                      b->bmp_ok, (unsigned long)b->bmp_pa, (int)b->bmp_c);

    if (o < (int)sizeof(line) - 72)
        o += snprintf(line + o, sizeof(line) - o,
                      " hx_ok=%d hx_rh=%d hx_t=%d hx_rhi=%d hx_ti=%d hx_flt=0x%02X",
                      b->hx_ok, (int)b->hx_rh, (int)b->hx_t,
                      b->hx_rhi, b->hx_ti, b->hx_flt);

    if (b->aq_ok && o < (int)sizeof(line) - 128)
        o += snprintf(line + o, sizeof(line) - o,
                      " aq_ok=%d aq_pm10=%d aq_pm25=%d aq_pm40=%d aq_pm100=%d"
                      " aq_rh=%d aq_t=%d aq_voc=%d aq_nox=%d aq_co2=%d",
                      b->aq_ok, (int)b->aq_pm10, (int)b->aq_pm25,
                      (int)b->aq_pm40, (int)b->aq_pm100,
                      (int)b->aq_rh, (int)b->aq_t,
                      (int)b->aq_voc, (int)b->aq_nox, (int)b->aq_co2);

    if (o < (int)sizeof(line) - 16)
        o += snprintf(line + o, sizeof(line) - o, " uv_mv=%u", b->uv_mv);

    o += snprintf(line + o, sizeof(line) - o, "\r\n");
    CAN_Bridge_TransmitLine(line, (uint16_t)o);
}

/* -------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/
void CAN_Bridge_Init(void)
{
    memset(&s_state, 0, sizeof(s_state));
    s_recv_mask      = 0U;
    s_burst_start_ms = 0U;
    s_seq            = 0U;
}

void CAN_Bridge_Poll(void)
{
    uint32_t id;
    uint8_t  data[8];

    /* Drain all pending CAN frames */
    while (CAN_Bridge_ReceiveFrame(&id, data) == 1) {
        if (id >= 0x100U && id <= 0x109U) {
            if (s_burst_start_ms == 0U)
                s_burst_start_ms = HAL_GetTick();  /* TODO: replace with your tick API */
            decode_frame(id, data);
        }
    }

    /* Emit on complete set or on burst timeout */
    if (s_recv_mask == FRAME_ALL_MASK ||
        (s_recv_mask != 0U &&
         (HAL_GetTick() - s_burst_start_ms) >= BURST_TIMEOUT_MS)) {  /* TODO: tick API */
        emit_tlm();
        s_recv_mask      = 0U;
        s_burst_start_ms = 0U;
    }
}

/* =========================================================================
 * HAL hooks — implement these for your ArduinoQ STM32 variant
 * =========================================================================
 *
 * CAN_Bridge_ReceiveFrame():
 *   Read one frame from the CAN RX FIFO.
 *   Return 1 if a frame was available and *id/*data8 were filled,
 *   return 0 if the FIFO was empty.
 *
 * CAN_Bridge_TransmitLine():
 *   Send `len` bytes of `line` over LPUSART (blocking or DMA).
 * =========================================================================*/

static int CAN_Bridge_ReceiveFrame(uint32_t *id, uint8_t *data8)
{
    /* TODO: adapt to your CAN handle and FIFO.
     *
     * Example with STM32 HAL CAN:
     *
     *   extern CAN_HandleTypeDef hcan1;
     *   if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0) return 0;
     *   CAN_RxHeaderTypeDef hdr;
     *   uint8_t buf[8];
     *   if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &hdr, buf) != HAL_OK) return 0;
     *   *id = hdr.StdId;
     *   memcpy(data8, buf, 8);
     *   return 1;
     */
    (void)id; (void)data8;
    return 0;
}

static void CAN_Bridge_TransmitLine(const char *line, uint16_t len)
{
    /* TODO: adapt to your LPUSART handle.
     *
     * Example with STM32 HAL UART:
     *
     *   extern UART_HandleTypeDef hlpuart1;
     *   HAL_UART_Transmit(&hlpuart1, (uint8_t *)line, len, HAL_MAX_DELAY);
     */
    (void)line; (void)len;
}
