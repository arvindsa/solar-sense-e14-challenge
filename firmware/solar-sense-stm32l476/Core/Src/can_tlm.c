#include "can_tlm.h"

/* Big-endian pack helpers */
#define PUT16(b, i, v) \
    do { (b)[i] = (uint8_t)((uint16_t)(v) >> 8); \
         (b)[(i)+1] = (uint8_t)(v); } while (0)
#define PUT32(b, i, v) \
    do { (b)[i]   = (uint8_t)((uint32_t)(v) >> 24); \
         (b)[(i)+1] = (uint8_t)((uint32_t)(v) >> 16); \
         (b)[(i)+2] = (uint8_t)((uint32_t)(v) >>  8); \
         (b)[(i)+3] = (uint8_t)(v); } while (0)

/* Fault bits that mean the bus is down or silent — no point queuing. */
#define CAN_FAULT_MASK  (HAL_CAN_ERROR_BOF | HAL_CAN_ERROR_EPV | \
                         HAL_CAN_ERROR_EWG | HAL_CAN_ERROR_ACK)

static HAL_StatusTypeDef send_frame(CAN_HandleTypeDef *hcan,
                                    uint32_t id,
                                    uint8_t *data)
{
    /* Bail immediately on any bus fault so we never block the main loop. */
    if (HAL_CAN_GetError(hcan) & CAN_FAULT_MASK) return HAL_ERROR;

    /* Bail if no TX mailbox is free right now (no waiting). With
       AutoRetransmission=ENABLE, unACKed frames hold their mailbox until
       the bus goes Bus-Off; all three can fill in <3 TLM frames if the
       bus isn't wired.  Drop-and-move-on is correct here. */
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) return HAL_BUSY;

    CAN_TxHeaderTypeDef hdr = {
        .StdId              = id,
        .ExtId              = 0,
        .IDE                = CAN_ID_STD,
        .RTR                = CAN_RTR_DATA,
        .DLC                = 8,
        .TransmitGlobalTime = DISABLE,
    };
    uint32_t mailbox;
    return HAL_CAN_AddTxMessage(hcan, &hdr, data, &mailbox);
}

HAL_StatusTypeDef CAN_TLM_Start(CAN_HandleTypeDef *hcan)
{
    /* Pass-all filter — SolarSense only transmits, so any Rx is incidental. */
    CAN_FilterTypeDef f = {
        .FilterIdHigh         = 0x0000U,
        .FilterIdLow          = 0x0000U,
        .FilterMaskIdHigh     = 0x0000U,
        .FilterMaskIdLow      = 0x0000U,
        .FilterFIFOAssignment = CAN_RX_FIFO0,
        .FilterBank           = 0,
        .FilterMode           = CAN_FILTERMODE_IDMASK,
        .FilterScale          = CAN_FILTERSCALE_32BIT,
        .FilterActivation     = ENABLE,
        .SlaveStartFilterBank = 14,
    };
    HAL_StatusTypeDef st = HAL_CAN_ConfigFilter(hcan, &f);
    if (st != HAL_OK) return st;
    return HAL_CAN_Start(hcan);
}

/* CAN_TLM_Send is fire-and-forget: individual frame drops (bus fault /
   no mailbox) are silently skipped so the main loop never stalls.
   Returns HAL_OK always — CAN is optional; UART is the primary stream. */
HAL_StatusTypeDef CAN_TLM_Send(CAN_HandleTypeDef *hcan,
                                const SolarSenseTLM_t *t)
{
    uint8_t d[8];

    /* 0x100 — system / power */
    PUT16(d, 0, t->bat_mv);
    PUT16(d, 2, t->bat_soc);
    d[4] = t->chg;
    d[5] = (t->r12v & 0x01U) | (uint8_t)((t->sd & 0x01U) << 1);
    PUT16(d, 6, t->uv_mv);
    send_frame(hcan, CAN_TLM_ID_SYS, d);

    /* 0x101–0x103 — solar panels */
    for (int i = 0; i < 3; i++) {
        d[0] = t->p_ok[i];
        d[1] = 0U;
        PUT16(d, 2, (uint16_t)t->p_mv[i]);
        PUT16(d, 4, (uint16_t)t->p_ma[i]);
        PUT16(d, 6, (uint16_t)t->p_mw[i]);
        send_frame(hcan, CAN_TLM_ID_P1 + (uint32_t)i, d);
    }

    /* 0x104 — thermocouple */
    d[0] = t->tc3_ok;
    d[1] = t->tc3_fault;
    PUT16(d, 2, (uint16_t)t->tc3_c);
    PUT16(d, 4, (uint16_t)t->tc3_cj);
    d[6] = 0U; d[7] = 0U;
    send_frame(hcan, CAN_TLM_ID_TC, d);

    /* 0x105 — BMP280 */
    d[0] = t->bmp_ok;
    d[1] = 0U;
    PUT16(d, 2, (uint16_t)t->bmp_c);
    PUT32(d, 4, t->bmp_pa);
    send_frame(hcan, CAN_TLM_ID_BMP, d);

    /* 0x106 — HX94C */
    d[0] = t->hx_ok;
    d[1] = t->hx_flt;
    PUT16(d, 2, (uint16_t)t->hx_rh);
    PUT16(d, 4, (uint16_t)t->hx_t);
    d[6] = t->hx_rhi;
    d[7] = t->hx_ti;
    send_frame(hcan, CAN_TLM_ID_HX, d);

    /* 0x107 — air quality PM */
    d[0] = t->aq_ok;
    d[1] = 0U;
    PUT16(d, 2, (uint16_t)t->aq_pm10);
    PUT16(d, 4, (uint16_t)t->aq_pm25);
    PUT16(d, 6, (uint16_t)t->aq_pm40);
    send_frame(hcan, CAN_TLM_ID_AQPM, d);

    /* 0x108 — air quality gas 1 */
    PUT16(d, 0, (uint16_t)t->aq_pm100);
    PUT16(d, 2, (uint16_t)t->aq_rh);
    PUT16(d, 4, (uint16_t)t->aq_t);
    PUT16(d, 6, (uint16_t)t->aq_voc);
    send_frame(hcan, CAN_TLM_ID_AQG1, d);

    /* 0x109 — air quality gas 2 */
    PUT16(d, 0, (uint16_t)t->aq_nox);
    PUT16(d, 2, (uint16_t)t->aq_co2);
    d[4] = 0U; d[5] = 0U; d[6] = 0U; d[7] = 0U;
    send_frame(hcan, CAN_TLM_ID_AQG2, d);

    return HAL_OK;
}
