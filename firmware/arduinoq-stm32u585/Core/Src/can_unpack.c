#include "can_unpack.h"

/* ── Little-endian read helpers ──────────────────────────────────────────── */

static inline uint16_t get_u16(const uint8_t *d)
{
    return (uint16_t)(d[0] | ((uint16_t)d[1] << 8));
}

static inline int16_t get_i16(const uint8_t *d)
{
    return (int16_t)get_u16(d);
}

static inline uint32_t get_u32(const uint8_t *d)
{
    return (uint32_t)(d[0] | ((uint32_t)d[1] << 8)
                            | ((uint32_t)d[2] << 16)
                            | ((uint32_t)d[3] << 24));
}

static inline int32_t get_i32(const uint8_t *d)
{
    return (int32_t)get_u32(d);
}

static inline uint64_t get_u64(const uint8_t *d)
{
    return (uint64_t)get_u32(d) | ((uint64_t)get_u32(d + 4) << 32);
}

/* ── Unpack functions ────────────────────────────────────────────────────── */

/* 0x101–0x103  Panel V + I  [v_mv u16][i_ua i32]  DLC=6 */
void can_unpack_panel(const uint8_t *in, SS_Panel_t *s)
{
    s->v_mv = get_u16(in);
    s->i_ua = get_i32(in + 2);
    s->p_uw = (int32_t)((int64_t)s->v_mv * s->i_ua / 1000);
}

/* 0x104  UV counts  [uv_counts u16]  DLC=2 */
void can_unpack_uv(const uint8_t *in, SS_UV_t *s)
{
    s->uv_counts = get_u16(in);
}

/* 0x201  Thermocouple  [surface i16][cold i16]  DLC=4 */
void can_unpack_thermo(const uint8_t *in, SS_Thermo_t *s)
{
    s->surface_c10 = get_i16(in);
    s->cold_c10    = get_i16(in + 2);
}

/* 0x202  Battery  [bat_mv u16][soc u8]  DLC=3 */
void can_unpack_battery(const uint8_t *in, SS_Battery_t *s)
{
    s->bat_mv = get_u16(in);
    s->soc    = in[2];
}

/* 0x301  SEN66 PM  [pm1 u16][pm2.5 u16][pm4 u16][pm10 u16]  DLC=8 */
void can_unpack_sen66_pm(const uint8_t *in, SS_SEN66_PM_t *s)
{
    s->pm1_x10  = get_u16(in);
    s->pm25_x10 = get_u16(in + 2);
    s->pm4_x10  = get_u16(in + 4);
    s->pm10_x10 = get_u16(in + 6);
}

/* 0x302  SEN66 gas  [co2 u16][voc u16][nox u16]  DLC=6 */
void can_unpack_sen66_gas(const uint8_t *in, SS_SEN66_Gas_t *s)
{
    s->co2_ppm = get_u16(in);
    s->voc     = get_u16(in + 2);
    s->nox     = get_u16(in + 4);
}

/* 0x303  BME280  [press u32][temp i16]  DLC=6 */
void can_unpack_bme280(const uint8_t *in, SS_BME280_t *s)
{
    s->press_pa  = get_u32(in);
    s->temp_c100 = get_i16(in + 4);
}

/* 0x304  HX94C + rain  [rh u16][temp i16][rain u8]  DLC=5 */
void can_unpack_hx94c_rain(const uint8_t *in, SS_HX94C_Rain_t *s)
{
    s->rh_pct100 = get_u16(in);
    s->temp_c100 = get_i16(in + 2);
    s->rain      = in[4];
}

/* 0x7E0  Time sync  [utc_ms u64]  DLC=8 */
void can_unpack_time_sync(const uint8_t *in, SS_TimeSync_t *s)
{
    s->utc_ms = get_u64(in);
}

/* 0x7E1  Heartbeat  [seq u32][flags u8][crc16 u16]  DLC=7 */
void can_unpack_heartbeat(const uint8_t *in, SS_Heartbeat_t *s)
{
    s->seq   = get_u32(in);
    s->flags = in[4];
    s->crc16 = get_u16(in + 5);
}
