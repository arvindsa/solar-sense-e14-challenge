#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdint.h>

/*
 * SolarSense CAN protocol — shared between panel-side sender (F103) and
 * gateway receiver (QSTM / U585).
 *
 * Byte order: little-endian (LSB first). Both nodes are ARM Cortex —
 * no byte-swap needed on either end.
 *
 * Fixed-point scale factors:
 *   voltage  : u16, millivolts (mV)
 *   current  : i32, microamps  (µA)
 *   power    : i32, microwatts (µW)  — computed at gateway, not transmitted
 *   temp     : i16, °C × 10   (MAX31855)  or  °C × 100 (BME280, HX94C)
 *   humidity : u16, % × 100
 *   pressure : u32, Pa
 *   PM       : u16, µg/m³ × 10
 */

/* ── CAN IDs ──────────────────────────────────────────────────────────────── */

/* Fast — 5 Hz */
#define SS_ID_PANEL1       0x101U  /* Panel 1 — clean reference     DLC=6 */
#define SS_ID_PANEL2       0x102U  /* Panel 2 — soiling test        DLC=6 */
#define SS_ID_PANEL3       0x103U  /* Panel 3 — soiling test        DLC=6 */
#define SS_ID_UV           0x104U  /* GUVA-S12SD UV ADC counts      DLC=2 */

/* Medium — 1 Hz */
#define SS_ID_THERMO       0x201U  /* MAX31855 thermocouple (P1)    DLC=4 */
#define SS_ID_BATTERY      0x202U  /* MAX17048 fuel gauge           DLC=3 */

/* Slow — 0.2 Hz */
#define SS_ID_SEN66_PM     0x301U  /* SEN66 particulates            DLC=8 */
#define SS_ID_SEN66_GAS    0x302U  /* SEN66 CO2 + VOC + NOx         DLC=6 */
#define SS_ID_BME280       0x303U  /* BME280 pressure + board temp  DLC=6 */
#define SS_ID_HX94C_RAIN   0x304U  /* HX94C ambient + rain flag     DLC=5 */

/* Diagnostics — bidirectional */
#define SS_ID_TIME_SYNC    0x7E0U  /* gateway → panel, UTC ms       DLC=8 */
#define SS_ID_HEARTBEAT    0x7E1U  /* panel → gateway, seq+flags    DLC=7 */

/* ── Heartbeat flags byte ─────────────────────────────────────────────────── */
#define SS_FLAG_BACKFILL   (1U << 7)  /* frame is replayed from SD backlog */
#define SS_FLAG_SD_ERR     (1U << 6)  /* SD card write failed              */
#define SS_FLAG_CAN_ERR    (1U << 5)  /* CAN bus-error counter tripped     */
#define SS_FLAG_BAT_LOW    (1U << 4)  /* MAX17048 ALRT asserted            */

/* ── Decoded message structs ──────────────────────────────────────────────── */

typedef struct {
    uint16_t v_mv;  /* panel terminal voltage, mV                */
    int32_t  i_ua;  /* panel current, µA                         */
    int32_t  p_uw;  /* power = v_mv * i_ua / 1000 µW (gateway)  */
} SS_Panel_t;

typedef struct {
    uint16_t uv_counts; /* 12-bit ADC counts, 0–4095 */
} SS_UV_t;

typedef struct {
    int16_t surface_c10; /* thermocouple temp, °C × 10  (Panel 1 bonded) */
    int16_t cold_c10;    /* cold-junction temp, °C × 10 (board ambient)  */
} SS_Thermo_t;

typedef struct {
    uint16_t bat_mv; /* battery voltage, mV    */
    uint8_t  soc;    /* state of charge, 0–100 */
} SS_Battery_t;

typedef struct {
    uint16_t pm1_x10;  /* PM1.0,  µg/m³ × 10 */
    uint16_t pm25_x10; /* PM2.5,  µg/m³ × 10 */
    uint16_t pm4_x10;  /* PM4.0,  µg/m³ × 10 */
    uint16_t pm10_x10; /* PM10,   µg/m³ × 10 */
} SS_SEN66_PM_t;

typedef struct {
    uint16_t co2_ppm; /* CO2,       ppm   */
    uint16_t voc;     /* VOC index, 1–500 */
    uint16_t nox;     /* NOx index, 1–500 */
} SS_SEN66_Gas_t;

typedef struct {
    uint32_t press_pa;  /* atmospheric pressure, Pa           */
    int16_t  temp_c100; /* board-local temperature, °C × 100 */
} SS_BME280_t;

typedef struct {
    uint16_t rh_pct100;  /* ambient humidity,    % × 100  */
    int16_t  temp_c100;  /* ambient temperature, °C × 100 */
    uint8_t  rain;       /* 1 = wet, 0 = dry              */
} SS_HX94C_Rain_t;

typedef struct {
    uint64_t utc_ms; /* wall-clock UTC, milliseconds */
} SS_TimeSync_t;

typedef struct {
    uint32_t seq;   /* SD record sequence number  */
    uint8_t  flags; /* SS_FLAG_* bitmask           */
    uint16_t crc16; /* CRC-16 over logical record  */
} SS_Heartbeat_t;

#endif /* CAN_PROTOCOL_H */
