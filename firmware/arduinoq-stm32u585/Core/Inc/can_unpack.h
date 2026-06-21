#ifndef CAN_UNPACK_H
#define CAN_UNPACK_H

#include "can_protocol.h"

/*
 * Unpack functions: deserialise a received CAN data buffer into a typed struct.
 * Byte order: little-endian (LSB first), matching can_pack on the sender side.
 * SS_Panel_t.p_uw is computed here (not transmitted).
 */

void can_unpack_panel      (const uint8_t *in, SS_Panel_t      *s);
void can_unpack_uv         (const uint8_t *in, SS_UV_t          *s);
void can_unpack_thermo     (const uint8_t *in, SS_Thermo_t      *s);
void can_unpack_battery    (const uint8_t *in, SS_Battery_t     *s);
void can_unpack_sen66_pm   (const uint8_t *in, SS_SEN66_PM_t    *s);
void can_unpack_sen66_gas  (const uint8_t *in, SS_SEN66_Gas_t   *s);
void can_unpack_bme280     (const uint8_t *in, SS_BME280_t      *s);
void can_unpack_hx94c_rain (const uint8_t *in, SS_HX94C_Rain_t  *s);
void can_unpack_time_sync  (const uint8_t *in, SS_TimeSync_t    *s);
void can_unpack_heartbeat  (const uint8_t *in, SS_Heartbeat_t   *s);

#endif /* CAN_UNPACK_H */
