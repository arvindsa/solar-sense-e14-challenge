/* can_bridge.h — ArduinoQ STM32 CAN → LPUSART bridge
 *
 * Receives SolarSense telemetry frames (CAN IDs 0x100–0x109 at 500 kbps),
 * reassembles them, and forwards a TLM text line over LPUSART to the
 * ArduinoQ Linux side (/dev/ttyHS1 at 115200 8N1).
 *
 * Integration steps for your ArduinoQ firmware:
 *   1. Call CAN_Bridge_Init() once after CAN and LPUSART peripherals are ready.
 *   2. Call CAN_Bridge_Poll() from your main loop (or from the CAN Rx FIFO0
 *      message-pending callback).
 *   3. Implement the two weak-linked stubs at the bottom of can_bridge.c
 *      (CAN_Bridge_ReceiveFrame / CAN_Bridge_TransmitLine) to hook your HAL
 *      handles.
 *
 * TODO: adapt peripheral handles and IRQ names to your ArduinoQ STM32 variant.
 */
#ifndef CAN_BRIDGE_H
#define CAN_BRIDGE_H

#include <stdint.h>

/* Call once after CAN and LPUSART init. */
void CAN_Bridge_Init(void);

/* Call periodically (from main loop or CAN Rx callback) to drain the
   CAN FIFO and emit a TLM line when a complete frame set is received. */
void CAN_Bridge_Poll(void);

#endif /* CAN_BRIDGE_H */
