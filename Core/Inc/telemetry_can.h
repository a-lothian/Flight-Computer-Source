/*
 * telemetry_can.h
 *
 * Live telemetry to the Telemetry Board over CAN2 (PB12 RX, PB13 TX). The
 * Telemetry Board reassembles each packet and forwards it over LoRa to the
 * ground station.
 *
 * Bus: classic CAN 2.0A, 500 kbit/s, standard 11-bit IDs, data frames. Here
 * PCLK1 is 4 MHz, so a bit is 8 time quanta: prescaler 1, BS1 5, BS2 2,
 * SJW 1, sample point 75%.
 *
 * Framing (the Telemetry Board's receiver must match it): the packet's bytes
 * go out in order as consecutive frames with standard ID
 * TELEMETRY_CAN_BASE_ID + chunk index, 8 bytes each except the last, so
 * DLC = min(8, size - 8 * index), all queued back to back. For
 * TelemetryRecordMini (53 bytes) that is 7 frames, IDs 0x0AA-0x0B0, the last
 * one 5 bytes. Nothing else may be sent on those IDs.
 */

#ifndef INC_TELEMETRY_CAN_H_
#define INC_TELEMETRY_CAN_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

#define TELEMETRY_CAN_BASE_ID      0x0AAu
#define TELEMETRY_CAN_BITRATE      500000u

/* Check hcan is set up as above (CubeMX regeneration can silently reset it)
 * and start it. Returns HAL_ERROR if the setup is wrong, or HAL_CAN_Start's
 * error if the bus never goes idle (e.g. no transceiver). Telemetry only:
 * nothing in flight depends on it. */
HAL_StatusTypeDef telemetry_can_start(CAN_HandleTypeDef *hcan);

/* Queue one packet as consecutive frames, in order. Waits only while the 3
 * transmit mailboxes drain (about 1 ms at 500 kbit/s), at most
 * TELEMETRY_CAN_TIMEOUT_MS. If the previous packet is still stuck in the
 * mailboxes (no ACK, bus off), it is dropped and HAL_BUSY returned, so a
 * dead bus never stalls the loop for longer than that. */
HAL_StatusTypeDef telemetry_can_send(CAN_HandleTypeDef *hcan, const void *packet, uint32_t length);

/* True once CAN is running. Call it about once a second while it isn't: if
 * telemetry_can_start() timed out because the bus wasn't idle (RX held
 * dominant: transceiver unpowered or disconnected), the controller joins the
 * bus by itself once the bus goes idle, and this notices that without
 * waiting. A bad setup (HAL_ERROR before HAL_CAN_Start) never recovers. */
bool telemetry_can_poll(CAN_HandleTypeDef *hcan);

/* One status line: running or not, TEC, REC, the last error code (LEC) and
 * the packets queued / dropped since the previous line. Prints only while a
 * debugger is reading SWO, so it costs nothing in flight. A TEC climbing
 * with LEC 3 (no ACK) means the frames go out but nothing receives them;
 * LEC 5 (bit dominant error) means they don't reach the bus at all. */
void telemetry_can_print_status(CAN_HandleTypeDef *hcan);

#endif /* INC_TELEMETRY_CAN_H_ */
