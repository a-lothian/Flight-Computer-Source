/*
 * telemetry_can.c
 *
 * See telemetry_can.h.
 */

#include "telemetry_can.h"
#include <stdio.h>

/* longest a send may wait for mailboxes, ms (HAL_GetTick resolution 1 ms) */
#define TELEMETRY_CAN_TIMEOUT_MS   3u
#define ALL_TX_MAILBOXES           (CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2)

/* packets since the last telemetry_can_print_status() */
static uint32_t packets_queued;
static uint32_t packets_dropped;

HAL_StatusTypeDef telemetry_can_start(CAN_HandleTypeDef *hcan)
{
    /* bit time = (1 + BS1 + BS2) quanta of Prescaler / PCLK1 */
    uint32_t bs1 = ((hcan->Init.TimeSeg1 & CAN_BTR_TS1_Msk) >> CAN_BTR_TS1_Pos) + 1u;
    uint32_t bs2 = ((hcan->Init.TimeSeg2 & CAN_BTR_TS2_Msk) >> CAN_BTR_TS2_Pos) + 1u;
    uint32_t quanta = hcan->Init.Prescaler * (1u + bs1 + bs2);
    if (quanta == 0u || HAL_RCC_GetPCLK1Freq() != TELEMETRY_CAN_BITRATE * quanta) {
        return HAL_ERROR;
    }
    /* frames must leave in the order queued, not by ID */
    if (hcan->Init.TransmitFifoPriority != ENABLE || hcan->Init.Mode != CAN_MODE_NORMAL) {
        return HAL_ERROR;
    }
    return HAL_CAN_Start(hcan);
}

HAL_StatusTypeDef telemetry_can_send(CAN_HandleTypeDef *hcan, const void *packet, uint32_t length)
{
    const uint8_t *bytes = (const uint8_t *)packet;
    uint32_t frames = (length + 7u) / 8u;
    if (frames == 0u || TELEMETRY_CAN_BASE_ID + frames - 1u > 0x7FFu) {
        return HAL_ERROR;
    }

    /* the last packet hasn't left: nothing is ACKing, or the bus is off.
     * Drop it rather than wait, and let the next call try afresh */
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) < 3u) {
        (void)HAL_CAN_AbortTxRequest(hcan, ALL_TX_MAILBOXES);
        packets_dropped++;
        return HAL_BUSY;
    }

    CAN_TxHeaderTypeDef h = {0};
    h.IDE = CAN_ID_STD;
    h.RTR = CAN_RTR_DATA;
    h.TransmitGlobalTime = DISABLE;

    uint32_t start = HAL_GetTick();
    for (uint32_t i = 0; i < frames; i++) {
        uint32_t offset = i * 8u;
        uint32_t len = length - offset;
        if (len > 8u) {
            len = 8u;
        }
        h.StdId = TELEMETRY_CAN_BASE_ID + i;
        h.DLC = len;

        while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0u) {
            if (HAL_GetTick() - start > TELEMETRY_CAN_TIMEOUT_MS) {
                /* a partial packet is discarded by the receiver anyway */
                (void)HAL_CAN_AbortTxRequest(hcan, ALL_TX_MAILBOXES);
                packets_dropped++;
                return HAL_TIMEOUT;
            }
        }
        uint32_t mailbox;
        HAL_StatusTypeDef st = HAL_CAN_AddTxMessage(hcan, &h, &bytes[offset], &mailbox);
        if (st != HAL_OK) {
            (void)HAL_CAN_AbortTxRequest(hcan, ALL_TX_MAILBOXES);
            packets_dropped++;
            return st;
        }
    }
    packets_queued++;
    return HAL_OK;
}

bool telemetry_can_poll(CAN_HandleTypeDef *hcan)
{
    if (hcan->State == HAL_CAN_STATE_LISTENING) {
        return true;
    }
    /* HAL_CAN_Start gives up after 10 ms waiting for the controller to leave
     * init mode, but it has already asked it to, and the controller does so
     * by itself after 11 recessive bits. So once it has, finish what
     * HAL_CAN_Start would have done */
    if (hcan->State == HAL_CAN_STATE_ERROR && (hcan->ErrorCode & HAL_CAN_ERROR_TIMEOUT) != 0u &&
        (hcan->Instance->MCR & CAN_MCR_INRQ) == 0u && (hcan->Instance->MSR & CAN_MSR_INAK) == 0u) {
        hcan->ErrorCode = HAL_CAN_ERROR_NONE;
        hcan->State = HAL_CAN_STATE_LISTENING;
        return true;
    }
    return false;
}

void telemetry_can_print_status(CAN_HandleTypeDef *hcan)
{
    uint32_t queued = packets_queued;
    uint32_t dropped = packets_dropped;
    packets_queued = 0;
    packets_dropped = 0;

    /* printf's formatting would cost loop time with nobody listening */
    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) == 0u || (ITM->TER & 1u) == 0u) {
        return;
    }

    const char *state;
    if (hcan->State == HAL_CAN_STATE_LISTENING) {
        state = "on";
    } else if (hcan->State == HAL_CAN_STATE_ERROR && (hcan->ErrorCode & HAL_CAN_ERROR_TIMEOUT) != 0u) {
        state = "waiting for an idle bus";
    } else {
        state = "off (bad setup, no retry)";
    }
    uint32_t esr = hcan->Instance->ESR;
    printf("CAN %s: tec=%lu rec=%lu lec=%lu%s, packets queued %lu dropped %lu\r\n", state,
           (unsigned long)((esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos),
           (unsigned long)((esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos),
           (unsigned long)((esr & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos),
           (esr & CAN_ESR_BOFF) != 0u ? " bus-off" :
           (esr & CAN_ESR_EPVF) != 0u ? " error-passive" :
           (esr & CAN_ESR_EWGF) != 0u ? " error-warning" : "",
           (unsigned long)queued, (unsigned long)dropped);
}
