/* First: the HAL module switches the #if below reads. */
#include "can_priv.h"

#if !defined(ESP_PLATFORM) && CONFIG_CAN_BXCAN

#include <stdlib.h>
#include <string.h>

#include "bxcan.h"
#include "can_log.h"

/*
 * Every name below is the HAL's (stm32f4xx_hal_can.h v1.8.5, the same on
 * F1/F2/F7/L4) or CMSIS's (stm32f405xx.h); nothing is a register value of
 * this library's own.
 *
 *   HAL_CAN_STATE_READY, _LISTENING     :48-53, initialised / started
 *   CAN_ID_STD, CAN_ID_EXT              :435-436
 *   CAN_RTR_DATA, CAN_RTR_REMOTE        :444-445
 *   CAN_RX_FIFO0                        :453
 *   CAN_IT_RX_FIFO0_MSG_PENDING         :521
 *   CAN_ESR_EWGF, _EPVF, _BOFF          stm32f405xx.h:1879-1886
 *   CAN_ESR_TEC, _REC (+ _Pos)          stm32f405xx.h:1895-1900
 *   CAN_RF0R_FOVR0                      stm32f405xx.h:1812-1813
 */
/* CAN_TX_MAILBOX0..2, stm32f4xx_hal_can.h:462-464. */
#define TX_MAILBOXES    3

#define TRY(expr) do { int _e = (expr); if (_e) return _e; } while (0)

struct bxcan_t {
    struct can_dev     can;     /* the category view; priv points back */
    CAN_HandleTypeDef *hcan;
    bool               use_irq;
    struct can_ring    ring;
};

/* ---- frames ------------------------------------------------------------- */

static bool fifo_take(bxcan_handle_t d, struct can_frame *frame)
{
    CAN_RxHeaderTypeDef h;
    uint8_t data[8];

    if (HAL_CAN_GetRxFifoFillLevel(d->hcan, CAN_RX_FIFO0) == 0 ||
        HAL_CAN_GetRxMessage(d->hcan, CAN_RX_FIFO0, &h, data) != HAL_OK)
        return false;
    memset(frame, 0, sizeof(*frame));
    frame->ext = h.IDE == CAN_ID_EXT;
    frame->id = frame->ext ? h.ExtId : h.StdId;
    frame->rtr = h.RTR == CAN_RTR_REMOTE;
    frame->dlc = h.DLC > CAN_FRAME_DATA_MAX ? CAN_FRAME_DATA_MAX :
                                              (uint8_t)h.DLC;
    if (!frame->rtr)
        memcpy(frame->data, data, frame->dlc);
    return true;
}

void bxcan_irq(bxcan_handle_t dev)
{
    struct can_frame frame;

    if (dev == NULL)
        return;
    while (fifo_take(dev, &frame)) {
        frame.t_us = can_time_us();
        (void)can_ring_put(&dev->ring, &frame);
    }
    can_irq(&dev->can);
}

/* ---- the can_dev ops ---------------------------------------------------- */

static bxcan_handle_t of(struct can_dev *can)
{
    return can->priv;
}

static int op_send(struct can_dev *can, const struct can_frame *frame,
                   uint32_t timeout_ms)
{
    bxcan_handle_t d = of(can);
    int64_t start = can_time_us();
    CAN_TxHeaderTypeDef h = {
        .StdId = frame->ext ? 0 : frame->id,
        .ExtId = frame->ext ? frame->id : 0,
        .IDE = frame->ext ? CAN_ID_EXT : CAN_ID_STD,
        .RTR = frame->rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA,
        .DLC = frame->dlc,
        .TransmitGlobalTime = DISABLE,
    };
    uint32_t mailbox;

    while (HAL_CAN_GetTxMailboxesFreeLevel(d->hcan) == 0) {
        if (can_elapsed_ms(start) >= timeout_ms)
            return -ETIMEDOUT;
        can_yield();
    }
    return can_hal_errno(HAL_CAN_AddTxMessage(d->hcan, &h, frame->data,
                                              &mailbox));
}

static int op_recv(struct can_dev *can, struct can_frame *frame,
                   uint32_t timeout_ms)
{
    bxcan_handle_t d = of(can);
    int64_t start = can_time_us();

    for (;;) {
        if (d->use_irq ? can_ring_get(&d->ring, frame) :
                         fifo_take(d, frame))
            return 0;
        if (can_elapsed_ms(start) >= timeout_ms)
            return -ETIMEDOUT;
        can_yield();
    }
}

static int op_get_status(struct can_dev *can, struct can_status *status)
{
    bxcan_handle_t d = of(can);
    uint32_t esr = d->hcan->Instance->ESR;

    if (HAL_CAN_GetState(d->hcan) != HAL_CAN_STATE_LISTENING)
        status->link = CAN_LINK_STOPPED;
    else if (esr & CAN_ESR_BOFF)
        status->link = CAN_LINK_BUS_OFF;
    else if (esr & CAN_ESR_EPVF)
        status->link = CAN_LINK_PASSIVE;
    else if (esr & CAN_ESR_EWGF)
        status->link = CAN_LINK_WARNING;
    else
        status->link = CAN_LINK_ACTIVE;
    status->tec = (uint16_t)((esr & CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos);
    status->rec = (uint16_t)((esr & CAN_ESR_REC_Msk) >> CAN_ESR_REC_Pos);
    /* The ring counts what it dropped; the FIFO has an overrun flag. */
    status->rx_lost = d->ring.dropped +
                      !!(d->hcan->Instance->RF0R & CAN_RF0R_FOVR0);
    status->rx_pending = d->use_irq ?
                         can_ring_count(&d->ring) :
                         HAL_CAN_GetRxFifoFillLevel(d->hcan, CAN_RX_FIFO0);
    status->tx_pending = TX_MAILBOXES -
                         HAL_CAN_GetTxMailboxesFreeLevel(d->hcan);
    return 0;
}

static int op_start(struct can_dev *can)
{
    return can_hal_errno(HAL_CAN_Start(of(can)->hcan));
}

static int op_stop(struct can_dev *can)
{
    return can_hal_errno(HAL_CAN_Stop(of(can)->hcan));
}

/* No set_filters, set_bitrate, set_opmode or recover: see bxcan.h. */
static const struct can_ops bxcan_ops = {
    .send = op_send,
    .recv = op_recv,
    .get_status = op_get_status,
    .start = op_start,
    .stop = op_stop,
};

/* ---- init --------------------------------------------------------------- */

static int init(const bxcan_config_t *cfg, bxcan_handle_t *out)
{
    HAL_CAN_StateTypeDef state;
    bxcan_handle_t d;
    int err = 0;

    if (cfg == NULL || out == NULL || cfg->hcan == NULL)
        return -EINVAL;
    state = HAL_CAN_GetState(cfg->hcan);
    if (state != HAL_CAN_STATE_READY && state != HAL_CAN_STATE_LISTENING)
        return -EINVAL;     /* not HAL_CAN_Init()ed */

    d = calloc(1, sizeof *d);
    if (d == NULL)
        return -ENOMEM;
    d->can.name = "bxcan";
    d->can.ops = &bxcan_ops;
    d->can.priv = d;
    d->hcan = cfg->hcan;
    d->use_irq = cfg->use_irq;
    can_ring_init(&d->ring);

    if (state == HAL_CAN_STATE_READY)
        err = can_hal_errno(HAL_CAN_Start(d->hcan));
    if (!err && d->use_irq)
        err = can_hal_errno(HAL_CAN_ActivateNotification(
            d->hcan, CAN_IT_RX_FIFO0_MSG_PENDING));
    if (err) {
        can_err(&d->can, "would not start: %d", err);
        free(d);
        return err;
    }
    *out = d;
    return 0;
}

int bxcan_init(const bxcan_config_t *cfg, bxcan_handle_t *out)
{
    return init(cfg, out);
}

int bxcan_del(bxcan_handle_t dev)
{
    int err = 0;

    if (dev == NULL)
        return -EINVAL;
    if (dev->use_irq)
        err = can_hal_errno(HAL_CAN_DeactivateNotification(
            dev->hcan, CAN_IT_RX_FIFO0_MSG_PENDING));
    if (HAL_CAN_GetState(dev->hcan) == HAL_CAN_STATE_LISTENING) {
        int stop = can_hal_errno(HAL_CAN_Stop(dev->hcan));

        err = err ? err : stop;
    }
    free(dev);
    return err;
}

struct can_dev *bxcan_can(bxcan_handle_t dev)
{
    return dev ? &dev->can : NULL;
}

#endif /* !ESP_PLATFORM && CONFIG_CAN_BXCAN */
