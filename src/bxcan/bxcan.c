/* First: the HAL module switches the #if below reads. */
#include "can_priv.h"

#if !defined(ESP_PLATFORM) && CONFIG_CAN_BXCAN

#include <stdlib.h>
#include <string.h>

#include "bxcan.h"
#include "can_log.h"
#include "fdcan/fdcan_math.h"

/*
 * Every name below is the HAL's (stm32f4xx_hal_can.h v1.8.5, the same on
 * F1/F2/F7/L4) or CMSIS's (stm32f405xx.h); the bit layouts they stand for
 * are RM0090 Rev 3's, chapter 24 (docs/rm0090.pdf, gitignored).
 *
 *   HAL_CAN_STATE_READY, _LISTENING     :48-53, initialised / started
 *   CAN_MODE_NORMAL, _SILENT, _SILENT_LOOPBACK   :337-340; CAN_BTR's SILM
 *                                       and LBKM (RM0090 24.9.2)
 *   CAN_ID_STD, CAN_ID_EXT              :435-436
 *   CAN_RTR_DATA, CAN_RTR_REMOTE        :444-445
 *   CAN_RX_FIFO0                        :453
 *   CAN_IT_RX_FIFO0_MSG_PENDING         :521
 *   CAN_MCR_INRQ, _ABOM; CAN_MSR_INAK   stm32f405xx.h:1671, :1689, :1702
 *                                       (RM0090 24.9.2)
 *   CAN_ESR_EWGF, _EPVF, _BOFF          stm32f405xx.h:1879-1886
 *   CAN_ESR_TEC, _REC (+ _Pos)          stm32f405xx.h:1895-1900
 *   CAN_BTR_TS1_Pos, _TS2_Pos, _SJW_Pos stm32f405xx.h:1906-1919
 *   CAN_RF0R_FOVR0                      stm32f405xx.h:1812-1813
 *   CAN_FMR_FINIT, _CAN2SB (+ _Pos)     stm32f405xx.h:2211-2214
 *                                       (RM0090 24.9.4)
 */
/* CAN_TX_MAILBOX0..2, stm32f4xx_hal_can.h:462-464. */
#define TX_MAILBOXES    3
#define ALL_MAILBOXES   (CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2)

/*
 * CAN_BTR's fields (RM0090 24.9.2), each holding its value less one: BRP
 * 10 bits, TS1 4, TS2 3, SJW 2. RM0090 24.7.7 gives the same ranges: BS1
 * 1..16 quanta, BS2 1..8, SJW 1..4. The search is the FDCAN's, at the
 * FDCAN's nominal sample point, 87.5%.
 */
static const struct fdcan_timing_limits bxcan_limits = {
    .prescaler_max = 1024,
    .seg1_min = 1, .seg1_max = 16,
    .seg2_min = 1, .seg2_max = 8,
    .sjw_max = 4,
};

/* RM0090 24.2: "Bit rates up to 1 Mbit/s". */
#define BITRATE_MAX     1000000

/*
 * The filter banks: 28, shared by CAN1 and CAN2 and held in CAN1's
 * registers; CAN1 has those below CAN_FMR.CAN2SB, CAN2 the rest (RM0090
 * 24.3.4, 24.9.4). A part with one CAN, or CAN3, has its own 14 (the
 * HAL's IS_CAN_FILTER_BANK_SINGLE, stm32f4xx_hal_can.h:809).
 */
#define BANKS_DUAL      28
#define BANKS_SINGLE    14

/* Filter bank register bits, 32-bit scale (RM0090 Figure 231):
 * STID[10:0] at 31:21, EXID[17:0] and STID together at 31:3, IDE 2. */
#define BANK_STD_ID_POS 21
#define BANK_EXT_ID_POS 3
#define BANK_IDE        (1u << 2)

/* How long entering initialisation mode may take: the HAL's own
 * CAN_TIMEOUT_VALUE, stm32f4xx_hal_can.c:237. */
#define INIT_TIMEOUT_MS 10

#define TRY(expr) do { int _e = (expr); if (_e) return _e; } while (0)

struct bxcan_t {
    struct can_dev     can;     /* the category view; priv points back */
    CAN_HandleTypeDef *hcan;
    bool               use_irq;
    struct can_ring    ring;
    bool               recovering;  /* can_recover() asked, BOFF not
                                     * yet seen clear; see op_get_status() */
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

    /* Silent mode "cannot start a transmission" (RM0090 24.5.1). */
    if (d->hcan->Init.Mode == CAN_MODE_SILENT)
        return -EOPNOTSUPP;
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

    /*
     * BOFF stays set through the recovery; with ABOM the controller starts
     * it by itself (RM0090 24.7.6). After can_recover() without ABOM, the
     * latch says RECOVERING until a look here finds BOFF clear. The bxCAN
     * has no flag for a recovery under way (CAN_MSR, CAN_ESR: 24.9.2), so
     * a recovery and a second bus-off both between two looks read as the
     * first recovery still going. That takes TEC from 0 past 255 again,
     * 32 failed transmissions at least (ISO 11898-1: 8 per transmit error),
     * between two looks. The way out stays open: can_recover() acts
     * whenever BOFF is set, latch or not.
     */
    if (!(esr & CAN_ESR_BOFF))
        d->recovering = false;
    if (HAL_CAN_GetState(d->hcan) != HAL_CAN_STATE_LISTENING)
        status->link = CAN_LINK_STOPPED;
    else if (esr & CAN_ESR_BOFF)
        status->link = (d->recovering ||
                        (d->hcan->Instance->MCR & CAN_MCR_ABOM)) ?
                       CAN_LINK_RECOVERING : CAN_LINK_BUS_OFF;
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

/*
 * HAL_CAN_Init() again with the handle's Init as it now stands: it writes
 * CAN_MCR's options and CAN_BTR, which takes writes in initialisation mode
 * only (RM0090 24.9.1), and nothing else -- the filter banks and CAN_IER
 * stand. Running again afterwards if it was running before. Frames still
 * queued are dropped first, as HAL_FDCAN_Init() drops the FDCAN's:
 * initialisation mode keeps the mailboxes, and they would go out under the
 * new settings, or in silent mode never. An abort empties a
 * pending mailbox at once and one on the bus by the end of its frame
 * (RM0090 24.7.1, "Abort"), which HAL_CAN_Stop() waits for (24.4.1).
 */
static int reinit(bxcan_handle_t d)
{
    bool running = HAL_CAN_GetState(d->hcan) == HAL_CAN_STATE_LISTENING;

    if (running) {
        TRY(can_hal_errno(HAL_CAN_AbortTxRequest(d->hcan, ALL_MAILBOXES)));
        TRY(can_hal_errno(HAL_CAN_Stop(d->hcan)));
    }
    TRY(can_hal_errno(HAL_CAN_Init(d->hcan)));
    if (running)
        TRY(can_hal_errno(HAL_CAN_Start(d->hcan)));
    return 0;
}

/*
 * The bxCAN's clock is APB1's (RM0090 6.3.15, CAN1EN in RCC_APB1ENR), and
 * a quantum is BRP + 1 of its periods (24.7.7). The HAL's Init holds the
 * prescaler as is and the three segments at their CAN_BTR positions, less
 * one, as its CAN_BS1_xTQ, CAN_BS2_xTQ and CAN_SJW_xTQ do
 * (stm32f4xx_hal_can.h:350-391).
 */
static int op_set_bitrate(struct can_dev *can, uint32_t bitrate)
{
    bxcan_handle_t d = of(can);
    struct fdcan_timing t;

    if (bitrate > BITRATE_MAX)
        return -EINVAL;
    TRY(fdcan_timing_find(HAL_RCC_GetPCLK1Freq(), bitrate,
                          FDCAN_NOMINAL_SAMPLE_PERMILLE, &bxcan_limits, &t));
    d->hcan->Init.Prescaler = t.prescaler;
    d->hcan->Init.TimeSeg1 = (t.seg1 - 1) << CAN_BTR_TS1_Pos;
    d->hcan->Init.TimeSeg2 = (t.seg2 - 1) << CAN_BTR_TS2_Pos;
    d->hcan->Init.SyncJumpWidth = (t.sjw - 1) << CAN_BTR_SJW_Pos;
    return reinit(d);
}

/*
 * Loopback is loop back combined with silent mode: CANTX held recessive
 * and CANRX cut off, so off the bus, as on the FDCAN and the MCP2515.
 * Listen is silent mode. RM0090 24.5.1-24.5.3.
 */
static int op_set_opmode(struct can_dev *can, enum can_opmode mode)
{
    bxcan_handle_t d = of(can);

    switch (mode) {
    case CAN_OPMODE_LOOPBACK:
        d->hcan->Init.Mode = CAN_MODE_SILENT_LOOPBACK;
        break;
    case CAN_OPMODE_LISTEN:
        d->hcan->Init.Mode = CAN_MODE_SILENT;
        break;
    default:
        d->hcan->Init.Mode = CAN_MODE_NORMAL;
        break;
    }
    return reinit(d);
}

/* An identifier or mask at its place in a 32-bit bank (RM0090 Figure
 * 231), IDE set for an extended one. */
static uint32_t bank_bits(uint32_t id, bool ext)
{
    return ext ? (id << BANK_EXT_ID_POS) | BANK_IDE :
                 id << BANK_STD_ID_POS;
}

/*
 * The controller's own banks, the first CAN_FILTER_MAX of them, each one
 * filter in 32-bit mask mode into FIFO 0; the rest of its banks are turned
 * off, as RM0090 24.7.4 asks of unused ones. No filters: one bank of mask
 * 0, which passes everything. Scale, mode and FIFO are written in filter
 * initialisation mode, FINIT (24.9.1); reception pauses meanwhile (24.4.1).
 * -ENOSPC when CAN2SB leaves this controller fewer than CAN_FILTER_MAX.
 */
static int op_set_filters(struct can_dev *can,
                          const struct can_filter *filters, size_t count)
{
    bxcan_handle_t d = of(can);
    CAN_TypeDef *master = d->hcan->Instance;
    uint32_t first = 0, end = BANKS_SINGLE;
    uint32_t used = count ? (uint32_t)count : 1;

#if defined(CAN2)
#if defined(CAN3)
    if (master != CAN3)
#endif
    {
        uint32_t slave_start = (CAN1->FMR & CAN_FMR_CAN2SB) >>
                               CAN_FMR_CAN2SB_Pos;

        /* 6 bits, but "in the range 0 to 27", 28 giving CAN1 all
         * (RM0090 24.9.4); more is no layout at all. */
        if (slave_start > BANKS_DUAL)
            return -EINVAL;
        first = master == CAN2 ? slave_start : 0;
        end = master == CAN2 ? BANKS_DUAL : slave_start;
        master = CAN1;
    }
#endif
    if (first + CAN_FILTER_MAX > end)
        return -ENOSPC;

    master->FMR |= CAN_FMR_FINIT;
    for (uint32_t bank = first; bank < end; bank++)
        master->FA1R &= ~(1u << bank);
    for (uint32_t i = 0; i < used; i++) {
        uint32_t bank = first + i;
        uint32_t bit = 1u << bank;

        master->FS1R |= bit;        /* 32-bit */
        master->FM1R &= ~bit;       /* mask mode */
        master->FFA1R &= ~bit;      /* FIFO 0 */
        master->sFilterRegister[bank].FR1 =
            count ? bank_bits(filters[i].id, filters[i].ext) : 0;
        /* The mask always compares IDE, never RTR: the format must
         * match, data and remote frames both pass. */
        master->sFilterRegister[bank].FR2 =
            count ? bank_bits(filters[i].mask, filters[i].ext) | BANK_IDE : 0;
        master->FA1R |= bit;
    }
    master->FMR &= ~CAN_FMR_FINIT;
    return 0;
}

/*
 * Leaving bus-off takes 128 occurrences of 11 recessive bits either way;
 * with ABOM the controller sets out on its own, without it once software
 * has set and cleared CAN_MCR.INRQ (RM0090 24.7.6, CAN_MCR.ABOM in
 * 24.9.2). INRQ is held until CAN_MSR.INAK says initialisation mode was
 * entered (24.4.1). Status reads RECOVERING until BOFF clears. -ENETDOWN
 * when the controller is stopped, -EBUSY when it is not in bus-off.
 */
static int op_recover(struct can_dev *can)
{
    bxcan_handle_t d = of(can);
    CAN_TypeDef *regs = d->hcan->Instance;
    int64_t start;
    int err = 0;

    if (HAL_CAN_GetState(d->hcan) != HAL_CAN_STATE_LISTENING)
        return -ENETDOWN;
    if (!(regs->ESR & CAN_ESR_BOFF))
        return -EBUSY;
    if (regs->MCR & CAN_MCR_ABOM)
        return 0;
    regs->MCR |= CAN_MCR_INRQ;
    start = can_time_us();
    while (!(regs->MSR & CAN_MSR_INAK)) {
        if (can_elapsed_ms(start) >= INIT_TIMEOUT_MS) {
            err = -ETIMEDOUT;
            break;
        }
        can_yield();
    }
    regs->MCR &= ~CAN_MCR_INRQ;
    d->recovering = !err;
    return err;
}

static int op_start(struct can_dev *can)
{
    return can_hal_errno(HAL_CAN_Start(of(can)->hcan));
}

static int op_stop(struct can_dev *can)
{
    return can_hal_errno(HAL_CAN_Stop(of(can)->hcan));
}

static const struct can_ops bxcan_ops = {
    .send = op_send,
    .recv = op_recv,
    .get_status = op_get_status,
    .set_filters = op_set_filters,
    .set_bitrate = op_set_bitrate,
    .set_opmode = op_set_opmode,
    .recover = op_recover,
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
