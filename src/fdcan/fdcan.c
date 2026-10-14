/* First: the HAL module switches the #if below reads. */
#include "can_priv.h"

#if !defined(ESP_PLATFORM) && CONFIG_CAN_FDCAN

#include <stdlib.h>
#include <string.h>

#include "can_log.h"
#include "fdcan_can.h"
#include "fdcan_math.h"

/*
 * Every name below is the HAL's (stm32h5xx_hal_fdcan.h, STM32H5 HAL v1.7.0)
 * or CMSIS's (stm32h563xx.h, cmsis_device_h5 v1.7.0); nothing is a register
 * value of this library's own.
 *
 *   HAL_FDCAN_STATE_READY, _BUSY        :51-52, initialised / started
 *   FDCAN_FRAME_CLASSIC, _FD_NO_BRS, _FD_BRS   :538-540
 *   FDCAN_MODE_NORMAL, _BUS_MONITORING, _INTERNAL_LOOPBACK   :548-551
 *   FDCAN_STANDARD_ID, _EXTENDED_ID     :592-593
 *   FDCAN_DATA_FRAME, _REMOTE_FRAME     :601-602
 *   FDCAN_ESI_ACTIVE, _PASSIVE          :633-634
 *   FDCAN_BRS_OFF, _ON                  :642-643
 *   FDCAN_CLASSIC_CAN, FDCAN_FD_CAN     :651-652
 *   FDCAN_NO_TX_EVENTS                  :660
 *   FDCAN_FILTER_MASK                   :671, FilterID1 id, FilterID2 mask
 *   FDCAN_FILTER_TO_RXFIFO0             :681
 *   FDCAN_RX_FIFO0                      :704
 *   FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT   :768, :770
 *   FDCAN_FILTER_REMOTE                 :778
 *   FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST    :855
 *   FDCAN_IT_RX_FIFO0_NEW_MESSAGE       :922
 *   FDCAN_CCCR_INIT                     stm32h563xx.h:9104
 *   FDCAN_DBTP_TDC; FDCAN_TDCR_TDCO_Pos, _TDCF_Pos   as hal_fdcan.c:1785, :1810
 *                                       uses them; the fields are 7 bits
 *                                       (RM0481 Rev 5, 60.4.3, 60.4.14)
 *   __HAL_FDCAN_CLEAR_FLAG              :1111; IR flags clear on a 1
 *                                       (RM0481 60.4.15)
 *   FDCAN_CKDIV_PDIV                    stm32h563xx.h:9610
 *   RCC_PERIPHCLK_FDCAN                 stm32h5xx_hal_rcc_ex.h:576
 */
/* The Tx FIFO's elements: SRAMCAN_TFQ_NBR, stm32h5xx_hal_fdcan.c:216. */
#define TX_FIFO_ELEMENTS	3

/* Every received frame lands in a 64-byte buffer: the HAL copies the
 * length its DLC table gives, up to 64 (stm32h5xx_hal_fdcan.c:246). */
#define RX_DATA_MAX		CANFD_FRAME_DATA_MAX

#define RING_LEN		16	/* a power of two */

#define TRY(expr) do { int _e = (expr); if (_e) return _e; } while (0)

/*
 * The frames fdcan_irq() hands over, as the core's can_ring does for
 * Classic frames: one writer, the ISR; one reader, the driver.
 */
struct fdcan_ring {
	struct canfd_frame slot[RING_LEN];
	uint32_t head;		/* written by the ISR side only */
	uint32_t tail;		/* written by the reader only */
	uint32_t dropped;	/* frames that found the ring full */
};

struct fdcan_t {
	struct can_dev can;	/* the category view; priv points back */
	FDCAN_HandleTypeDef *hfdcan;
	bool use_irq;
	struct fdcan_ring ring;
	uint32_t fd_dropped;	/* FD frames the Classic recv passed over */
	bool tdc;		/* transmitter delay compensation on */
	uint32_t tdc_offset;	/* its TDCO, in mtq */
	uint32_t tdc_filter;	/* its TDCF, in mtq */
	uint32_t fifo_lost;	/* RF0L flags seen and cleared */
};

/*
 * Rx FIFO 0's message-lost flag, counted and cleared, so rx_lost counts
 * each time the FIFO overflowed (since the last look). One writer: the ISR
 * with use_irq, the thread without.
 */
static void note_lost(fdcan_handle_t d)
{
	if (__HAL_FDCAN_GET_FLAG(d->hfdcan, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST)) {
		__HAL_FDCAN_CLEAR_FLAG(d->hfdcan,
				       FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST);
		d->fifo_lost++;
	}
}

/* ---- the ring ------------------------------------------------------- */

static bool ring_put(struct fdcan_ring *ring, const struct canfd_frame *frame)
{
	uint32_t head = ring->head;
	uint32_t tail = __atomic_load_n(&ring->tail, __ATOMIC_ACQUIRE);

	if (head - tail >= RING_LEN) {
		ring->dropped++;
		return false;
	}
	ring->slot[head & (RING_LEN - 1)] = *frame;
	__atomic_store_n(&ring->head, head + 1, __ATOMIC_RELEASE);
	return true;
}

static bool ring_get(struct fdcan_ring *ring, struct canfd_frame *out)
{
	uint32_t tail = ring->tail;
	uint32_t head = __atomic_load_n(&ring->head, __ATOMIC_ACQUIRE);

	if (head == tail)
		return false;
	*out = ring->slot[tail & (RING_LEN - 1)];
	__atomic_store_n(&ring->tail, tail + 1, __ATOMIC_RELEASE);
	return true;
}

static uint32_t ring_count(const struct fdcan_ring *ring)
{
	return __atomic_load_n(&ring->head, __ATOMIC_ACQUIRE) - ring->tail;
}

/* ---- frames --------------------------------------------------------- */

static bool fifo_take(fdcan_handle_t d, struct canfd_frame *frame)
{
	FDCAN_RxHeaderTypeDef h;
	uint8_t data[RX_DATA_MAX];

	if (HAL_FDCAN_GetRxFifoFillLevel(d->hfdcan, FDCAN_RX_FIFO0) == 0 ||
	    HAL_FDCAN_GetRxMessage(d->hfdcan, FDCAN_RX_FIFO0, &h, data) != HAL_OK)
		return false;
	memset(frame, 0, sizeof(*frame));
	frame->ext = h.IdType == FDCAN_EXTENDED_ID;
	frame->id = h.Identifier;
	frame->rtr = h.RxFrameType == FDCAN_REMOTE_FRAME;
	frame->fd = h.FDFormat == FDCAN_FD_CAN;
	frame->brs = h.BitRateSwitch == FDCAN_BRS_ON;
	frame->esi = h.ErrorStateIndicator == FDCAN_ESI_PASSIVE;
	frame->len = canfd_dlc_to_len((uint8_t)h.DataLength);
	/* A Classic DLC of 9 to 15 still means 8 bytes. */
	if (!frame->fd && frame->len > CAN_FRAME_DATA_MAX)
		frame->len = CAN_FRAME_DATA_MAX;
	if (!frame->rtr)
		memcpy(frame->data, data, frame->len);
	return true;
}

void fdcan_irq(fdcan_handle_t dev)
{
	struct canfd_frame frame;

	if (dev == NULL)
		return;
	note_lost(dev);
	while (fifo_take(dev, &frame)) {
		frame.t_us = can_time_us();
		(void)ring_put(&dev->ring, &frame);
	}
	can_irq(&dev->can);
}

static fdcan_handle_t of(struct can_dev *can)
{
	return can->priv;
}

/* The next frame from the ring or the FIFO, waiting on the clock. */
static int take(fdcan_handle_t d, struct canfd_frame *frame,
		uint32_t timeout_ms, int64_t start)
{
	for (;;) {
		if (!d->use_irq)
			note_lost(d);
		if (d->use_irq ? ring_get(&d->ring, frame) :
				 fifo_take(d, frame))
			return 0;
		if (can_elapsed_ms(start) >= timeout_ms)
			return -ETIMEDOUT;
		can_yield();
	}
}

static int send(fdcan_handle_t d, const struct canfd_frame *frame,
		uint32_t timeout_ms)
{
	uint32_t format = d->hfdcan->Init.FrameFormat;
	int64_t start = can_time_us();
	FDCAN_TxHeaderTypeDef h = {
		.Identifier = frame->id,
		.IdType = frame->ext ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID,
		.TxFrameType = frame->rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME,
		.DataLength = (uint32_t)canfd_len_to_dlc(frame->len),
		.ErrorStateIndicator = FDCAN_ESI_ACTIVE,
		.BitRateSwitch = frame->brs ? FDCAN_BRS_ON : FDCAN_BRS_OFF,
		.FDFormat = frame->fd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN,
		.TxEventFifoControl = FDCAN_NO_TX_EVENTS,
		.MessageMarker = 0,
	};

	if (d->hfdcan->Init.Mode == FDCAN_MODE_BUS_MONITORING)
		return -EOPNOTSUPP;	/* listen-only never sends */
	if ((frame->fd && format == FDCAN_FRAME_CLASSIC) ||
	    (frame->brs && format != FDCAN_FRAME_FD_BRS))
		return -EOPNOTSUPP;	/* the app's Init.FrameFormat */
	while (HAL_FDCAN_GetTxFifoFreeLevel(d->hfdcan) == 0) {
		if (can_elapsed_ms(start) >= timeout_ms)
			return -ETIMEDOUT;
		can_yield();
	}
	return can_hal_errno(HAL_FDCAN_AddMessageToTxFifoQ(d->hfdcan, &h,
							   frame->data));
}

/* ---- the can_dev ops ------------------------------------------------ */

static int op_send(struct can_dev *can, const struct can_frame *frame,
		   uint32_t timeout_ms)
{
	struct canfd_frame fd = {
		.id = frame->id,
		.len = frame->dlc,
		.ext = frame->ext,
		.rtr = frame->rtr,
	};

	memcpy(fd.data, frame->data, frame->dlc);
	return send(of(can), &fd, timeout_ms);
}

static int op_send_fd(struct can_dev *can, const struct canfd_frame *frame,
		      uint32_t timeout_ms)
{
	return send(of(can), frame, timeout_ms);
}

/* Classic frames only: an FD frame is passed over and counted. */
static int op_recv(struct can_dev *can, struct can_frame *frame,
		   uint32_t timeout_ms)
{
	fdcan_handle_t d = of(can);
	int64_t start = can_time_us();
	struct canfd_frame fd;

	for (;;) {
		TRY(take(d, &fd, timeout_ms, start));
		if (!fd.fd)
			break;
		d->fd_dropped++;
	}
	memset(frame, 0, sizeof(*frame));
	frame->id = fd.id;
	frame->dlc = fd.len;
	frame->ext = fd.ext;
	frame->rtr = fd.rtr;
	memcpy(frame->data, fd.data, fd.len);
	frame->t_us = fd.t_us;
	return 0;
}

static int op_recv_fd(struct can_dev *can, struct canfd_frame *frame,
		      uint32_t timeout_ms)
{
	return take(of(can), frame, timeout_ms, can_time_us());
}

static int op_get_status(struct can_dev *can, struct can_status *status)
{
	fdcan_handle_t d = of(can);
	FDCAN_ProtocolStatusTypeDef protocol;
	FDCAN_ErrorCountersTypeDef counters;

	if (HAL_FDCAN_GetState(d->hfdcan) != HAL_FDCAN_STATE_BUSY) {
		status->link = CAN_LINK_STOPPED;
	} else {
		TRY(can_hal_errno(HAL_FDCAN_GetProtocolStatus(d->hfdcan,
							      &protocol)));
		/* In bus-off the controller sets INIT itself; once it is
		 * cleared, it is waiting out the recovery. */
		if (protocol.BusOff)
			status->link = (d->hfdcan->Instance->CCCR &
					FDCAN_CCCR_INIT) ?
				       CAN_LINK_BUS_OFF : CAN_LINK_RECOVERING;
		else if (protocol.ErrorPassive)
			status->link = CAN_LINK_PASSIVE;
		else if (protocol.Warning)
			status->link = CAN_LINK_WARNING;
		else
			status->link = CAN_LINK_ACTIVE;
	}
	TRY(can_hal_errno(HAL_FDCAN_GetErrorCounters(d->hfdcan, &counters)));
	status->tec = (uint16_t)counters.TxErrorCnt;
	status->rec = (uint16_t)counters.RxErrorCnt;
	/* What the ring dropped, and each overflow of the FIFO. */
	if (!d->use_irq)
		note_lost(d);
	status->rx_lost = d->ring.dropped + d->fifo_lost;
	status->rx_pending = d->use_irq ?
			     ring_count(&d->ring) :
			     HAL_FDCAN_GetRxFifoFillLevel(d->hfdcan,
							  FDCAN_RX_FIFO0);
	status->tx_pending = TX_FIFO_ELEMENTS -
			     HAL_FDCAN_GetTxFifoFreeLevel(d->hfdcan);
	status->rx_fd_dropped = d->fd_dropped;
	return 0;
}

/*
 * HAL_FDCAN_Init() again with the handle's Init as it now stands, then the
 * filters, the global filter and the notification, all of which it clears
 * or needs stopped (ConfigGlobalFilter() wants the READY state); running
 * again afterwards if it was running before. The filter lists are sized to
 * the filters given.
 */
static int reinit(fdcan_handle_t d, const struct can_filter *filters,
		  size_t count)
{
	FDCAN_HandleTypeDef *h = d->hfdcan;
	bool running = HAL_FDCAN_GetState(h) == HAL_FDCAN_STATE_BUSY;
	uint32_t std_index = 0, ext_index = 0;

	if (running)
		TRY(can_hal_errno(HAL_FDCAN_Stop(h)));
	h->Init.StdFiltersNbr = 0;
	h->Init.ExtFiltersNbr = 0;
	for (size_t i = 0; i < count; i++) {
		if (filters[i].ext)
			h->Init.ExtFiltersNbr++;
		else
			h->Init.StdFiltersNbr++;
	}
	TRY(can_hal_errno(HAL_FDCAN_Init(h)));
	/* HAL_FDCAN_Init() rewrites DBTP whole, TDC bit included
	 * (stm32h5xx_hal_fdcan.c:505): set it again. Both calls want READY. */
	if (d->tdc) {
		TRY(can_hal_errno(HAL_FDCAN_ConfigTxDelayCompensation(
			h, d->tdc_offset, d->tdc_filter)));
		TRY(can_hal_errno(HAL_FDCAN_EnableTxDelayCompensation(h)));
	}
	for (size_t i = 0; i < count; i++) {
		FDCAN_FilterTypeDef f = {
			.IdType = filters[i].ext ? FDCAN_EXTENDED_ID :
						   FDCAN_STANDARD_ID,
			.FilterIndex = filters[i].ext ? ext_index++ :
							std_index++,
			.FilterType = FDCAN_FILTER_MASK,
			.FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
			.FilterID1 = filters[i].id,
			.FilterID2 = filters[i].mask,
		};

		TRY(can_hal_errno(HAL_FDCAN_ConfigFilter(h, &f)));
	}
	/* No filters: take everything. Some: only what they pass. */
	TRY(can_hal_errno(HAL_FDCAN_ConfigGlobalFilter(
		h, count ? FDCAN_REJECT : FDCAN_ACCEPT_IN_RX_FIFO0,
		count ? FDCAN_REJECT : FDCAN_ACCEPT_IN_RX_FIFO0,
		FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE)));
	if (d->use_irq)
		TRY(can_hal_errno(HAL_FDCAN_ActivateNotification(
			h, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0)));
	if (running)
		TRY(can_hal_errno(HAL_FDCAN_Start(h)));
	return 0;
}

static int op_set_filters(struct can_dev *can,
			  const struct can_filter *filters, size_t count)
{
	return reinit(of(can), filters, count);
}

/* The FDCAN clock: the kernel clock over CKDIV's divider. */
static uint32_t clock_hz(void)
{
	return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN) /
	       fdcan_clock_divisor(FDCAN_CONFIG->CKDIV & FDCAN_CKDIV_PDIV);
}

/* The rate a set of timing fields makes, 0 for fields not set. */
static uint32_t rate_of(uint32_t clock, uint32_t prescaler, uint32_t seg1,
			uint32_t seg2)
{
	return prescaler ? clock / (prescaler * (1 + seg1 + seg2)) : 0;
}

/*
 * Up to 1 Mbit/s, and with BRS no faster than the data phase as it stands:
 * the data rate is "between nominal bitrate and 8 Mbit/s" (RM0481 Rev 5,
 * 60.3.4, p.2893).
 */
static int op_set_bitrate(struct can_dev *can, uint32_t bitrate)
{
	fdcan_handle_t d = of(can);
	const FDCAN_InitTypeDef *init = &d->hfdcan->Init;
	uint32_t clock = clock_hz();
	uint32_t data = rate_of(clock, init->DataPrescaler, init->DataTimeSeg1,
				init->DataTimeSeg2);
	struct fdcan_timing t;

	if (bitrate > FDCAN_NOMINAL_BITRATE_MAX)
		return -EINVAL;
	if (init->FrameFormat == FDCAN_FRAME_FD_BRS && data && bitrate > data)
		return -EINVAL;
	TRY(fdcan_timing_find(clock, bitrate,
			      FDCAN_NOMINAL_SAMPLE_PERMILLE,
			      &fdcan_nominal_limits, &t));
	d->hfdcan->Init.NominalPrescaler = t.prescaler;
	d->hfdcan->Init.NominalTimeSeg1 = t.seg1;
	d->hfdcan->Init.NominalTimeSeg2 = t.seg2;
	d->hfdcan->Init.NominalSyncJumpWidth = t.sjw;
	return reinit(d, can->filters, can->filter_count);
}

/*
 * Up to 8 Mbit/s and no slower than the nominal phase (RM0481 60.3.4).
 * Above 1 Mbit/s with transmitter delay compensation, its offset at the data
 * sample point; see fdcan_math.h.
 */
static int op_set_data_bitrate(struct can_dev *can, uint32_t bitrate)
{
	fdcan_handle_t d = of(can);
	uint32_t clock = clock_hz();
	struct fdcan_timing t;
	int offset = 0;

	if (d->hfdcan->Init.FrameFormat != FDCAN_FRAME_FD_BRS)
		return -EOPNOTSUPP;	/* no data phase without BRS */
	if (bitrate > FDCAN_DATA_BITRATE_MAX ||
	    bitrate < rate_of(clock, d->hfdcan->Init.NominalPrescaler,
			      d->hfdcan->Init.NominalTimeSeg1,
			      d->hfdcan->Init.NominalTimeSeg2))
		return -EINVAL;
	TRY(fdcan_timing_find(clock, bitrate, FDCAN_DATA_SAMPLE_PERMILLE,
			      &fdcan_data_limits, &t));
	if (bitrate > FDCAN_TDC_ABOVE_BITRATE) {
		offset = fdcan_tdc_offset(&t);
		if (offset < 0)
			return offset;
		/* The search takes the smallest prescaler, so 1 or 2 when one
		 * fits exactly; see FDCAN_TDC_PRESCALER_MAX. */
		if (t.prescaler > FDCAN_TDC_PRESCALER_MAX)
			can_warn(can, "data prescaler %lu with delay "
				 "compensation; 1 or 2 is advised",
				 (unsigned long)t.prescaler);
	}
	d->tdc = bitrate > FDCAN_TDC_ABOVE_BITRATE;
	d->tdc_offset = (uint32_t)offset;
	d->tdc_filter = 0;
	d->hfdcan->Init.DataPrescaler = t.prescaler;
	d->hfdcan->Init.DataTimeSeg1 = t.seg1;
	d->hfdcan->Init.DataTimeSeg2 = t.seg2;
	d->hfdcan->Init.DataSyncJumpWidth = t.sjw;
	return reinit(d, can->filters, can->filter_count);
}

/* Loopback is internal: off the bus, as on the MCP2515. */
static int op_set_opmode(struct can_dev *can, enum can_opmode mode)
{
	fdcan_handle_t d = of(can);

	switch (mode) {
	case CAN_OPMODE_LOOPBACK:
		d->hfdcan->Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
		break;
	case CAN_OPMODE_LISTEN:
		d->hfdcan->Init.Mode = FDCAN_MODE_BUS_MONITORING;
		break;
	default:
		d->hfdcan->Init.Mode = FDCAN_MODE_NORMAL;
		break;
	}
	return reinit(d, can->filters, can->filter_count);
}

/*
 * Bus-off sets INIT; clearing it, as HAL_FDCAN_Start() does to leave
 * initialisation, has the controller wait out 129 occurrences of 11
 * recessive bits before it takes part again, and reset its error counters
 * (RM0481 Rev 5, 60.4.13, note under FDCAN_PSR, p.2930). Status reads
 * RECOVERING until then. -ENETDOWN when the
 * controller is stopped, -EBUSY when it is not in bus-off.
 */
static int op_recover(struct can_dev *can)
{
	fdcan_handle_t d = of(can);
	FDCAN_ProtocolStatusTypeDef protocol;

	if (HAL_FDCAN_GetState(d->hfdcan) != HAL_FDCAN_STATE_BUSY)
		return -ENETDOWN;
	TRY(can_hal_errno(HAL_FDCAN_GetProtocolStatus(d->hfdcan, &protocol)));
	if (!protocol.BusOff)
		return -EBUSY;
	d->hfdcan->Instance->CCCR &= ~FDCAN_CCCR_INIT;
	return 0;
}

static int op_start(struct can_dev *can)
{
	return can_hal_errno(HAL_FDCAN_Start(of(can)->hfdcan));
}

static int op_stop(struct can_dev *can)
{
	return can_hal_errno(HAL_FDCAN_Stop(of(can)->hfdcan));
}

static const struct can_ops fdcan_ops = {
	.send = op_send,
	.recv = op_recv,
	.get_status = op_get_status,
	.set_filters = op_set_filters,
	.set_bitrate = op_set_bitrate,
	.set_opmode = op_set_opmode,
	.recover = op_recover,
	.start = op_start,
	.stop = op_stop,
	.send_fd = op_send_fd,
	.recv_fd = op_recv_fd,
	.set_data_bitrate = op_set_data_bitrate,
};

/* ---- init ----------------------------------------------------------- */

int fdcan_init(const fdcan_config_t *cfg, fdcan_handle_t *out)
{
	HAL_FDCAN_StateTypeDef state;
	fdcan_handle_t d;
	int err = 0;

	if (cfg == NULL || out == NULL || cfg->hfdcan == NULL)
		return -EINVAL;
	state = HAL_FDCAN_GetState(cfg->hfdcan);
	if (state != HAL_FDCAN_STATE_READY && state != HAL_FDCAN_STATE_BUSY)
		return -EINVAL;		/* not HAL_FDCAN_Init()ed */

	d = calloc(1, sizeof *d);
	if (d == NULL)
		return -ENOMEM;
	d->can.name = "fdcan";
	d->can.ops = &fdcan_ops;
	d->can.priv = d;
	d->hfdcan = cfg->hfdcan;
	d->use_irq = cfg->use_irq;
	/* A delay compensation the app set up itself is kept: every reinit
	 * writes it back, as HAL_FDCAN_Init() clears it. */
	d->tdc = d->hfdcan->Instance->DBTP & FDCAN_DBTP_TDC;
	d->tdc_offset = (d->hfdcan->Instance->TDCR >> FDCAN_TDCR_TDCO_Pos) &
			0x7F;
	d->tdc_filter = (d->hfdcan->Instance->TDCR >> FDCAN_TDCR_TDCF_Pos) &
			0x7F;

	/* Started already: the app's filters and global filter stand. */
	if (state == HAL_FDCAN_STATE_READY) {
		err = can_hal_errno(HAL_FDCAN_ConfigGlobalFilter(
			d->hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0,
			FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE,
			FDCAN_FILTER_REMOTE));
		if (!err)
			err = can_hal_errno(HAL_FDCAN_Start(d->hfdcan));
	}
	if (!err && d->use_irq)
		err = can_hal_errno(HAL_FDCAN_ActivateNotification(
			d->hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0));
	if (err) {
		can_err(&d->can, "would not start: %d", err);
		free(d);
		return err;
	}
	*out = d;
	return 0;
}

int fdcan_del(fdcan_handle_t dev)
{
	int err = 0;

	if (dev == NULL)
		return -EINVAL;
	if (dev->use_irq)
		err = can_hal_errno(HAL_FDCAN_DeactivateNotification(
			dev->hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE));
	if (HAL_FDCAN_GetState(dev->hfdcan) == HAL_FDCAN_STATE_BUSY) {
		int stop = can_hal_errno(HAL_FDCAN_Stop(dev->hfdcan));

		err = err ? err : stop;
	}
	free(dev);
	return err;
}

struct can_dev *fdcan_can(fdcan_handle_t dev)
{
	return dev ? &dev->can : NULL;
}

#endif /* !ESP_PLATFORM && CONFIG_CAN_FDCAN */
