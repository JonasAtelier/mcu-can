/*
 * Host-side check of the FDCAN driver against a fake of the HAL's FDCAN
 * module (hal_stubs/stm32_hal_stub.h, the STM32H5 HAL's names and values):
 * HAL_FDCAN_Init() between RESET, READY and BUSY, start and stop, the filter
 * lists it sizes from Init, the global filter (READY only, as the HAL has
 * it), a three-element Tx FIFO, Rx FIFO 0 with its fill level and
 * message-lost flag, the new-message notification, and the protocol status
 * and error counters.
 */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32_hal_stub.h"

#include "can.h"
#include "fdcan_can.h"

/* ------------------------------------------------------- the fake HAL --- */

static FDCAN_GlobalTypeDef regs;
static FDCAN_HandleTypeDef hfdcan = { .Instance = &regs };
#define tdc_enabled	(!!(regs.DBTP & FDCAN_DBTP_TDC))

static struct {
	FDCAN_RxHeaderTypeDef h;
	uint8_t data[64];
} fifo[24];
static int fifo_count;
static uint32_t tx_free = 3;
static FDCAN_TxHeaderTypeDef tx_h[8];
static uint8_t tx_data[8][64];
static int tx_count;
static FDCAN_FilterTypeDef filters_std[28], filters_ext[8];
static int filter_writes;
static uint32_t global_std, global_ext, global_rtr_std, global_rtr_ext;
static int global_writes;
static FDCAN_InitTypeDef init_seen;
static int inits, starts, stops;
static uint32_t tdc_offset_seen, tdc_filter_seen;
static int tdc_configs;
static HAL_StatusTypeDef init_ret = HAL_OK, start_ret = HAL_OK,
			 add_ret = HAL_OK, filter_ret = HAL_OK;
static uint32_t notifications;
static FDCAN_ProtocolStatusTypeDef protocol;
static FDCAN_ErrorCountersTypeDef counters;
static uint32_t kernel_hz = 80000000;

HAL_FDCAN_StateTypeDef HAL_FDCAN_GetState(const FDCAN_HandleTypeDef *h)
{
	return h->State;
}

/* Clears the message RAM, filters included, as the HAL's does. */
HAL_StatusTypeDef HAL_FDCAN_Init(FDCAN_HandleTypeDef *h)
{
	assert(h == &hfdcan);
	assert(h->State != HAL_FDCAN_STATE_BUSY && "stopped first");
	if (init_ret != HAL_OK)
		return init_ret;
	init_seen = h->Init;
	h->Instance->DBTP &= ~FDCAN_DBTP_TDC;	/* DBTP is written whole */
	memset(filters_std, 0, sizeof(filters_std));
	memset(filters_ext, 0, sizeof(filters_ext));
	h->Instance->CCCR |= FDCAN_CCCR_INIT;
	h->State = HAL_FDCAN_STATE_READY;
	inits++;
	return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *h)
{
	assert(h == &hfdcan);
	if (start_ret != HAL_OK)
		return start_ret;
	if (h->State != HAL_FDCAN_STATE_READY)
		return HAL_ERROR;
	h->Instance->CCCR &= ~FDCAN_CCCR_INIT;
	h->State = HAL_FDCAN_STATE_BUSY;
	starts++;
	return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *h)
{
	assert(h == &hfdcan);
	if (h->State != HAL_FDCAN_STATE_BUSY)
		return HAL_ERROR;
	h->Instance->CCCR |= FDCAN_CCCR_INIT;
	h->State = HAL_FDCAN_STATE_READY;
	stops++;
	return HAL_OK;
}

/* READY or BUSY; the index must be inside the list Init sized. */
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *h,
					 const FDCAN_FilterTypeDef *f)
{
	assert(h == &hfdcan);
	if (filter_ret != HAL_OK)
		return filter_ret;
	assert(h->State == HAL_FDCAN_STATE_READY ||
	       h->State == HAL_FDCAN_STATE_BUSY);
	if (f->IdType == FDCAN_EXTENDED_ID) {
		assert(f->FilterIndex < h->Init.ExtFiltersNbr);
		filters_ext[f->FilterIndex] = *f;
	} else {
		assert(f->FilterIndex < h->Init.StdFiltersNbr);
		filters_std[f->FilterIndex] = *f;
	}
	filter_writes++;
	return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *h,
					       uint32_t std, uint32_t ext,
					       uint32_t rtr_std,
					       uint32_t rtr_ext)
{
	assert(h == &hfdcan);
	if (h->State != HAL_FDCAN_STATE_READY)
		return HAL_ERROR;
	global_std = std;
	global_ext = ext;
	global_rtr_std = rtr_std;
	global_rtr_ext = rtr_ext;
	global_writes++;
	return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h,
						const FDCAN_TxHeaderTypeDef *header,
						const uint8_t *data)
{
	assert(h == &hfdcan && tx_free > 0);
	if (add_ret != HAL_OK)
		return add_ret;
	if (h->State != HAL_FDCAN_STATE_BUSY)
		return HAL_ERROR;
	tx_h[tx_count % 8] = *header;
	/* What the HAL copies: the DLC's length. */
	memcpy(tx_data[tx_count % 8], data,
	       canfd_dlc_to_len((uint8_t)header->DataLength));
	tx_count++;
	return HAL_OK;
}

uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef *h)
{
	assert(h == &hfdcan);
	return tx_free;
}

uint32_t HAL_FDCAN_GetRxFifoFillLevel(const FDCAN_HandleTypeDef *h,
				      uint32_t fifo_no)
{
	assert(h == &hfdcan && fifo_no == FDCAN_RX_FIFO0);
	return (uint32_t)(fifo_count > 3 ? 3 : fifo_count);
}

HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h,
					 uint32_t fifo_no,
					 FDCAN_RxHeaderTypeDef *header,
					 uint8_t *data)
{
	assert(h == &hfdcan && fifo_no == FDCAN_RX_FIFO0 && fifo_count > 0);
	*header = fifo[0].h;
	memcpy(data, fifo[0].data, canfd_dlc_to_len((uint8_t)header->DataLength));
	memmove(fifo, fifo + 1, sizeof(fifo[0]) * (size_t)--fifo_count);
	return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(const FDCAN_HandleTypeDef *h,
					      FDCAN_ProtocolStatusTypeDef *s)
{
	assert(h == &hfdcan);
	*s = protocol;
	return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(const FDCAN_HandleTypeDef *h,
					     FDCAN_ErrorCountersTypeDef *c)
{
	assert(h == &hfdcan);
	*c = counters;
	return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *h,
						 uint32_t its, uint32_t buffers)
{
	assert(h == &hfdcan && buffers == 0);
	notifications |= its;
	return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_DeactivateNotification(FDCAN_HandleTypeDef *h,
						   uint32_t its)
{
	assert(h == &hfdcan);
	notifications &= ~its;
	return HAL_OK;
}

/* Both READY only, as the HAL's. */
HAL_StatusTypeDef HAL_FDCAN_ConfigTxDelayCompensation(FDCAN_HandleTypeDef *h,
						      uint32_t offset,
						      uint32_t filter)
{
	assert(h == &hfdcan && offset <= 0x7F && filter <= 0x7F);
	if (h->State != HAL_FDCAN_STATE_READY)
		return HAL_ERROR;
	h->Instance->TDCR = (filter << FDCAN_TDCR_TDCF_Pos) |
			    (offset << FDCAN_TDCR_TDCO_Pos);
	tdc_offset_seen = offset;
	tdc_filter_seen = filter;
	tdc_configs++;
	return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_EnableTxDelayCompensation(FDCAN_HandleTypeDef *h)
{
	assert(h == &hfdcan);
	if (h->State != HAL_FDCAN_STATE_READY)
		return HAL_ERROR;
	h->Instance->DBTP |= FDCAN_DBTP_TDC;
	return HAL_OK;
}

uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint64_t clk)
{
	assert(clk == RCC_PERIPHCLK_FDCAN);
	return kernel_hz;
}

/* A frame on the bus, into FIFO 0. dlc is the 4-bit code. */
static void bus_frame(uint32_t id, bool ext, bool rtr, bool fd, bool brs,
		      uint32_t dlc)
{
	fifo[fifo_count].h = (FDCAN_RxHeaderTypeDef){
		.Identifier = id,
		.IdType = ext ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID,
		.RxFrameType = rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME,
		.DataLength = dlc,
		.ErrorStateIndicator = FDCAN_ESI_ACTIVE,
		.BitRateSwitch = brs ? FDCAN_BRS_ON : FDCAN_BRS_OFF,
		.FDFormat = fd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN,
	};
	for (int i = 0; i < 64; i++)
		fifo[fifo_count].data[i] = (uint8_t)(0xA0 + i);
	fifo_count++;
}

/* -------------------------------------------------------------- cases --- */

static int allocs, frees, calloc_fail;

void *__real_calloc(size_t n, size_t size);
void __real_free(void *p);

void *__wrap_calloc(size_t n, size_t size);
void *__wrap_calloc(size_t n, size_t size)
{
	if (calloc_fail)
		return NULL;
	allocs++;
	return __real_calloc(n, size);
}

void __wrap_free(void *p);
void __wrap_free(void *p)
{
	if (p)
		frees++;
	__real_free(p);
}

/* As CubeMX leaves it: initialised, not started, in the given format. */
static fdcan_handle_t up(bool use_irq, uint32_t format)
{
	fdcan_config_t cfg = { .hfdcan = &hfdcan, .use_irq = use_irq };
	fdcan_handle_t d = NULL;

	hfdcan.State = HAL_FDCAN_STATE_READY;
	hfdcan.Init = (FDCAN_InitTypeDef){ .FrameFormat = format,
					   .Mode = FDCAN_MODE_NORMAL };
	regs = (FDCAN_GlobalTypeDef){ .CCCR = FDCAN_CCCR_INIT };
	fifo_count = tx_count = 0;
	tx_free = 3;
	notifications = 0;
	protocol = (FDCAN_ProtocolStatusTypeDef){ 0 };
	counters = (FDCAN_ErrorCountersTypeDef){ 0 };
	inits = starts = stops = filter_writes = global_writes = 0;
	assert(fdcan_init(&cfg, &d) == 0 && d);
	return d;
}

static void down(fdcan_handle_t d)
{
	assert(fdcan_del(d) == 0);
	assert(hfdcan.State == HAL_FDCAN_STATE_READY && notifications == 0);
	assert(allocs == frees);
}

static void test_init(void)
{
	fdcan_config_t cfg = { .hfdcan = &hfdcan };
	fdcan_handle_t d;

	d = up(false, FDCAN_FRAME_CLASSIC);
	assert(starts == 1 && hfdcan.State == HAL_FDCAN_STATE_BUSY);
	/* Everything into FIFO 0 until filters are set; remote frames go
	 * through the filters too. */
	assert(global_writes == 1 && global_std == FDCAN_ACCEPT_IN_RX_FIFO0 &&
	       global_ext == FDCAN_ACCEPT_IN_RX_FIFO0 &&
	       global_rtr_std == FDCAN_FILTER_REMOTE &&
	       global_rtr_ext == FDCAN_FILTER_REMOTE);
	assert(notifications == 0 && inits == 0);
	assert(!strcmp(fdcan_can(d)->name, "fdcan"));
	assert(fdcan_can(NULL) == NULL);
	down(d);

	d = up(true, FDCAN_FRAME_CLASSIC);
	assert(notifications == FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
	down(d);

	/* Started by the app already: left as the app set it. */
	hfdcan.State = HAL_FDCAN_STATE_BUSY;
	starts = global_writes = 0;
	assert(fdcan_init(&cfg, &d) == 0 && starts == 0 && global_writes == 0);
	down(d);

	/* Not HAL_FDCAN_Init()ed. */
	hfdcan.State = HAL_FDCAN_STATE_RESET;
	assert(fdcan_init(&cfg, &d) == -EINVAL);
	hfdcan.State = HAL_FDCAN_STATE_ERROR;
	assert(fdcan_init(&cfg, &d) == -EINVAL);
	assert(fdcan_init(NULL, &d) == -EINVAL);
	assert(fdcan_init(&cfg, NULL) == -EINVAL);
	cfg.hfdcan = NULL;
	assert(fdcan_init(&cfg, &d) == -EINVAL);

	cfg.hfdcan = &hfdcan;
	hfdcan.State = HAL_FDCAN_STATE_READY;
	start_ret = HAL_BUSY;
	assert(fdcan_init(&cfg, &d) == -EAGAIN);
	assert(strstr(stub_log_last, "fdcan: would not start"));
	start_ret = HAL_OK;
	calloc_fail = 1;
	assert(fdcan_init(&cfg, &d) == -ENOMEM);
	calloc_fail = 0;
	assert(allocs == frees);
	assert(fdcan_del(NULL) == -EINVAL);
}

static void test_send_classic(void)
{
	fdcan_handle_t d = up(false, FDCAN_FRAME_CLASSIC);
	struct can_dev *can = fdcan_can(d);
	struct can_frame f = { .id = 0x123, .dlc = 8,
			       .data = { 1, 2, 3, 4, 5, 6, 7, 8 } };
	struct canfd_frame fd = { .id = 0x1, .len = 12, .fd = true };
	uint32_t t0;

	assert(can_send(can, &f, 10) == 0 && tx_count == 1);
	assert(tx_h[0].Identifier == 0x123 && tx_h[0].IdType == FDCAN_STANDARD_ID &&
	       tx_h[0].TxFrameType == FDCAN_DATA_FRAME &&
	       tx_h[0].DataLength == 8 && tx_h[0].FDFormat == FDCAN_CLASSIC_CAN &&
	       tx_h[0].BitRateSwitch == FDCAN_BRS_OFF &&
	       tx_h[0].ErrorStateIndicator == FDCAN_ESI_ACTIVE &&
	       tx_h[0].TxEventFifoControl == FDCAN_NO_TX_EVENTS);
	assert(!memcmp(tx_data[0], f.data, 8));

	f = (struct can_frame){ .id = 0x1ABCDEF0, .ext = true, .rtr = true,
				.dlc = 3 };
	assert(can_send(can, &f, 10) == 0);
	assert(tx_h[1].IdType == FDCAN_EXTENDED_ID &&
	       tx_h[1].Identifier == 0x1ABCDEF0 &&
	       tx_h[1].TxFrameType == FDCAN_REMOTE_FRAME &&
	       tx_h[1].DataLength == 3);

	/* A Classic controller refuses FD frames, but takes Classic ones
	 * through the FD call. */
	assert(can_send_fd(can, &fd, 10) == -EOPNOTSUPP);
	fd = (struct canfd_frame){ .id = 0x2, .len = 1, .data = { 0x5A } };
	assert(can_send_fd(can, &fd, 10) == 0 && tx_count == 3);
	assert(tx_h[2].FDFormat == FDCAN_CLASSIC_CAN && tx_data[2][0] == 0x5A);

	/* A full Tx FIFO: wait on the clock, then give up. */
	tx_free = 0;
	t0 = stub_tick;
	assert(can_send(can, &f, 20) == -ETIMEDOUT);
	assert(stub_tick - t0 >= 20);
	tx_free = 3;
	add_ret = HAL_ERROR;
	assert(can_send(can, &f, 20) == -EIO);
	add_ret = HAL_OK;
	down(d);
}

static void test_send_fd(void)
{
	fdcan_handle_t d = up(false, FDCAN_FRAME_FD_BRS);
	struct can_dev *can = fdcan_can(d);
	struct canfd_frame f = { .id = 0x7FF, .len = 64, .fd = true,
				 .brs = true };

	for (int i = 0; i < 64; i++)
		f.data[i] = (uint8_t)i;
	assert(can_send_fd(can, &f, 10) == 0);
	assert(tx_h[0].FDFormat == FDCAN_FD_CAN &&
	       tx_h[0].BitRateSwitch == FDCAN_BRS_ON &&
	       tx_h[0].DataLength == 15 && tx_data[0][63] == 63);
	f.len = 20;
	f.brs = false;
	assert(can_send_fd(can, &f, 10) == 0 && tx_h[1].DataLength == 11 &&
	       tx_h[1].BitRateSwitch == FDCAN_BRS_OFF);
	down(d);

	/* FD without BRS: FD frames yes, BRS no. */
	d = up(false, FDCAN_FRAME_FD_NO_BRS);
	can = fdcan_can(d);
	assert(can_send_fd(can, &f, 10) == 0);
	f.brs = true;
	assert(can_send_fd(can, &f, 10) == -EOPNOTSUPP && tx_count == 1);
	down(d);
}

static void test_recv_polled(void)
{
	fdcan_handle_t d = up(false, FDCAN_FRAME_FD_BRS);
	struct can_dev *can = fdcan_can(d);
	struct can_frame f;
	struct canfd_frame fd;
	struct can_status s;

	bus_frame(0x321, false, false, false, false, 8);
	assert(can_recv(can, &f, 10) == 0);
	assert(f.id == 0x321 && !f.ext && !f.rtr && f.dlc == 8 &&
	       f.data[7] == 0xA7 && f.t_us != 0);

	/* A Classic DLC past 8 is 8 bytes. */
	bus_frame(0x1, false, false, false, false, 12);
	assert(can_recv(can, &f, 10) == 0 && f.dlc == 8);

	/* Remote: the length asked, no data. */
	bus_frame(0x10000, true, true, false, false, 4);
	assert(can_recv(can, &f, 10) == 0);
	assert(f.ext && f.rtr && f.dlc == 4 && f.data[0] == 0);

	/* FD frames: can_recv() passes over them and counts them;
	 * can_recv_fd() takes them whole. */
	bus_frame(0x50, false, false, true, true, 13);
	bus_frame(0x51, false, false, false, false, 1);
	assert(can_recv(can, &f, 10) == 0 && f.id == 0x51);
	assert(can_get_status(can, &s) == 0 && s.rx_fd_dropped == 1);
	bus_frame(0x52, false, false, true, true, 15);
	assert(can_recv_fd(can, &fd, 10) == 0);
	assert(fd.id == 0x52 && fd.fd && fd.brs && !fd.esi && fd.len == 64 &&
	       fd.data[63] == (uint8_t)(0xA0 + 63));
	bus_frame(0x53, false, false, true, false, 9);
	fifo[0].h.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	assert(can_recv_fd(can, &fd, 10) == 0);
	assert(fd.len == 12 && !fd.brs && fd.esi);

	/* Nothing waiting: a timeout, on the clock. */
	assert(can_recv(can, &f, 5) == -ETIMEDOUT);
	assert(can_recv_fd(can, &fd, 5) == -ETIMEDOUT);
	down(d);
}

static void test_recv_irq(void)
{
	fdcan_handle_t d = up(true, FDCAN_FRAME_FD_BRS);
	struct can_dev *can = fdcan_can(d);
	struct canfd_frame fd;
	struct can_status s;

	/* fdcan_irq() drains the FIFO into the ring, stamping each. */
	stub_tick = 500;
	bus_frame(0x1, false, false, false, false, 2);
	bus_frame(0x2, false, false, true, true, 10);
	fdcan_irq(d);
	assert(fifo_count == 0 && can_irq_pending(can));
	stub_tick = 900;
	assert(can_recv_fd(can, &fd, 0) == 0 && fd.id == 0x1 &&
	       fd.t_us == 500000);
	assert(can_recv_fd(can, &fd, 0) == 0 && fd.id == 0x2 && fd.len == 16);
	assert(can_recv_fd(can, &fd, 0) == -ETIMEDOUT);

	/* A full ring drops and counts. */
	for (int i = 0; i < 20; i++)
		bus_frame((uint32_t)i, false, false, false, false, 1);
	fdcan_irq(d);
	assert(can_get_status(can, &s) == 0 && s.rx_lost == 4 &&
	       s.rx_pending == 16);
	fdcan_irq(NULL);
	down(d);
}

static void test_status(void)
{
	fdcan_handle_t d = up(false, FDCAN_FRAME_CLASSIC);
	struct can_dev *can = fdcan_can(d);
	struct can_status s;

	counters.TxErrorCnt = 97;
	counters.RxErrorCnt = 5;
	assert(can_get_status(can, &s) == 0);
	assert(s.link == CAN_LINK_ACTIVE && s.tec == 97 && s.rec == 5);
	protocol.Warning = 1;
	assert(can_get_status(can, &s) == 0 && s.link == CAN_LINK_WARNING);
	protocol.ErrorPassive = 1;
	assert(can_get_status(can, &s) == 0 && s.link == CAN_LINK_PASSIVE);
	/* Bus-off: the controller has set INIT itself. */
	protocol.BusOff = 1;
	regs.CCCR |= FDCAN_CCCR_INIT;
	assert(can_get_status(can, &s) == 0 && s.link == CAN_LINK_BUS_OFF);
	regs.CCCR &= ~FDCAN_CCCR_INIT;
	assert(can_get_status(can, &s) == 0 && s.link == CAN_LINK_RECOVERING);

	/* Pending counts and the FIFO's message-lost flag. */
	protocol = (FDCAN_ProtocolStatusTypeDef){ 0 };
	bus_frame(1, false, false, false, false, 0);
	bus_frame(2, false, false, false, false, 0);
	tx_free = 1;
	regs.IR = FDCAN_IR_RF0L;
	assert(can_get_status(can, &s) == 0);
	assert(s.rx_pending == 2 && s.tx_pending == 2 && s.rx_lost == 1);
	/* The flag is cleared once counted, and each overflow counts. */
	assert(!(regs.IR & FDCAN_IR_RF0L));
	assert(can_get_status(can, &s) == 0 && s.rx_lost == 1);
	regs.IR = FDCAN_IR_RF0L;
	assert(can_get_status(can, &s) == 0 && s.rx_lost == 2);
	regs.IR = FDCAN_IR_RF0L;
	fifo_count = 0;
	assert(can_recv(can, &(struct can_frame){ 0 }, 0) == -ETIMEDOUT);
	assert(!(regs.IR & FDCAN_IR_RF0L));
	assert(can_get_status(can, &s) == 0 && s.rx_lost == 3);
	tx_free = 3;

	assert(can_stop(can) == 0);
	assert(can_get_status(can, &s) == 0 && s.link == CAN_LINK_STOPPED);
	assert(can_start(can) == 0);
	down(d);
}

static void test_recover(void)
{
	fdcan_handle_t d = up(false, FDCAN_FRAME_CLASSIC);
	struct can_dev *can = fdcan_can(d);

	assert(can_recover(can) == -EBUSY);		/* not bus-off */
	protocol.BusOff = 1;
	regs.CCCR |= FDCAN_CCCR_INIT;
	assert(can_recover(can) == 0);
	assert(!(regs.CCCR & FDCAN_CCCR_INIT));		/* waiting it out */
	assert(can_recover(can) == 0);			/* still recovering */
	assert(can_stop(can) == 0);
	assert(can_recover(can) == -ENETDOWN);
	down(d);
}

static void test_filters(void)
{
	fdcan_handle_t d = up(false, FDCAN_FRAME_CLASSIC);
	struct can_dev *can = fdcan_can(d);
	struct can_filter two[2] = {
		{ .id = 0x120, .mask = 0x7F0 },
		{ .id = 0x1F000000, .mask = 0x1FF00000, .ext = true },
	};

	/* Re-initialised with the lists sized to the filters, each written
	 * as a mask filter into FIFO 0, the rest rejected, running again. */
	assert(can_set_filters(can, two, 2) == 0);
	assert(stops == 1 && inits == 1 && starts == 2);
	assert(init_seen.StdFiltersNbr == 1 && init_seen.ExtFiltersNbr == 1);
	assert(filters_std[0].FilterID1 == 0x120 &&
	       filters_std[0].FilterID2 == 0x7F0 &&
	       filters_std[0].FilterType == FDCAN_FILTER_MASK &&
	       filters_std[0].FilterConfig == FDCAN_FILTER_TO_RXFIFO0 &&
	       filters_std[0].IdType == FDCAN_STANDARD_ID);
	assert(filters_ext[0].FilterID1 == 0x1F000000 &&
	       filters_ext[0].FilterID2 == 0x1FF00000 &&
	       filters_ext[0].IdType == FDCAN_EXTENDED_ID);
	assert(global_std == FDCAN_REJECT && global_ext == FDCAN_REJECT);
	assert(hfdcan.State == HAL_FDCAN_STATE_BUSY);

	/* Two of one format: indexes 0 and 1. */
	two[1] = (struct can_filter){ .id = 0x300, .mask = 0x700 };
	assert(can_set_filters(can, two, 2) == 0);
	assert(init_seen.StdFiltersNbr == 2 && init_seen.ExtFiltersNbr == 0 &&
	       filters_std[1].FilterID1 == 0x300);

	/* None: everything accepted again. */
	assert(can_set_filters(can, NULL, 0) == 0);
	assert(init_seen.StdFiltersNbr == 0 &&
	       global_std == FDCAN_ACCEPT_IN_RX_FIFO0 &&
	       global_ext == FDCAN_ACCEPT_IN_RX_FIFO0);

	/* While stopped: configured, left stopped. */
	assert(can_stop(can) == 0);
	starts = 0;
	assert(can_set_filters(can, two, 1) == 0 && starts == 0 &&
	       hfdcan.State == HAL_FDCAN_STATE_READY);
	assert(can_start(can) == 0);

	/* A HAL failure comes back, and the core keeps the old filters. */
	filter_ret = HAL_ERROR;
	assert(can_set_filters(can, two, 2) == -EIO);
	filter_ret = HAL_OK;
	assert(can->filter_count == 1);

	/* With use_irq the notification survives the re-init. */
	down(d);
	d = up(true, FDCAN_FRAME_CLASSIC);
	notifications = 0;
	assert(can_set_filters(fdcan_can(d), two, 1) == 0);
	assert(notifications == FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
	down(d);
}

static void test_bitrate(void)
{
	fdcan_handle_t d = up(false, FDCAN_FRAME_FD_BRS);
	struct can_dev *can = fdcan_can(d);
	struct can_filter one = { .id = 0x5, .mask = 0x7FF };

	assert(can_set_filters(can, &one, 1) == 0);

	/* 80 MHz kernel clock, no divider: 1 Mbit/s is 80 quanta. */
	stub_fdcan_config.CKDIV = 0;
	kernel_hz = 80000000;
	assert(can_set_bitrate(can, 1000000) == 0);
	assert(init_seen.NominalPrescaler == 1 &&
	       init_seen.NominalTimeSeg1 == 69 &&
	       init_seen.NominalTimeSeg2 == 10 &&
	       init_seen.NominalSyncJumpWidth == 10);
	/* The filters set before are written again after the re-init. */
	assert(init_seen.StdFiltersNbr == 1 && filters_std[0].FilterID1 == 0x5);
	assert(global_std == FDCAN_REJECT);

	/* CKDIV code 2 divides by 4: 20 MHz, 500 kbit/s is 40 quanta. */
	stub_fdcan_config.CKDIV = 2;
	assert(can_set_bitrate(can, 500000) == 0);
	assert(init_seen.NominalPrescaler == 1 &&
	       init_seen.NominalTimeSeg1 + init_seen.NominalTimeSeg2 + 1 == 40);
	stub_fdcan_config.CKDIV = 0;
	assert(can_set_bitrate(can, 300000) == -EINVAL);	/* not exact */
	/* Past RM0481's 1 Mbit/s nominal, even when the clock makes it. */
	inits = 0;
	assert(can_set_bitrate(can, 2000000) == -EINVAL && inits == 0);

	/* The data phase at 1 Mbit/s: no delay compensation. */
	assert(can_set_bitrate(can, 1000000) == 0);
	assert(can_set_data_bitrate(can, 1000000) == 0);
	assert(init_seen.DataPrescaler == 2 && init_seen.DataTimeSeg1 == 29 &&
	       init_seen.DataTimeSeg2 == 10 && init_seen.DataSyncJumpWidth == 10);
	assert(!tdc_enabled);

	/* 2 Mbit/s: 40 quanta of one clock, sample point after 30 -- the
	 * offset, in mtq, and compensation on; the filter window off. */
	tdc_configs = 0;
	assert(can_set_data_bitrate(can, 2000000) == 0);
	assert(init_seen.DataPrescaler == 1 && init_seen.DataTimeSeg1 == 29);
	assert(tdc_enabled && tdc_offset_seen == 30 && tdc_filter_seen == 0 &&
	       tdc_configs == 1);
	assert(hfdcan.State == HAL_FDCAN_STATE_BUSY);

	/* RM0481's top, 8 Mbit/s: 10 quanta, sample after 8. */
	assert(can_set_data_bitrate(can, 8000000) == 0);
	assert(tdc_enabled && tdc_offset_seen == 8);

	/* Any re-init sets it again: HAL_FDCAN_Init() clears it. */
	assert(can_set_filters(can, &one, 1) == 0);
	assert(tdc_enabled && tdc_offset_seen == 8);

	/* Past 8 Mbit/s, or slower than the nominal phase: refused, nothing
	 * touched. */
	inits = 0;
	assert(can_set_data_bitrate(can, 10000000) == -EINVAL && inits == 0);
	assert(can_set_data_bitrate(can, 500000) == -EINVAL && inits == 0);
	assert(tdc_enabled);

	/* Back to 1 Mbit/s: compensation off after the re-init. */
	assert(can_set_data_bitrate(can, 1000000) == 0 && !tdc_enabled);

	/* An offset past TDCO's 127 mtq is refused: a 400 MHz clock at
	 * 2 Mbit/s takes prescaler 5, sample point after 30 quanta, 150 mtq.
	 * The nominal phase is set at 80 MHz, so it reads 5 Mbit/s here; set
	 * it at 400 MHz first. */
	kernel_hz = 400000000;
	assert(can_set_bitrate(can, 1000000) == 0);
	inits = 0;
	assert(can_set_data_bitrate(can, 2000000) == -EINVAL && inits == 0);
	kernel_hz = 80000000;
	assert(hfdcan.State == HAL_FDCAN_STATE_BUSY);
	init_ret = HAL_ERROR;
	assert(can_set_bitrate(can, 1000000) == -EIO);
	init_ret = HAL_OK;
	hfdcan.State = HAL_FDCAN_STATE_READY;	/* as the failure left it */
	assert(can_start(can) == 0);
	down(d);

	/* No BRS, no data phase to set. */
	d = up(false, FDCAN_FRAME_FD_NO_BRS);
	assert(can_set_data_bitrate(fdcan_can(d), 1000000) == -EOPNOTSUPP);
	down(d);
}

static void test_opmode(void)
{
	fdcan_handle_t d = up(false, FDCAN_FRAME_CLASSIC);
	struct can_dev *can = fdcan_can(d);
	struct can_frame f = { .id = 0x1, .dlc = 1 };

	assert(can_set_opmode(can, CAN_OPMODE_LOOPBACK) == 0);
	assert(init_seen.Mode == FDCAN_MODE_INTERNAL_LOOPBACK);
	assert(can_send(can, &f, 10) == 0);
	assert(can_set_opmode(can, CAN_OPMODE_LISTEN) == 0);
	assert(init_seen.Mode == FDCAN_MODE_BUS_MONITORING);
	assert(can_send(can, &f, 10) == -EOPNOTSUPP);	/* listen-only */
	assert(can_set_opmode(can, CAN_OPMODE_NORMAL) == 0);
	assert(init_seen.Mode == FDCAN_MODE_NORMAL);
	assert(can_send(can, &f, 10) == 0);
	assert(hfdcan.State == HAL_FDCAN_STATE_BUSY);
	down(d);
}

/* With use_irq, fdcan_irq() counts the overflow, one writer. */
static void test_lost_irq(void)
{
	fdcan_handle_t d = up(true, FDCAN_FRAME_CLASSIC);
	struct can_status s;

	regs.IR = FDCAN_IR_RF0L;
	assert(can_get_status(fdcan_can(d), &s) == 0 && s.rx_lost == 0);
	fdcan_irq(d);
	assert(!(regs.IR & FDCAN_IR_RF0L));
	assert(can_get_status(fdcan_can(d), &s) == 0 && s.rx_lost == 1);
	down(d);
}

/*
 * A delay compensation the app set up (CubeMX, or by hand after
 * HAL_FDCAN_Init()) survives every re-init the library does.
 */
static void test_app_tdc(void)
{
	fdcan_config_t cfg = { .hfdcan = &hfdcan };
	struct can_filter one = { .id = 0x5, .mask = 0x7FF };
	fdcan_handle_t d = NULL;
	struct can_dev *can;

	hfdcan.State = HAL_FDCAN_STATE_READY;
	hfdcan.Init = (FDCAN_InitTypeDef){ .FrameFormat = FDCAN_FRAME_FD_BRS,
					   .Mode = FDCAN_MODE_NORMAL };
	regs = (FDCAN_GlobalTypeDef){
		.CCCR = FDCAN_CCCR_INIT,
		.DBTP = FDCAN_DBTP_TDC,
		.TDCR = (13U << FDCAN_TDCR_TDCO_Pos) | (5U << FDCAN_TDCR_TDCF_Pos),
	};
	stub_fdcan_config.CKDIV = 0;
	kernel_hz = 80000000;
	assert(fdcan_init(&cfg, &d) == 0);
	can = fdcan_can(d);

	assert(can_set_filters(can, &one, 1) == 0);
	assert(tdc_enabled && tdc_offset_seen == 13 && tdc_filter_seen == 5);
	assert(can_set_opmode(can, CAN_OPMODE_LOOPBACK) == 0);
	assert(tdc_enabled && tdc_offset_seen == 13 && tdc_filter_seen == 5);
	assert(can_set_bitrate(can, 500000) == 0);
	assert(tdc_enabled && tdc_offset_seen == 13 && tdc_filter_seen == 5);
	assert(((regs.TDCR >> FDCAN_TDCR_TDCO_Pos) & 0x7F) == 13);
	/* can_set_data_bitrate() replaces it with its own: offset at the
	 * sample point, filter window off. */
	assert(can_set_data_bitrate(can, 2000000) == 0);
	assert(tdc_enabled && tdc_offset_seen == 30 && tdc_filter_seen == 0);
	assert(can_set_filters(can, &one, 1) == 0 && tdc_filter_seen == 0);
	down(d);

	/* Without it, none is invented. */
	d = up(false, FDCAN_FRAME_FD_BRS);
	assert(can_set_filters(fdcan_can(d), &one, 1) == 0 && !tdc_enabled);
	down(d);
}

/* With BRS the nominal rate may not pass the data rate (RM0481 60.3.4). */
static void test_nominal_under_data(void)
{
	fdcan_handle_t d = up(false, FDCAN_FRAME_FD_BRS);
	struct can_dev *can = fdcan_can(d);

	stub_fdcan_config.CKDIV = 0;
	kernel_hz = 80000000;
	assert(can_set_bitrate(can, 500000) == 0);
	assert(can_set_data_bitrate(can, 500000) == 0);
	inits = 0;
	assert(can_set_bitrate(can, 1000000) == -EINVAL && inits == 0);
	assert(can_set_bitrate(can, 250000) == 0);
	assert(can_set_bitrate(can, 500000) == 0);	/* equal is fine */
	down(d);

	/* Classic: no data phase to compare with. */
	d = up(false, FDCAN_FRAME_CLASSIC);
	hfdcan.Init.DataPrescaler = 1;
	hfdcan.Init.DataTimeSeg1 = 100;
	hfdcan.Init.DataTimeSeg2 = 59;		/* 500 kbit/s, unused */
	assert(can_set_bitrate(fdcan_can(d), 1000000) == 0);
	down(d);
}

/*
 * 240 MHz at 2 Mbit/s: neither prescaler 1 (120 quanta) nor 2 (60) keeps
 * seg1 within 32, so 3 it is (40 quanta, offset 90 mtq) -- taken, with a
 * warning that 1 or 2 is advised.
 */
static void test_tdc_prescaler(void)
{
	fdcan_handle_t d = up(false, FDCAN_FRAME_FD_BRS);
	struct can_dev *can = fdcan_can(d);

	stub_fdcan_config.CKDIV = 0;
	kernel_hz = 240000000;
	assert(can_set_bitrate(can, 1000000) == 0);
	stub_log_last[0] = '\0';
	assert(can_set_data_bitrate(can, 2000000) == 0);
	assert(init_seen.DataPrescaler == 3 && init_seen.DataTimeSeg1 == 29);
	assert(tdc_enabled && tdc_offset_seen == 90);
	assert(strstr(stub_log_last, "data prescaler 3"));
	/* 80 MHz: prescaler 1, no warning. (The data phase first: at 80 MHz
	 * the old data fields read 667 kbit/s, under a 1 Mbit/s nominal.) */
	kernel_hz = 80000000;
	stub_log_last[0] = '\0';
	assert(can_set_data_bitrate(can, 2000000) == 0);
	assert(init_seen.DataPrescaler == 1 && stub_log_last[0] == '\0');
	assert(can_set_bitrate(can, 1000000) == 0);
	down(d);
}

int main(void)
{
	test_init();
	test_send_classic();
	test_send_fd();
	test_recv_polled();
	test_recv_irq();
	test_status();
	test_recover();
	test_filters();
	test_bitrate();
	test_opmode();
	test_lost_irq();
	test_app_tdc();
	test_nominal_under_data();
	test_tdc_prescaler();
	printf("test_fdcan: all passed\n");
	return 0;
}
