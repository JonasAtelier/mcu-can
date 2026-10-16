/*
 * Host-side check of the bxCAN driver against a fake of the HAL's CAN
 * module (hal_stubs/stm32_hal_stub.h): start and stop between READY and
 * LISTENING, three transmit mailboxes, receive FIFO 0 with its fill level,
 * the FIFO 0 message-pending notification, the error status register the
 * driver reads TEC, REC and the error flags from, HAL_CAN_Init() writing
 * BTR from the handle's Init, and the registers the driver writes itself:
 * the filter banks in CAN1 and MCR's INRQ for the bus-off recovery.
 */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32_hal_stub.h"

#include "bxcan.h"
#include "can.h"

/* ------------------------------------------------------- the fake HAL --- */

static CAN_HandleTypeDef hcan = { .Instance = CAN1 };

static struct {
	CAN_RxHeaderTypeDef h;
	uint8_t data[8];
} fifo[8];
static int fifo_count;
static uint32_t free_mailboxes = 3;
static CAN_TxHeaderTypeDef tx_h[8];
static uint8_t tx_data[8][8];
static int tx_count;
static HAL_StatusTypeDef add_ret = HAL_OK, start_ret = HAL_OK;
static HAL_StatusTypeDef init_ret = HAL_OK;
static uint32_t notifications;
static int starts, stops, inits;
static uint32_t aborted;	/* the mailboxes the last abort named */
static int aborted_at_stop;	/* aborts before the last stop */
static int aborts;
static bool ack_init;	/* the fake enters initialisation mode on INRQ */

/*
 * As the stub's, at least Delay + 1 ticks; and with ack_init, a controller
 * that sets INAK once INRQ has been set -- only then, so a driver that
 * never sets INRQ waits in vain.
 */
void HAL_Delay(uint32_t Delay)
{
	stub_tick += Delay + 1;
	if (ack_init && (stub_can1.MCR & CAN_MCR_INRQ))
		stub_can1.MSR |= CAN_MSR_INAK;
}

HAL_CAN_StateTypeDef HAL_CAN_GetState(const CAN_HandleTypeDef *h)
{
	return h->State;
}

/* An F405 at 168 MHz: APB1 at 42 MHz. */
uint32_t HAL_RCC_GetPCLK1Freq(void)
{
	return 42000000;
}

/* As the HAL's: BTR from Init, and READY; the driver stops it first. */
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef *h)
{
	assert(h == &hcan && h->State == HAL_CAN_STATE_READY);
	if (init_ret != HAL_OK)
		return init_ret;
	h->Instance->BTR = h->Init.Mode | h->Init.SyncJumpWidth |
			   h->Init.TimeSeg1 | h->Init.TimeSeg2 |
			   (h->Init.Prescaler - 1);
	inits++;
	return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *h)
{
	assert(h == &hcan);
	if (start_ret != HAL_OK)
		return start_ret;
	if (h->State != HAL_CAN_STATE_READY)
		return HAL_ERROR;
	h->State = HAL_CAN_STATE_LISTENING;
	starts++;
	return HAL_OK;
}

/* Every queued frame dropped, as RM0090 24.7.1's abort ends. */
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *h,
					 uint32_t mailboxes)
{
	assert(h == &hcan && h->State == HAL_CAN_STATE_LISTENING);
	aborted = mailboxes;
	aborts++;
	free_mailboxes = 3;
	return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *h)
{
	assert(h == &hcan);
	if (h->State != HAL_CAN_STATE_LISTENING)
		return HAL_ERROR;
	aborted_at_stop = aborts;
	h->State = HAL_CAN_STATE_READY;
	stops++;
	return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h,
				       const CAN_TxHeaderTypeDef *header,
				       const uint8_t data[], uint32_t *mailbox)
{
	assert(h == &hcan && free_mailboxes > 0);
	if (add_ret != HAL_OK)
		return add_ret;
	tx_h[tx_count % 8] = *header;
	memcpy(tx_data[tx_count % 8], data, 8);
	tx_count++;
	*mailbox = 1;
	return HAL_OK;
}

uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *h)
{
	assert(h == &hcan);
	return free_mailboxes;
}

uint32_t HAL_CAN_GetRxFifoFillLevel(const CAN_HandleTypeDef *h,
				    uint32_t fifo_no)
{
	assert(h == &hcan && fifo_no == CAN_RX_FIFO0);
	return (uint32_t)(fifo_count > 3 ? 3 : fifo_count);
}

HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h, uint32_t fifo_no,
				       CAN_RxHeaderTypeDef *header,
				       uint8_t data[])
{
	assert(h == &hcan && fifo_no == CAN_RX_FIFO0 && fifo_count > 0);
	*header = fifo[0].h;
	memcpy(data, fifo[0].data, 8);
	memmove(fifo, fifo + 1, sizeof(fifo[0]) * (size_t)--fifo_count);
	return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h,
					       uint32_t its)
{
	assert(h == &hcan);
	notifications |= its;
	return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef *h,
						 uint32_t its)
{
	assert(h == &hcan);
	notifications &= ~its;
	return HAL_OK;
}

static void bus_frame(uint32_t id, bool ext, bool rtr, uint32_t dlc)
{
	fifo[fifo_count].h = (CAN_RxHeaderTypeDef){
		.StdId = ext ? 0 : id,
		.ExtId = ext ? id : 0,
		.IDE = ext ? CAN_ID_EXT : CAN_ID_STD,
		.RTR = rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA,
		.DLC = dlc,
	};
	for (int i = 0; i < 8; i++)
		fifo[fifo_count].data[i] = (uint8_t)(0xB0 + i);
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

static bxcan_handle_t up(bool use_irq)
{
	bxcan_config_t cfg = { .hcan = &hcan, .use_irq = use_irq };
	bxcan_handle_t d = NULL;

	hcan.State = HAL_CAN_STATE_READY;
	fifo_count = tx_count = 0;
	free_mailboxes = 3;
	hcan.Instance = CAN1;
	hcan.Init = (CAN_InitTypeDef){ 0 };
	stub_can1 = (CAN_TypeDef){ 0 };
	/* CAN2's banks from 14, half each: CAN_FMR's reset CAN2SB. */
	stub_can1.FMR = 14u << CAN_FMR_CAN2SB_Pos;
	assert(bxcan_init(&cfg, &d) == 0 && d);
	return d;
}

static void down(bxcan_handle_t d)
{
	assert(bxcan_del(d) == 0);
	assert(hcan.State == HAL_CAN_STATE_READY && notifications == 0);
	assert(allocs == frees);
}

static void test_init(void)
{
	bxcan_config_t cfg = { .hcan = &hcan };
	bxcan_handle_t d;

	starts = 0;
	d = up(false);
	assert(starts == 1 && hcan.State == HAL_CAN_STATE_LISTENING);
	assert(notifications == 0);
	assert(!strcmp(bxcan_can(d)->name, "bxcan"));
	assert(bxcan_can(NULL) == NULL);
	down(d);

	d = up(true);
	assert(notifications == CAN_IT_RX_FIFO0_MSG_PENDING);
	down(d);

	/* Started by the app already: left running, not started twice. */
	hcan.State = HAL_CAN_STATE_LISTENING;
	starts = 0;
	assert(bxcan_init(&cfg, &d) == 0 && starts == 0);
	down(d);

	/* Not HAL_CAN_Init()ed. */
	hcan.State = HAL_CAN_STATE_RESET;
	assert(bxcan_init(&cfg, &d) == -EINVAL);
	hcan.State = HAL_CAN_STATE_ERROR;
	assert(bxcan_init(&cfg, &d) == -EINVAL);
	assert(bxcan_init(NULL, &d) == -EINVAL);
	assert(bxcan_init(&cfg, NULL) == -EINVAL);
	cfg.hcan = NULL;
	assert(bxcan_init(&cfg, &d) == -EINVAL);

	cfg.hcan = &hcan;
	hcan.State = HAL_CAN_STATE_READY;
	start_ret = HAL_BUSY;
	assert(bxcan_init(&cfg, &d) == -EAGAIN);
	assert(strstr(stub_log_last, "bxcan: would not start"));
	start_ret = HAL_OK;
	calloc_fail = 1;
	assert(bxcan_init(&cfg, &d) == -ENOMEM);
	calloc_fail = 0;
	assert(allocs == frees);
	assert(bxcan_del(NULL) == -EINVAL);
}

static void test_send(void)
{
	bxcan_handle_t d = up(false);
	struct can_dev *can = bxcan_can(d);
	struct can_frame f = { .id = 0x123, .dlc = 2, .data = { 5, 6 } };
	uint32_t t0;

	assert(can_send(can, &f, 0) == 0);
	assert(tx_h[0].StdId == 0x123 && tx_h[0].IDE == CAN_ID_STD &&
	       tx_h[0].RTR == CAN_RTR_DATA && tx_h[0].DLC == 2 &&
	       tx_h[0].TransmitGlobalTime == DISABLE && tx_data[0][1] == 6);
	f = (struct can_frame){ .id = 0x1ABCDE, .ext = true, .rtr = true,
				.dlc = 4 };
	assert(can_send(can, &f, 0) == 0);
	assert(tx_h[1].ExtId == 0x1ABCDE && tx_h[1].StdId == 0 &&
	       tx_h[1].IDE == CAN_ID_EXT && tx_h[1].RTR == CAN_RTR_REMOTE &&
	       tx_h[1].DLC == 4);

	/* No mailbox free: wait on the clock, then -ETIMEDOUT. */
	free_mailboxes = 0;
	t0 = stub_tick;
	assert(can_send(can, &f, 0) == -ETIMEDOUT && stub_tick == t0);
	assert(can_send(can, &f, 20) == -ETIMEDOUT && stub_tick - t0 >= 20);
	free_mailboxes = 1;
	add_ret = HAL_ERROR;
	assert(can_send(can, &f, 0) == -EIO);
	add_ret = HAL_OK;
	down(d);
}

static void test_recv_polled(void)
{
	bxcan_handle_t d = up(false);
	struct can_dev *can = bxcan_can(d);
	struct can_frame f;
	uint32_t t0;

	bus_frame(0x321, false, false, 8);
	bus_frame(0x10, true, true, 3);
	bus_frame(0x11, false, false, 0);
	stub_tick = 5000;
	assert(can_recv(can, &f, 0) == 0);
	assert(f.id == 0x321 && !f.ext && !f.rtr && f.dlc == 8 &&
	       f.data[7] == 0xB7 && f.t_us == 5000000);
	assert(can_recv(can, &f, 0) == 0 && f.id == 0x10 && f.ext && f.rtr &&
	       f.dlc == 3 && f.data[0] == 0);
	assert(can_recv(can, &f, 0) == 0 && f.id == 0x11 && f.dlc == 0);
	t0 = stub_tick;
	assert(can_recv(can, &f, 0) == -ETIMEDOUT && stub_tick == t0);
	assert(can_recv(can, &f, 12) == -ETIMEDOUT && stub_tick - t0 >= 12);
	down(d);
}

static void test_recv_irq(void)
{
	bxcan_handle_t d = up(true);
	struct can_dev *can = bxcan_can(d);
	struct can_frame f;
	struct can_status st;

	/* The ISR empties the FIFO, stamping each frame then. */
	stub_tick = 7000;
	bus_frame(0x1, false, false, 1);
	bus_frame(0x2, false, false, 1);
	bxcan_irq(d);
	assert(fifo_count == 0 && can_irq_pending(can));
	stub_tick = 9000;
	assert(can_recv(can, &f, 0) == 0 && f.id == 1 && f.t_us == 7000000);
	assert(can_recv(can, &f, 0) == 0 && f.id == 2);
	assert(can_recv(can, &f, 0) == -ETIMEDOUT);

	/* A full ring drops and counts. */
	for (int burst = 0; burst < 6; burst++) {
		for (int i = 0; i < 3; i++)
			bus_frame((uint32_t)(burst * 3 + i), false, false, 0);
		bxcan_irq(d);
	}
	assert(can_get_status(can, &st) == 0);
	assert(st.rx_pending == 16 && st.rx_lost == 2);
	for (uint32_t i = 0; i < 16; i++)
		assert(can_recv(can, &f, 0) == 0 && f.id == i);
	bxcan_irq(NULL);
	down(d);
}

static void test_status(void)
{
	bxcan_handle_t d = up(false);
	struct can_dev *can = bxcan_can(d);
	struct can_status st;

	stub_can1.ESR = (100u << CAN_ESR_TEC_Pos) | (7u << CAN_ESR_REC_Pos) |
		   CAN_ESR_EWGF;
	free_mailboxes = 1;
	bus_frame(1, false, false, 0);
	assert(can_get_status(can, &st) == 0);
	assert(st.link == CAN_LINK_WARNING && st.tec == 100 && st.rec == 7);
	assert(st.tx_pending == 2 && st.rx_pending == 1 && st.rx_lost == 0);
	stub_can1.ESR |= CAN_ESR_EPVF;
	assert(can_get_status(can, &st) == 0 && st.link == CAN_LINK_PASSIVE);
	stub_can1.ESR |= CAN_ESR_BOFF;
	assert(can_get_status(can, &st) == 0 && st.link == CAN_LINK_BUS_OFF);
	stub_can1.ESR = 0;
	stub_can1.RF0R = CAN_RF0R_FOVR0;
	assert(can_get_status(can, &st) == 0 && st.link == CAN_LINK_ACTIVE);
	assert(st.rx_lost == 1);

	/* Start and stop are the HAL's. */
	assert(can_stop(can) == 0 && hcan.State == HAL_CAN_STATE_READY);
	assert(can_get_status(can, &st) == 0 && st.link == CAN_LINK_STOPPED);
	assert(can_stop(can) == -EIO);
	assert(can_start(can) == 0 && hcan.State == HAL_CAN_STATE_LISTENING);
	down(d);
}

#define BTR_FIELDS(prescaler, seg1, seg2, sjw)                        \
	((((sjw) - 1u) << CAN_BTR_SJW_Pos) | (((seg2) - 1u) << CAN_BTR_TS2_Pos) | \
	 (((seg1) - 1u) << CAN_BTR_TS1_Pos) | ((prescaler) - 1u))

static void test_bitrate(void)
{
	bxcan_handle_t d = up(false);
	struct can_dev *can = bxcan_can(d);

	/* 42 MHz / 500 kbit/s = 84 clocks: prescaler 6, 1 + 11 + 2 quanta,
	 * the sample point at 12 of 14, the nearest to 87.5%. */
	starts = stops = inits = 0;
	assert(can_set_bitrate(can, 500000) == 0);
	assert(hcan.Init.Prescaler == 6);
	assert(hcan.Init.TimeSeg1 == 10u << CAN_BTR_TS1_Pos);
	assert(hcan.Init.TimeSeg2 == 1u << CAN_BTR_TS2_Pos);
	assert(hcan.Init.SyncJumpWidth == 1u << CAN_BTR_SJW_Pos);
	assert(stub_can1.BTR == BTR_FIELDS(6, 11, 2, 2));
	/* Stopped for HAL_CAN_Init(), and running again. */
	assert(stops == 1 && inits == 1 && starts == 1);
	assert(hcan.State == HAL_CAN_STATE_LISTENING);

	assert(can_set_bitrate(can, 1000000) == 0);
	assert(stub_can1.BTR == BTR_FIELDS(3, 11, 2, 2));
	assert(can_set_bitrate(can, 125000) == 0);
	assert(stub_can1.BTR == BTR_FIELDS(21, 13, 2, 2));

	/* Over 1 Mbit/s, or no exact fit: refused, nothing touched. */
	inits = 0;
	assert(can_set_bitrate(can, 1000001) == -EINVAL);
	assert(can_set_bitrate(can, 333333) == -EINVAL);
	assert(inits == 0 && stub_can1.BTR == BTR_FIELDS(21, 13, 2, 2));

	/* Stopped: set, and left stopped. */
	assert(can_stop(can) == 0);
	starts = 0;
	assert(can_set_bitrate(can, 250000) == 0);
	assert(stub_can1.BTR == BTR_FIELDS(12, 11, 2, 2));
	assert(starts == 0 && hcan.State == HAL_CAN_STATE_READY);
	init_ret = HAL_ERROR;
	assert(can_set_bitrate(can, 500000) == -EIO);
	init_ret = HAL_OK;
	assert(can_start(can) == 0);
	down(d);
}

static void test_opmode(void)
{
	bxcan_handle_t d = up(false);
	struct can_dev *can = bxcan_can(d);
	struct can_frame f = { .id = 0x55, .dlc = 1 };

	hcan.Init.Prescaler = 6;
	starts = inits = 0;
	/* Two frames queued: dropped before the controller stops. */
	free_mailboxes = 1;
	aborts = aborted_at_stop = 0;
	aborted = 0;
	assert(can_set_opmode(can, CAN_OPMODE_LOOPBACK) == 0);
	assert(aborts == 1 && aborted_at_stop == 1);
	assert(aborted == (CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 |
			   CAN_TX_MAILBOX2));
	{
		struct can_status st;

		assert(can_get_status(can, &st) == 0 && st.tx_pending == 0);
	}
	assert(hcan.Init.Mode == CAN_MODE_SILENT_LOOPBACK);
	assert((stub_can1.BTR & (CAN_BTR_LBKM | CAN_BTR_SILM)) ==
	       (CAN_BTR_LBKM | CAN_BTR_SILM));
	assert(inits == 1 && starts == 1);
	tx_count = 0;
	assert(can_send(can, &f, 0) == 0 && tx_count == 1);

	/* Listen-only never sends. */
	assert(can_set_opmode(can, CAN_OPMODE_LISTEN) == 0);
	assert(hcan.Init.Mode == CAN_MODE_SILENT);
	assert(can_send(can, &f, 0) == -EOPNOTSUPP && tx_count == 1);

	assert(can_set_opmode(can, CAN_OPMODE_NORMAL) == 0);
	assert(hcan.Init.Mode == CAN_MODE_NORMAL);
	assert((stub_can1.BTR & (CAN_BTR_LBKM | CAN_BTR_SILM)) == 0);
	assert(can_send(can, &f, 0) == 0 && tx_count == 2);
	down(d);
}

#define IDE	(1u << 2)	/* CAN_FxR's IDE bit, RM0090 Figure 231 */

static void test_filters(void)
{
	bxcan_handle_t d = up(false);
	struct can_dev *can = bxcan_can(d);
	struct can_filter fl[2] = {
		{ .id = 0x123, .mask = 0x7F0 },
		{ .id = 0x1ABCDE, .mask = 0x1FFFFF00, .ext = true },
	};

	/* The app's: banks 0 and 5 of CAN1 in list mode on FIFO 1, and bank
	 * 20 of CAN2's. */
	stub_can1.FA1R = (1u << 0) | (1u << 5) | (1u << 20);
	stub_can1.FM1R = stub_can1.FFA1R = (1u << 0) | (1u << 1);
	assert(can_set_filters(can, fl, 2) == 0);
	/* Two 32-bit mask banks on FIFO 0, the rest of CAN1's off. */
	assert(stub_can1.FA1R == ((1u << 0) | (1u << 1) | (1u << 20)));
	assert((stub_can1.FS1R & 3u) == 3u);
	assert((stub_can1.FM1R & 3u) == 0 && (stub_can1.FFA1R & 3u) == 0);
	assert(stub_can1.sFilterRegister[0].FR1 == 0x123u << 21);
	assert(stub_can1.sFilterRegister[0].FR2 == ((0x7F0u << 21) | IDE));
	assert(stub_can1.sFilterRegister[1].FR1 == ((0x1ABCDEu << 3) | IDE));
	assert(stub_can1.sFilterRegister[1].FR2 ==
	       ((0x1FFFFF00u << 3) | IDE));
	assert(!(stub_can1.FMR & CAN_FMR_FINIT));
	assert((stub_can1.FMR & CAN_FMR_CAN2SB) == 14u << CAN_FMR_CAN2SB_Pos);

	/* None: one bank, mask 0, takes everything. */
	assert(can_set_filters(can, NULL, 0) == 0);
	assert(stub_can1.FA1R == ((1u << 0) | (1u << 20)));
	assert(stub_can1.sFilterRegister[0].FR1 == 0 &&
	       stub_can1.sFilterRegister[0].FR2 == 0);

	/* CAN2's banks start at CAN2SB, in CAN1's registers. */
	hcan.Instance = CAN2;
	assert(can_set_filters(can, fl, 1) == 0);
	assert(stub_can1.FA1R == ((1u << 0) | (1u << 14)));
	assert(stub_can1.sFilterRegister[14].FR1 == 0x123u << 21);

	/* CAN2's registers never touched: the banks live in CAN1. */
	{
		static const CAN_TypeDef untouched;

		assert(!memcmp(&stub_can2, &untouched, sizeof untouched));
	}

	/* CAN2SB past 28 is no layout: refused, from either side. */
	stub_can1.FMR = 40u << CAN_FMR_CAN2SB_Pos;
	assert(can_set_filters(can, fl, 1) == -EINVAL);
	hcan.Instance = CAN1;
	assert(can_set_filters(can, fl, 1) == -EINVAL);
	hcan.Instance = CAN2;

	/* Fewer than two banks for this CAN: refused, nothing touched. */
	stub_can1.FMR = 27u << CAN_FMR_CAN2SB_Pos;
	assert(can_set_filters(can, fl, 1) == -ENOSPC);
	hcan.Instance = CAN1;
	stub_can1.FMR = 1u << CAN_FMR_CAN2SB_Pos;
	assert(can_set_filters(can, fl, 1) == -ENOSPC);
	assert(stub_can1.FA1R == ((1u << 0) | (1u << 14)));
	down(d);
}

static void test_recover(void)
{
	bxcan_handle_t d = up(false);
	struct can_dev *can = bxcan_can(d);
	struct can_status st;
	uint32_t t0;

	assert(can_recover(can) == -EBUSY);	/* not in bus-off */
	assert(can_stop(can) == 0);
	assert(can_recover(can) == -ENETDOWN);
	assert(can_start(can) == 0);

	/* Automatic bus-off management: the controller is on it already. */
	stub_can1.ESR = CAN_ESR_BOFF;
	stub_can1.MCR = CAN_MCR_ABOM;
	assert(can_get_status(can, &st) == 0 &&
	       st.link == CAN_LINK_RECOVERING);
	assert(can_recover(can) == 0 && stub_can1.MCR == CAN_MCR_ABOM);

	/* Without: bus-off until asked, then INRQ set and cleared. */
	stub_can1.MCR = 0;
	assert(can_get_status(can, &st) == 0 && st.link == CAN_LINK_BUS_OFF);
	ack_init = true;
	assert(can_recover(can) == 0);
	/* INAK came, so INRQ was set; and it is clear again. */
	assert((stub_can1.MSR & CAN_MSR_INAK) &&
	       !(stub_can1.MCR & CAN_MCR_INRQ));
	ack_init = false;
	assert(can_get_status(can, &st) == 0 &&
	       st.link == CAN_LINK_RECOVERING);
	stub_can1.ESR = 0;
	assert(can_get_status(can, &st) == 0 && st.link == CAN_LINK_ACTIVE);
	stub_can1.ESR = CAN_ESR_BOFF;	/* a second bus-off */
	assert(can_get_status(can, &st) == 0 && st.link == CAN_LINK_BUS_OFF);

	/* No acknowledge: -ETIMEDOUT on the clock, INRQ cleared again. */
	stub_can1.MSR = 0;
	t0 = stub_tick;
	assert(can_recover(can) == -ETIMEDOUT && stub_tick - t0 >= 10);
	assert(!(stub_can1.MCR & CAN_MCR_INRQ));
	assert(can_get_status(can, &st) == 0 && st.link == CAN_LINK_BUS_OFF);
	stub_can1.ESR = 0;
	down(d);
}

int main(void)
{
	test_init();
	test_send();
	test_recv_polled();
	test_recv_irq();
	test_status();
	test_bitrate();
	test_opmode();
	test_filters();
	test_recover();
	printf("test_bxcan: all passed\n");
	return 0;
}
