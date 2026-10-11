/*
 * Host-side check of the bxCAN driver against a fake of the HAL's CAN
 * module (hal_stubs/stm32_hal_stub.h): start and stop between READY and
 * LISTENING, three transmit mailboxes, receive FIFO 0 with its fill level,
 * the FIFO 0 message-pending notification, and the error status register
 * the driver reads TEC, REC and the error flags from.
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

static CAN_TypeDef regs;
static CAN_HandleTypeDef hcan = { .Instance = &regs };

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
static uint32_t notifications;
static int starts, stops;

HAL_CAN_StateTypeDef HAL_CAN_GetState(const CAN_HandleTypeDef *h)
{
	return h->State;
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

HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *h)
{
	assert(h == &hcan);
	if (h->State != HAL_CAN_STATE_LISTENING)
		return HAL_ERROR;
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
	regs = (CAN_TypeDef){ 0 };
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

	regs.ESR = (100u << CAN_ESR_TEC_Pos) | (7u << CAN_ESR_REC_Pos) |
		   CAN_ESR_EWGF;
	free_mailboxes = 1;
	bus_frame(1, false, false, 0);
	assert(can_get_status(can, &st) == 0);
	assert(st.link == CAN_LINK_WARNING && st.tec == 100 && st.rec == 7);
	assert(st.tx_pending == 2 && st.rx_pending == 1 && st.rx_lost == 0);
	regs.ESR |= CAN_ESR_EPVF;
	assert(can_get_status(can, &st) == 0 && st.link == CAN_LINK_PASSIVE);
	regs.ESR |= CAN_ESR_BOFF;
	assert(can_get_status(can, &st) == 0 && st.link == CAN_LINK_BUS_OFF);
	regs.ESR = 0;
	regs.RF0R = CAN_RF0R_FOVR0;
	assert(can_get_status(can, &st) == 0 && st.link == CAN_LINK_ACTIVE);
	assert(st.rx_lost == 1);

	/* What the app's Init owns is not this driver's to change. */
	{
		struct can_filter fl = { .id = 1, .mask = 0x7FF };

		assert(can_set_filters(can, &fl, 1) == -EOPNOTSUPP);
		assert(can_set_bitrate(can, 500000) == -EOPNOTSUPP);
		assert(can_set_opmode(can, CAN_OPMODE_LOOPBACK) == -EOPNOTSUPP);
		assert(can_recover(can) == -EOPNOTSUPP);
	}

	/* Start and stop are the HAL's. */
	assert(can_stop(can) == 0 && hcan.State == HAL_CAN_STATE_READY);
	assert(can_get_status(can, &st) == 0 && st.link == CAN_LINK_STOPPED);
	assert(can_stop(can) == -EIO);
	assert(can_start(can) == 0 && hcan.State == HAL_CAN_STATE_LISTENING);
	down(d);
}

int main(void)
{
	test_init();
	test_send();
	test_recv_polled();
	test_recv_irq();
	test_status();
	printf("test_bxcan: all passed\n");
	return 0;
}
