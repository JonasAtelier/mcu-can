/*
 * bxCAN on an STM32F4: receive on the FIFO 0 interrupt, send a counter once
 * a second.
 *
 * A snippet, not a project: the clock tree, the printf() retarget and the
 * peripheral are the app's -- here hcan1 as CubeMX generates it
 * (MX_CAN1_Init()) with its bit timing, CAN_MODE_NORMAL and AutoBusOff
 * enabled, a filter that sends everything to FIFO 0, and CAN1_RX0 enabled
 * in the NVIC with HAL_CAN_IRQHandler() in its handler. Needs
 * HAL_CAN_MODULE_ENABLED, stm32f4xx_hal_can.c in the build, and
 * CAN_BXCAN = y.
 */
#include <stdio.h>

#include CMSIS_device_header

#include "bxcan.h"
#include "can.h"
#include "can_log.h"

extern CAN_HandleTypeDef hcan1;		/* from MX_CAN1_Init() */

static bxcan_handle_t bx;

/*
 * All IDs into FIFO 0: one 32-bit mask filter, ID and mask 0. On a part
 * with two CANs the banks are shared, and those below SlaveStartFilterBank
 * belong to CAN1 (stm32f4xx_hal_can.h:143-148), so bank 0 needs it above
 * 0; 14 gives each CAN half of the 28 banks (:131-132).
 */
static void accept_all(void)
{
	CAN_FilterTypeDef f = {
		.FilterBank = 0,
		.FilterMode = CAN_FILTERMODE_IDMASK,
		.FilterScale = CAN_FILTERSCALE_32BIT,
		.FilterFIFOAssignment = CAN_RX_FIFO0,
		.FilterActivation = CAN_FILTER_ENABLE,
		.SlaveStartFilterBank = 14,
	};

	HAL_CAN_ConfigFilter(&hcan1, &f);
}

void can_example_init(void)
{
	bxcan_config_t cfg = { .hcan = &hcan1, .use_irq = true };
	int err;

	accept_all();
	err = bxcan_init(&cfg, &bx);
	if (err)
		printf("bxcan_init: %d\n", err);
}

/* The HAL calls it from HAL_CAN_IRQHandler(). */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan == &hcan1)
		bxcan_irq(bx);
}

/* From the main loop, once a second. */
void can_example_poll(void)
{
	static uint32_t counter;
	struct can_dev *can = bxcan_can(bx);
	struct can_frame tx = {
		.id = 0x200,
		.dlc = 4,
		.data = { (uint8_t)(counter >> 24), (uint8_t)(counter >> 16),
			  (uint8_t)(counter >> 8), (uint8_t)counter },
	};
	struct can_frame rx;
	struct can_status st;

	if (!can)
		return;
	if (can_send(can, &tx, 10) == 0)
		counter++;
	while (can_recv(can, &rx, 0) == 0)
		can_log_frame(can, &rx);
	if (can_get_status(can, &st) == 0 && st.link != CAN_LINK_ACTIVE)
		printf("bxcan: link %d tec %u rec %u lost %u\n", (int)st.link,
		       st.tec, st.rec, (unsigned)st.rx_lost);
}
