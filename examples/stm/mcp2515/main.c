/*
 * MCP2515 on an STM32: send a counter once a second and print what
 * arrives.
 *
 * A snippet, not a project: the clock tree, the printf() retarget, the SPI
 * and the GPIOs are the app's -- here hspi1 as CubeMX generates it
 * (MX_SPI1_Init()): master, mode 0 (CPOL low, CPHA 1 edge), 8-bit, 10 MHz
 * or less (DS Section 12.1); PA4 a push-pull output for CS, idle high; PB0
 * an input with pull-up for nINT. Needs HAL_SPI_MODULE_ENABLED and
 * CAN_MCP2515 = y.
 *
 * CRYSTAL_HZ must match the crystal next to the chip; see docs/hardware.md.
 */
#include <stdio.h>

#include CMSIS_device_header

#include "can.h"
#include "can_log.h"
#include "mcp2515.h"

#define CRYSTAL_HZ  8000000

extern SPI_HandleTypeDef hspi1;		/* from MX_SPI1_Init() */

static mcp2515_handle_t chip;

void can_example_init(void)
{
	mcp2515_config_t cfg = {
		.spi = &hspi1,
		.cs_port = GPIOA,
		.cs_pin = GPIO_PIN_4,
		.int_port = GPIOB,
		.int_pin = GPIO_PIN_0,
		.crystal_hz = CRYSTAL_HZ,
		.bitrate = 500000,
	};
	int err = mcp2515_init(&cfg, &chip);

	if (err)
		printf("mcp2515_init: %d\n", err);
}

/* From the main loop, once a second. */
void can_example_poll(void)
{
	static uint32_t counter;
	struct can_dev *can = mcp2515_can(chip);
	struct can_frame tx = {
		.id = 0x123,
		.dlc = 4,
		.data = { (uint8_t)(counter >> 24), (uint8_t)(counter >> 16),
			  (uint8_t)(counter >> 8), (uint8_t)counter },
	};
	struct can_frame rx;

	if (!can)
		return;
	if (can_send(can, &tx, 100) == 0)
		counter++;
	while (can_recv(can, &rx, 0) == 0)
		can_log_frame(can, &rx);
}
