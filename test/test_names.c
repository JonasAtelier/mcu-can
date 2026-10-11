/*
 * The coexistence check, at compile time: one translation unit with every
 * header an ESP-IDF app holding both esp-sn65hvd230-can and mcu-can may
 * include together -- hvd230can.h (fixtures/, a verbatim copy of
 * esp-sn65hvd230-can 28593dc), can.h, can_log.h, mcp2515.h, twai_can.h
 * and the legacy driver/twai.h they sit on. A type, tag, enumerator or
 * prototype both define fails the build, and so, under -Werror, does a
 * macro defined twice differently. The link-time half -- no global symbol
 * defined by both libraries -- is checked on a real firmware build with nm.
 */
#include <stdio.h>

#include "hvd230can.h"

#include "can.h"
#include "can_log.h"
#include "mcp2515.h"
#include "twai_can.h"

#include "driver/twai.h"

int main(void)
{
	/* One of each side's types, so each header is used, not skipped. */
	hvd230_t hvd;
	can_frame_t old_frame = { .id = 1 };
	can_sta sta = CAN_OK;
	can_state state = CAN_STATE_RUNNING;
	struct can_frame frame = { .id = 2 };
	struct can_status status = { .link = CAN_LINK_ACTIVE };
	enum can_opmode mode = CAN_OPMODE_NORMAL;
	mcp2515_frame_t chip_frame = { .id = 3 };
	twaican_config_t twai = { .rs_gpio = TWAICAN_NO_GPIO };
	twai_message_t msg = { .identifier = 4 };

	(void)hvd;
	printf("test_names: %u %u %d %d %u %d %u %d %d %u: no clash\n",
	       (unsigned)old_frame.id, (unsigned)frame.id, sta, state,
	       (unsigned)chip_frame.id, (int)status.link, (unsigned)mode,
	       twai.rs_gpio, CAN_MAX_DLC + CAN_FRAME_DATA_MAX,
	       (unsigned)msg.identifier);
	return 0;
}
