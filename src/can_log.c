#include <stdint.h>

#include "can_log.h"

/*
 * Decimal text of v, written backwards from end. newlib-nano's printf, the
 * one stm-forge links, has no %lld, so the timestamp cannot go through PRId64.
 */
static const char *can_format_i64(char *end, int64_t v)
{
	uint64_t u = v < 0 ? 0 - (uint64_t)v : (uint64_t)v;

	*--end = '\0';
	do {
		*--end = (char)('0' + u % 10);
		u /= 10;
	} while (u);
	if (v < 0)
		*--end = '-';
	return end;
}

void can_log_frame(const struct can_dev *can, const struct can_frame *frame)
{
	char t[21];	/* "-9223372036854775808" and its NUL */
	char data[3 * CAN_FRAME_DATA_MAX + 1] = "";
	static const char hex[] = "0123456789ABCDEF";
	unsigned int n = frame->dlc > CAN_FRAME_DATA_MAX ? CAN_FRAME_DATA_MAX :
							     frame->dlc;

	if (!frame->rtr)
		for (unsigned int i = 0; i < n; i++) {
			data[3 * i] = ' ';
			data[3 * i + 1] = hex[frame->data[i] >> 4];
			data[3 * i + 2] = hex[frame->data[i] & 0xF];
			data[3 * i + 3] = '\0';
		}
	can_info(can, "t %s us  %s %0*lX  [%u]%s%s",
		 can_format_i64(t + sizeof(t), frame->t_us),
		 frame->ext ? "ext" : "std", frame->ext ? 8 : 3,
		 (unsigned long)frame->id, (unsigned)frame->dlc,
		 frame->rtr ? " remote" : "", data);
}
