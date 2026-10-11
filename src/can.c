#include <errno.h>
#include <string.h>

#ifdef ESP_PLATFORM
#include "esp_attr.h"
#define CAN_IRAM	IRAM_ATTR
#else
#define CAN_IRAM
#endif

#include "can.h"
#include "can_priv.h"

static bool can_bound(const struct can_dev *can)
{
	return can && can->ops && can->ops->send && can->ops->recv &&
	       can->ops->get_status;
}

static uint32_t can_id_max(bool ext)
{
	return ext ? CAN_FRAME_EXT_ID_MAX : CAN_FRAME_STD_ID_MAX;
}

int can_send(struct can_dev *can, const struct can_frame *frame,
	     uint32_t timeout_ms)
{
	if (!can_bound(can) || !frame || frame->dlc > CAN_FRAME_DATA_MAX ||
	    frame->id > can_id_max(frame->ext))
		return -EINVAL;
	return can->ops->send(can, frame, timeout_ms);
}

bool can_filter_pass(const struct can_filter *filters, size_t count,
		     const struct can_frame *frame)
{
	if (!count)
		return true;
	for (size_t i = 0; i < count; i++) {
		const struct can_filter *f = &filters[i];

		if (f->ext == frame->ext &&
		    ((frame->id ^ f->id) & f->mask) == 0)
			return true;
	}
	return false;
}

int can_recv(struct can_dev *can, struct can_frame *frame,
	     uint32_t timeout_ms)
{
	int64_t start = can_time_us();

	if (!can_bound(can) || !frame)
		return -EINVAL;
	for (;;) {
		struct can_frame tmp = { 0 };
		uint32_t elapsed = can_elapsed_ms(start);
		int ret;

		ret = can->ops->recv(can, &tmp,
				     elapsed >= timeout_ms ? 0 :
				     timeout_ms - elapsed);
		if (ret)
			return ret;
		if (!tmp.t_us)
			tmp.t_us = can_time_us();
		if (can_filter_pass(can->filters, can->filter_count, &tmp)) {
			*frame = tmp;
			return 0;
		}
		/* Dropped; it spent the time it took. */
		if (can_elapsed_ms(start) >= timeout_ms)
			return -ETIMEDOUT;
	}
}

int can_set_filters(struct can_dev *can, const struct can_filter *filters,
		    size_t count)
{
	int ret;

	if (!can_bound(can) || (count && !filters))
		return -EINVAL;
	if (!can->ops->set_filters)
		return -EOPNOTSUPP;
	if (count > CAN_FILTER_MAX)
		return -ENOSPC;
	for (size_t i = 0; i < count; i++)
		if (filters[i].id > can_id_max(filters[i].ext) ||
		    filters[i].mask > can_id_max(filters[i].ext))
			return -EINVAL;

	ret = can->ops->set_filters(can, filters, count);
	if (ret)
		return ret;
	if (count)
		memcpy(can->filters, filters, count * sizeof(*filters));
	can->filter_count = (uint8_t)count;
	return 0;
}

int can_set_bitrate(struct can_dev *can, uint32_t bitrate)
{
	if (!can_bound(can) || !bitrate)
		return -EINVAL;
	if (!can->ops->set_bitrate)
		return -EOPNOTSUPP;
	return can->ops->set_bitrate(can, bitrate);
}

int can_set_opmode(struct can_dev *can, enum can_opmode mode)
{
	if (!can_bound(can) || (mode != CAN_OPMODE_NORMAL &&
				mode != CAN_OPMODE_LOOPBACK &&
				mode != CAN_OPMODE_LISTEN))
		return -EINVAL;
	if (!can->ops->set_opmode)
		return -EOPNOTSUPP;
	return can->ops->set_opmode(can, mode);
}

int can_get_status(struct can_dev *can, struct can_status *status)
{
	struct can_status tmp = { 0 };
	int ret;

	if (!can_bound(can) || !status)
		return -EINVAL;
	ret = can->ops->get_status(can, &tmp);
	if (ret)
		return ret;
	*status = tmp;
	return 0;
}

int can_recover(struct can_dev *can)
{
	if (!can_bound(can))
		return -EINVAL;
	if (!can->ops->recover)
		return -EOPNOTSUPP;
	return can->ops->recover(can);
}

int can_start(struct can_dev *can)
{
	if (!can_bound(can))
		return -EINVAL;
	if (!can->ops->start)
		return -EOPNOTSUPP;
	return can->ops->start(can);
}

int can_stop(struct can_dev *can)
{
	if (!can_bound(can))
		return -EINVAL;
	if (!can->ops->stop)
		return -EOPNOTSUPP;
	return can->ops->stop(can);
}

/* A count, not a flag: a flag cleared by the thread could lose an edge
 * that landed between its look and its clear. In IRAM on ESP-IDF, for an
 * ISR that runs with flash off; it calls nothing. */
CAN_IRAM void can_irq(struct can_dev *can)
{
	if (can)
		can->irq_count++;
}

bool can_irq_pending(struct can_dev *can)
{
	uint32_t now;

	if (!can)
		return false;
	now = can->irq_count;
	if (now == can->irq_seen)
		return false;
	can->irq_seen = now;
	return true;
}

uint32_t can_elapsed_ms(int64_t start)
{
	int64_t us = can_time_us() - start;

	if (us <= 0)
		return 0;
	return us / 1000 > UINT32_MAX ? UINT32_MAX : (uint32_t)(us / 1000);
}

/* ------------------------------------------------------------- ring -- */

void can_ring_init(struct can_ring *ring)
{
	memset(ring, 0, sizeof(*ring));
}

/*
 * One writer and one reader, each owning one index: the writer fills the
 * slot before it publishes head, the reader copies it out before it
 * publishes tail. Acquire and release order that on any core.
 */
bool can_ring_put(struct can_ring *ring, const struct can_frame *frame)
{
	uint32_t head = ring->head;
	uint32_t tail = __atomic_load_n(&ring->tail, __ATOMIC_ACQUIRE);

	if (head - tail >= CAN_RING_LEN) {
		ring->dropped++;
		return false;
	}
	ring->slot[head & (CAN_RING_LEN - 1)] = *frame;
	__atomic_store_n(&ring->head, head + 1, __ATOMIC_RELEASE);
	return true;
}

bool can_ring_get(struct can_ring *ring, struct can_frame *out)
{
	uint32_t tail = ring->tail;
	uint32_t head = __atomic_load_n(&ring->head, __ATOMIC_ACQUIRE);

	if (head == tail)
		return false;
	*out = ring->slot[tail & (CAN_RING_LEN - 1)];
	__atomic_store_n(&ring->tail, tail + 1, __ATOMIC_RELEASE);
	return true;
}

uint32_t can_ring_count(const struct can_ring *ring)
{
	return __atomic_load_n(&ring->head, __ATOMIC_ACQUIRE) - ring->tail;
}
