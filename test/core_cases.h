/*
 * The core's cases -- frame checks, the receive timestamp, filters in
 * software on top of the chip's, the optional ops, status, the IRQ count,
 * the frame ring and the log -- shared by esp/test_core.c and
 * stm/test_core.c. Each includes this after providing:
 *
 *   LOG_PREFIX             what the platform's log puts before "can: "
 *   stub_log_last          the last log line
 *   now_us(), set_now_us() the platform's clock behind can_time_us()
 *   pass_ms(ms)            let time pass, as a wait for a frame would
 */

/* A queue of frames the fake chip hands out, one per recv. */
static struct can_frame queue[16];
static int queued, served;
static int fake_ret, recvs;
static uint32_t last_recv_timeout;
static struct can_frame sent;
static uint32_t sent_timeout;
static int sends;

static int fake_send(struct can_dev *c, const struct can_frame *f,
		     uint32_t timeout_ms)
{
	(void)c;
	sent = *f;
	sent_timeout = timeout_ms;
	sends++;
	return fake_ret;
}

static int fake_recv(struct can_dev *c, struct can_frame *f,
		     uint32_t timeout_ms)
{
	(void)c;
	recvs++;
	last_recv_timeout = timeout_ms;
	if (fake_ret)
		return fake_ret;
	if (served == queued) {
		pass_ms(timeout_ms);		/* nothing came */
		return -ETIMEDOUT;
	}
	pass_ms(3);
	*f = queue[served++];
	return 0;
}

static struct can_status fake_status;

static int fake_get_status(struct can_dev *c, struct can_status *s)
{
	(void)c;
	/* The core hands over a zeroed struct; a chip fills what it has. */
	assert(s->tec == 0 && s->rx_lost == 0 && s->link == CAN_LINK_STOPPED);
	if (fake_ret)
		return fake_ret;
	s->link = fake_status.link;
	s->tec = fake_status.tec;
	return 0;
}

static int filter_calls, filter_ret;
static size_t filter_count_seen;

static int fake_set_filters(struct can_dev *c, const struct can_filter *f,
			    size_t count)
{
	(void)c;
	(void)f;
	filter_calls++;
	filter_count_seen = count;
	return filter_ret;
}

static uint32_t bitrate_seen;
static enum can_opmode opmode_seen;
static int recovers, starts, stops;

static int fake_set_bitrate(struct can_dev *c, uint32_t b)
{
	(void)c;
	bitrate_seen = b;
	return 0;
}

static int fake_set_opmode(struct can_dev *c, enum can_opmode m)
{
	(void)c;
	opmode_seen = m;
	return 0;
}

static int fake_recover(struct can_dev *c)
{
	(void)c;
	recovers++;
	return 0;
}

static int fake_start(struct can_dev *c)
{
	(void)c;
	starts++;
	return 0;
}

static int fake_stop(struct can_dev *c)
{
	(void)c;
	stops++;
	return 0;
}

static const struct can_ops full_ops = {
	.send = fake_send,
	.recv = fake_recv,
	.get_status = fake_get_status,
	.set_filters = fake_set_filters,
	.set_bitrate = fake_set_bitrate,
	.set_opmode = fake_set_opmode,
	.recover = fake_recover,
	.start = fake_start,
	.stop = fake_stop,
};

static const struct can_ops bare_ops = {
	.send = fake_send,
	.recv = fake_recv,
	.get_status = fake_get_status,
};

static struct can_dev dev;

static void reset(const struct can_ops *ops)
{
	memset(&dev, 0, sizeof(dev));
	dev.name = "fake";
	dev.ops = ops;
	queued = served = 0;
	fake_ret = filter_ret = 0;
	recvs = sends = filter_calls = 0;
	recovers = starts = stops = 0;
	set_now_us(1000000);
}

static struct can_frame std_frame(uint32_t id)
{
	return (struct can_frame){ .id = id, .dlc = 2, .data = { 1, 2 } };
}

static struct can_frame ext_frame(uint32_t id)
{
	return (struct can_frame){ .id = id, .ext = true, .dlc = 1 };
}

static void test_bound(void)
{
	struct can_dev none = { .name = "none" };
	struct can_ops half = { .send = fake_send, .recv = fake_recv };
	struct can_frame f = std_frame(1);
	struct can_status s;

	reset(&full_ops);
	assert(can_send(NULL, &f, 0) == -EINVAL);
	assert(can_recv(NULL, &f, 0) == -EINVAL);
	assert(can_send(&dev, NULL, 0) == -EINVAL);
	assert(can_recv(&dev, NULL, 0) == -EINVAL);
	assert(can_get_status(&dev, NULL) == -EINVAL);
	assert(can_send(&none, &f, 0) == -EINVAL);
	/* get_status is required, as send and recv are. */
	none.ops = &half;
	assert(can_get_status(&none, &s) == -EINVAL);
	assert(can_start(&none) == -EINVAL);
	assert(sends == 0 && recvs == 0);
}

static void test_send_checks(void)
{
	struct can_frame f;

	reset(&full_ops);
	f = std_frame(CAN_FRAME_STD_ID_MAX);
	assert(can_send(&dev, &f, 7) == 0);
	assert(sends == 1 && sent.id == 0x7FF && sent_timeout == 7);
	f.id = CAN_FRAME_STD_ID_MAX + 1;
	assert(can_send(&dev, &f, 0) == -EINVAL);

	f = ext_frame(CAN_FRAME_EXT_ID_MAX);
	assert(can_send(&dev, &f, 0) == 0 && sent.ext);
	f.id = CAN_FRAME_EXT_ID_MAX + 1;
	assert(can_send(&dev, &f, 0) == -EINVAL);

	f = std_frame(1);
	f.dlc = CAN_FRAME_DATA_MAX;
	assert(can_send(&dev, &f, 0) == 0);
	f.dlc = CAN_FRAME_DATA_MAX + 1;
	assert(can_send(&dev, &f, 0) == -EINVAL);
	assert(sends == 3);

	/* The chip's error comes back as it is. */
	f.dlc = 0;
	fake_ret = -EIO;
	assert(can_send(&dev, &f, 0) == -EIO);
}

static void test_recv_stamp(void)
{
	struct can_frame f;

	reset(&full_ops);
	queue[queued++] = std_frame(0x10);
	queue[queued] = std_frame(0x11);
	queue[queued++].t_us = 42;		/* stamped by the chip */

	assert(can_recv(&dev, &f, 100) == 0);
	assert(f.id == 0x10 && f.t_us == now_us());
	assert(can_recv(&dev, &f, 100) == 0);
	assert(f.id == 0x11 && f.t_us == 42);

	/* Nothing: the chip's -ETIMEDOUT, and the frame untouched. */
	f.id = 0x99;
	assert(can_recv(&dev, &f, 20) == -ETIMEDOUT && f.id == 0x99);
	assert(last_recv_timeout == 20);

	fake_ret = -EIO;
	assert(can_recv(&dev, &f, 20) == -EIO);
}

static void test_filter_pass(void)
{
	struct can_filter fl[2] = {
		{ .id = 0x120, .mask = 0x7F0 },
		{ .id = 0x18FF0000, .mask = 0x1FFF0000, .ext = true },
	};
	struct can_frame f;

	f = std_frame(0x12A);
	assert(can_filter_pass(fl, 0, &f));
	assert(can_filter_pass(fl, 1, &f));
	f = std_frame(0x13A);
	assert(!can_filter_pass(fl, 2, &f));
	/* Same low bits, other format: not a match. */
	f = ext_frame(0x12A);
	assert(!can_filter_pass(fl, 1, &f));
	f = ext_frame(0x18FF1234);
	assert(can_filter_pass(fl, 2, &f));
	f = ext_frame(0x18FE1234);
	assert(!can_filter_pass(fl, 2, &f));
	/* Mask 0: every frame of its format. */
	fl[0].mask = 0;
	f = std_frame(0x7FF);
	assert(can_filter_pass(fl, 1, &f));
	f = ext_frame(0x7FF);
	assert(!can_filter_pass(fl, 1, &f));
}

static void test_filters(void)
{
	struct can_filter fl[3] = {
		{ .id = 0x100, .mask = 0x7FF },
		{ .id = 0x200, .mask = 0x7FF },
		{ .id = 0x300, .mask = 0x7FF },
	};
	struct can_filter bad;
	struct can_frame f;

	reset(&full_ops);
	assert(can_set_filters(&dev, NULL, 1) == -EINVAL);
	assert(can_set_filters(&dev, fl, CAN_FILTER_MAX + 1) == -ENOSPC);
	bad = (struct can_filter){ .id = 0x800, .mask = 0x7FF };
	assert(can_set_filters(&dev, &bad, 1) == -EINVAL);
	bad = (struct can_filter){ .id = 0x1, .mask = 0xFFF };
	assert(can_set_filters(&dev, &bad, 1) == -EINVAL);
	bad = (struct can_filter){ .id = 0x1, .mask = 0x3FFFFFFF, .ext = true };
	assert(can_set_filters(&dev, &bad, 1) == -EINVAL);
	assert(filter_calls == 0);

	assert(can_set_filters(&dev, fl, 2) == 0);
	assert(filter_calls == 1 && filter_count_seen == 2);
	assert(dev.filter_count == 2 && dev.filters[1].id == 0x200);

	/* What the chip let through but the filters do not is dropped. */
	queue[queued++] = std_frame(0x300);
	queue[queued++] = ext_frame(0x100);
	queue[queued++] = std_frame(0x200);
	assert(can_recv(&dev, &f, 100) == 0 && f.id == 0x200 && !f.ext);
	assert(served == 3);

	/* Dropped frames count against the timeout: 3 ms each here. */
	queued = served = 0;
	for (int i = 0; i < 10; i++)
		queue[queued++] = std_frame(0x300);
	assert(can_recv(&dev, &f, 7) == -ETIMEDOUT);
	assert(served == 3);
	/* 0 polls: one look, then -ETIMEDOUT behind a dropped one. */
	served = 0;
	assert(can_recv(&dev, &f, 0) == -ETIMEDOUT && served == 1);

	/* A chip that refuses keeps the old filters. */
	filter_ret = -ENOSPC;
	assert(can_set_filters(&dev, fl, 1) == -ENOSPC);
	assert(dev.filter_count == 2);
	filter_ret = 0;

	/* 0 accepts all again. */
	assert(can_set_filters(&dev, NULL, 0) == 0 && dev.filter_count == 0);
	queued = served = 0;
	queue[queued++] = ext_frame(0x1234);
	assert(can_recv(&dev, &f, 0) == 0 && f.id == 0x1234);

	reset(&bare_ops);
	assert(can_set_filters(&dev, fl, 1) == -EOPNOTSUPP);
	assert(dev.filter_count == 0);
}

static void test_optional_ops(void)
{
	reset(&full_ops);
	assert(can_set_bitrate(&dev, 0) == -EINVAL);
	assert(can_set_bitrate(&dev, 250000) == 0 && bitrate_seen == 250000);
	assert(can_set_opmode(&dev, (enum can_opmode)7) == -EINVAL);
	assert(can_set_opmode(&dev, CAN_OPMODE_LISTEN) == 0 &&
	       opmode_seen == CAN_OPMODE_LISTEN);
	assert(can_recover(&dev) == 0 && recovers == 1);
	assert(can_start(&dev) == 0 && starts == 1);
	assert(can_stop(&dev) == 0 && stops == 1);

	reset(&bare_ops);
	assert(can_set_bitrate(&dev, 250000) == -EOPNOTSUPP);
	assert(can_set_opmode(&dev, CAN_OPMODE_NORMAL) == -EOPNOTSUPP);
	assert(can_recover(&dev) == -EOPNOTSUPP);
	assert(can_start(&dev) == -EOPNOTSUPP);
	assert(can_stop(&dev) == -EOPNOTSUPP);
	assert(can_recover(NULL) == -EINVAL);
}

static void test_status(void)
{
	struct can_status s;

	reset(&full_ops);
	fake_status = (struct can_status){ .link = CAN_LINK_PASSIVE,
					   .tec = 130 };
	memset(&s, 0xA5, sizeof(s));
	assert(can_get_status(&dev, &s) == 0);
	assert(s.link == CAN_LINK_PASSIVE && s.tec == 130 && s.rec == 0 &&
	       s.bus_errors == 0);

	/* A failed read leaves the caller's struct alone. */
	fake_ret = -EIO;
	s.tec = 7;
	assert(can_get_status(&dev, &s) == -EIO && s.tec == 7);
}

static void test_irq(void)
{
	reset(&full_ops);
	assert(!can_irq_pending(&dev));
	can_irq(&dev);
	can_irq(&dev);
	assert(can_irq_pending(&dev));
	assert(!can_irq_pending(&dev));
	can_irq(&dev);
	assert(can_irq_pending(&dev));
	can_irq(NULL);
	assert(!can_irq_pending(NULL));
}

static void test_ring(void)
{
	struct can_ring r;
	struct can_frame f;

	can_ring_init(&r);
	assert(!can_ring_get(&r, &f) && can_ring_count(&r) == 0);
	for (uint32_t i = 0; i < CAN_RING_LEN; i++) {
		f = std_frame(i);
		assert(can_ring_put(&r, &f));
	}
	f = std_frame(0x7FF);
	assert(!can_ring_put(&r, &f) && r.dropped == 1);
	assert(can_ring_count(&r) == CAN_RING_LEN);
	assert(can_ring_get(&r, &f) && f.id == 0);
	/* One taken, one more fits -- in the slot just freed. */
	f = std_frame(0x55);
	assert(can_ring_put(&r, &f));
	for (uint32_t i = 1; i < CAN_RING_LEN; i++)
		assert(can_ring_get(&r, &f) && f.id == i);
	assert(can_ring_get(&r, &f) && f.id == 0x55);
	assert(!can_ring_get(&r, &f));

	/* Free-running indexes wrap. */
	r.head = r.tail = UINT32_MAX;
	f = std_frame(1);
	assert(can_ring_put(&r, &f) && can_ring_put(&r, &f));
	assert(can_ring_count(&r) == 2);
	assert(can_ring_get(&r, &f) && can_ring_get(&r, &f));
	assert(can_ring_count(&r) == 0);
}

static void test_elapsed(void)
{
	set_now_us(5000000);
	assert(can_elapsed_ms(now_us()) == 0);
	assert(can_elapsed_ms(now_us() + 1000) == 0);	/* the future */
	pass_ms(12);
	assert(can_elapsed_ms(5000000) == 12);
}

static void test_log(void)
{
	struct can_frame f = std_frame(0x12A);

	reset(&full_ops);
	f.t_us = 1234567;
	can_log_frame(&dev, &f);
	assert(!strcmp(stub_log_last, LOG_PREFIX("I")
		       "can: fake: t 1234567 us  std 12A  [2] 01 02"));

	f = ext_frame(0x18FF00A1);
	f.data[0] = 0xEF;
	f.t_us = -5;
	can_log_frame(&dev, &f);
	assert(!strcmp(stub_log_last, LOG_PREFIX("I")
		       "can: fake: t -5 us  ext 18FF00A1  [1] EF"));

	f.rtr = true;
	f.dlc = 4;
	can_log_frame(&dev, &f);
	assert(strstr(stub_log_last, "[4] remote") &&
	       !strstr(stub_log_last, "EF"));
}

static void run_core_cases(void)
{
	test_bound();
	test_send_checks();
	test_recv_stamp();
	test_filter_pass();
	test_filters();
	test_optional_ops();
	test_status();
	test_irq();
	test_ring();
	test_elapsed();
	test_log();
}
