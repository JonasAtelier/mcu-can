/*
 * A fake MCP2515 at SPI level, and this library's cases against it, shared
 * by esp/test_mcp2515.c and stm/test_mcp2515.c. The fake keeps the register
 * file and models what the driver relies on (DS20001801J): the instruction
 * set (Table 12-1) with READ and WRITE auto-incrementing, BIT MODIFY,
 * READ STATUS (Figure 12-8), LOAD TX BUFFER and RTS (Figures 12-5, 12-6),
 * READ RX BUFFER clearing its RXnIF when CS rises (Section 12.4); RESET to
 * Configuration mode with CANCTRL 0x87 (Register 10-1); CANSTAT and
 * CANCTRL at every xEh/xFh (Table 11-1); a requested mode that may take a
 * few looks to show (Section 10.0); CNF, filter and mask registers
 * writable only in Configuration mode, which clears TEC and REC (Section
 * 10.1); acceptance by RXM, mask and filter with EXIDE (Section 4.5,
 * Registers 4-1, 4-11), RXB0 rolling over into RXB1 with BUKT, and RXnOVR
 * when neither takes a frame (Register 6-3); nINT low while an enabled
 * flag is set (Section 7.0); loopback handing sent frames straight back.
 * Each platform file feeds it from its own SPI calls -- spi_fails() first,
 * then fake_spi() -- and provides:
 *
 *   RET(err)               what the handle API returns for a negative errno
 *   cfg_of(int_wired)      a config on that platform's SPI bus, 8 MHz, 500k
 *   spi_live()             SPI devices the driver has attached and not removed
 *   SPI_LIVE_UP            what spi_live() is with a handle up: 1, or 0 where
 *                          the port attaches nothing to count
 *   now_us(), pass_ms()    the clock behind can_time_us()
 */

#include "mcp2515_math.h"
#include "mcp2515_regs.h"

extern char stub_log_last[256];

/* ------------------------------------------------------- the fake chip --- */

struct sent_frame {
	uint8_t buf;		/* TXBn it went out of */
	uint8_t raw[13];	/* SIDH..D7 as loaded */
};

static struct {
	bool present;
	int miso;		/* what an absent chip reads as: 0x00 or 0xFF */
	uint8_t reg[128];
	int mode_lag;		/* CANSTAT looks before a new mode shows */
	int lag_left;
	bool mode_stuck;	/* never leaves Configuration mode */
	bool cnf_stuck;		/* CNF writes ignored */
	bool tx_hold;		/* RTS leaves TXREQ set: nothing goes out */
	struct sent_frame sent[16];
	int sent_count;
	int xfers, status_reads, resets;
	size_t last_load_len;	/* bytes after a LOAD TX BUFFER instruction */
	int config_writes_refused;
} chip;

static int spi_fail_at = -1;	/* fail the transfer with this number */

static bool spi_fails(void)
{
	return chip.xfers++ == spi_fail_at;
}

static uint8_t opmode(void)
{
	return chip.reg[MCP2515_CANSTAT] & MCP2515_MODE_MASK;
}

static void chip_reset(void)
{
	memset(chip.reg, 0, sizeof(chip.reg));
	chip.reg[MCP2515_CANCTRL] = 0x87;	/* Table 11-2 POR */
	chip.reg[MCP2515_CANSTAT] = MCP2515_MODE_CONFIG;
	chip.lag_left = 0;
	chip.resets++;
}

static void chip_power_on(void)
{
	memset(&chip, 0, sizeof(chip));
	chip.present = true;
	chip_reset();
	chip.resets = 0;
	spi_fail_at = -1;
}

/* Registers only Configuration mode may write (Section 10.1, 4.5.5). */
static bool config_only(uint8_t a)
{
	return (a <= 0x0B) || (a >= 0x10 && a <= 0x1B) ||
	       (a >= 0x20 && a <= 0x2A);
}

static uint8_t canstat_addr(uint8_t a)
{
	return (a & 0x0F) == 0x0E ? MCP2515_CANSTAT :
	       (a & 0x0F) == 0x0F ? MCP2515_CANCTRL : a;
}

static void request_mode(void)
{
	uint8_t want = chip.reg[MCP2515_CANCTRL] & MCP2515_MODE_MASK;

	if (want == opmode() || chip.mode_stuck)
		return;
	chip.lag_left = chip.mode_lag;
	if (chip.lag_left == 0) {
		if (want == MCP2515_MODE_CONFIG) {
			chip.reg[MCP2515_TEC] = 0;
			chip.reg[MCP2515_TEC + 1] = 0;
		}
		chip.reg[MCP2515_CANSTAT] =
			(chip.reg[MCP2515_CANSTAT] & ~MCP2515_MODE_MASK) | want;
	}
}

static uint8_t read_reg(uint8_t a)
{
	a = canstat_addr(a & 0x7F);
	if (a == MCP2515_CANSTAT && chip.lag_left && --chip.lag_left == 0) {
		uint8_t want = chip.reg[MCP2515_CANCTRL] & MCP2515_MODE_MASK;

		if (want == MCP2515_MODE_CONFIG)
			chip.reg[MCP2515_TEC] = chip.reg[MCP2515_TEC + 1] = 0;
		chip.reg[MCP2515_CANSTAT] = want;
	}
	return chip.reg[a];
}

static void write_reg(uint8_t a, uint8_t v)
{
	a = canstat_addr(a & 0x7F);
	if (a == MCP2515_CANSTAT)
		return;				/* read-only */
	if (config_only(a) && opmode() != MCP2515_MODE_CONFIG) {
		chip.config_writes_refused++;
		return;
	}
	if (a >= 0x28 && a <= 0x2A && chip.cnf_stuck)
		return;
	chip.reg[a] = v;
	if (a == MCP2515_CANCTRL)
		request_mode();
}

/* Does the 4-byte filter at f with the mask at m accept this frame? */
static bool accepts(uint8_t f, uint8_t m, const uint8_t *raw)
{
	bool ext = raw[1] & 0x08;

	if (((chip.reg[f + 1] & 0x08) != 0) != ext)	/* EXIDE */
		return false;
	for (int i = 0; i < 4; i++) {
		uint8_t mask = chip.reg[m + i];

		if (i == 1)
			mask &= 0xE3;			/* SID2:0, EID17:16 */
		if (!ext && i >= 2) {
			/* A standard frame's first data bytes (Register 4-1). */
			uint8_t d = raw[5 + (i - 2)];

			if ((d ^ chip.reg[f + i]) & mask)
				return false;
			continue;
		}
		if ((raw[i] ^ chip.reg[f + i]) & mask)
			return false;
	}
	return true;
}

static bool buffer_takes(int n, const uint8_t *raw)
{
	static const uint8_t rxf[6] = MCP2515_RXF_ADDRS;
	uint8_t ctrl = chip.reg[n ? MCP2515_RXB1CTRL : MCP2515_RXB0CTRL];

	if ((ctrl & 0x60) == 0x60)
		return true;
	for (int i = n ? 2 : 0; i < (n ? 6 : 2); i++)
		if (accepts(rxf[i], n ? MCP2515_RXM1SIDH : MCP2515_RXM0SIDH,
			    raw))
			return true;
	return false;
}

static void store(int n, const uint8_t *raw)
{
	memcpy(&chip.reg[n ? 0x71 : 0x61], raw, 13);
	chip.reg[MCP2515_CANINTF] |= n ? MCP2515_INT_RX1IF : MCP2515_INT_RX0IF;
}

/* A frame off the bus, raw as the RX buffers hold it. */
static void deliver_raw(const uint8_t *raw)
{
	bool full0 = chip.reg[MCP2515_CANINTF] & MCP2515_INT_RX0IF;
	bool full1 = chip.reg[MCP2515_CANINTF] & MCP2515_INT_RX1IF;

	if (opmode() == MCP2515_MODE_CONFIG)
		return;
	if (buffer_takes(0, raw)) {
		if (!full0)
			store(0, raw);
		else if ((chip.reg[MCP2515_RXB0CTRL] & MCP2515_RXB0_BUKT) &&
			 !full1)
			store(1, raw);
		else
			chip.reg[MCP2515_EFLG] |= MCP2515_EFLG_RX0OVR;
		return;
	}
	if (buffer_takes(1, raw)) {
		if (!full1)
			store(1, raw);
		else
			chip.reg[MCP2515_EFLG] |= MCP2515_EFLG_RX1OVR;
	}
}

static void deliver(uint32_t id, bool ext, bool rtr, uint8_t dlc,
		    const uint8_t *data)
{
	uint8_t raw[13] = { 0 };

	mcp2515_pack_id(id, ext, raw);
	if (rtr && !ext)
		raw[1] |= MCP2515_SIDL_SRR;
	raw[4] = (uint8_t)((rtr && ext ? MCP2515_DLC_RTR : 0) | dlc);
	if (data)
		memcpy(&raw[5], data, dlc > 8 ? 8 : dlc);
	deliver_raw(raw);
}

static void transmit(int n)
{
	uint8_t base = (uint8_t)(0x30 + 0x10 * n);
	struct sent_frame *s = &chip.sent[chip.sent_count++ % 16];

	s->buf = (uint8_t)n;
	memcpy(s->raw, &chip.reg[base + 1], 13);
	if (chip.tx_hold)
		return;
	chip.reg[base] &= ~0x08;		/* TXREQ */
	if (opmode() == MCP2515_MODE_LOOPBACK) {
		uint8_t raw[13];
		bool ext = s->raw[1] & 0x08;
		bool rtr = s->raw[4] & MCP2515_DLC_RTR;

		memcpy(raw, s->raw, 13);
		/* Received: a standard remote frame says so in SRR. */
		if (!ext) {
			raw[1] = (uint8_t)(raw[1] & ~MCP2515_SIDL_SRR) |
				 (rtr ? MCP2515_SIDL_SRR : 0);
			raw[4] &= (uint8_t)~MCP2515_DLC_RTR;
		}
		deliver_raw(raw);
	}
}

/* Let held frames go: TXREQ clear on all three buffers. */
static void release_tx(void)
{
	chip.tx_hold = false;
	for (int b = 0; b < 3; b++)
		chip.reg[0x30 + 0x10 * b] &= (uint8_t)~0x08;
}

static bool int_low(void)
{
	return chip.reg[MCP2515_CANINTE] & chip.reg[MCP2515_CANINTF];
}

/* One CS window: out[0] the instruction, in[] what came back. */
static void fake_spi(const uint8_t *out, uint8_t *in, size_t n)
{
	uint8_t cmd = out[0];

	memset(in, chip.present ? 0 : chip.miso, n);
	if (!chip.present)
		return;

	if (cmd == MCP2515_RESET) {
		chip_reset();
	} else if (cmd == MCP2515_READ) {
		for (size_t i = 2; i < n; i++)
			in[i] = read_reg((uint8_t)(out[1] + i - 2));
	} else if (cmd == MCP2515_WRITE) {
		for (size_t i = 2; i < n; i++)
			write_reg((uint8_t)(out[1] + i - 2), out[i]);
	} else if (cmd == MCP2515_BIT_MODIFY) {
		uint8_t a = canstat_addr(out[1] & 0x7F);
		uint8_t mask = out[2];

		assert(n == 4);
		if (a == MCP2515_EFLG)
			mask &= MCP2515_EFLG_OVR;	/* the writable bits */
		write_reg(a, (uint8_t)((chip.reg[a] & ~mask) | (out[3] & mask)));
	} else if (cmd == MCP2515_READ_STATUS) {
		uint8_t intf = chip.reg[MCP2515_CANINTF];
		uint8_t st = intf & 0x03;

		for (int b = 0; b < 3; b++) {
			if (chip.reg[0x30 + 0x10 * b] & 0x08)
				st |= (uint8_t)MCP2515_ST_TXREQ(b);
			if (intf & (0x04 << b))
				st |= (uint8_t)(0x08 << (2 * b));
		}
		chip.status_reads++;
		for (size_t i = 1; i < n; i++)
			in[i] = st;			/* repeats */
	} else if ((cmd & 0xF8) == MCP2515_LOAD_TX) {
		static const uint8_t start[6] = { 0x31, 0x36, 0x41, 0x46,
						  0x51, 0x56 };
		uint8_t a;

		assert((cmd & 0x07) < 6);
		a = start[cmd & 0x07];
		chip.last_load_len = n - 1;
		for (size_t i = 1; i < n; i++)
			chip.reg[a + i - 1] = out[i];
	} else if ((cmd & 0xF8) == MCP2515_RTS) {
		assert(n == 1);
		for (int b = 0; b < 3; b++)
			if (cmd & (1 << b)) {
				chip.reg[0x30 + 0x10 * b] |= 0x08;
				transmit(b);
			}
	} else if ((cmd & 0xF9) == MCP2515_READ_RX) {
		static const uint8_t start[4] = { 0x61, 0x66, 0x71, 0x76 };
		uint8_t a = start[(cmd >> 1) & 0x03];

		for (size_t i = 1; i < n; i++)
			in[i] = chip.reg[a + i - 1];
		/* Cleared when CS rises (Section 12.4). */
		chip.reg[MCP2515_CANINTF] &= (cmd & MCP2515_READ_RX1) ?
					     (uint8_t)~MCP2515_INT_RX1IF :
					     (uint8_t)~MCP2515_INT_RX0IF;
	} else {
		assert(!"unknown instruction");
	}
}

/* -------------------------------------------------------------- cases --- */

static int allocs, frees;

void *__real_calloc(size_t n, size_t size);
void __real_free(void *p);
static int calloc_fail;

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

static mcp2515_handle_t up(bool int_wired)
{
	mcp2515_config_t cfg = cfg_of(int_wired);
	mcp2515_handle_t d = NULL;

	chip_power_on();
	assert(mcp2515_init(&cfg, &d) == RET(0) && d);
	return d;
}

static void down(mcp2515_handle_t d)
{
	assert(mcp2515_del(d) == RET(0));
	assert(spi_live() == 0 && allocs == frees);
}

static void test_init(void)
{
	mcp2515_handle_t d = up(false);

	assert(chip.resets == 1);
	/* 8 MHz, 500 kbit/s: the table row, SOF clear. */
	assert(chip.reg[0x2A] == 0x00 && chip.reg[0x29] == 0x90 &&
	       chip.reg[0x28] == 0x02);
	assert(chip.reg[MCP2515_RXB0CTRL] == 0x64);
	assert(chip.reg[MCP2515_RXB1CTRL] == 0x60);
	assert(chip.reg[MCP2515_CANINTE] == 0);
	assert(opmode() == MCP2515_MODE_NORMAL);
	/* CANCTRL's other bits are left as reset left them: CLKEN, CLKPRE. */
	assert((chip.reg[MCP2515_CANCTRL] & 0x1F) == 0x07);
	assert(spi_live() == SPI_LIVE_UP);
	assert(!strcmp(mcp2515_can(d)->name, "mcp2515"));
	down(d);

	/* nINT wired: the two receive flags drive it. */
	d = up(true);
	assert(chip.reg[MCP2515_CANINTE] == 0x03);
	down(d);

	/* A 16 MHz crystal at 1 Mbit/s, in loopback. */
	{
		mcp2515_config_t cfg = cfg_of(false);

		chip_power_on();
		cfg.crystal_hz = 16000000;
		cfg.bitrate = 1000000;
		cfg.loopback = true;
		assert(mcp2515_init(&cfg, &d) == RET(0));
		assert(chip.reg[0x2A] == 0x00 && chip.reg[0x29] == 0xD0 &&
		       chip.reg[0x28] == 0x02);
		assert(opmode() == MCP2515_MODE_LOOPBACK);
		down(d);
	}
	{
		mcp2515_config_t cfg = cfg_of(false);

		chip_power_on();
		cfg.listen_only = true;
		assert(mcp2515_init(&cfg, &d) == RET(0));
		assert(opmode() == MCP2515_MODE_LISTEN);
		down(d);
	}

	/* The mode takes a few looks to show: the driver waits for it. */
	chip_power_on();
	chip.mode_lag = 2;		/* a second look, a tick later */
	{
		mcp2515_config_t cfg = cfg_of(false);

		assert(mcp2515_init(&cfg, &d) == RET(0));
		assert(opmode() == MCP2515_MODE_NORMAL);
		down(d);
	}
}

static void test_init_failures(void)
{
	mcp2515_config_t cfg = cfg_of(false);
	mcp2515_handle_t d = NULL;
	int64_t t0;

	allocs = frees = 0;
	assert(mcp2515_init(NULL, &d) == RET(-EINVAL));
	assert(mcp2515_init(&cfg, NULL) == RET(-EINVAL));

	/* Refused before the bus is touched: no table row, or both modes. */
	chip_power_on();
	cfg.bitrate = 1000000;			/* 8 MHz: PS2 would be 1 TQ */
	assert(mcp2515_init(&cfg, &d) == RET(-EINVAL));
	assert(strstr(stub_log_last, "no bit timing for a 8000000 Hz"));
	cfg.bitrate = 500000;
	cfg.crystal_hz = 12000000;
	assert(mcp2515_init(&cfg, &d) == RET(-EINVAL));
	cfg = cfg_of(false);
	cfg.loopback = cfg.listen_only = true;
	assert(mcp2515_init(&cfg, &d) == RET(-EINVAL));
	assert(chip.xfers == 0 && allocs == 0);

	/* Nothing there: MISO floating high or low. */
	cfg = cfg_of(false);
	for (int level = 0; level < 2; level++) {
		chip_power_on();
		chip.present = false;
		chip.miso = level ? 0xFF : 0x00;
		assert(mcp2515_init(&cfg, &d) == RET(-ENODEV));
		assert(strstr(stub_log_last, "no MCP2515: CANSTAT reads"));
		assert(spi_live() == 0 && allocs == frees);
	}

	/* The timing does not stick. */
	chip_power_on();
	chip.cnf_stuck = true;
	assert(mcp2515_init(&cfg, &d) == RET(-ENODEV));
	assert(strstr(stub_log_last, "bit timing did not stick"));
	assert(spi_live() == 0 && allocs == frees);

	/* It never leaves Configuration mode: -EPROTO, after the wait, which
	 * is on the clock (esp-mcp2515-can's wait was no wait at 100 Hz). */
	chip_power_on();
	chip.mode_stuck = true;
	t0 = now_us();
	assert(mcp2515_init(&cfg, &d) == RET(-EPROTO));
	assert(now_us() - t0 >= 10000 + 10000);	/* reset wait + mode wait */
	assert(strstr(stub_log_last, "would not leave configuration mode"));
	assert(spi_live() == 0 && allocs == frees);

	/* Every transfer of a good init, failed in turn. */
	chip_power_on();
	{
		int good;

		assert(mcp2515_init(&cfg, &d) == RET(0));
		good = chip.xfers;
		assert(mcp2515_del(d) == RET(0));
		for (int i = 0; i < good; i++) {
			chip_power_on();
			spi_fail_at = i;
			assert(mcp2515_init(&cfg, &d) != RET(0));
			assert(spi_live() == 0 && allocs == frees);
		}
	}

	chip_power_on();
	calloc_fail = 1;
	assert(mcp2515_init(&cfg, &d) == RET(-ENOMEM));
	calloc_fail = 0;
	assert(chip.xfers == 0);
}

static void test_send(void)
{
	mcp2515_handle_t d = up(false);
	mcp2515_frame_t f = { .id = 0x123, .len = 3, .data = { 1, 2, 3 } };
	int64_t t0;

	assert(mcp2515_send(d, &f, 0) == RET(0));
	assert(chip.sent_count == 1 && chip.sent[0].buf == 0);
	/* SIDH, SIDL, EID8, EID0, DLC, data (Registers 3-3 to 3-8). */
	assert(chip.sent[0].raw[0] == 0x24 && chip.sent[0].raw[1] == 0x60);
	assert(chip.sent[0].raw[4] == 3 && chip.sent[0].raw[7] == 3);

	f = (mcp2515_frame_t){ .id = 0x18FF00A1, .len = 0, .extended = true,
			       .remote = true };
	assert(mcp2515_send(d, &f, 0) == RET(0));
	{
		uint32_t id;
		bool ext;

		mcp2515_unpack_id(chip.sent[1].raw, &id, &ext);
		assert(id == 0x18FF00A1 && ext);
		assert(chip.sent[1].raw[4] == MCP2515_DLC_RTR);
		assert(chip.last_load_len == 5);
	}
	/* A remote frame's DLC is what it asks for; no data goes with it. */
	f = (mcp2515_frame_t){ .id = 0x55, .len = 4, .remote = true,
			       .data = { 1, 2, 3, 4 } };
	assert(mcp2515_send(d, &f, 0) == RET(0));
	assert(chip.last_load_len == 5 && chip.sent[2].raw[4] == (0x40 | 4));
	f.remote = false;
	assert(mcp2515_send(d, &f, 0) == RET(0) && chip.last_load_len == 9);

	/* Buffers held: TXB0, TXB1, TXB2, then none free. */
	chip.tx_hold = true;
	chip.sent_count = 0;
	f = (mcp2515_frame_t){ .id = 0x7FF, .len = 8 };
	for (int b = 0; b < 3; b++) {
		assert(mcp2515_send(d, &f, 0) == RET(0));
		assert(chip.sent[b].buf == b);
	}
	t0 = now_us();
	assert(mcp2515_send(d, &f, 0) == RET(-ETIMEDOUT));
	assert(now_us() == t0);			/* 0: one look, no wait */
	assert(mcp2515_send(d, &f, 35) == RET(-ETIMEDOUT));
	assert(now_us() - t0 >= 35000);
	assert(now_us() - t0 < 35000 + 25000);
	release_tx();

	f.len = 9;
	assert(mcp2515_send(d, &f, 0) == RET(-EINVAL));
	f.len = 0;
	f.id = 0x800;
	assert(mcp2515_send(d, &f, 0) == RET(-EINVAL));
	f.extended = true;
	assert(mcp2515_send(d, &f, 0) == RET(0));
	f.id = 0x20000000;
	assert(mcp2515_send(d, &f, 0) == RET(-EINVAL));
	assert(mcp2515_send(NULL, &f, 0) == RET(-EINVAL));
	assert(mcp2515_send(d, NULL, 0) == RET(-EINVAL));
	down(d);
}

static void test_receive(void)
{
	mcp2515_handle_t d = up(false);
	mcp2515_frame_t f;
	int64_t t0;
	static const uint8_t data[8] = { 9, 8, 7, 6, 5, 4, 3, 2 };

	t0 = now_us();
	assert(mcp2515_receive(d, &f, 0) == RET(-ETIMEDOUT));
	assert(now_us() == t0);
	assert(mcp2515_receive(d, &f, 45) == RET(-ETIMEDOUT));
	assert(now_us() - t0 >= 45000 && now_us() - t0 < 45000 + 25000);

	deliver(0x321, false, false, 8, data);
	deliver(0x1ABCDE, true, false, 2, data);
	assert(chip.reg[MCP2515_CANINTF] == 0x03);	/* rolled over */
	assert(mcp2515_receive(d, &f, 0) == RET(0));
	assert(f.id == 0x321 && !f.extended && !f.remote && f.len == 8 &&
	       f.data[7] == 2);
	assert(mcp2515_receive(d, &f, 0) == RET(0));
	assert(f.id == 0x1ABCDE && f.extended && f.len == 2 && f.data[1] == 8 &&
	       f.data[2] == 0);
	assert(chip.reg[MCP2515_CANINTF] == 0);

	/* Remote frames: SRR for standard, DLC.RTR for extended. */
	deliver(0x10, false, true, 4, NULL);
	assert(mcp2515_receive(d, &f, 0) == RET(0));
	assert(f.remote && !f.extended && f.len == 4);
	deliver(0x10, true, true, 1, NULL);
	assert(mcp2515_receive(d, &f, 0) == RET(0));
	assert(f.remote && f.extended && f.len == 1);

	/* A DLC of 15 on the wire: eight bytes stored. */
	deliver(0x11, false, false, 15, data);
	assert(mcp2515_receive(d, &f, 0) == RET(0) && f.len == 8);

	/* Three frames, two buffers: the third is an overflow. */
	deliver(1, false, false, 0, NULL);
	deliver(2, false, false, 0, NULL);
	deliver(3, false, false, 0, NULL);
	{
		mcp2515_status_t st;

		assert(mcp2515_read_status(d, &st) == RET(0));
		assert(st.rx_overflow);
		assert(mcp2515_clear_errors(d) == RET(0));
		assert(chip.reg[MCP2515_EFLG] == 0);
	}
	assert(mcp2515_receive(NULL, &f, 0) == RET(-EINVAL));
	assert(mcp2515_receive(d, NULL, 0) == RET(-EINVAL));
	down(d);

	/* nINT wired and high: no SPI poll at all. */
	d = up(true);
	chip.status_reads = 0;
	assert(mcp2515_receive(d, &f, 20) == RET(-ETIMEDOUT));
	assert(chip.status_reads == 0);
	deliver(0x55, false, false, 1, data);
	assert(int_low());
	assert(mcp2515_receive(d, &f, 20) == RET(0) && f.id == 0x55);
	assert(!int_low());
	down(d);
}

static void test_loopback(void)
{
	mcp2515_config_t cfg = cfg_of(false);
	mcp2515_handle_t d;
	mcp2515_frame_t f = { .id = 0x7A, .len = 2, .data = { 0xAB, 0xCD } };
	mcp2515_frame_t g;

	chip_power_on();
	cfg.loopback = true;
	assert(mcp2515_init(&cfg, &d) == RET(0));
	assert(mcp2515_send(d, &f, 10) == RET(0));
	assert(mcp2515_receive(d, &g, 10) == RET(0));
	assert(g.id == 0x7A && g.len == 2 && g.data[1] == 0xCD && !g.remote);
	f.remote = true;
	f.extended = false;
	assert(mcp2515_send(d, &f, 10) == RET(0));
	assert(mcp2515_receive(d, &g, 10) == RET(0) && g.remote && !g.extended);
	down(d);
}

static void test_filter(void)
{
	mcp2515_handle_t d = up(false);
	mcp2515_frame_t f;

	assert(mcp2515_set_filter(d, 0x123, 0x7F0, false) == RET(0));
	assert(chip.config_writes_refused == 0);
	assert(opmode() == MCP2515_MODE_NORMAL);
	assert(chip.reg[MCP2515_RXB0CTRL] == 0x04);
	assert(chip.reg[MCP2515_RXB1CTRL] == 0x00);
	for (int i = 0; i < 6; i++) {
		static const uint8_t rxf[6] = MCP2515_RXF_ADDRS;

		assert(chip.reg[rxf[i]] == 0x24 && chip.reg[rxf[i] + 1] == 0x60);
	}

	deliver(0x12F, false, false, 0, NULL);	/* passes */
	deliver(0x133, false, false, 0, NULL);	/* not */
	deliver(0x12F, true, false, 0, NULL);	/* extended: not */
	assert(mcp2515_receive(d, &f, 0) == RET(0) && f.id == 0x12F);
	assert(mcp2515_receive(d, &f, 0) == RET(-ETIMEDOUT));

	/* Extended. */
	assert(mcp2515_set_filter(d, 0x18FF0000, 0x1FFF0000, true) == RET(0));
	deliver(0x18FF1234, true, false, 0, NULL);
	deliver(0x18FE1234, true, false, 0, NULL);
	deliver(0x123, false, false, 0, NULL);
	assert(mcp2515_receive(d, &f, 0) == RET(0) && f.id == 0x18FF1234);
	assert(mcp2515_receive(d, &f, 0) == RET(-ETIMEDOUT));

	/* mask 0: everything, as at power-on. */
	assert(mcp2515_set_filter(d, 0, 0, false) == RET(0));
	assert(chip.reg[MCP2515_RXB0CTRL] == 0x64);
	deliver(0x1, true, false, 0, NULL);
	assert(mcp2515_receive(d, &f, 0) == RET(0) && f.id == 1 && f.extended);

	assert(mcp2515_set_filter(d, 0x800, 0x7FF, false) == RET(-EINVAL));
	assert(mcp2515_set_filter(NULL, 0, 0, false) == RET(-EINVAL));

	/* A failed write still puts the chip back on the bus. */
	spi_fail_at = chip.xfers + 3;
	assert(mcp2515_set_filter(d, 0x123, 0x7FF, false) != RET(0));
	assert(opmode() == MCP2515_MODE_NORMAL);
	down(d);
}

static void test_category(void)
{
	mcp2515_handle_t d = up(false);
	struct can_dev *can = mcp2515_can(d);
	struct can_filter fl[2] = {
		{ .id = 0x100, .mask = 0x7FF },
		{ .id = 0x1ABCDE, .mask = CAN_FRAME_EXT_ID_MAX, .ext = true },
	};
	struct can_frame f = { .id = 0x42, .dlc = 1, .data = { 7 } };
	struct can_status st;

	assert(mcp2515_can(NULL) == NULL);
	assert(can_send(can, &f, 0) == 0 && chip.sent_count == 1);
	assert(chip.sent[0].raw[4] == 1 && chip.sent[0].raw[5] == 7);

	/* Two filters: one per buffer. */
	assert(can_set_filters(can, fl, 2) == 0);
	assert(chip.reg[0x00] == 0x20 && chip.reg[0x04] == 0x20);
	assert(chip.reg[0x09] & 0x08 && chip.reg[0x19] & 0x08);
	deliver(0x1ABCDE, true, false, 0, NULL);
	deliver(0x100, false, false, 0, NULL);
	deliver(0x101, false, false, 0, NULL);
	assert(can_recv(can, &f, 0) == 0 && f.id == 0x100 && f.t_us == now_us());
	assert(can_recv(can, &f, 0) == 0 && f.id == 0x1ABCDE && f.ext);
	assert(can_recv(can, &f, 0) == -ETIMEDOUT);
	/* RXB1 has a mask of its own: one EID bit off is out. */
	deliver(0x1ABCDF, true, false, 0, NULL);
	assert(!(chip.reg[MCP2515_CANINTF] & MCP2515_INT_RX1IF));
	/* One filter goes on both buffers. */
	assert(can_set_filters(can, &fl[1], 1) == 0);
	assert(chip.reg[0x00] == chip.reg[0x10]);
	assert(can_set_filters(can, NULL, 0) == 0);
	assert(chip.reg[MCP2515_RXB0CTRL] == 0x64);

	/* Bit rate: a table row, written and read back. */
	assert(can_set_bitrate(can, 250000) == 0);
	assert(chip.reg[0x2A] == 0x00 && chip.reg[0x29] == 0xB1 &&
	       chip.reg[0x28] == 0x05);
	assert(opmode() == MCP2515_MODE_NORMAL);
	assert(can_set_bitrate(can, 1000000) == -EINVAL);	/* 8 MHz */

	assert(can_set_opmode(can, CAN_OPMODE_LISTEN) == 0);
	assert(opmode() == MCP2515_MODE_LISTEN);
	assert(can_set_opmode(can, CAN_OPMODE_NORMAL) == 0);

	/* Status from TEC, REC and EFLG. */
	chip.reg[MCP2515_TEC] = 130;
	chip.reg[MCP2515_TEC + 1] = 5;
	chip.reg[MCP2515_EFLG] = MCP2515_EFLG_TXEP | MCP2515_EFLG_EWARN;
	assert(can_get_status(can, &st) == 0);
	assert(st.link == CAN_LINK_PASSIVE && st.tec == 130 && st.rec == 5);
	chip.reg[MCP2515_EFLG] = MCP2515_EFLG_EWARN;
	assert(can_get_status(can, &st) == 0 && st.link == CAN_LINK_WARNING);
	/* Bus-off: TEC is past 255, so past 128 too. */
	chip.reg[MCP2515_EFLG] = MCP2515_EFLG_TXBO | MCP2515_EFLG_TXEP |
				 MCP2515_EFLG_RX1OVR;
	assert(can_get_status(can, &st) == 0 && st.link == CAN_LINK_BUS_OFF);
	assert(st.rx_lost == 1);
	chip.reg[MCP2515_EFLG] = 0;
	deliver(0x1, false, false, 0, NULL);
	chip.tx_hold = true;
	assert(can_send(can, &f, 0) == 0 && can_send(can, &f, 0) == 0);
	assert(can_get_status(can, &st) == 0 && st.link == CAN_LINK_ACTIVE);
	assert(st.rx_pending == 1 && st.tx_pending == 2);
	release_tx();

	/* Recovery is the chip's own. */
	assert(can_recover(can) == 0);

	/* Stop: Configuration mode, which clears the counters; settings made
	 * while stopped leave it there; start goes back. */
	assert(can_stop(can) == 0 && opmode() == MCP2515_MODE_CONFIG);
	assert(chip.reg[MCP2515_TEC] == 0);
	assert(can_get_status(can, &st) == 0 && st.link == CAN_LINK_STOPPED);
	assert(can_set_filters(can, fl, 1) == 0);
	assert(can_set_opmode(can, CAN_OPMODE_LOOPBACK) == 0);
	assert(opmode() == MCP2515_MODE_CONFIG);
	assert(can_start(can) == 0 && opmode() == MCP2515_MODE_LOOPBACK);
	down(d);
}

static void test_status_handle(void)
{
	mcp2515_handle_t d = up(false);
	mcp2515_status_t st;

	chip.reg[MCP2515_TEC] = 96;
	chip.reg[MCP2515_TEC + 1] = 128;
	chip.reg[MCP2515_EFLG] = MCP2515_EFLG_EWARN | MCP2515_EFLG_RXEP;
	assert(mcp2515_read_status(d, &st) == RET(0));
	assert(st.tx_errors == 96 && st.rx_errors == 128 && st.warning &&
	       st.passive && !st.bus_off && !st.rx_overflow);
	/* Only the two overflow bits are the MCU's to clear. */
	chip.reg[MCP2515_EFLG] |= MCP2515_EFLG_OVR;
	assert(mcp2515_clear_errors(d) == RET(0));
	assert(chip.reg[MCP2515_EFLG] ==
	       (MCP2515_EFLG_EWARN | MCP2515_EFLG_RXEP));
	assert(mcp2515_read_status(NULL, &st) == RET(-EINVAL));
	assert(mcp2515_read_status(d, NULL) == RET(-EINVAL));
	assert(mcp2515_clear_errors(NULL) == RET(-EINVAL));
	assert(mcp2515_del(NULL) == RET(-EINVAL));
	down(d);
}

static void run_mcp2515_cases(void)
{
	test_init();
	test_init_failures();
	test_send();
	test_receive();
	test_loopback();
	test_filter();
	test_category();
	test_status_handle();
}
