/*
 * Host-side check of the FDCAN bit-timing search (src/fdcan/fdcan_math.h):
 * worked rows, the limits from the HAL's field comments, the clock divider
 * codes, and a sweep that holds the invariants on every result.
 */
#include <assert.h>
#include <stdio.h>

#include "fdcan_math.h"

static void row(uint32_t clock, uint32_t bitrate, uint32_t permille,
		const struct fdcan_timing_limits *limits, uint32_t prescaler,
		uint32_t seg1, uint32_t seg2, uint32_t sjw)
{
	struct fdcan_timing t;

	assert(fdcan_timing_find(clock, bitrate, permille, limits, &t) == 0);
	assert(t.prescaler == prescaler && t.seg1 == seg1 && t.seg2 == seg2 &&
	       t.sjw == sjw);
}

/*
 * Worked by hand. 80 MHz, 1 Mbit/s: 80 quanta of one clock; 87.5% of 80 is
 * 70 quanta to the sample point, so seg1 69 and seg2 10. 8 MHz, 1 Mbit/s:
 * 8 quanta, 87.5% is 7, seg2 would be 1 -- under the nominal minimum of 2,
 * so the sample point moves to 6 of 8. 250 MHz, 125 kbit/s: 2000 ticks; the
 * first prescaler whose seg1 fits in 256 is 8 (250 quanta, seg1 218).
 */
static void test_rows(void)
{
	row(80000000, 1000000, 875, &fdcan_nominal_limits, 1, 69, 10, 10);
	row(80000000, 500000, 875, &fdcan_nominal_limits, 1, 139, 20, 20);
	row(8000000, 1000000, 875, &fdcan_nominal_limits, 1, 5, 2, 2);
	row(8000000, 125000, 875, &fdcan_nominal_limits, 1, 55, 8, 8);
	row(250000000, 125000, 875, &fdcan_nominal_limits, 8, 218, 31, 31);
	/* Data: seg1 at most 32, so 80 MHz at 1 Mbit/s takes prescaler 2:
	 * 40 quanta, 75% is 30, seg1 29, seg2 10. */
	row(80000000, 1000000, 750, &fdcan_data_limits, 2, 29, 10, 10);
	row(80000000, 2000000, 750, &fdcan_data_limits, 1, 29, 10, 10);
	/* seg2 has a ceiling too: at a 50% data sample point, prescaler 2
	 * would give 40 quanta, seg1 19 but seg2 20, past 16; 4 gives 20. */
	row(80000000, 1000000, 500, &fdcan_data_limits, 4, 9, 10, 10);
}

static void test_refused(void)
{
	struct fdcan_timing t = { 0 };

	/* Not an exact division. */
	assert(fdcan_timing_find(80000000, 300000, 875,
				 &fdcan_nominal_limits, &t) == -EINVAL);
	assert(fdcan_timing_find(7500000, 1000000, 875,
				 &fdcan_nominal_limits, &t) == -EINVAL);
	/* Too few quanta: one bit is 1 tick, 1 + 2 + 2 is the least. */
	assert(fdcan_timing_find(1000000, 1000000, 875,
				 &fdcan_nominal_limits, &t) == -EINVAL);
	assert(fdcan_timing_find(4000000, 1000000, 875,
				 &fdcan_nominal_limits, &t) == -EINVAL);
	assert(fdcan_timing_find(5000000, 1000000, 875,
				 &fdcan_nominal_limits, &t) == 0);
	/* Data needs 1 + 1 + 1. */
	assert(fdcan_timing_find(2000000, 1000000, 750,
				 &fdcan_data_limits, &t) == -EINVAL);
	assert(fdcan_timing_find(3000000, 1000000, 750,
				 &fdcan_data_limits, &t) == 0);
	/* Too many: past the largest prescaler with the most quanta. */
	assert(fdcan_timing_find(250000000, 400, 875,
				 &fdcan_nominal_limits, &t) == -EINVAL);
	assert(fdcan_timing_find(0, 1000000, 875,
				 &fdcan_nominal_limits, &t) == -EINVAL);
	assert(fdcan_timing_find(80000000, 0, 875,
				 &fdcan_nominal_limits, &t) == -EINVAL);
}

/* FDCAN_CLOCK_DIV1..DIV30: code 0 divides by 1, n by 2n. */
static void test_divisor(void)
{
	assert(fdcan_clock_divisor(0) == 1);
	assert(fdcan_clock_divisor(1) == 2);
	assert(fdcan_clock_divisor(5) == 10);
	assert(fdcan_clock_divisor(15) == 30);
	assert(fdcan_clock_divisor(0x10) == 1);	/* PDIV is four bits */
}

static void check(const struct fdcan_timing_limits *limits, uint32_t clock,
		  uint32_t bitrate, uint32_t permille)
{
	struct fdcan_timing t;
	uint32_t quanta, sample;
	int64_t off;

	if (fdcan_timing_find(clock, bitrate, permille, limits, &t))
		return;
	quanta = 1 + t.seg1 + t.seg2;
	/* The bit is exactly the rate asked for. */
	assert((uint64_t)t.prescaler * quanta * bitrate == clock);
	assert(t.prescaler >= 1 && t.prescaler <= limits->prescaler_max);
	assert(t.seg1 >= limits->seg1_min && t.seg1 <= limits->seg1_max);
	assert(t.seg2 >= limits->seg2_min && t.seg2 <= limits->seg2_max);
	assert(t.sjw >= 1 && t.sjw <= t.seg2 && t.sjw <= limits->sjw_max);
	/* The sample point within half a quantum of the target, unless a
	 * segment minimum pushed it. */
	sample = 1 + t.seg1;
	off = (int64_t)sample * 1000 - (int64_t)quanta * permille;
	if (t.seg2 > limits->seg2_min && t.seg1 > limits->seg1_min)
		assert(off >= -500 && off <= 500);
}

static void test_sweep(void)
{
	static const uint32_t clocks[] = { 8000000, 16000000, 20000000,
					   40000000, 48000000, 64000000,
					   80000000, 100000000, 120000000,
					   160000000, 250000000 };
	static const uint32_t rates[] = { 10000, 20000, 50000, 100000, 125000,
					  250000, 500000, 800000, 1000000,
					  2000000, 4000000, 5000000, 8000000 };

	for (size_t c = 0; c < sizeof(clocks) / sizeof(clocks[0]); c++)
		for (size_t r = 0; r < sizeof(rates) / sizeof(rates[0]); r++) {
			check(&fdcan_nominal_limits, clocks[c], rates[r], 875);
			check(&fdcan_data_limits, clocks[c], rates[r], 750);
		}
}

/*
 * The TDC offset: the data sample point in mtq, prescaler x (1 + seg1),
 * up to TDCO's 127 (RM0481 60.4.14).
 */
static void test_tdc_offset(void)
{
	struct fdcan_timing t = { .prescaler = 1, .seg1 = 29, .seg2 = 10 };

	assert(fdcan_tdc_offset(&t) == 30);
	t = (struct fdcan_timing){ .prescaler = 2, .seg1 = 29 };
	assert(fdcan_tdc_offset(&t) == 60);
	t = (struct fdcan_timing){ .prescaler = 1, .seg1 = 126 };
	assert(fdcan_tdc_offset(&t) == 127);
	t = (struct fdcan_timing){ .prescaler = 1, .seg1 = 127 };
	assert(fdcan_tdc_offset(&t) == -EINVAL);
	t = (struct fdcan_timing){ .prescaler = 5, .seg1 = 29 };
	assert(fdcan_tdc_offset(&t) == -EINVAL);
	/* Every data timing at 80 MHz from 2 to 8 Mbit/s fits. */
	for (uint32_t rate = 2000000; rate <= 8000000; rate += 1000000) {
		if (fdcan_timing_find(80000000, rate, 750, &fdcan_data_limits,
				      &t))
			continue;
		assert(fdcan_tdc_offset(&t) > 0);
	}
}

int main(void)
{
	test_rows();
	test_refused();
	test_divisor();
	test_sweep();
	test_tdc_offset();
	printf("test_fdcan_math: all passed\n");
	return 0;
}
