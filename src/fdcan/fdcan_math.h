/*
 * FDCAN bit timing, as pure arithmetic: compiled and asserted on the host.
 *
 * A bit is one sync quantum plus seg1 plus seg2 time quanta, and a quantum
 * is prescaler periods of the FDCAN clock: the STM32 HAL writes NBTP and
 * DBTP from exactly those four fields of FDCAN_InitTypeDef, each less one
 * (stm32h5xx_hal_fdcan.c, HAL_FDCAN_Init, v1.7.0). The ranges are the HAL's,
 * from the same fields' comments (stm32h5xx_hal_fdcan.h:82-110):
 *
 *   nominal  prescaler 1..512  seg1 2..256  seg2 2..128  sjw 1..128
 *   data     prescaler 1..32   seg1 1..32   seg2 1..16   sjw 1..16
 *
 * The FDCAN clock is the kernel clock divided by CKDIV.PDIV: 0 divides by
 * 1, n by 2n (FDCAN_CLOCK_DIV1..DIV30, stm32h5xx_hal_fdcan.h:560-575).
 *
 * RM0481 Rev 5, 60.3.3 "Bit timing" (p.2892), gives the same bit: one sync
 * quantum, BS1 = TSEG1 + 1 and BS2 = TSEG2 + 1 quanta of (BRP + 1) clock
 * periods, the registers holding each value less one. Its section 60.3.4
 * (p.2893) puts the nominal bit rate at up to 1 Mbit/s.
 *
 * The sample points are this library's policy, not a standard's: 87.5% for
 * the nominal phase, 75% for the data phase.
 */
#pragma once

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#define FDCAN_NOMINAL_SAMPLE_PERMILLE	875
#define FDCAN_DATA_SAMPLE_PERMILLE	750

/* RM0481 Rev 5, 60.3.4 (p.2893): "Nominal bitrate: up to 1 Mbit/s". */
#define FDCAN_NOMINAL_BITRATE_MAX	1000000

struct fdcan_timing {
	uint32_t prescaler;
	uint32_t seg1;
	uint32_t seg2;
	uint32_t sjw;
};

/* RM0481 Rev 5, 60.3.4 (p.2893): "Data bitrate: between nominal bitrate
 * and 8 Mbit/s". */
#define FDCAN_DATA_BITRATE_MAX		8000000

/*
 * Transmitter delay compensation (TDC), RM0481 Rev 5, 60.3.4 "Operating
 * modes", its "Transceiver delay compensation" part under "CAN FD
 * operation", pp.2896-2897; FDCAN_DBTP.TDC, 60.4.3, p.2921; FDCAN_TDCR,
 * 60.4.14, p.2931:
 *
 *   - A transmitting node reads its own bits back through the transceiver,
 *     late by the loop delay. Without TDC, a loop delay longer than the data
 *     phase's time before the sample point is a bit error, which is what
 *     limits the data rate.
 *   - With TDC (FDCAN_DBTP.TDC), each bit is checked at a secondary sample
 *     point (SSP): the delay the controller measures in each FD frame plus
 *     the offset TDCO, in mtq -- one period of the FDCAN time-quantum clock,
 *     so in clock periods, not quanta. TDCO is 0..127; delay plus TDCO must
 *     stay under six data bits, and is capped at 127 mtq.
 *   - TDCF, a filter window, sets a minimum SSP; 0 leaves it off.
 *
 * Where the offset goes is this library's choice: at the data sample point,
 * prescaler x (1 + seg1) mtq into the bit (RM0481's example is half a bit).
 * When TDC is on is the library's choice too: above 1 Mbit/s, the most the
 * nominal phase may run at (60.3.4); at or below, with the 75% data sample
 * point, the loop delay would have to exceed 750 ns.
 */
#define FDCAN_TDC_ABOVE_BITRATE		1000000
#define FDCAN_TDC_OFFSET_MAX		127

/*
 * With TDC, a data prescaler of 1 or 2 is advised: CAN in Automation's
 * guidance for CAN FD bit timing (CiA 601-3, as the review relayed it; the
 * document is not in docs/) and ISO 11898-1, not RM0481. The timing search
 * already tries prescalers smallest first, so it lands on 1 or 2 whenever
 * either makes the rate exactly; when neither does, the rate is still
 * taken, and the driver warns.
 */
#define FDCAN_TDC_PRESCALER_MAX		2

/* The SSP offset for a data timing: its sample point, in mtq. -EINVAL past
 * what TDCO holds. */
static inline int fdcan_tdc_offset(const struct fdcan_timing *data)
{
	uint32_t offset = data->prescaler * (1 + data->seg1);

	return offset <= FDCAN_TDC_OFFSET_MAX ? (int)offset : -EINVAL;
}

struct fdcan_timing_limits {
	uint32_t prescaler_max;
	uint32_t seg1_min, seg1_max;
	uint32_t seg2_min, seg2_max;
	uint32_t sjw_max;
};

static const struct fdcan_timing_limits fdcan_nominal_limits = {
	.prescaler_max = 512,
	.seg1_min = 2, .seg1_max = 256,
	.seg2_min = 2, .seg2_max = 128,
	.sjw_max = 128,
};

static const struct fdcan_timing_limits fdcan_data_limits = {
	.prescaler_max = 32,
	.seg1_min = 1, .seg1_max = 32,
	.seg2_min = 1, .seg2_max = 16,
	.sjw_max = 16,
};

/* The divisor of a CKDIV.PDIV code. */
static inline uint32_t fdcan_clock_divisor(uint32_t pdiv)
{
	pdiv &= 0x0F;
	return pdiv ? 2 * pdiv : 1;
}

/*
 * The timing that makes bitrate exactly from clock_hz, with the sample point
 * nearest sample_permille. The smallest prescaler that fits wins: the most
 * quanta per bit, so the finest sample point. sjw is as wide as seg2 and
 * the limits allow. -EINVAL when no prescaler makes the rate exactly.
 */
static inline int fdcan_timing_find(uint32_t clock_hz, uint32_t bitrate,
				    uint32_t sample_permille,
				    const struct fdcan_timing_limits *limits,
				    struct fdcan_timing *timing)
{
	uint32_t ticks;

	if (!clock_hz || !bitrate || clock_hz % bitrate)
		return -EINVAL;
	ticks = clock_hz / bitrate;
	for (uint32_t prescaler = 1; prescaler <= limits->prescaler_max;
	     prescaler++) {
		uint32_t quanta, sample, seg1, seg2;

		if (ticks % prescaler)
			continue;
		quanta = ticks / prescaler;
		if (quanta < 1 + limits->seg1_min + limits->seg2_min)
			break;		/* fewer still from here on */
		/* Quanta up to the sample point, the sync quantum included. */
		sample = (quanta * sample_permille + 500) / 1000;
		if (sample < 1 + limits->seg1_min)
			sample = 1 + limits->seg1_min;
		if (quanta - sample < limits->seg2_min)
			sample = quanta - limits->seg2_min;
		seg1 = sample - 1;
		seg2 = quanta - sample;
		if (seg1 > limits->seg1_max || seg2 > limits->seg2_max)
			continue;
		timing->prescaler = prescaler;
		timing->seg1 = seg1;
		timing->seg2 = seg2;
		timing->sjw = seg2 < limits->sjw_max ? seg2 : limits->sjw_max;
		return 0;
	}
	return -EINVAL;
}
