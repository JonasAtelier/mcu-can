/*
 * The MCP2515's identifier packing and bit-timing table, on the host: every
 * row decoded back into a bit rate and held to the rules of the datasheet,
 * DS20001801J, Section 5:
 *
 *   TQ = 2 x (BRP + 1) / FOSC                       Register 5-1, p.44
 *   bit = SyncSeg + PropSeg + PS1 + PS2 TQ          Section 5.2, p.39
 *   SyncSeg 1; PropSeg 1-8; PS1 1-8; PS2 2-8        Section 5.2.3, p.40
 *   PS2 >= IPT = 2 TQ                               Section 5.2.5, p.40
 *   PropSeg + PS1 >= PS2; PS2 > SJW                 Section 5.5, p.43
 *   SJW 1-4                                         Section 5.2.6
 *   sample point "about 60-70%"                     Section 5.5
 *
 * and to this library's two changes from esp-mcp2515-can: 8 MHz at 1 Mbit/s
 * refused, CNF3.SOF never set.
 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "mcp2515_math.h"

static const uint32_t crystals[] = { 4000000, 8000000, 10000000, 12000000,
				     16000000, 20000000 };
static const uint32_t rates[] = { 10000, 20000, 50000, 100000, 125000,
				  250000, 500000, 800000, 1000000 };

/* The rows there should be, and nothing else. */
static bool expected(uint32_t xtal, uint32_t rate)
{
	if (xtal == 8000000)
		return rate == 125000 || rate == 250000 || rate == 500000;
	if (xtal == 16000000)
		return rate == 125000 || rate == 250000 || rate == 500000 ||
		       rate == 1000000;
	return false;
}

static void check_row(uint32_t xtal, uint32_t rate, const uint8_t cnf[3])
{
	unsigned int sjw = (cnf[0] >> 6) + 1;
	unsigned int brp = cnf[0] & 0x3F;
	bool btlmode = cnf[1] & 0x80;
	bool sam = cnf[1] & 0x40;
	unsigned int ps1 = ((cnf[1] >> 3) & 7) + 1;
	unsigned int prop = (cnf[1] & 7) + 1;
	bool sof = cnf[2] & 0x80;
	bool wakfil = cnf[2] & 0x40;
	unsigned int ps2 = (cnf[2] & 7) + 1;
	unsigned int tq = 1 + prop + ps1 + ps2;
	unsigned int sample_pct = 100 * (1 + prop + ps1) / tq;

	/* PS2 comes from CNF3 only with BTLMODE set (Register 5-2). */
	assert(btlmode);
	assert(cnf[2] & 0x38 ? false : true);	/* CNF3 bits 5:3 unused */
	assert(prop >= 1 && prop <= 8 && ps1 >= 1 && ps1 <= 8);
	assert(ps2 >= 2 && ps2 <= 8);
	assert(sjw >= 1 && sjw <= 4);
	assert(prop + ps1 >= ps2);
	assert(ps2 > sjw);
	/* FOSC / (2 (BRP + 1) x TQ per bit), exactly. */
	assert((uint64_t)rate * 2 * (brp + 1) * tq == xtal);
	assert(sample_pct >= 55 && sample_pct <= 75);
	assert(!sof && !wakfil);
	/* Triple sampling as esp-mcp2515-can had it: the 16 MHz rows. */
	assert(sam == (xtal == 16000000));
	printf("  %2u MHz %7u: CNF1 %02X CNF2 %02X CNF3 %02X  %2u TQ, "
	       "sample %u%%, SJW %u\n", (unsigned)(xtal / 1000000),
	       (unsigned)rate, cnf[0], cnf[1], cnf[2], tq, sample_pct, sjw);
}

static void test_timing(void)
{
	int rows = 0;

	for (size_t i = 0; i < sizeof(crystals) / sizeof(crystals[0]); i++)
		for (size_t j = 0; j < sizeof(rates) / sizeof(rates[0]); j++) {
			uint8_t cnf[3] = { 0xEE, 0xEE, 0xEE };
			bool found = mcp2515_timing(crystals[i], rates[j],
						    cnf);

			assert(found == expected(crystals[i], rates[j]));
			if (!found) {
				/* Untouched. */
				assert(cnf[0] == 0xEE && cnf[2] == 0xEE);
				continue;
			}
			check_row(crystals[i], rates[j], cnf);
			rows++;
		}
	assert(rows == 7);

	/* The 8 MHz rows at 125k and 250k are Section 5.5's own example,
	 * 1 + 2 + 7 + 6 = 16 TQ. */
	{
		uint8_t cnf[3];

		assert(mcp2515_timing(8000000, 125000, cnf));
		assert(cnf[1] == 0xB1 && cnf[2] == 0x05);
	}
}

static void check_id(uint32_t id, bool ext, uint8_t b0, uint8_t b1,
		     uint8_t b2, uint8_t b3)
{
	uint8_t raw[4];
	uint32_t back;
	bool back_ext;

	mcp2515_pack_id(id, ext, raw);
	assert(raw[0] == b0 && raw[1] == b1 && raw[2] == b2 && raw[3] == b3);
	mcp2515_unpack_id(raw, &back, &back_ext);
	assert(back == id && back_ext == ext);
}

static void test_ids(void)
{
	/* Standard: SID10:3 in SIDH, SID2:0 in SIDL 7:5 (Registers 3-3, 3-4). */
	check_id(0x000, false, 0x00, 0x00, 0, 0);
	check_id(0x123, false, 0x24, 0x60, 0, 0);
	check_id(0x7FF, false, 0xFF, 0xE0, 0, 0);
	/* Extended: SID as the top 11 bits, EXIDE, EID17:16 in SIDL 1:0, then
	 * EID15:8 and EID7:0 (Registers 3-4 to 3-6). */
	check_id(0x18FF00A1, true, 0xC7, 0xEB, 0x00, 0xA1);
	check_id(0x1FFFFFFF, true, 0xFF, 0xEB, 0xFF, 0xFF);
	check_id(0x00000001, true, 0x00, 0x08, 0x00, 0x01);
	check_id(0x00030000, true, 0x00, 0x0B, 0x00, 0x00);

	/* Out of range is masked, not spilled into the neighbours. */
	{
		uint8_t raw[4];

		mcp2515_pack_id(0xFFFFF800 | 0x123, false, raw);
		assert(raw[0] == 0x24 && raw[1] == 0x60);
	}
}

int main(void)
{
	test_timing();
	test_ids();
	printf("test_mcp2515_math: all passed\n");
	return 0;
}
