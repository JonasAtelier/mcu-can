/*
 * Pure identifier packing and bit timing, split out so they can be compiled
 * and asserted on the host (see test/test_mcp2515_math.c). Every constant
 * here is quoted from docs/mcp2515.pdf (DS20001801J).
 *
 * This is where an MCP2515 driver silently goes wrong. Nothing checks that
 * the crystal you declared is the crystal fitted, and nothing on a CAN bus
 * reports "your bit time is wrong" -- the bus just looks dead, or worse,
 * looks fine until a second node joins.
 */
#ifndef MCP2515_MATH_H
#define MCP2515_MATH_H

#include <stdbool.h>
#include <stdint.h>

#define MCP2515_SID_MAX 0x7FFu          /* 11-bit standard identifier */
#define MCP2515_EID_MAX 0x1FFFFFFFu     /* 29-bit extended identifier */

/*
 * Pack an identifier into the four bytes SIDH, SIDL, EID8, EID0, the layout
 * shared by the transmit buffers (Register 3-3 to 3-6), the receive buffers
 * (Register 4-4 to 4-7) and the acceptance filters (Register 4-10 to 4-13).
 *
 * SIDL bit 3 is EXIDE on a transmit buffer or a filter, IDE on a receive
 * buffer: same bit, "this identifier is 29-bit" either way. In the mask
 * registers (Register 4-15) that bit is unimplemented and reads back 0, so
 * writing it there is harmless.
 */
static inline void mcp2515_pack_id(uint32_t id, bool extended, uint8_t out[4])
{
    if (extended) {
        id &= MCP2515_EID_MAX;
        out[0] = (uint8_t)(id >> 21);                    /* SID[10:3] */
        out[1] = (uint8_t)((((id >> 18) & 0x07u) << 5) | /* SID[2:0]  */
                           0x08u |                       /* EXIDE     */
                           ((id >> 16) & 0x03u));        /* EID[17:16] */
        out[2] = (uint8_t)(id >> 8);                     /* EID[15:8] */
        out[3] = (uint8_t)id;                            /* EID[7:0]  */
    } else {
        id &= MCP2515_SID_MAX;
        out[0] = (uint8_t)(id >> 3);
        out[1] = (uint8_t)((id & 0x07u) << 5);
        out[2] = 0;
        out[3] = 0;
    }
}

/* The inverse, for a frame read out of a receive buffer. */
static inline void mcp2515_unpack_id(const uint8_t in[4], uint32_t *id,
                                     bool *extended)
{
    bool ext = (in[1] & 0x08u) != 0;    /* IDE, Register 4-5 bit 3 */

    if (ext)
        *id = ((uint32_t)in[0] << 21) |
              ((uint32_t)(in[1] & 0xE0u) << 13) |
              ((uint32_t)(in[1] & 0x03u) << 16) |
              ((uint32_t)in[2] << 8) |
              (uint32_t)in[3];
    else
        *id = ((uint32_t)in[0] << 3) | (uint32_t)(in[1] >> 5);

    *extended = ext;
}

/*
 * Bit timing, as a table rather than a formula.
 *
 * A formula has to choose PropSeg, PS1, PS2 and SJW, and the division rarely
 * lands on a legal split; when it does not, the failure is a sample point a
 * few percent off, which works on the bench and drops frames in the field.
 * Seven rows that were each checked against the arithmetic are fewer lines
 * and provably right -- test_mcp2515_math.c decodes every one of them back
 * into a bit rate and checks it against the rules of Section 5.
 *
 * cnf[0] is CNF1 (Register 5-1: SJW[1:0], BRP[5:0]),
 * cnf[1] is CNF2 (Register 5-2: BTLMODE, SAM, PHSEG1[2:0], PRSEG[2:0]),
 * cnf[2] is CNF3 (Register 5-3: SOF, WAKFIL, PHSEG2[2:0]).
 *
 * Nominal bit time = 1 + (PRSEG+1) + (PHSEG1+1) + (PHSEG2+1) quanta, and
 * TQ = 2 x (BRP+1) / FOSC. Sample points land between 56% and 75%, inside
 * the "about 60-70%" the datasheet asks for in Section 5.5 (the 8 MHz rows
 * at 125k and 250k are its own worked example: 1 + 2 + 7 + 6 = 16 TQ).
 *
 * Two changes from esp-mcp2515-can's table:
 *
 *   - 8 MHz at 1 Mbit/s is gone. Four quanta is all an 8 MHz crystal leaves
 *     for a 1 us bit, so its PS2 was 1 TQ: under the 2 TQ minimum
 *     ("PS2min = IPT = 2 TQs", Section 5.2.5, p.40) and not more than SJW
 *     (Section 5.5, p.43). Fit a 16 MHz crystal for 1 Mbit/s.
 *   - CNF3.SOF is 0 in every row. With CLKEN = 1, its power-on value
 *     (Register 10-1), SOF = 1 turns the CLKOUT pin into a start-of-frame
 *     signal (Register 5-3, p.45); four of the old rows set it, for no
 *     reason given. SOF = 0 keeps CLKOUT the clock output it is at reset.
 *
 * The 16 MHz rows keep CNF2.SAM = 1, three samples per bit, as they had.
 */
static inline bool mcp2515_timing(uint32_t crystal_hz, uint32_t bitrate,
                                  uint8_t cnf[3])
{
    static const struct {
        uint32_t crystal_hz;
        uint32_t bitrate;
        uint8_t  cnf[3];
    } table[] = {
        { 8000000,  125000, { 0x01, 0xB1, 0x05 } },
        { 8000000,  250000, { 0x00, 0xB1, 0x05 } },
        { 8000000,  500000, { 0x00, 0x90, 0x02 } },
        { 16000000,  125000, { 0x03, 0xF0, 0x06 } },
        { 16000000,  250000, { 0x41, 0xF1, 0x05 } },
        { 16000000,  500000, { 0x00, 0xF0, 0x06 } },
        { 16000000, 1000000, { 0x00, 0xD0, 0x02 } },
    };

    for (unsigned i = 0; i < sizeof(table) / sizeof(table[0]); i++) {
        if (table[i].crystal_hz != crystal_hz || table[i].bitrate != bitrate)
            continue;
        cnf[0] = table[i].cnf[0];
        cnf[1] = table[i].cnf[1];
        cnf[2] = table[i].cnf[2];
        return true;
    }

    return false;
}

#endif /* MCP2515_MATH_H */
