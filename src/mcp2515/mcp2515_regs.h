/*
 * MCP2515 SPI instructions, registers and bits. Every value carries its
 * table, figure or register number in the datasheet, DS20001801J
 * (docs/mcp2515.pdf); the numbers are those of esp-mcp2515-can 210a3c0.
 */
#ifndef MCP2515_REGS_H
#define MCP2515_REGS_H

/* ---- SPI instruction set, Table 12-1 (p.67) ----------------------------- */

#define MCP2515_RESET       0xC0
#define MCP2515_READ        0x03
#define MCP2515_WRITE       0x02
#define MCP2515_BIT_MODIFY  0x05    /* instruction, address, mask, data */
#define MCP2515_READ_STATUS 0xA0
#define MCP2515_RTS         0x80    /* | 1<<n for TXBn, Figure 12-6 */
#define MCP2515_LOAD_TX     0x40    /* | abc, Figure 12-5: 0/2/4 TXB0/1/2 */
#define MCP2515_READ_RX     0x90    /* | nm<<1, Figure 12-3: 90 RXB0, 94 RXB1 */
#define MCP2515_READ_RX1    0x04    /* n = 1: RXB1SIDH */

/* The byte READ STATUS returns, Figure 12-8 (p.70). TXREQ for buffer n is
 * bit 2+2n. */
#define MCP2515_ST_RX0IF    (1u << 0)
#define MCP2515_ST_RX1IF    (1u << 1)
#define MCP2515_ST_TXREQ(n) (1u << (2 + 2 * (n)))

/* ---- registers, Table 11-1 and Table 11-2 (p.63) ------------------------ */

#define MCP2515_RXF0SIDH    0x00    /* Register 4-10 to 4-13; six filters */
#define MCP2515_RXM0SIDH    0x20    /* Register 4-14 to 4-16 */
#define MCP2515_RXM1SIDH    0x24
#define MCP2515_CANSTAT     0x0E    /* Register 10-2 */
#define MCP2515_CANCTRL     0x0F    /* Register 10-1 */
#define MCP2515_TEC         0x1C    /* Register 6-1; REC 0x1D right after */
#define MCP2515_CNF3        0x28    /* Register 5-3; CNF2 0x29, CNF1 0x2A */
#define MCP2515_CANINTE     0x2B    /* Register 7-1 */
#define MCP2515_CANINTF     0x2C    /* Register 7-2 */
#define MCP2515_EFLG        0x2D    /* Register 6-3 */
#define MCP2515_RXB0CTRL    0x60    /* Register 4-1 */
#define MCP2515_RXB1CTRL    0x70    /* Register 4-2 */

/* Table 11-1. The six filters are not evenly spaced: RXF2 ends at 0x0B
 * and RXF3 starts at 0x10. RXF0-1 feed RXB0, RXF2-5 feed RXB1. */
#define MCP2515_RXF_ADDRS   { 0x00, 0x04, 0x08, 0x10, 0x14, 0x18 }

/* CANCTRL REQOP[2:0] / CANSTAT OPMOD[2:0], both in bits 7:5 (Register 10-1,
 * p.60). */
#define MCP2515_MODE_MASK       0xE0u
#define MCP2515_MODE_NORMAL     0x00u
#define MCP2515_MODE_SLEEP      0x20u
#define MCP2515_MODE_LOOPBACK   0x40u
#define MCP2515_MODE_LISTEN     0x60u
#define MCP2515_MODE_CONFIG     0x80u

/* RXBnCTRL, Register 4-1 and 4-2 (p.27-28). */
#define MCP2515_RXM_FILTER  0x00u   /* RXM = 00: apply the masks and filters */
#define MCP2515_RXM_ALL     0x60u   /* RXM = 11: masks and filters off */
#define MCP2515_RXB0_BUKT   0x04u   /* roll a full RXB0 over into RXB1 */

/* CANINTE / CANINTF, Register 7-1 and 7-2 (p.53-54). */
#define MCP2515_INT_RX0IF   0x01u
#define MCP2515_INT_RX1IF   0x02u

/* EFLG, Register 6-3 (p.50) and Table 11-2. */
#define MCP2515_EFLG_EWARN  0x01u   /* TEC or REC reached 96 */
#define MCP2515_EFLG_RXEP   0x08u   /* REC reached 128 */
#define MCP2515_EFLG_TXEP   0x10u   /* TEC reached 128 */
#define MCP2515_EFLG_TXBO   0x20u   /* TEC reached 255 */
#define MCP2515_EFLG_RX0OVR 0x40u
#define MCP2515_EFLG_RX1OVR 0x80u
#define MCP2515_EFLG_OVR    (MCP2515_EFLG_RX0OVR | MCP2515_EFLG_RX1OVR)

/* TXBnDLC (Register 3-7, p.22) and RXBnDLC (Register 4-8, p.32) both put
 * RTR at bit 6. */
#define MCP2515_DLC_RTR     0x40u
#define MCP2515_DLC_LEN     0x0Fu
/* RXBnSIDL SRR, Register 4-5 bit 4 (p.30): the RTR flag of a *standard*
 * frame. */
#define MCP2515_SIDL_SRR    0x10u

#define MCP2515_SPI_MAX_HZ  10000000    /* Section 12.1 and p.1: 10 MHz */

#endif /* MCP2515_REGS_H */
