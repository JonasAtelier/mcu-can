# Chip reference

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

Sources, cited by section, table, register or file and line:

| Short | Document |
|---|---|
| DS | Microchip MCP2515 datasheet, DS20001801J, `docs/mcp2515.pdf` (gitignored) |
| SLOS | TI SN65HVD230 datasheet, SLOS346O, `docs/sn65hvd230.pdf` (gitignored) |
| IDF | ESP-IDF v5.5.5, `components/` |
| HAL | STM32CubeF4 HAL v1.8.5, `stm32f4xx_hal_can.h` |
| CMSIS | STM32F4 CMSIS device header `stm32f405xx.h` |

ST's reference manual for the bxCAN (RM0090) is not among them: it could
not be obtained. What it alone would say -- the bit timing and filter
register layouts -- this library does not write. See [bxCAN](#bxcan).

## MCP2515

### SPI instructions and registers

| What | Value | Source |
|---|---|---|
| RESET, READ, WRITE, BIT MODIFY, READ STATUS | C0, 03, 02, 05, A0 | DS Table 12-1, p.67 |
| LOAD TX BUFFER | `0100 0abc`: 0/2/4 for TXB0/1/2 SIDH | DS Table 12-1, Figure 12-5 |
| RTS | `1000 0nnn`, one bit per TX buffer | DS Figure 12-6 |
| READ RX BUFFER | `1001 0nm0`: 90 for RXB0SIDH, 94 for RXB1SIDH; clears RXnIF when CS rises | DS Table 12-1, Figure 12-3, Section 12.4 |
| READ STATUS byte | RX0IF, RX1IF, then TXREQ and TXnIF of TXB0..2 | DS Figure 12-8, p.70 |
| RXFn, RXMn | 00, 04, 08, 10, 14, 18; 20, 24 | DS Table 11-1, p.63 |
| CANSTAT, CANCTRL | xE, xF | DS Table 11-1, Registers 10-1, 10-2 |
| TEC, REC | 1C, 1D | DS Table 11-1, Register 6-1 |
| CNF3, CNF2, CNF1 | 28, 29, 2A | DS Table 11-1, Registers 5-1 to 5-3 |
| CANINTE, CANINTF, EFLG | 2B, 2C, 2D | DS Table 11-1, Registers 7-1, 7-2, 6-3 |
| RXB0CTRL, RXB1CTRL | 60, 70; RXM bits 6:5, BUKT bit 2 | DS Registers 4-1, 4-2, p.27-28 |
| REQOP / OPMOD | bits 7:5: 000 Normal, 010 Loopback, 011 Listen-Only, 100 Configuration | DS Register 10-1, p.60 |
| EFLG | RX1OVR, RX0OVR, TXBO, TXEP, RXEP, TXWAR, RXWAR, EWARN from bit 7 | DS Table 11-2 |
| RTR | TXBnDLC / RXBnDLC bit 6 (extended), RXBnSIDL.SRR bit 4 (standard) | DS Registers 3-7, 4-8, 4-5 |
| SPI clock | 10 MHz at most, mode 0,0 or 1,1 | DS p.1, Section 12.1 |

### Bit timing

TQ = 2 x (BRP + 1) / FOSC (DS Register 5-1, p.44); a bit is SyncSeg (1 TQ)
+ PropSeg + PS1 + PS2 (DS Section 5.2, p.39). The rules, DS Sections 5.2.3
to 5.5, p.40-43: PropSeg and PS1 1-8 TQ; PS2 2-8 TQ, never under the
2 TQ information processing time ("PS2min = IPT = 2 TQs", Section 5.2.5);
PropSeg + PS1 >= PS2; PS2 > SJW; SJW 1-4; the sample point "about 60-70%".
CNF1..3 are writable only in Configuration mode (Section 5.7).

The rows, as `test/test_mcp2515_math` decodes and checks them:

| Crystal | Rate | CNF1 CNF2 CNF3 | TQ | Sample | SJW |
|---|---|---|---|---|---|
| 8 MHz | 125k | 01 B1 05 | 16 | 62% | 1 |
| 8 MHz | 250k | 00 B1 05 | 16 | 62% | 1 |
| 8 MHz | 500k | 00 90 02 | 8 | 62% | 1 |
| 16 MHz | 125k | 03 F0 06 | 16 | 56% | 1 |
| 16 MHz | 250k | 41 F1 05 | 16 | 62% | 2 |
| 16 MHz | 500k | 00 F0 06 | 16 | 56% | 1 |
| 16 MHz | 1M | 00 D0 02 | 8 | 62% | 1 |

The 8 MHz rows at 125k and 250k are DS Section 5.5's own example (1 + 2 +
7 + 6 = 16 TQ). Three decisions against esp-mcp2515-can's table:

- **8 MHz at 1 Mbit/s is refused.** Its row, 00 80 80, had four quanta: PS2
  of 1 TQ, under the IPT and not more than SJW. It may work on a short
  bench bus and has no resynchronisation margin. Fit a 16 MHz crystal.
- **SOF = 0 in every row.** CNF3 bit 7 with CLKEN = 1 -- CANCTRL's
  power-on value, 1000 0111 (Table 11-2) -- makes CLKOUT a start-of-frame
  signal instead of the clock (DS Register 5-3, p.45). The old 16 MHz rows
  set it with no reason given; the driver never touches CANCTRL's CLKEN
  either, so CLKOUT stays the clock divided by 8.
- **SAM kept** in the 16 MHz rows (CNF2 bit 6, three samples per bit), as
  they had it.

### Modes and filters

A mode request takes effect only after pending transmissions complete, and
must be checked in CANSTAT (DS Section 10.0, p.59); the driver polls it for
up to 10 ms on the clock, `-EPROTO` after. Entering Configuration mode
clears TEC and REC (DS Section 10.1), so a filter change, a bit rate change
and `can_stop()` all do.

Each receive buffer has its own mask: RXF0-1 with RXM0 feed RXB0, RXF2-5
with RXM1 feed RXB1 (DS Section 4.5). A filter's EXIDE bit applies it to
extended or to standard frames only (DS Register 4-11), so the chip itself
tells the formats apart. With RXM = 00, the EID bytes of filter and mask
apply to a standard frame's first two data bytes (DS Register 4-1); the
driver writes them as 0 for a standard filter, so they never decide. BUKT
rolls a frame RXB0 cannot take into RXB1.

### Bus-off

The chip leaves bus-off by itself after 128 occurrences of 11 consecutive
recessive bits, with no MCU action (DS p.47). EFLG's TXBO, TXEP, RXEP and
EWARN follow the counters; only RX0OVR and RX1OVR are the MCU's to clear
(DS Register 6-3), which `mcp2515_clear_errors()` does.

## TWAI

The driver sits on ESP-IDF's legacy driver, `driver/twai.h`, through its
`_v2` calls, which take a controller number.

| What | Source |
|---|---|
| bit timing: ESP-IDF's `TWAI_TIMING_CONFIG_*()`; on the ESP32-S3 (APB 80 MHz) 500k is 8 MHz quanta, tseg1 11, tseg2 4, sjw 2 | IDF `hal/include/hal/twai_types_deprecated.h:54-81`, the S3's 500k at :79 |
| one install per controller | IDF `driver/twai/twai.c:491` |
| uninstall from STOPPED or BUS_OFF only | twai.c:608 |
| start from STOPPED, stop from RUNNING | twai.c:652, :684 |
| transmit refused in listen-only, and outside RUNNING | twai.c:717, :722 |
| self-reception for a frame sent with `self` | twai.c:742 |
| recovery from BUS_OFF only; afterwards STOPPED | twai.c:889, :252-256 |
| error warning limit 96 | IDF `hal/twai_hal_v1.c:17`, `:62` |
| error passive at TEC or REC >= 128 | IDF `hal/include/hal/twai_hal.h:44` |
| single filter: ID at bit 21 (11-bit) or bit 3 (29-bit), mask inverted | IDF `hal/twai_hal_v1.c:111-122` |
| legacy and new driver not together | twai.c:1002-1018 |

The legacy status reports the state (stopped, running, bus-off,
recovering) and the counters, not the warning and passive states; the
driver derives those from the counters with the two limits above.

The single filter compares raw bits, and a standard filter's bits can match
an extended frame's top bits: the hardware does not tell the formats apart.
`can_recv()` checks the format in software. The filter, mode and bit rate
go to the driver at install, so they change only while stopped.

Choices declared against esp-sn65hvd230-can, whose API this driver does
not copy:

- Waits on the driver's queues round up to whole ticks, plus one: long,
  never short (`can_ticks()`); 0 polls.
- Remote frames are sent and received (`rtr`); esp-sn65hvd230-can refused
  them.
- No callback table and no `dispatch()`: `can_recv()` in the app's loop
  does that job.
- No process-wide lock: the driver itself refuses a second owner.

The SN65HVD230's Rs pin: low is high-speed mode, high (> 0.75 VCC) its
listen-only standby, driver off and receiver on (SLOS 10.4.1, 10.4.3);
waking takes 1.5 us at most (SLOS 8.10).

## bxCAN

Every name the driver uses is the HAL's or CMSIS's:

| What | Source |
|---|---|
| `HAL_CAN_STATE_READY`, `_LISTENING`: initialised, started | HAL :48-53 |
| `CAN_ID_STD`, `CAN_ID_EXT`; `CAN_RTR_DATA`, `CAN_RTR_REMOTE` | HAL :435-436, :444-445 |
| `CAN_RX_FIFO0`; three TX mailboxes `CAN_TX_MAILBOX0..2` | HAL :453, :462-464 |
| `CAN_IT_RX_FIFO0_MSG_PENDING` | HAL :521 |
| `HAL_CAN_Start/Stop/AddTxMessage/GetTxMailboxesFreeLevel/GetRxMessage/GetRxFifoFillLevel` | HAL :683-696 |
| `HAL_CAN_ActivateNotification/DeactivateNotification`, `HAL_CAN_GetState` | HAL :707-708, :744 |
| ESR: `CAN_ESR_EWGF`, `_EPVF`, `_BOFF`; `CAN_ESR_TEC`, `_REC` and their positions | CMSIS :1879-1886, :1895-1900 |
| FIFO 0 overrun `CAN_RF0R_FOVR0`; fill level `CAN_RF0R_FMP0`, two bits | CMSIS :1812-1813, :1806-1807 |

Without RM0090, and declared so:

- `can_set_bitrate()`, `can_set_filters()`, `can_set_opmode()` and
  `can_recover()` are `-EOPNOTSUPP`. Bit timing, mode and automatic bus-off
  management are the app's `HAL_CAN_Init()`; filters its
  `HAL_CAN_ConfigFilter()`, as CubeMX generates them.
- The link state reads ESR's three flags as the HAL names them, bus-off
  first, then passive, then warning; nothing more is inferred.
