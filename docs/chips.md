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
| HAL5 | STM32H5 HAL driver v1.7.0 (STMicroelectronics/stm32h5xx_hal_driver, commit 4b27033), `stm32h5xx_hal_fdcan.h`, `stm32h5xx_hal_fdcan.c`, `stm32h5xx_hal_rcc_ex.h`; BSD-3-Clause |
| CMSIS5 | cmsis_device_h5 v1.7.0 (commit 884b8dc), `stm32h563xx.h` |

The bxCAN's reference manual is RM0090 Rev 3 (STM32F40x, STM32F41x,
STM32F42x and STM32F43x, Doc ID 018909), `docs/rm0090.pdf` (gitignored),
chapter 24. It arrived after the driver was written against the HAL; with
it the driver sets the bit rate, the mode and the filter banks and leaves
bus-off. See [bxCAN](#bxcan).

The FDCAN's reference manual is: RM0481 Rev 5 (STM32H523/533xx,
STM32H543/553xx, STM32H562xx and STM32H563/573xx), `docs/rm0481.pdf`
(gitignored), chapter 60. It arrived after the driver was written against
the HAL, and was used to check the bit time, the bit-rate limits and the
bus-off recovery below. The FDCAN driver needs no register layout of its
own -- the H5 HAL has a call for every setting. See [FDCAN](#fdcan).

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
| `HAL_CAN_Init()`: writes MCR's options and BTR from `Init`, nothing else | HAL :649 |
| `CAN_MODE_NORMAL`, `_SILENT`, `_SILENT_LOOPBACK` | HAL :337-340 |
| `CAN_SJW_xTQ`, `CAN_BS1_xTQ`, `CAN_BS2_xTQ`: the value less one, at its BTR position | HAL :350-391 |
| MCR `CAN_MCR_INRQ`, `_ABOM`; MSR `CAN_MSR_INAK` | CMSIS :1671, :1689, :1702 |
| BTR `CAN_BTR_TS1_Pos`, `_TS2_Pos`, `_SJW_Pos` | CMSIS :1906-1919 |
| FMR `CAN_FMR_FINIT`, `CAN_FMR_CAN2SB` and its position | CMSIS :2211-2214 |

What RM0090 Rev 3 gives, and the driver does with it:

| What | Value | Source |
|---|---|---|
| Bit rate | up to 1 Mbit/s | 24.2 |
| Clock | APB1 (CAN1EN in RCC_APB1ENR) | 6.3.15 |
| Bit time | 1 + (TS1 + 1) + (TS2 + 1) quanta of (BRP + 1) APB1 periods | 24.7.7, Figure 235 |
| BTR fields | BRP[9:0], TS1[3:0] at 19:16, TS2[2:0] at 22:20, SJW[1:0] at 25:24; SILM 31, LBKM 30; written in initialisation mode only | 24.9.2, 24.9.1 |
| Listen | silent mode: receives, sends recessive bits only, "cannot start a transmission" -- so `can_send()` is `-EOPNOTSUPP` | 24.5.1 |
| Loopback | loop back combined with silent: CANRX cut off, CANTX held recessive -- off the bus | 24.5.3 |
| Filter banks | 28, shared: CAN1 below FMR.CAN2SB (reset 14), CAN2 from it; in CAN1's registers | 24.3.4, 24.9.4 |
| 32-bit bank | ID in FR1, mask in FR2: STID[10:0] at 31:21, EXID[17:0] at 20:3, IDE 2, RTR 1 | Figure 231 |
| Bank setup | FS1R scale, FM1R mode, FFA1R FIFO: writable with FMR.FINIT set; FINIT pauses reception; unused banks left off | 24.9.1, 24.4.1, 24.7.4 |
| Bus-off | BOFF while TEC > 255; left after 128 x 11 recessive bits, by itself with MCR.ABOM, else once software has set and cleared MCR.INRQ | 24.7.6, CAN_MCR in 24.9.2 |
| Initialisation mode | INRQ set; entered once MSR.INAK is set | 24.4.1 |

Policy, not RM0090's: the sample point, 87.5%, as the FDCAN's nominal
phase; the filters' bank layout -- one 32-bit mask bank per filter, IDE
compared, RTR not, all into FIFO 0; and waiting at most 10 ms for INAK, the
HAL's own `CAN_TIMEOUT_VALUE` (stm32f4xx_hal_can.c:237). The link state
reads ESR's flags bus-off first, then passive, then warning.

## FDCAN

Every name the driver uses is the HAL's or CMSIS's:

| What | Source |
|---|---|
| `HAL_FDCAN_STATE_READY`, `_BUSY`: initialised, started | HAL5 fdcan.h:51-52 |
| `FDCAN_FRAME_CLASSIC`, `_FD_NO_BRS`, `_FD_BRS` | HAL5 fdcan.h:538-540 |
| `FDCAN_MODE_NORMAL`, `_BUS_MONITORING`, `_INTERNAL_LOOPBACK` | HAL5 fdcan.h:548-551 |
| `FDCAN_STANDARD_ID`, `_EXTENDED_ID`; `FDCAN_DATA_FRAME`, `_REMOTE_FRAME` | HAL5 fdcan.h:592-593, :601-602 |
| `FDCAN_ESI_*`, `FDCAN_BRS_*`, `FDCAN_CLASSIC_CAN`, `FDCAN_FD_CAN`, `FDCAN_NO_TX_EVENTS` | HAL5 fdcan.h:633-660 |
| `FDCAN_FILTER_MASK` (FilterID1 the id, FilterID2 the mask), `FDCAN_FILTER_TO_RXFIFO0` | HAL5 fdcan.h:671, :681 |
| `FDCAN_RX_FIFO0`; `FDCAN_ACCEPT_IN_RX_FIFO0`, `FDCAN_REJECT`; `FDCAN_FILTER_REMOTE` | HAL5 fdcan.h:704, :768, :770, :778 |
| `FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST`, `FDCAN_IT_RX_FIFO0_NEW_MESSAGE` | HAL5 fdcan.h:855, :922 |
| Filter lists: 0-28 standard, 0-8 extended elements | HAL5 fdcan.h:112-116 |
| Tx FIFO of 3 elements, Rx FIFO 0 of 3 | HAL5 fdcan.c:213, :216 (`SRAMCAN_RF0_NBR`, `SRAMCAN_TFQ_NBR`) |
| DLC to length: 0-8, 12, 16, 20, 24, 32, 48, 64 | HAL5 fdcan.c:246 (`DLCtoBytes`); fdcan.h:610-625 (`FDCAN_DLC_BYTES_*`) |
| CCCR.INIT; CKDIV.PDIV | CMSIS5 :9104, :9610 |
| `RCC_PERIPHCLK_FDCAN`, `HAL_RCCEx_GetPeriphCLKFreq()` | HAL5 rcc_ex.h:576, :4774 |

### What the HAL does that the driver leans on

- `HAL_FDCAN_Init()` writes the bit timing from `Init` as given, each field
  less one, into NBTP, and into DBTP only with `FDCAN_FRAME_FD_BRS`; it
  sizes the filter lists from `StdFiltersNbr`/`ExtFiltersNbr` and **zeroes
  the whole message RAM**, filters included (fdcan.c:3447-3470). So every
  setting change re-initialises, then writes the filters back.
- `HAL_FDCAN_ConfigGlobalFilter()` only works in the READY state (fdcan.c
  :1380-1397): the driver stops the controller first.
- `HAL_FDCAN_Start()` leaves initialisation by clearing CCCR.INIT
  (fdcan.c:1994-2003); bus-off sets INIT again, and `can_recover()` clears
  it the same way.
- `HAL_FDCAN_GetRxMessage()` copies as many bytes as the DLC's length,
  up to 64, whatever the frame's format: the driver reads into a 64-byte
  buffer, and hands a Classic DLC of 9 to 15 up as 8.
- The kernel clock divider CKDIV is common to every FDCAN instance and set
  only from FDCAN1's `Init.ClockDivider` (fdcan.h:61-65, fdcan.c:414-418);
  the driver reads the register, not the handle.

### What RM0481 confirms

| Fact | RM0481 Rev 5 |
|---|---|
| bit time = 1 sync quantum + BS1 + BS2; tq = (BRP + 1) clock periods; BS1 = TSEG1 + 1, BS2 = TSEG2 + 1 quanta | 60.3.3 "Bit timing", p.2892 |
| the time-quantum clock at most the APB clock | 60.3.3, p.2893 |
| nominal bit rate up to 1 Mbit/s; data bit rate from the nominal up to 8 Mbit/s | 60.3.4 "Operating modes", p.2893 |
| bus-off sets INIT; once the CPU clears it, 129 x 11 recessive bits, then the error counters reset | 60.4.13, the note under FDCAN_PSR, p.2930 |
| transceiver delay compensation: the loop delay past the time before the sample point is a bit error; with DBTP.TDC the bit is checked at a secondary sample point, measured delay + TDCO, in mtq (FDCAN clock periods); delay + TDCO under six data bits, at most 127 mtq | 60.3.4, "Transceiver delay compensation", pp.2896-2897 |
| DBTP.TDC, bit 23 | 60.4.3, p.2921 |
| TDCR: TDCO[6:0] bits 14:8, TDCF[6:0] bits 6:0, 0-127 mtq, writable only with CCE and INIT set | 60.4.14, p.2931 |

`can_set_bitrate()` refuses more than 1 Mbit/s accordingly, and
`can_set_data_bitrate()` more than 8 Mbit/s or less than the nominal rate.

### Transmitter delay compensation

Above 1 Mbit/s, `can_set_data_bitrate()` turns TDC on: TDCO at the data
sample point, `DataPrescaler x (1 + DataTimeSeg1)` mtq into the bit, TDCF 0,
through `HAL_FDCAN_ConfigTxDelayCompensation()` and
`HAL_FDCAN_EnableTxDelayCompensation()` (both READY only). At 1 Mbit/s and
below it stays off. The threshold and the offset are this library's choice:
RM0481's own example puts the SSP half a bit in; at or below 1 Mbit/s with a
75% sample point the loop delay would have to exceed 750 ns to matter.
`HAL_FDCAN_Init()` writes DBTP whole (stm32h5xx_hal_fdcan.c:505), clearing
TDC, so every re-initialisation sets it again. An offset past TDCO's 127 mtq
-- a fast FDCAN clock at a high data rate -- is `-EINVAL`: use CKDIV or a
slower clock.

An app-configured TDC is preserved: `fdcan_init()` reads DBTP.TDC and
TDCR's TDCO and TDCF as the app left them, and every re-initialisation
writes them back. If CubeMX sets a data rate above 1 Mbit/s without TDC,
call `can_set_data_bitrate()`, which turns it on.

With TDC a data prescaler of 1 or 2 is advised -- CAN in Automation's CAN
FD bit-timing guidance (CiA 601-3) and ISO 11898-1, as the review relayed
them; neither is in `docs/`, and RM0481 does not say it. The timing search
tries the smallest prescaler first, so it lands on 1 or 2 whenever one of
them makes the rate exactly. When neither does -- 240 MHz at 2 Mbit/s needs
3 -- the rate is still taken and the driver logs a warning.

With BRS, `can_set_bitrate()` also refuses a nominal rate above the data
rate the handle's `Init` sets: the data rate is "between nominal bitrate and
8 Mbit/s" (RM0481 60.3.4).

### Bit timing

A bit is one sync quantum plus `TimeSeg1` plus `TimeSeg2`, each quantum
`Prescaler` FDCAN clock periods. The ranges are the HAL's field comments
(fdcan.h:82-110):

| | Prescaler | TimeSeg1 | TimeSeg2 | SyncJumpWidth |
|---|---|---|---|---|
| nominal | 1-512 | 2-256 | 2-128 | 1-128 |
| data | 1-32 | 1-32 | 1-16 | 1-16 |

`src/fdcan/fdcan_math.h` takes the smallest prescaler that makes the rate
exactly within those ranges, puts the sample point nearest 87.5% (nominal)
or 75% (data) -- this library's policy, not a standard's -- and widens SJW
to TimeSeg2. `test/test_fdcan_math.c` works rows by hand and sweeps eleven
clocks by thirteen rates for the invariants.

### Not done, and why

- **Measuring or configuring the transceiver's loop delay.** The controller
  measures it in every FD frame (FDCAN_PSR.TDCV); the driver neither reads
  it nor checks RM0481's "delay plus offset under six data bits".
- **The TDC filter window** (TDCF) stays 0, off.
- **Rx FIFO 1, Tx buffers, the Tx event FIFO, timestamps from the
  controller, high-priority messages**: not used. Frames are stamped with
  `can_time_us()` when taken from the FIFO.

