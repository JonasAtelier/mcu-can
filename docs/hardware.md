# Hardware

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

None of this has been checked on a bench with this library.

## Every controller needs a transceiver

A CAN controller -- the MCP2515, the TWAI, the bxCAN -- speaks logic levels
on TX and RX. A transceiver turns them into the differential CANH/CANL pair.
With a 3.3 V MCU, the SN65HVD230 runs from the same 3.3 V supply (SLOS
pin functions: VCC, "Transceiver 3.3V supply voltage").

Terminate the bus at both ends with resistors equal to the cable's 120 Ω
characteristic impedance, and keep stubs short (SLOS 11.2.1.1, citing ISO
11898). A bench setup with two boards needs both terminators, wherever they
sit.

## MCP2515

| Pin | To |
|---|---|
| SCK, SI, SO, CS | the MCU's SPI: mode 0,0 or 1,1, 10 MHz at most (DS Section 12.1) |
| INT | an MCU input, optional: `cfg.int_gpio` / `int_port` |
| TXCAN, RXCAN | the transceiver's TXD and RXD |
| OSC1, OSC2 | the crystal |
| VDD | 2.7 V to 5.5 V (DS p.1) |

**The crystal.** 8 MHz and 16 MHz are the two the timing table knows; read
the part number on the can and set `crystal_hz` to match. A wrong value is
a wrong bit rate, and nothing on the bus says so -- it just looks dead.
1 Mbit/s needs the 16 MHz one. The chip takes up to 25 MHz at 2.7-5.5 V,
40 MHz at 4.5-5.5 V (DS Oscillator Timing, p.75).

**INT** is driven low while an enabled flag is pending (DS Section 7.0).
With it wired, the driver enables the two receive flags and skips the SPI
poll while it is high; on ESP-IDF it also turns on the pin's pull-up, as
esp-mcp2515-can did.

**CLKOUT** carries the oscillator divided by 8 from power-on (CANCTRL
CLKEN = 1, CLKPRE = 11, DS Register 10-1), and this driver leaves it so:
it never sets CNF3.SOF, which would turn it into a start-of-frame signal.

A breakout that runs the MCP2515 from 5 V drives SO and INT at 5 V; check
that a 3.3 V MCU's pins take it before wiring them straight in.

## TWAI (ESP32)

TX and RX go to the transceiver's TXD and RXD on any GPIOs the TWAI can be
routed to; `cfg.tx_gpio`, `cfg.rx_gpio`.

**The SN65HVD230's Rs pin (pin 8)**, when wired to a GPIO
(`cfg.rs_gpio`): low (< 1.2 V) is high-speed mode, high (> 0.75 VCC) the
listen-only standby, driver off and receiver on (SLOS 10.4.1, 10.4.3).
`twaican_init()` drives it low before starting the controller. Left
unwired, Rs is whatever the board ties it to.
**Leave `rs_gpio` at `TWAICAN_NO_GPIO` (-1) when it is not wired**: 0 is
GPIO0.

## bxCAN (STM32)

CAN1_TX and CAN1_RX (or CAN2's) go to the transceiver. Their alternate
function and the bit timing come from CubeMX, as `HAL_CAN_MspInit()` and
`hcan.Init`; this library uses what the app set up.

## FDCAN (STM32)

FDCANx_TX and FDCANx_RX go to a CAN FD-capable transceiver; pins, the
alternate function and the kernel clock come from CubeMX
(`HAL_FDCAN_MspInit()`). The kernel clock decides which bit rates exist:
`can_set_bitrate()` takes only rates it divides into exactly, so pick a
clock that is a multiple of every rate the bus uses (an 8 MHz HSE makes the
Classic rates up to 1 Mbit/s; 80 MHz makes them with a finer sample point).
CKDIV, the divider all FDCAN instances share, is set once through FDCAN1's
`Init.ClockDivider` (stm32h5xx_hal_fdcan.h:61-65). The time-quantum clock
must not run faster than the FDCAN's APB clock (RM0481 60.3.3).

Fast data phases depend on the transceiver: above 1 Mbit/s the driver uses
transmitter delay compensation, and the transceiver's loop delay plus the
offset (the data sample point, 75% of a bit) must stay under six data bits
(RM0481 60.3.4). The loop delay is in the transceiver's datasheet.

### On the Nexus-S

`hardware/nexus-s` (GPIO.md, README.md) wires the STM32H563's two FDCANs to
two TCAN1044V transceivers:

| | Pins | STB | Bus |
|---|---|---|---|
| FDCAN1 | PD0 RX, PD1 TX, AF9 | PD3 | CAN FD, CAN1A/CAN1B |
| FDCAN2 | PB5 RX, PB6 TX, AF9 | PB7 | Classic 1 Mbit/s, the ESP32 TWAI nodes, CAN2A/CAN2B |

Each STB has the transceiver's internal pull-up: the transceiver stays in
standby until the firmware drives STB low (`examples/stm/fdcan/`). Keep
FDCAN2 at `FDCAN_FRAME_CLASSIC`: a TWAI is Classic CAN, and an FD frame on
its bus is an error frame to every one of them. FDCAN2's TX is on PB6 rather
than PB13 because the ROM bootloader drives FDCAN2_TX on PB13.
