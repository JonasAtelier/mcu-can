# API usage

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

## The handles and the category view

| Call | Does |
|---|---|
| `mcp2515_init(&cfg, &h)` | resets the chip, checks it answers in Configuration mode, writes and reads back the bit timing, accepts every frame, enables nINT's two receive flags if `cfg.int_gpio`/`int_port` is wired, and enters the running mode |
| `mcp2515_send/receive/set_filter/read_status/clear_errors/del` | esp-mcp2515-can's calls, unchanged |
| `mcp2515_can(h)` | the handle as a `struct can_dev` |
| `twaican_init(&cfg, &h)` | ESP-IDF: installs the legacy driver on `cfg.controller` and starts it, transceiver awake |
| `twaican_standby/wake(h)` | the SN65HVD230's Rs pin, when `cfg.rs_gpio` is wired |
| `twaican_service(h)` | bus-off recovery on a timer; call it from the app's loop |
| `twaican_del(h)` / `twaican_can(h)` | stop and uninstall / the `struct can_dev` |
| `bxcan_init(&cfg, &h)` | HAL: takes the app's `HAL_CAN_Init()`ed handle and starts it unless the app has |
| `bxcan_irq(h)` | from `HAL_CAN_RxFifo0MsgPendingCallback()`: FIFO 0 into the ring |
| `bxcan_del(h)` / `bxcan_can(h)` | stop / the `struct can_dev` |
| `fdcan_init(&cfg, &h)` | HAL: takes the app's `HAL_FDCAN_Init()`ed handle; unless the app has started it, every frame into Rx FIFO 0 and start |
| `fdcan_irq(h)` | from `HAL_FDCAN_RxFifo0Callback()`: Rx FIFO 0 into the ring |
| `fdcan_del(h)` / `fdcan_can(h)` | stop / the `struct can_dev` |

and through any `struct can_dev`:

| Call | Does |
|---|---|
| `can_send(c, &f, ms)` | queue one frame, waiting up to `ms` for room |
| `can_recv(c, &f, ms)` | the next frame that passes the filters, stamped |
| `can_send_fd(c, &f, ms)` / `can_recv_fd(c, &f, ms)` | the same with `struct canfd_frame`: CAN FD and Classic frames |
| `can_set_filters(c, filters, n)` | accept only what passes one of `n` filters; 0 accepts all |
| `can_set_bitrate(c, bps)` | a new bit rate |
| `can_set_data_bitrate(c, bps)` | CAN FD: the bit rate of a BRS frame's data phase |
| `can_set_opmode(c, mode)` | normal, loopback or listen-only |
| `can_get_status(c, &st)` | link state, error counters, losses, what waits |
| `can_recover(c)` | leave bus-off |
| `can_start(c)` / `can_stop(c)` | onto the bus / off it |
| `can_irq(c)` / `can_irq_pending(c)` | an INT line counted from an ISR, seen in a thread |
| `can_log_frame(c, &f)` | one info line |

On ESP-IDF the chip calls return `esp_err_t`; on the HAL, 0 or a negative
errno; every `can_*` call errno on both. An optional op a controller does
not have is `-EOPNOTSUPP`.

## Frames

`struct can_frame` is `id`, `dlc` (0-8), `ext` (29-bit), `rtr` (remote),
`data[8]` and `t_us`. A remote frame carries no data; its `dlc` is the
length it asks for. `can_send()` refuses a `dlc` over 8 or an `id` past
its format's range with `-EINVAL`. `can_recv()` stamps `t_us` with
`can_time_us()` when the frame comes out of the driver -- or, on bxCAN with
`use_irq`, when `bxcan_irq()` took it out of the FIFO.

A Classic DLC of 9 to 15 on the wire is handed up as 8.

**CAN FD.** `struct canfd_frame` is `id`, `len`, `ext`, `rtr`, `fd`, `brs`,
`esi`, `data[64]` and `t_us`. `len` is the length in bytes, not the DLC:
0-8 for Classic, and for FD one of 0-8, 12, 16, 20, 24, 32, 48, 64;
`canfd_len_to_dlc()` and `canfd_dlc_to_len()` convert. `can_send_fd()`
refuses another FD length, an FD remote frame (FD has none), a Classic
`len` over 8 and `brs` without `fd` with `-EINVAL`. `esi`, on a received FD
frame, says its sender was error passive.

The FD calls work on every controller. On the MCP2515, TWAI and bxCAN,
`can_send_fd()` sends a Classic frame through `can_send()` and refuses an
FD one with `-EOPNOTSUPP`, and `can_recv_fd()` returns `can_recv()`'s
frames with `fd` false. On the FDCAN, `can_recv_fd()` returns FD and
Classic frames alike, while `can_recv()` returns Classic ones only and
counts the FD frames it passes over in `can_status.rx_fd_dropped`: use one
or the other per controller.

What an FDCAN puts on the wire is its app's `Init.FrameFormat`:
`FDCAN_FRAME_CLASSIC` refuses FD frames, `FDCAN_FRAME_FD_NO_BRS` refuses
`brs`, `FDCAN_FRAME_FD_BRS` takes both (`-EOPNOTSUPP` for what it refuses).

## Filters

A `struct can_filter` passes a frame when `ext` matches and
`(id & mask) == (filter.id & mask)`; mask 0 passes every frame of that
format. `can_recv()` checks the filters again in software, so what a
controller's hardware lets through beyond them never reaches the app.

| Controller | Filters | When |
|---|---|---|
| MCP2515 | up to 2: the first on RXB0, the second on RXB1; one alone goes on both | any time; Configuration mode for the writes, which clears TEC and REC |
| TWAI | 1, the single filter mode; it cannot tell standard from extended frames, the software check does | only while stopped |
| bxCAN | 2: each a 32-bit mask bank into FIFO 0, standard or extended; the controller's other banks turned off; none is one bank of mask 0 | any time; filter initialisation mode pauses reception meanwhile |
| FDCAN | 2: each a mask filter element into Rx FIFO 0, standard or extended; the rest rejected; none accepts all | any time: stops, re-initialises, starts again |

On the FDCAN, `can_set_filters()`, `can_set_bitrate()`,
`can_set_data_bitrate()` and `can_set_opmode()` all rewrite the handle's
`Init` and run `HAL_FDCAN_Init()` again, which clears the message RAM, then
write the filters last set back. Filters the app wrote through
`HAL_FDCAN_ConfigFilter()` itself do not survive any of them.

`mcp2515_set_filter(h, id, mask, ext)` keeps esp-mcp2515-can's meaning: one
filter on both buffers, and mask 0 accepts **everything**, both formats.

## Modes, bit rate, start and stop

| | MCP2515 | TWAI | bxCAN | FDCAN |
|---|---|---|---|---|
| `can_set_opmode()` | any time; while stopped, at start | while stopped | any time (re-init) | any time (re-init) |
| loopback | Loopback mode | NO_ACK, every frame sent with self-reception | `CAN_MODE_SILENT_LOOPBACK`: off the bus | `FDCAN_MODE_INTERNAL_LOOPBACK`: off the bus |
| listen-only | Listen-Only mode | LISTEN_ONLY; `can_send()` is `-EOPNOTSUPP` | `CAN_MODE_SILENT`; sending is `-EOPNOTSUPP` | `FDCAN_MODE_BUS_MONITORING`; sending is `-EOPNOTSUPP` |
| `can_set_bitrate()` | 125k, 250k, 500k; 1M at 16 MHz | 25k, 50k, 100k, 125k, 250k, 500k, 800k, 1M, while stopped | up to 1M, any rate APB1 divides into exactly; sample point 87.5% (re-init) | up to 1M, any rate the FDCAN clock divides into exactly; sample point 87.5% |
| `can_set_data_bitrate()` | `-EOPNOTSUPP` | `-EOPNOTSUPP` | `-EOPNOTSUPP` | with `FDCAN_FRAME_FD_BRS` only, from the nominal rate up to 8M, exact; sample point 75%; above 1M with transmitter delay compensation |
| `can_stop()` | Configuration mode: off the bus, counters cleared | stop and uninstall; queued frames dropped | `HAL_CAN_Stop()` | `HAL_FDCAN_Stop()` |
| `can_start()` | back to the running mode | install with the settings as they are, then start | `HAL_CAN_Start()` | `HAL_FDCAN_Start()` |

The FDCAN clock is the kernel clock (`HAL_RCCEx_GetPeriphCLKFreq
(RCC_PERIPHCLK_FDCAN)`) over the CKDIV divider. A rate it does not divide
into exactly is `-EINVAL`: an 8 MHz HSE makes 1 Mbit/s with 8 quanta and a
75% sample point, 80 MHz makes 80 quanta at 87.5%. The sample points are
this library's policy. The nominal rate stops at 1 Mbit/s and the data rate
at 8 Mbit/s, no slower than the nominal one, RM0481's limits (60.3.4).
Above 1 Mbit/s the data phase runs with transmitter delay compensation, its
offset at the data sample point ([chips](chips.md#transmitter-delay-compensation)).
The transceiver's loop delay plus that offset must stay under six data bits
(RM0481 60.3.4): check it against the transceiver's datasheet.

The bxCAN's clock is APB1's (`HAL_RCC_GetPCLK1Freq()`); the same search
finds its timing within CAN_BTR's ranges, 1 to 16 quanta before the sample
point and 1 to 8 after (RM0090 24.7.7). 42 MHz makes 500 kbit/s and
1 Mbit/s with 14 quanta, the sample point at 12. On the bxCAN and the FDCAN
both, `can_set_bitrate()` and `can_set_opmode()` stop a running controller,
run the HAL's init again with the new setting and start it again. **Frames
still queued to send are dropped** on both: the FDCAN's init clears them,
and the bxCAN driver aborts its three mailboxes first
(`HAL_CAN_AbortTxRequest()`), so nothing goes out under the new setting
that was queued under the old.

The TWAI's settings that need a stopped controller answer `-EBUSY`
otherwise (`ESP_ERR_INVALID_STATE` through the handle API).

## Status and recovery

`can_get_status()` fills `struct can_status`; counters a controller does not
keep read 0.

| Field | MCP2515 | TWAI | bxCAN | FDCAN |
|---|---|---|---|---|
| `link` | from EFLG: TXBO, TXEP/RXEP, EWARN | from the driver's state and the counters: 96 warning, 128 passive | from ESR: BOFF (RECOVERING with MCR.ABOM set or after `can_recover()`), EPVF, EWGF; STOPPED unless started | from `HAL_FDCAN_GetProtocolStatus()`: BusOff (BUS_OFF while CCCR.INIT is set, RECOVERING after), ErrorPassive, Warning; STOPPED unless started |
| `tec`, `rec` | TEC, REC | the driver's counters (TEC 256 in bus-off) | ESR | `HAL_FDCAN_GetErrorCounters()` |
| `rx_lost` | 1 while EFLG.RXnOVR is set | missed plus overrun | ring drops, plus 1 while FIFO 0's overrun flag is set | ring drops, plus each Rx FIFO 0 overflow: the message-lost flag counted and cleared |
| `tx_failed`, `arb_lost`, `bus_errors` | 0 | the driver's counts | 0 | 0 |
| `rx_pending`, `tx_pending` | full RX buffers, TX buffers with TXREQ | the driver's queues | ring or FIFO level, busy mailboxes | ring or FIFO level, busy Tx FIFO elements of 3 |
| `rx_fd_dropped` | 0 | 0 | 0 | FD frames `can_recv()` passed over |

Leaving bus-off:

- **MCP2515**: on its own, once the bus has been idle for 128 x 11
  recessive bits. `can_recover()` returns 0 and does nothing.
- **TWAI**: `can_recover()` starts the recovery (`-EBUSY` unless bus-off).
  The driver leaves the controller stopped after it; `can_start()` starts it
  again. Or set `cfg.restart_ms` and call `twaican_service()` from the app's
  loop, which does both, as esp-sn65hvd230-can's `hvd230_service()` does.
- **bxCAN**: with the app's `Init.AutoBusOff` (MCR.ABOM) the controller
  leaves bus-off by itself after 128 x 11 recessive bits, `link` reading
  RECOVERING meanwhile, and `can_recover()` returns 0. Without it,
  `can_recover()` enters and leaves initialisation mode, which lets that
  wait start (RM0090 24.7.6); `-ETIMEDOUT` if initialisation mode is not
  entered within 10 ms. `-EBUSY` unless bus-off, `-ENETDOWN` while
  stopped, as on the FDCAN. The controller has no flag for a recovery under
  way, so RECOVERING after `can_recover()` is the driver's note, kept until
  it sees BOFF clear: a recovery and a second bus-off both between two
  `can_get_status()` calls still read RECOVERING. `can_recover()` works
  whenever BOFF is set, so call it again if RECOVERING lasts longer than
  the bus could take.
- **FDCAN**: bus-off sets CCCR.INIT and the controller stays off the bus.
  `can_recover()` clears INIT, as `HAL_FDCAN_Start()` does to leave
  initialisation; the controller then waits 129 x 11 recessive bits by
  itself, resets its error counters (RM0481 60.4.13), and `link` reads
  RECOVERING until it is back.
  `-EBUSY` unless bus-off, `-ENETDOWN` while stopped.

## The INT line

The MCP2515 driver samples nINT itself when `cfg.int_gpio` / `int_port` is
wired: a high pin skips the SPI poll. To sleep until a frame comes instead,
call `can_irq(can)` from the pin's ISR or EXTI callback and wait on
`can_irq_pending(can)` in a thread before `can_recv()`. `bxcan_irq()` calls
`can_irq()` itself.

## bxCAN

The app owns the peripheral's clock, pins and first setup. Before
`bxcan_init()`:

```c
hcan1.Init.Prescaler = ...;          /* from CubeMX: the bit timing */
hcan1.Init.Mode = CAN_MODE_NORMAL;
hcan1.Init.AutoBusOff = ENABLE;      /* leave bus-off on its own */
HAL_CAN_Init(&hcan1);
HAL_CAN_ConfigFilter(&hcan1, &filter);   /* to FIFO 0 */
```

After that, bit rate, filters and mode are the library's to set too (see
above). `can_set_filters()` takes over all the controller's filter banks:
on a part with two CANs, CAN1's are those below `CAN_FMR.CAN2SB` and
CAN2's the rest, both in CAN1's registers (RM0090 24.9.4); it needs two
of them, `-ENOSPC` otherwise; a CAN2SB past 28, which RM0090 does not
define, is `-EINVAL`. A filter the app set on FIFO 1 is turned off with the
rest.

With `use_irq`, enable the CAN RX0 interrupt in the NVIC and call
`bxcan_irq()` from the HAL's callback; the driver turns on the FIFO 0
message-pending notification itself. Without, `can_recv()` polls FIFO 0,
whose fill level is two bits (CMSIS `CAN_RF0R_FMP0`): three frames at most.
Poll faster than they come, or lose them.

## FDCAN

The app owns the peripheral's clock, pins and first setup, CubeMX-style,
before `fdcan_init()`:

```c
hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;  /* or _CLASSIC, _FD_NO_BRS */
hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
HAL_FDCAN_Init(&hfdcan1);
```

`fdcan_init()` then, unless the app has started the controller already,
sets the global filter to take every frame into Rx FIFO 0 and starts it.
After that, bit rates, filters and the mode are the library's to set (see
above). With `use_irq`, enable the FDCAN's interrupt line 0 in the NVIC and
call `fdcan_irq()` from `HAL_FDCAN_RxFifo0Callback()`; the driver turns on
the new-message notification itself. Without, `can_recv()` polls Rx FIFO 0,
which holds three frames (`SRAMCAN_RF0_NBR`, stm32h5xx_hal_fdcan.c:213):
poll faster than they come, or lose them.

## Errors

| errno | esp_err_t (handle API) | Means |
|---|---|---|
| `-EINVAL` | `ESP_ERR_INVALID_ARG` | a bad argument or config; no timing row for the crystal and rate |
| `-ENODEV` | `ESP_ERR_NOT_FOUND` | no MCP2515 answers, or its timing did not stick |
| `-EPROTO` | `ESP_ERR_INVALID_RESPONSE` | the MCP2515 would not change mode within 10 ms |
| `-ETIMEDOUT` | `ESP_ERR_TIMEOUT` | nothing within the timeout |
| `-EBUSY` | `ESP_ERR_INVALID_STATE` | TWAI: not in a state for that (running, recovering, installed elsewhere); FDCAN: `can_recover()` when not bus-off |
| `-EOPNOTSUPP` | `ESP_ERR_NOT_SUPPORTED` | the controller has no such op, or listen-only refused a send |
| `-ENOSPC` | -- | more filters than the controller has |
| `-ENETDOWN` | `ESP_FAIL` | TWAI stopped, or its transceiver in standby; FDCAN stopped, for `can_recover()` |
| `-ENOMEM` | `ESP_ERR_NO_MEM` | allocation |
| `-EIO`, `-EAGAIN` | `ESP_FAIL` | the bus or the HAL failed |

## Changes from esp-mcp2515-can

`mcp2515.h` keeps every type and call: a token-level diff of the
preprocessed header against esp-mcp2515-can 210a3c0 removes nothing. What
changed in behaviour:

- **Waits are on the clock.** esp-mcp2515-can slept with
  `vTaskDelay(pdMS_TO_TICKS(1))` and counted `portTICK_PERIOD_MS` per loop;
  at ESP-IDF's default 100 Hz that is no wait at all, so a 100 ms timeout
  ended after ten quick polls. Every wait now runs to its deadline on
  `can_time_us()`.
- **8 MHz at 1 Mbit/s is refused** (`ESP_ERR_INVALID_ARG`); its PS2 broke
  the datasheet's 2 TQ minimum. See [chips](chips.md#bit-timing).
- **CNF3.SOF is never set**: CLKOUT stays a clock output. The four 16 MHz
  rows and the 8 MHz 1 Mbit/s row set it before.
- `mcp2515_del()` returns `ESP_OK` once the device is removed; it returned
  `spi_bus_remove_device()`'s code.
- New: the STM32 HAL, `mcp2515_can()`, and through it two filters, bit
  rate and mode changes, stop and start.
