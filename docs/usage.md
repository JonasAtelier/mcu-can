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

and through any `struct can_dev`:

| Call | Does |
|---|---|
| `can_send(c, &f, ms)` | queue one frame, waiting up to `ms` for room |
| `can_recv(c, &f, ms)` | the next frame that passes the filters, stamped |
| `can_set_filters(c, filters, n)` | accept only what passes one of `n` filters; 0 accepts all |
| `can_set_bitrate(c, bps)` | a new bit rate |
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

A DLC of 9 to 15 on the wire is handed up as 8: no controller here stores
more than eight data bytes.

## Filters

A `struct can_filter` passes a frame when `ext` matches and
`(id & mask) == (filter.id & mask)`; mask 0 passes every frame of that
format. `can_recv()` checks the filters again in software, so what a
controller's hardware lets through beyond them never reaches the app.

| Controller | Filters | When |
|---|---|---|
| MCP2515 | up to 2: the first on RXB0, the second on RXB1; one alone goes on both | any time; Configuration mode for the writes, which clears TEC and REC |
| TWAI | 1, the single filter mode; it cannot tell standard from extended frames, the software check does | only while stopped |
| bxCAN | none: `-EOPNOTSUPP`; the app calls `HAL_CAN_ConfigFilter()` | -- |

`mcp2515_set_filter(h, id, mask, ext)` keeps esp-mcp2515-can's meaning: one
filter on both buffers, and mask 0 accepts **everything**, both formats.

## Modes, bit rate, start and stop

| | MCP2515 | TWAI | bxCAN |
|---|---|---|---|
| `can_set_opmode()` | any time; while stopped, at start | while stopped | `-EOPNOTSUPP` (the app's `Init.Mode`) |
| loopback | Loopback mode | NO_ACK, every frame sent with self-reception | -- |
| listen-only | Listen-Only mode | LISTEN_ONLY; `can_send()` is `-EOPNOTSUPP` | -- |
| `can_set_bitrate()` | 125k, 250k, 500k; 1M at 16 MHz | 25k, 50k, 100k, 125k, 250k, 500k, 800k, 1M, while stopped | `-EOPNOTSUPP` (the app's `Init`) |
| `can_stop()` | Configuration mode: off the bus, counters cleared | stop and uninstall; queued frames dropped | `HAL_CAN_Stop()` |
| `can_start()` | back to the running mode | install with the settings as they are, then start | `HAL_CAN_Start()` |

The TWAI's settings that need a stopped controller answer `-EBUSY`
otherwise (`ESP_ERR_INVALID_STATE` through the handle API).

## Status and recovery

`can_get_status()` fills `struct can_status`; counters a controller does not
keep read 0.

| Field | MCP2515 | TWAI | bxCAN |
|---|---|---|---|
| `link` | from EFLG: TXBO, TXEP/RXEP, EWARN | from the driver's state and the counters: 96 warning, 128 passive | from ESR: BOFF, EPVF, EWGF; STOPPED unless started |
| `tec`, `rec` | TEC, REC | the driver's counters (TEC 256 in bus-off) | ESR |
| `rx_lost` | 1 while EFLG.RXnOVR is set | missed plus overrun | ring drops, plus 1 while FIFO 0's overrun flag is set |
| `tx_failed`, `arb_lost`, `bus_errors` | 0 | the driver's counts | 0 |
| `rx_pending`, `tx_pending` | full RX buffers, TX buffers with TXREQ | the driver's queues | ring or FIFO level, busy mailboxes |

Leaving bus-off:

- **MCP2515**: on its own, once the bus has been idle for 128 x 11
  recessive bits. `can_recover()` returns 0 and does nothing.
- **TWAI**: `can_recover()` starts the recovery (`-EBUSY` unless bus-off).
  The driver leaves the controller stopped after it; `can_start()` starts it
  again. Or set `cfg.restart_ms` and call `twaican_service()` from the app's
  loop, which does both, as esp-sn65hvd230-can's `hvd230_service()` does.
- **bxCAN**: the app's `Init.AutoBusOff`; `can_recover()` is
  `-EOPNOTSUPP`.

## The INT line

The MCP2515 driver samples nINT itself when `cfg.int_gpio` / `int_port` is
wired: a high pin skips the SPI poll. To sleep until a frame comes instead,
call `can_irq(can)` from the pin's ISR or EXTI callback and wait on
`can_irq_pending(can)` in a thread before `can_recv()`. `bxcan_irq()` calls
`can_irq()` itself.

## bxCAN

The app owns the peripheral. Before `bxcan_init()`:

```c
hcan1.Init.Prescaler = ...;          /* from CubeMX: the bit timing */
hcan1.Init.Mode = CAN_MODE_NORMAL;
hcan1.Init.AutoBusOff = ENABLE;      /* leave bus-off on its own */
HAL_CAN_Init(&hcan1);
HAL_CAN_ConfigFilter(&hcan1, &filter);   /* to FIFO 0 */
```

With `use_irq`, enable the CAN RX0 interrupt in the NVIC and call
`bxcan_irq()` from the HAL's callback; the driver turns on the FIFO 0
message-pending notification itself. Without, `can_recv()` polls FIFO 0,
whose fill level is two bits (CMSIS `CAN_RF0R_FMP0`): three frames at most.
Poll faster than they come, or lose them.

## Errors

| errno | esp_err_t (handle API) | Means |
|---|---|---|
| `-EINVAL` | `ESP_ERR_INVALID_ARG` | a bad argument or config; no timing row for the crystal and rate |
| `-ENODEV` | `ESP_ERR_NOT_FOUND` | no MCP2515 answers, or its timing did not stick |
| `-EPROTO` | `ESP_ERR_INVALID_RESPONSE` | the MCP2515 would not change mode within 10 ms |
| `-ETIMEDOUT` | `ESP_ERR_TIMEOUT` | nothing within the timeout |
| `-EBUSY` | `ESP_ERR_INVALID_STATE` | TWAI: not in a state for that (running, recovering, installed elsewhere) |
| `-EOPNOTSUPP` | `ESP_ERR_NOT_SUPPORTED` | the controller has no such op, or listen-only refused a send |
| `-ENOSPC` | -- | more filters than the controller has |
| `-ENETDOWN` | `ESP_FAIL` | TWAI stopped, or its transceiver in standby |
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
