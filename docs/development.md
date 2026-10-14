# Architecture, driver development and tests

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

## Repository layout

```
include/                    public API, the only directory apps include
  can.h                     struct can_dev, frames, filters, status, modes
  can_log.h                 can_err/warn/info/dbg, can_log_frame()
  mcp2515.h                 esp-mcp2515-can's handle API, the HAL's, mcp2515_can()
  twai_can.h                the TWAI on ESP-IDF's legacy driver, twaican_can()
  bxcan.h                   bxCAN on the HAL's CAN module, bxcan_irq(), bxcan_can()
  fdcan_can.h               FDCAN on the HAL's FDCAN module, fdcan_irq(), fdcan_can()
src/                        private to the library
  can.c                     the core: checks, the software filter, the stamp, the IRQ count, the frame ring, the CAN FD calls and their Classic fallback
  can_log.c                 can_log_frame()
  can_priv.h                bus ops, attach and probe, errno maps, waits, the frame ring
  can_port_esp.c            all of that on ESP-IDF; empty elsewhere
  can_port_stm.c            all of that on the STM32 HAL; empty under ESP-IDF
  mcp2515/                  mcp2515.c  mcp2515_regs.h  mcp2515_math.h
  twai/                     twai_can.c (ESP-IDF only)
  bxcan/                    bxcan.c (STM32 HAL only)
  fdcan/                    fdcan.c (STM32 HAL only)  fdcan_math.h
test/                       host tests, `make -C test test`
  test_mcp2515_math.c       every bit-timing row against the datasheet's rules; identifier packing
  test_fdcan_math.c         the FDCAN timing search: hand-worked rows, refusals, a sweep of invariants
  test_names.c              esp-sn65hvd230-can's header and this library's in one file
  core_cases.h              core cases, shared by esp/ and stm/
  mcp2515_cases.h           a fake MCP2515 at SPI level and its cases, shared
  esp/                      suites on idf_stubs/, through can_port_esp.c, plus the TWAI
  stm/                      suites on hal_stubs/, through can_port_stm.c, plus bxCAN and FDCAN
  idf_stubs/  hal_stubs/    the slice of ESP-IDF and of the HAL the code calls
  fixtures/                 hvd230can.h, a verbatim copy, for test_names
examples/
  esp/<name>/               ESP-IDF projects: mcp2515 (esp-mcp2515-can's), twai, dual
  stm/<name>/               STM32 snippets: mcp2515, bxcan, fdcan
docs/                       the five guides; the datasheets here are gitignored
CMakeLists.txt              ESP-IDF only; stm-forge globs src/**/*.c instead
Kconfig                     CAN and CAN_<CHIP>, for ESP-IDF and both forges' menus
idf_component.yml           ESP-IDF component manifest
LICENSE                     MIT
```

Naming inside `src/<chip>/`, the same in every category library:

| File | Holds |
|---|---|
| `<chip>.c` | the driver: init, del, the ops, the chip's calls |
| `<chip>_regs.h` | register map, each value with its source citation |
| `<chip>_proto.h` | serial protocol, for parts without registers |
| `<chip>_math.h` | pure conversions, compiled and asserted on the host |
| `<chip>_priv.h` | driver state, only if needed |

The TWAI, bxCAN and FDCAN drivers have no register file: every value they
use is ESP-IDF's or the HAL's own name. They are `twai_can.c` and `bxcan.c`
rather than `twai.c` -- ESP-IDF has a `twai.c` of its own -- and the FDCAN's
header is `fdcan_can.h`, since CubeMX generates an `fdcan.h`. The MCP2515 driver keeps
the original's four-space style; the library's own files use tabs.

**Names next to esp-sn65hvd230-can.** Apps link both, so nothing here may
share a name with it: not a header, not a source file, not a type, tag,
enumerator, macro or global symbol. Everything public is `can_*`, `canfd_*`,
`CAN_FRAME_*`, `CANFD_FRAME_*`, `CAN_FILTER_*`, `CAN_LINK_*`,
`CAN_OPMODE_*`, `mcp2515_*`, `twaican_*`, `bxcan_*` or `fdcan_*`; `CAN_STATE_*`, `CAN_OK`, `CAN_MAX_DLC`,
`can_frame_t`, `can_state`, `can_lock` and the like are taken. Nor may a
name be one of the STM32 HAL's `CAN_*` (`CAN_MODE_*`, `CAN_ID_*`,
`CAN_IT_*`, ...) or `FDCAN_*`; the H5 HAL and its CMSIS header have no
lowercase `fdcan_` or `canfd_` name either (a grep finds only a local
variable, `fdcan_frequency`). `test_names` and the forge build's `nm` check hold this.

## Handles first

As in the other category libraries: each handle embeds a `struct can_dev`,
init fills it and points `priv` back at the handle, and `<chip>_can()`
hands it out. `send`, `recv` and `get_status` are required; the rest are
optional and `-EOPNOTSUPP` when absent, which is how bxCAN says it does not
set what the app's HAL setup owns.

The core checks what every controller would: the frame's DLC and ID range,
each filter's ranges, the mode. It keeps the filters last set and checks
received frames against them in software, so a controller's hardware filter
only has to be a superset; the TWAI's is, since it cannot tell the formats
apart. It stamps frames the driver did not.

## Platform ports

The drivers in `src/` are the same C on both platforms; what differs is in
`src/can_port_esp.c` and `src/can_port_stm.c`: the bus ops, attach,
`can_i2c_probe()`, `can_sleep_ms()`, `can_yield()`, `can_time_us()`, and on
ESP-IDF `can_ticks()`.

What only one platform has stays in the driver behind `#ifdef ESP_PLATFORM`:

- **ESP-IDF:** the MCP2515's SPI device added to the app's host and its
  nINT GPIO configured, as esp-mcp2515-can did; the whole TWAI driver.
- **HAL:** the app's SPI handle and GPIOs; the whole bxCAN driver.

The handle calls of the MCP2515 and TWAI return `esp_err_t` on ESP-IDF and
errno on the HAL, through `CAN_HANDLE_RET()`; bxCAN's are errno.

**The frame ring.** bxCAN's interrupt hands frames to the reader through a
`struct can_ring` of `struct can_frame`: one writer, one reader, each owning
one free-running index, with acquire and release on the indices. A full
ring drops the new frame and counts it.

## Bus abstraction

The bus layer is mcu-lidar's: `struct can_bus_ops` with `transfer()`,
`receive()` for a part that streams, and `detach()`. Every category library
offers I2C, SPI and UART, used or not, and `test/{esp,stm}/test_port_*`
check all three. The MCP2515 uses SPI `transfer()` only: the instruction as
`cmd`, then what follows it, one chip-select frame per instruction. The
TWAI and bxCAN are on-chip and sit outside the bus layer, on their
platform's own driver.

## Adding a chip

1. `include/foo.h`: the handle API, platform types behind
   `#ifdef ESP_PLATFORM`, and `struct can_dev *foo_can(foo_handle_t)`.
2. `src/foo/foo.c`: include `can_priv.h` first, then wrap in
   `#if CONFIG_CAN_FOO`.
   - Embed a `struct can_dev`.
   - Give it `send`, `recv` and `get_status`, and whichever optional ops
     the chip can do. `recv` may leave `t_us` 0.
   - Filters: take up to `CAN_FILTER_MAX`; a hardware filter that passes
     more is fine, the core checks again.
   - CAN FD: `send_fd`, `recv_fd` and `set_data_bitrate`, optional. Without
     them the core carries Classic frames through `can_send_fd()` and
     `can_recv_fd()` itself. With them, `recv` returns Classic frames only
     and counts the FD ones it passes over in `rx_fd_dropped`.
3. In init: refuse a bad config, allocate, attach, check the chip answers,
   and only then set `can.ops` and `can.priv`. Wait with deadlines on
   `can_elapsed_ms()`. Work in errno; return `CAN_HANDLE_RET(err)`.
4. Add `CONFIG_CAN_FOO` to:
   - the no-chip checks in `CMakeLists.txt` and `src/can_port_stm.c`;
   - a HAL module `#error` in `src/can_priv.h`;
   - its sources in `CMakeLists.txt`;
   - `Kconfig`;
   - both forges' defconfigs;
   - `test/Makefile`.

## Tests

```sh
make -C test test
```

Host-only; no toolchain, HAL or ESP-IDF needed. Nothing is checked against
a real controller. esp-mcp2515-can's tests checked the timing table and the
driver on ESP-IDF stubs; their cases are covered here, and more.

The fake MCP2515 is a register file behind the SPI instruction set: modes
that may take a few looks to show, Configuration-only registers, the
filters with EXIDE and both masks, rollover and overflow, nINT, and
loopback. The fake TWAI driver keeps the legacy driver's state rules,
each cited to its line in `twai.c`; the fake HAL keeps the HAL's.

Each change is checked by a test that fails without it: 66 mutations of
the source were run, each on a scratch copy; 65 fail the suite. The one
that survives turns `can_irq_pending()`'s count into a flag the reader
clears, which differs only when an interrupt lands between the read and the
clear -- not something a single-threaded host test can show.

The FDCAN driver, its timing search, transmitter delay compensation and the
core's CAN FD calls had 48 more: 47 fail the suite. The survivor turns the timing search's early
`break`, once a prescaler leaves too few quanta, into `continue`: every
larger prescaler leaves fewer still, so the result is the same.

| Binary | Checks |
|---|---|
| `test_mcp2515_math` | **every timing row decoded and held to DS Section 5's rules**, exactly the seven rows and no others (8 MHz at 1 Mbit/s refused), SOF clear, SAM on the 16 MHz rows; identifier packing both ways against hand-worked bytes |
| `test_fdcan_math` | **hand-worked rows at 8, 80 and 250 MHz** for both phases, including a sample point pushed by seg2's minimum and a prescaler skipped for seg2's ceiling; inexact rates, too few and too many quanta refused; the CKDIV codes; **a sweep of 11 clocks by 13 rates: the bit exactly the rate, every field in the HAL's range, the sample point within half a quantum**; the TDC offset at the data sample point in mtq, 127 the most, every 80 MHz data rate from 2 to 8 Mbit/s fitting |
| `test_names` | hvd230can.h, can.h, can_log.h, mcp2515.h, twai_can.h and driver/twai.h in one file under `-Werror` |
| `esp/test_core`, `stm/test_core` | the DLC table both ways and every length no DLC carries; **`can_send_fd()`'s checks (FD lengths, no FD remote, Classic over 8, BRS without FD) and its Classic fallback**; `can_recv_fd()` with the software filter and its fallback through `can_recv()`; `can_set_data_bitrate()`; the frame and filter checks; the stamp; **the software filter, its format check, and the dropped frames counting against the timeout**; refused filters kept; the optional ops and their `-EOPNOTSUPP`; status zeroed for the driver and untouched on failure; the IRQ count; the ring's full, wrap and order; the log line; HAL `can_dbg()` |
| `esp/test_mcp2515`, `stm/test_mcp2515` | init: reset, the timing written and read back, RXB0/1, CANINTE with and without nINT, the running mode, a mode that takes a second look; **no chip (MISO low and high), timing that does not stick, a chip stuck in Configuration mode after waits on the clock**, every transfer of init failed in turn, nothing leaked; send through all three buffers, **the timeout on the clock**, remote frames with no data; receive in order across both buffers, remote frames by SRR and by DLC, DLC 15, overflow and its clearing, nINT skipping the poll; loopback; `set_filter()` in Configuration mode and back, standard, extended and mask 0, and back on the bus after a failed write; the category: two filters on two masks, bit rate, modes, status from EFLG, stop and start. ESP-IDF: the SPI device's mode and clock, nINT's pull-up, the `esp_err_t` codes. HAL: the handle's SPI mode checks, one CS frame per instruction |
| `esp/test_twai` | install with defaults and every setting, both controllers, every IDF rate; the Rs pin low before start; refused configs, a second owner, install failures, nothing leaked; frames both ways, **the ticks rounded up**, a full queue waiting its time; loopback as NO_ACK with self-reception; listen-only refusing to send; settings `-EBUSY` while running, applied at the next start, **the filter's ID at bit 21 or 3 and its mask inverted**, an extended frame through a standard filter dropped; status and the 96/128 limits; recovery, and a restart after it; `twaican_service()` waiting `restart_ms` and restarting; standby refusing sends |
| `stm/test_bxcan` | start unless started, the notification with `use_irq`, refused handles, failures leaking nothing; the TX header for standard, extended and remote frames; no mailbox, waiting on the clock; polled receive; **`bxcan_irq()` draining FIFO 0 with its own stamp**, the ring dropping and counting; status from ESR and FIFO 0's overrun; the four `-EOPNOTSUPP`s; start and stop |
| `stm/test_fdcan` | start unless started with every frame accepted, the notification with `use_irq`, refused handles, failures leaking nothing; the TX header for Classic, extended, remote, FD and BRS frames, the DLC codes; FD refused by a Classic controller and BRS by one without; a full Tx FIFO, waiting on the clock; polled receive, a Classic DLC past 8 as 8, remote frames without data; **`can_recv()` passing over FD frames and counting them, `can_recv_fd()` taking them whole with BRS and ESI**; `fdcan_irq()` draining FIFO 0 with its stamp, the ring dropping and counting; status from the protocol status and error counters, **BUS_OFF while INIT is set and RECOVERING after**, the message-lost flag, pending counts; recovery `-EBUSY`, clearing INIT, `-ENETDOWN` stopped; **filters: the lists sized from them, mask elements into FIFO 0, the rest rejected, none accepting all, kept across a bit-rate change, the notification surviving the re-init**; nominal and data timing from the kernel clock and CKDIV, the 1 Mbit/s nominal cap; **transmitter delay compensation off at 1 Mbit/s, on at 2 and 8 Mbit/s with the offset at the sample point and the filter window 0, set again after every re-init, off again back at 1 Mbit/s; an app's own TDC (offset and filter) kept through filters, mode and bit rate; 240 MHz at 2 Mbit/s taking prescaler 3 with a warning**; a nominal rate above the data rate refused with BRS; **the FIFO's message-lost flag counted and cleared, each overflow once, polled and from `fdcan_irq()`**; data rates past 8 Mbit/s, under the nominal, and with an offset past 127 mtq refused, nothing touched; no data phase without BRS; the three modes, listen-only refusing to send |
| `esp/test_port_esp`, `stm/test_port_stm` | the bus ops (I2C, SPI, UART transfer and `receive()`), the presence probe, the waits, `can_ticks()`, the errno and `esp_err_t` maps, the default clock |

All build under `-std=gnu11 -Wall -Wextra -Werror -pedantic` with UBSan, and
`-Wl,--wrap` (GNU ld) to count allocations and capture `printf()`.
