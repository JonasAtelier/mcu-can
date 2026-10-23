# mcu-can

<p align="center">
  <img alt="platform: ESP-IDF | STM32 HAL" src="https://img.shields.io/badge/platform-ESP--IDF%20%7C%20STM32%20HAL-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="language: C" src="https://img.shields.io/badge/language-C-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="license: MIT" src="https://img.shields.io/badge/license-MIT-f2bd68?style=flat-square&labelColor=0d2238">
</p>

A small C library for CAN controllers on **ESP-IDF 5.2+** and the **STM32
HAL**. The MCP2515, the ESP32's TWAI, the STM32's bxCAN and its FDCAN each
keep a handle API and hand out a `struct can_dev`: frames out and in,
acceptance filters, the operating mode, error counters and the bus state,
the same whichever controller carries them. The FDCAN adds CAN FD frames,
up to 64 bytes with bit-rate switching, through `can_send_fd()` and
`can_recv_fd()`, which work on the Classic controllers too. This library replaces
`esp-mcp2515-can`, whose `mcp2515.h` it keeps call for call; the STM32
port, the TWAI and bxCAN drivers and the category view are new.

`esp-sn65hvd230-can` stays as it is, and links next to this library: no
header, type, macro, file or global symbol is shared. **One owner per
controller**: drive a TWAI with one of them, not both.

The repository is named `mcu-can`; the C API, `CONFIG_CAN_*` options and
forge package key are `can`. The clone command below names the ESP-IDF
component directory `can` as well.

> **Hardware unverified.** Host tests use a simulated MCP2515, simulated
> ESP-IDF and HAL drivers, and platform stubs. Nothing here has run on a
> physical bus.

[Supported hardware](#supported-hardware) · [Drivers](#drivers) ·
[Install](#install) ·
[Quick start](#quick-start) · [Examples](#examples) ·
[Documentation](#documentation) · [Tests](#tests) · [License](#license)

## Supported hardware

| Controller | Interface | Notes |
|---|---|---|
| **MCP2515** | SPI | standalone CAN controller |
| **TWAI** | on-chip (ESP-IDF) | the ESP32's own CAN controller |
| **bxCAN** | on-chip (STM32 HAL) | the STM32's own CAN controller |
| **FDCAN** | on-chip (STM32 HAL) | the STM32's own CAN FD controller |

One Kconfig option per controller: `CAN_MCP2515`, `CAN_TWAI`, `CAN_BXCAN`,
`CAN_FDCAN`.
Wiring is in the [wiring guide](docs/hardware.md).

## Drivers

<details><summary><b>MCP2515</b> — SPI</summary>

8 or 16 MHz crystal; 125k, 250k, 500k, and 1M with 16 MHz.

- Platform: both
- Interface: SPI, mode 0,0 or 1,1, up to 10 MHz
- Examples: ESP-IDF [mcp2515](examples/esp/mcp2515/) (esp-mcp2515-can's) ·
  STM32 HAL [mcp2515](examples/stm/mcp2515/main.c)
- Examples (with the TWAI): ESP-IDF [dual](examples/esp/dual/), a
  TWAI-to-MCP2515 bridge through the category · STM32 HAL none
- **MCP2515: up to two filters** through the category, one per receive
  buffer.
- Chip reference: [MCP2515](docs/chips.md#mcp2515)

</details>

<details><summary><b>TWAI</b> — on-chip (ESP-IDF)</summary>

Every controller of the part; 25k to 1M; optional SN65HVD230 Rs pin.

- Platform: ESP-IDF
- Interface: on-chip, legacy `driver/twai.h`
- Examples: ESP-IDF [twai](examples/esp/twai/), the category view and bus-off
  recovery · STM32 HAL none
- Examples (with an MCP2515): ESP-IDF [dual](examples/esp/dual/), a
  TWAI-to-MCP2515 bridge through the category · STM32 HAL none
- **TWAI: filter, mode and bit rate only while stopped**, the legacy
  driver's install-time settings. One filter.
- Chip reference: [TWAI](docs/chips.md#twai)

</details>

<details><summary><b>bxCAN</b> — on-chip (STM32 HAL)</summary>

Bit rates up to 1 Mbit/s, filters, modes and bus-off recovery set here, after
the app's `HAL_CAN_Init()`.

- Platform: STM32 HAL
- Interface: on-chip, the HAL's CAN module
- Examples: ESP-IDF none · STM32 HAL [bxcan](examples/stm/bxcan/main.c), FIFO
  0 interrupt to `bxcan_irq()`
- **bxCAN checked against RM0090 (STM32F4) only**; the F1, F2, F7 and L4
  manuals are not in `docs/`.
- Chip reference: [bxCAN](docs/chips.md#bxcan)

</details>

<details><summary><b>FDCAN</b> — on-chip (STM32 HAL)</summary>

Classic and CAN FD; bit rates, filters, modes and bus-off recovery set here;
nominal up to 1 Mbit/s, data phase up to 8 Mbit/s with transmitter delay
compensation.

- Platform: STM32 HAL
- Interface: on-chip, the HAL's FDCAN module; checked against the STM32H5's
- Examples: ESP-IDF none · STM32 HAL [fdcan](examples/stm/fdcan/main.c), the
  Nexus-S's two buses: CAN FD on FDCAN1, Classic on FDCAN2
- **FDCAN checked against the STM32H5 HAL only**; another family's FDCAN
  HAL may differ.
- Chip reference: [FDCAN](docs/chips.md#fdcan)

</details>

## Install

### ESP-IDF

```sh
git clone https://github.com/JonasAtelier/mcu-can components/can
```

```ini
CONFIG_CAN=y
CONFIG_CAN_MCP2515=y
```

Other components declare `REQUIRES can driver`.

### esp-forge / stm-forge

```make
can_GIT=https://github.com/JonasAtelier/mcu-can@stable
```

```make
PKG_can             = y
CAN_MCP2515         = y
```

stm-forge must be at `ee921c3` or later. See the
[installation guide](docs/installation.md). **No chip driver is enabled by
default.**

## Quick start

On ESP-IDF, the MCP2515 exactly as esp-mcp2515-can:

```c
#include "mcp2515.h"

mcp2515_config_t cfg = {
    .host = SPI2_HOST, .cs_gpio = 10, .int_gpio = 9,
    .crystal_hz = 8000000, .bitrate = 500000,
};
mcp2515_handle_t chip;
mcp2515_frame_t f = { .id = 0x123, .len = 2, .data = { 1, 2 } };

ESP_ERROR_CHECK(mcp2515_init(&cfg, &chip));
mcp2515_send(chip, &f, 10);
```

The same through the category, with any of the three:

```c
struct can_dev *can = mcp2515_can(chip);    /* or twaican_can(), bxcan_can() */
struct can_frame rx;

if (can_recv(can, &rx, 100) == 0)
    can_log_frame(can, &rx);
```

- On ESP-IDF the chip calls return `esp_err_t`, as esp-mcp2515-can did; on
  the HAL, **0 or a negative errno**. `can_*` is errno on both.
- **Mind the crystal.** A wrong `crystal_hz` is a wrong bit rate, and the bus
  does not say so. Read the can on the crystal.
- bxCAN starts from the app's CubeMX setup, and `can_set_filters()` takes
  over all its filter banks; see [API usage](docs/usage.md#bxcan).

CAN FD, on an FDCAN set up for it:

```c
struct canfd_frame f = { .id = 0x300, .len = 64, .fd = true, .brs = true };

can_send_fd(fdcan_can(fd), &f, 10);
```

## Examples

Each driver's examples are listed under [Drivers](#drivers).

ESP-IDF examples locate this library through `EXTRA_COMPONENT_DIRS`, which
assumes the repository is checked out as `can`:

```sh
cd examples/esp/mcp2515
idf.py set-target esp32s3 # Replace with your board's target.
idf.py build
```

STM32 examples are snippets to integrate into your application.

## Scope and limitations

- **CAN FD on the FDCAN only.** The MCP2515, TWAI and bxCAN are Classic.
  Above 1 Mbit/s in the data phase the FDCAN turns on transmitter delay
  compensation with a fixed offset at the data sample point; whether a
  transceiver's loop delay fits is the board's to check (RM0481 60.3.4:
  delay plus offset under six data bits).
- **No locks.** One thread per device, or your own mutex.

## Documentation

| Guide | Contents |
|---|---|
| [Installation](docs/installation.md) | Forge, ESP-IDF, STM32 HAL, build options, logging, next to esp-sn65hvd230-can |
| [Hardware](docs/hardware.md) | Transceivers, termination, the MCP2515's pins and crystal, the TWAI's Rs pin, the STM32's CAN and FDCAN pins, the Nexus-S |
| [API usage](docs/usage.md) | Handles and the category view, frames and CAN FD frames, filters, modes, status and recovery, errors, changes from esp-mcp2515-can |
| [Chip reference](docs/chips.md) | The MCP2515's bit timing, instructions and registers; the TWAI's filter and states; bxCAN's timing, filter banks and bus-off; the FDCAN's HAL calls, timing ranges and DLC table; each cited |
| [Development](docs/development.md) | Layout, ports, bus abstraction, adding a driver and test coverage |

## Tests

```sh
make -C test test
```

Host-only; no toolchain or CAN hardware needed. See the
[test reference](docs/development.md#tests). Error logs during the suite are
expected: tests exercise failure paths.

## License

[MIT](LICENSE).
