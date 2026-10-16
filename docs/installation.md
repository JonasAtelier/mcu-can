# Installation

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

The repository is `mcu-can`; the forge package key, C API and configuration
options are `can`. The clone command below names the ESP-IDF component
directory `can`. It replaces `esp-mcp2515-can` (forge key `mcp2515`): a
project that `REQUIRES mcp2515` now `REQUIRES can`; `mcp2515.h` and its
calls are unchanged.

## With forge

stm-forge must be at `ee921c3` or later ("build: libraries may keep sources
in src/ subfolders"); older ones only compile `src/*.c`, so the chip drivers
in `src/<chip>/` are never built and the link fails with undefined
references to the chip functions. esp-forge is unaffected: the component's
CMakeLists lists its sources.

With [esp-forge](https://github.com/JonasAtelier/esp-forge) or
[stm-forge](https://github.com/JonasAtelier/stm-forge). The catalog line in
`profiles/pkgversion` is the same for both:

```make
can_GIT=https://github.com/JonasAtelier/mcu-can@stable
```

and a target turns the library and its chips on in
`configs/<target>_defconfig`:

```make
PKG_can             = y
CAN_MCP2515         = y
# CAN_TWAI is not set
# CAN_BXCAN is not set
# CAN_FDCAN is not set
# CAN_DEBUG is not set
```

or with `make menu`. esp-forge turns each line into `CONFIG_CAN_<CHIP>=y` in
`sdkconfig.defaults`; stm-forge passes it as `-DCONFIG_CAN_<CHIP>=1`.
`CAN_TWAI` is ignored by stm-forge, and `CAN_BXCAN` and `CAN_FDCAN` by
ESP-IDF. `CAN_DEBUG =
y` turns on `can_dbg()` lines on the HAL; on ESP-IDF the log level decides.

## ESP-IDF

Clone into your project's `components/` directory. A local component is
named after its directory, and that name is what you `REQUIRES`:

```sh
git clone https://github.com/JonasAtelier/mcu-can components/can
```

```cmake
idf_component_register(SRCS "main.c" REQUIRES can driver)
```

Enable the chips under `idf.py menuconfig` → **Component config → Classic
CAN subsystem**, or put `CONFIG_CAN_<CHIP>=y` in your `sdkconfig.defaults`
as each example in `examples/esp/` does. With the subsystem on and no chip
picked, the build warns. ESP-IDF 5.2 or newer is required: the `_v2` TWAI
calls and the controller number in `twai_general_config_t` arrived there.

### Next to esp-sn65hvd230-can

Both libraries link into one app: nothing in this one is named like
anything in that one -- no header (`hvd230can.h`, `internal.h`), no source
file (`core.c`, `io.c`, `health.c`), no type or macro (`can_frame_t`,
`can_sta`, `can_state`, `CAN_OK`, `CAN_STATE_*`, `CAN_MAX_DLC`,
`CAN_BITRATE_*`, ...), and no global symbol (`can_lock`, `hvd230_*`, ...).
`test/test_names.c` includes both libraries' headers in one file to keep it
so.

**One owner per TWAI controller.** esp-sn65hvd230-can drives its controller
through the same legacy driver; leave `CAN_TWAI` off for that controller.
The legacy driver refuses a second install on a controller already
installed (`ESP_ERR_INVALID_STATE`), so a clash shows at init, not on the
bus. The MCP2515 driver of this library sits next to it with no such
question.

## STM32 HAL

Copy `include/` and `src/` into an STM32 project, compile every `.c` under
`src/`, chip folders included, add `include/` and `src/` to the include path
(`src/` only needs to reach this library's own sources), and define what the
forge would: `CMSIS_device_header` as your family header (`"stm32f4xx.h"`,
`"stm32f7xx.h"`, `"stm32g0xx.h"`, `"stm32h5xx.h"`), `USE_HAL_DRIVER`, and
`CONFIG_CAN_<CHIP>=1`.

**No chip driver is built by default.** The core is always built and each
driver is opt-in: a chip whose option is absent or 0 compiles to nothing.
With the library on and no chip picked, the ESP-IDF CMakeLists warns and
`src/can_port_stm.c` emits a `#warning`. The TWAI driver compiles to nothing
on the HAL whatever its option says.

**The HAL modules are yours to enable:**

| Part | Needs in hal_conf | HAL source | Gives |
|---|---|---|---|
| MCP2515 | `HAL_SPI_MODULE_ENABLED` | `stm32<family>xx_hal_spi.c` | `mcp2515_init()` |
| bxCAN | `HAL_CAN_MODULE_ENABLED` | `stm32<family>xx_hal_can.c` | `bxcan_init()`, `bxcan_irq()` |
| FDCAN | `HAL_FDCAN_MODULE_ENABLED` | `stm32<family>xx_hal_fdcan.c` | `fdcan_init()`, `fdcan_irq()` |

Without the module a chip needs, the build stops with an `#error` in
`src/can_priv.h`. stm-forge's `mcu/stm32f4.cmake` lists
`stm32f4xx_hal_can.c`, which compiles to nothing until the app's hal_conf
enables the module; `mcu/stm32f7.cmake` does not yet. The G0 has
no bxCAN (its CAN is FDCAN). stm-forge's `mcu/stm32h5.cmake` lists
`stm32h5xx_hal_fdcan.c`; the `nucleo563` target (NUCLEO-H563ZI) is the H5
family's.

**The clock.** Received frames are stamped with `can_time_us()`, by default
`HAL_GetTick() * 1000`: millisecond steps. It is weak; define your own for
real microseconds (the DWT cycle counter, or a free-running timer).

**printf() is the log.** Lines go out through `printf()`, so retarget
newlib's `_write()` to a UART.

## Logging

`can_err()`, `can_warn()`, `can_info()` and `can_dbg()` prefix every line
with the device's name. On ESP-IDF they are `ESP_LOGx` under the one tag
`"can"`, so `esp_log_level_set("can", ...)` controls every controller. On
the HAL they are `printf("can: <name>: ...")`, and `can_dbg()` is compiled
in only with `CONFIG_CAN_DEBUG`. `can_log_frame()` prints one frame:

```
can: mcp2515: t 1234567 us  std 12A  [2] 01 02
```
