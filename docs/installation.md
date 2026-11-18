# Installation

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

The repository is `mcu-can`; the forge package key, C API and configuration
options are `can`. The clone command below names the ESP-IDF component
directory `can`. It replaces `esp-mcp2515-can` (forge key `mcp2515`): a
project that `REQUIRES mcp2515` now `REQUIRES can`; `mcp2515.h` and its
calls are unchanged.

Three step-by-step guides come first: [ESP-IDF](#esp-idf-step-by-step),
[STM32 with CMake](#with-cmake-a-cubemx-cmake-project) and
[STM32 without CMake](#without-cmake-stm32cubeide-a-makefile). None
needs a forge; if you build with esp-forge or stm-forge, see
[With forge](#with-forge). The steps use each chip's own CAN controller:
the TWAI on an ESP32, the bxCAN on an STM32F4. For the MCP2515, an SPI chip
that works on both, turn on `CAN_MCP2515` instead and start from its example
in `examples/esp/mcp2515` or `examples/stm/mcp2515`. The reference follows
the guides.

**Arduino is not supported yet.** TODO: make the library usable from
Arduino (Arduino IDE and PlatformIO).

## ESP-IDF, step by step

**You need** ESP-IDF 5.2 or newer, with `idf.py` working in your terminal, and
an ESP-IDF project: a folder with a `CMakeLists.txt` and a `main/` folder.
`idf.py create-project my-project` makes one. On the wire: a CAN transceiver
such as the SN65HVD230 between the ESP32 and the bus, 120 ohm at both ends of
the bus, and a second CAN node. A frame that no other node acknowledges is an
error, not a send.

ESP-IDF calls a library a **component**: a folder with its own
`CMakeLists.txt` that the build finds and compiles. This library is the
component `can`.

1. **Get the library.** Pick one way.

   - **Component manager**, the simplest: the first build downloads the
     library into `managed_components/can`. Create `main/idf_component.yml`:

     ```yaml
     dependencies:
       can:
         git: https://github.com/JonasAtelier/mcu-can.git
         version: stable
     ```

   - **Clone**, if you want to edit the library or build offline. In the
     project folder:

     ```sh
     git clone https://github.com/JonasAtelier/mcu-can components/can
     ```

     Keep the folder name `can`: ESP-IDF names a component after its folder.

   ```
   my-project/
   ├── CMakeLists.txt
   ├── sdkconfig.defaults       step 2
   ├── main/
   │   ├── CMakeLists.txt       step 3
   │   ├── idf_component.yml    step 1, component manager
   │   └── my-project.c         step 4
   └── components/
       └── can/                 step 1, clone
   ```

2. **Turn your controller on.** No controller driver is built until you do.
   Create `sdkconfig.defaults` in the project folder, a text file of settings
   you choose, with one line per driver:

   ```ini
   CONFIG_CAN_TWAI=y
   ```

   The other driver for ESP-IDF is `CONFIG_CAN_MCP2515`; `_BXCAN` and
   `_FDCAN` are STM32 controllers and build nothing here. ESP-IDF copies
   these lines into `sdkconfig`, the project's full settings file, only when
   it creates that file: **if `sdkconfig` already exists, delete it** so your
   line is read. `idf.py menuconfig` → **Component config → CAN subsystem
   (Classic CAN; CAN FD on the FDCAN)** does the same from a menu.

3. **Let your code see the library.** A new project's `main/CMakeLists.txt`
   has no `REQUIRES` and already sees every component. If yours has a
   `REQUIRES` list, add `can` and `driver` to it:

   ```cmake
   idf_component_register(SRCS "main.c" REQUIRES can driver)
   ```

4. **Write the smallest test.** Copy
   [examples/esp/twai/main/main.c](../examples/esp/twai/main/main.c) over
   your main source file. Set `PIN_TX`, `PIN_RX` and `PIN_RS` to your wiring
   (`PIN_RS` to `TWAICAN_NO_GPIO` if the transceiver's Rs pin is tied low),
   and `.bitrate` to your bus's, 500 kbit/s in the example. It sends a
   counter as ID 0x321 once a second and prints every frame it receives.
   [API usage](usage.md) explains the calls.

5. **Build.**

   ```sh
   idf.py set-target esp32    # your chip: esp32s3, esp32c3, ...
   idf.py build
   ```

   A good build ends with `Project build complete.` Flash it as any ESP-IDF
   app and open the serial monitor: each frame the other node sends prints a
   line ending in `can: twai: t 1234567 us  std 12A  [2] 01 02`, its time
   stamp, identifier and data bytes.

**If it fails**

| The build prints | Why | Fix |
|---|---|---|
| `Failed to resolve component 'can' required by component 'main': unknown name.` | The clone is not in a folder named `can` | Rename it to `components/can` |
| `fatal error: can.h: No such file or directory` | `main/CMakeLists.txt` has a `REQUIRES` list without `can` | Step 3 |
| `can: CONFIG_CAN is on but no chip driver is selected; set CONFIG_CAN_<CHIP>=y (esp-forge: CAN_<CHIP> = y)` (a CMake warning) | No driver turned on, often because `sdkconfig` existed before you edited `sdkconfig.defaults` | Step 2: delete `sdkconfig`, build again |

## STM32, step by step

**You need** a project for an STM32F4 or F7, the families here with a bxCAN,
that uses ST's **HAL**, the vendor library behind `HAL_Init()`,
`HAL_CAN_Start()` and friends, with CAN1 already set up. STM32CubeMX makes
such a project; with **Toolchain / IDE: CMake** it makes a CMake one, the
main case below. On the wire, as above: a transceiver, 120 ohm at both ends,
and a second node.

Two terms. A **HAL module** is the part of the HAL for one peripheral: CAN,
SPI, I2C. It is compiled only when it is switched on in **hal_conf**, the
file `Core/Inc/stm32f4xx_hal_conf.h` (`stm32f7xx_hal_conf.h` on an F7).
CubeMX switches a module on when you enable that peripheral.

### With CMake (a CubeMX CMake project)

1. **Get the library.** In the project folder, next to the top-level
   `CMakeLists.txt`:

   ```sh
   git clone https://github.com/JonasAtelier/mcu-can
   ```

   ```
   my-board/
   ├── CMakeLists.txt                  step 2
   ├── cmake/stm32cubemx/              made by CubeMX
   ├── Core/
   │   ├── Inc/stm32f4xx_hal_conf.h    hal_conf, step 3
   │   └── Src/main.c                  step 4
   ├── Drivers/                        the HAL, copied in by CubeMX
   └── mcu-can/                        step 1
   ```

2. **Add it to the build.** In the top `CMakeLists.txt`, after the line
   `add_subdirectory(cmake/stm32cubemx)`, add:

   ```cmake
   set(CONFIG_CAN_BXCAN ON)            # your controller; one line per driver
   set(MCU_HAL_TARGET stm32cubemx)
   set(MCU_DEVICE_HEADER stm32f4xx.h)  # stm32f7xx.h on an F7
   add_subdirectory(mcu-can)
   ```

   and add `can` to the link line CubeMX already wrote, so it reads:

   ```cmake
   target_link_libraries(${CMAKE_PROJECT_NAME} stm32cubemx can)
   ```

   Do not add `PRIVATE`: CubeMX's line has no such keyword, and CMake refuses
   to mix the two forms on one target.

   `stm32cubemx` is a CMake *target*, a named bundle of settings, that CubeMX
   makes to carry the HAL's include folders and defines; `MCU_HAL_TARGET`
   lends those to the library, which works with any CubeMX version.
   `MCU_DEVICE_HEADER` is your chip family's main header.

3. **Check the HAL module.** The bxCAN driver needs
   `#define HAL_CAN_MODULE_ENABLED` in hal_conf; enabling CAN1 in CubeMX does
   that. In CAN1's settings there, set the bit timing for your bus's bit
   rate (the prescaler and the two bit segments) and turn on automatic
   bus-off management; under its NVIC settings, enable the CAN1 RX0
   interrupt, so CubeMX calls `HAL_CAN_IRQHandler()` from it. The
   [HAL module table](#stm32-hal) lists what every driver needs.

4. **Write the smallest test.** It is
   [examples/stm/bxcan/main.c](../examples/stm/bxcan/main.c) fitted into
   CubeMX's `Core/Src/main.c`, sending ID 0x321 like the ESP-IDF example.
   Paste each part between the `USER CODE BEGIN` and `END` comments named,
   so a regenerate keeps it.

   After `/* USER CODE BEGIN Includes */`:

   ```c
   #include <stdio.h>

   #include "bxcan.h"
   #include "can.h"
   #include "can_log.h"
   ```

   After `/* USER CODE BEGIN 0 */`, just above `int main(void)`:

   ```c
   static bxcan_handle_t bx;

   /*
    * All IDs into FIFO 0: one 32-bit mask filter, ID and mask 0. The bxCAN
    * receives nothing until a filter lets frames in.
    */
   static void accept_all(void)
   {
     CAN_FilterTypeDef f = {
       .FilterBank = 0,
       .FilterMode = CAN_FILTERMODE_IDMASK,
       .FilterScale = CAN_FILTERSCALE_32BIT,
       .FilterFIFOAssignment = CAN_RX_FIFO0,
       .FilterActivation = CAN_FILTER_ENABLE,
       .SlaveStartFilterBank = 14,
     };

     HAL_CAN_ConfigFilter(&hcan1, &f);
   }
   ```

   After `/* USER CODE BEGIN 2 */`, which comes after `MX_CAN1_Init()`:

   ```c
     bxcan_config_t cfg = { .hcan = &hcan1, .use_irq = true };

     accept_all();
     if (bxcan_init(&cfg, &bx))
       Error_Handler(); /* the controller would not start */
   ```

   After `/* USER CODE BEGIN 3 */`, inside the `while (1)` loop:

   ```c
       static uint32_t counter;
       struct can_dev *can = bxcan_can(bx);
       struct can_frame tx = {
         .id = 0x321,
         .dlc = 4,
         .data = { (uint8_t)(counter >> 24), (uint8_t)(counter >> 16),
                   (uint8_t)(counter >> 8), (uint8_t)counter },
       };
       struct can_frame rx;
       struct can_status st;

       if (can_send(can, &tx, 10) == 0)
         counter++;
       while (can_recv(can, &rx, 0) == 0)
         can_log_frame(can, &rx);
       if (can_get_status(can, &st) == 0 && st.link != CAN_LINK_ACTIVE)
         printf("bxcan: link %d tec %u rec %u lost %u\n", (int)st.link,
                st.tec, st.rec, (unsigned)st.rx_lost);
       HAL_Delay(1000);
   ```

   After `/* USER CODE BEGIN 4 */`:

   ```c
   /* The HAL calls it from HAL_CAN_IRQHandler(), on the CAN1 RX0 interrupt. */
   void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
   {
     if (hcan == &hcan1)
       bxcan_irq(bx);
   }
   ```

   The lines go out through `printf()`: send that to a UART by writing
   newlib's `_write()`. [API usage](usage.md) explains the calls.

5. **Build** as before, for example `cmake --build` with your build folder.
   A good build shows a `Linking C executable` line and no error. On the
   UART, each frame the other node sends prints a line like
   `can: bxcan: t 1234567 us  std 12A  [2] 01 02`. A `bxcan: link ...` line
   instead means the bus is in trouble: often no other node, a wrong bit
   rate or missing termination.

**If it fails**

| You see | Why | Fix |
|---|---|---|
| `can: no chip driver is selected; set(CONFIG_CAN_<CHIP> ON) before add_subdirectory()` (a CMake warning), then ``undefined reference to `bxcan_init'`` | No driver turned on, or turned on after `add_subdirectory(mcu-can)` | Step 2 |
| `#error "CAN_BXCAN needs the HAL CAN module: ...` | The HAL module is off | Enable CAN1 in CubeMX and regenerate |
| `#error "can: CMSIS_device_header is not defined; ...` | `MCU_DEVICE_HEADER` is missing | Step 2 |
| `fatal error: stm32f4xx.h: No such file or directory` | `MCU_HAL_TARGET` is missing | Step 2 |
| `Target "stm32cubmx" not found.` (a CMake error) | `MCU_HAL_TARGET` is misspelt | Step 2 |
| `The plain signature for target_link_libraries has already been used with the target` | `PRIVATE` added to the link line | Step 2 |

### Without CMake (STM32CubeIDE, a Makefile)

1. Copy `include/` and `src/` of this repository into your project, for
   example to `Middlewares/mcu-can/`, keeping the driver folders inside
   `src/`.
2. Add every `.c` file under `src/` to the build, driver folders included.
3. Add two include paths, `Middlewares/mcu-can/include` and
   `Middlewares/mcu-can/src`, and two defines:
   `CMSIS_device_header="stm32f4xx.h"`, quotes included, and
   `CONFIG_CAN_BXCAN=1`, one per driver. CubeMX already defines
   `USE_HAL_DRIVER` and your device, such as `STM32F405xx`.
4. Steps 3 to 5 of the CMake guide apply as they are.

Where these settings usually live: in STM32CubeIDE, **Project → Properties →
C/C++ Build → Settings → Tool Settings → MCU GCC Compiler** (Include paths,
Preprocessor); in a CubeMX Makefile, `C_SOURCES`, `C_INCLUDES` and `C_DEFS`.
The build errors are the ones in the table above; the fix is the matching
define or include path instead of a CMake line. One more:
`#include expects "FILENAME" or <FILENAME>` means `CMSIS_device_header` lost
its quotes.

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

Setup: [ESP-IDF, step by step](#esp-idf-step-by-step). For reference:

- In `main/idf_component.yml`, `version` is a git ref; `stable` is the
  released one.
- A clone in `components/` is a local component, named after its folder;
  that name is what other components `REQUIRES`. The examples in
  `examples/esp/` need no name: their `main` lists no `REQUIRES`, and ESP-IDF
  gives `main` every component, so they build from any checkout name.
- With the subsystem on and no chip picked, the build warns.
- ESP-IDF 5.2 or newer is required: the `_v2` TWAI calls and the controller
  number in `twai_general_config_t` arrived there.

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

Setup: [With CMake](#with-cmake-a-cubemx-cmake-project) or
[Without CMake](#without-cmake-stm32cubeide-a-makefile). The
`add_subdirectory()` route builds a static library `can`. For reference:

- The families are the F4, F7, G0 and H5: `CMSIS_device_header` is
  `"stm32f4xx.h"`, `"stm32f7xx.h"`, `"stm32g0xx.h"` or `"stm32h5xx.h"`. The
  bxCAN driver is for the F4 and F7; the FDCAN driver is written against
  the H5's HAL (the G0 has no bxCAN); the MCP2515 works on any of them.
- Each `CONFIG_CAN_<CHIP>` (and `CONFIG_CAN_DEBUG`) that is true reaches the
  compiler as `CONFIG_CAN_<CHIP>=1`, for the library and your app.
- `MCU_HAL_TARGET` is a target that carries your `USE_HAL_DRIVER`, device
  define (`STM32F405xx`, ...) and the include dirs of the HAL, CMSIS and your
  `stm32f4xx_hal_conf.h`. The library copies only those include dirs and
  defines, never the target's sources, so this works with any CubeMX
  version. `MCU_DEVICE_HEADER` becomes `CMSIS_device_header`, which CubeMX
  does not define. Every `mcu-*` library reads these two, so set them once.
  Leave them unset if your project already provides both.
- The CPU flags (`-mcpu`, `-mfloat-abi`, ...) must reach the library too;
  CubeMX's toolchain file sets them for every target.
- Without CMake, `src/` only needs to be on the include path of this
  library's own sources; the defines are what stm-forge passes.

**No chip driver is built by default.** The core is always built and each
driver is opt-in: a chip whose option is absent or 0 compiles to nothing.
With the library on and no chip picked, the CMakeLists warns (ESP-IDF and
`add_subdirectory()`) and `src/can_port_stm.c` emits a `#warning`. The TWAI
driver compiles to nothing on the HAL whatever its option says, and does not
count as a chip picked there.

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
