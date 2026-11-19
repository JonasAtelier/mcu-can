# mcu-can

Send and receive CAN frames. The MCP2515 (SPI), ESP32 TWAI and STM32 bxCAN handle Classic CAN. STM32 FDCAN also handles CAN FD, with its HAL interface checked against the STM32H5.

**Platforms** · ESP-IDF 5.2+ and STM32 HAL. Hardware testing is pending. The host tests use simulated devices.

## Contents

- [Quick start](#quick-start)
- [Controllers and examples](#controllers-and-examples)
- [Using the library in your project](#using-the-library-in-your-project)
- [License](#license)

## Quick start

Use an MCP2515 and transceiver on a bus with a second active node and 120 Ω termination at each end. Set the SPI pins, oscillator frequency and bus bit rate in the example.

In a terminal with ESP-IDF activated, get the example and select the target for an ESP32. Change the target if your board uses another ESP32 chip.

```sh
git clone https://github.com/JonasAtelier/mcu-can.git can
cd can/examples/esp/mcp2515
idf.py set-target esp32
```

Check the [wiring guide](docs/hardware.md) and edit the settings in [main.c](examples/esp/mcp2515/main/main.c) to match your board. Connect the board by USB, then build, upload and open the serial monitor.

```sh
idf.py build flash monitor
```

The example sends a counter once a second and prints incoming frames. Match `CRYSTAL_HZ` to the crystal on the MCP2515 board to communicate at the selected bit rate.

## Controllers and examples

Use `can_send()` and `can_recv()` for Classic CAN across controllers. The FDCAN example configures one peripheral for CAN FD and a second for Classic CAN. [Frame and controller API](include/can.h)

| Controller | Driver header | ESP-IDF project | STM32 HAL example |
|---|---|---|---|
| [MCP2515](docs/chips.md#mcp2515) | [mcp2515.h](include/mcp2515.h) | [SPI controller](examples/esp/mcp2515/) | [SPI controller](examples/stm/mcp2515/main.c) |
| [ESP32 TWAI](docs/chips.md#twai) | [twai_can.h](include/twai_can.h) | [Transmit and receive](examples/esp/twai/) | — |
| [STM32 bxCAN](docs/chips.md#bxcan) | [bxcan.h](include/bxcan.h) | — | [Transmit and receive](examples/stm/bxcan/main.c) |
| [STM32 FDCAN](docs/chips.md#fdcan) | [fdcan_can.h](include/fdcan_can.h) | — | [Classic CAN and CAN FD](examples/stm/fdcan/main.c) |
| TWAI and MCP2515 | [can.h](include/can.h) | [Forward between two buses](examples/esp/dual/) | — |

A dash marks a hardware and platform combination without a dedicated example.

## Using the library in your project

- Add the component and enable the driver for your hardware. [Installation and build options](docs/installation.md)
- Open the ESP-IDF project for your hardware, or copy the STM32 example into a project with board setup already in place. [All ESP-IDF examples](examples/esp/) · [All STM32 examples](examples/stm/)
- Look up calls and chip-specific settings. [API usage](docs/usage.md) · [Hardware reference](docs/chips.md)

To run the simulated tests on your computer, run `make -C test test` from the library root. [Test requirements and coverage](docs/development.md)

## License

© 2026 JonasAtelier, [GPL-3.0-only](LICENSE).

[Browse the workshop catalog · ESP32 and STM32 drivers](https://github.com/JonasAtelier/jonas-atelier-libraries#esp32-and-stm32-drivers)
