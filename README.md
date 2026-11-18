# mcu-can

Send and receive CAN frames. The MCP2515 (SPI), ESP32 TWAI and STM32 bxCAN handle Classic CAN. STM32 FDCAN also handles CAN FD, with its HAL interface checked against the STM32H5.

**Platforms** · ESP-IDF 5.2+ and STM32 HAL. Hardware testing is pending. The host tests use simulated devices.

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

## Using the library in your project

- Add the component and enable the driver for your hardware. [Installation and build options](docs/installation.md)
- Start from an example for your chip. The STM32 examples fit into a project with board setup already in place. [ESP-IDF examples](examples/esp/) · [STM32 examples](examples/stm/)
- Look up calls and chip-specific settings. [API usage](docs/usage.md) · [Hardware reference](docs/chips.md)

To run the simulated tests on your computer, run `make -C test test` from the library root. [Test requirements and coverage](docs/development.md)

## License

[MIT](LICENSE).
