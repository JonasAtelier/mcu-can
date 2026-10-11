/*
 * mcp2515 — ESP-IDF driver for the Microchip MCP2515 stand-alone CAN
 * controller.
 *
 * The MCP2515 is a complete Classic CAN 2.0B controller in an 18-pin package,
 * spoken to over SPI. It does the whole protocol itself — arbitration, CRC,
 * acknowledgement, error counters, retransmission — and holds three transmit
 * and two receive buffers, so the MCU only moves whole frames in and out. It
 * still needs a transceiver (an MCP2551, TJA1050 or SN65HVD230) between it and
 * the physical bus.
 *
 * An ESP32 already has one CAN controller on board (TWAI). This chip is how
 * you get a second one, or a CAN bus on a part that has none.
 *
 * Every register and instruction constant carries its section, table or
 * register number from the datasheet in docs/mcp2515.pdf (DS20001801J).
 * Microchip publishes the full register map, so nothing here is guessed.
 */
/*
 * In mcu-can: the same calls on the STM32 HAL, where they return 0 or a
 * negative errno and the chip hangs off an SPI handle and a chip-select
 * GPIO the app set up; and mcp2515_can(), the handle as a struct can_dev.
 * Waits are now on the clock, so a timeout lasts what it says at any tick
 * rate. 8 MHz at 1 Mbit/s is refused, and CNF3.SOF is never set; see
 * docs/chips.md.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef ESP_PLATFORM
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#else
#include CMSIS_device_header
#endif

#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mcp2515_t *mcp2515_handle_t;

typedef struct {
    uint32_t id;
    uint8_t  len;        /* 0..8 */
    uint8_t  data[8];
    bool     extended;   /* 29-bit identifier */
    bool     remote;     /* RTR frame: a request, no data */
} mcp2515_frame_t;

#ifdef ESP_PLATFORM
typedef struct {
    spi_host_device_t host;        /* an SPI bus you have already initialised */
    gpio_num_t        cs_gpio;
    gpio_num_t        int_gpio;    /* nINT; GPIO_NUM_NC to poll the chip over SPI instead */
    int               spi_hz;      /* 0 selects 10 MHz, the chip's maximum */
    uint32_t          crystal_hz;  /* 0 selects 8 MHz. Many boards fit 16 MHz: read the can on the
                                    * crystal. Wrong here = wrong bitrate = a corrupted bus. */
    uint32_t          bitrate;     /* 0 selects 500000. Supported: 125000, 250000, 500000; 1000000 with a 16 MHz crystal */
    bool              listen_only; /* never ACK, never transmit: sniff a live bus without disturbing it */
    bool              loopback;    /* frames you send come straight back; test with no transceiver */
} mcp2515_config_t;
#elif defined(HAL_SPI_MODULE_ENABLED)
typedef struct {
    SPI_HandleTypeDef *spi;        /* HAL_SPI_Init()ed: mode 0,0 or 1,1, 8-bit frames, 10 MHz at most */
    GPIO_TypeDef      *cs_port;    /* a push-pull output, idle high */
    uint16_t           cs_pin;
    GPIO_TypeDef      *int_port;   /* nINT as an input with a pull-up; NULL to poll the chip over SPI */
    uint16_t           int_pin;
    uint32_t           crystal_hz; /* 0 selects 8 MHz; as above */
    uint32_t           bitrate;    /* 0 selects 500000. Supported: 125000, 250000, 500000; 1000000 with 16 MHz */
    bool               listen_only;
    bool               loopback;
} mcp2515_config_t;
#endif

typedef struct {
    uint8_t tx_errors;    /* TEC */
    uint8_t rx_errors;    /* REC */
    bool    warning;      /* a counter reached 96 */
    bool    passive;      /* a counter reached 128 */
    bool    bus_off;      /* TEC reached 255: the chip stopped transmitting */
    bool    rx_overflow;  /* a frame was lost because mcp2515_receive() was not called in time */
} mcp2515_status_t;

#ifdef ESP_PLATFORM
esp_err_t mcp2515_init(const mcp2515_config_t *cfg, mcp2515_handle_t *out);
/* Waits up to timeout_ms for a free transmit buffer. 0 means do not wait. ESP_ERR_TIMEOUT if none freed. */
esp_err_t mcp2515_send(mcp2515_handle_t dev, const mcp2515_frame_t *frame, uint32_t timeout_ms);
/* Waits up to timeout_ms for a frame. ESP_ERR_TIMEOUT if none arrived. */
esp_err_t mcp2515_receive(mcp2515_handle_t dev, mcp2515_frame_t *frame, uint32_t timeout_ms);
/* Accept only frames where (frame.id & mask) == (id & mask). mask 0 accepts everything (the power-on state). Applies to both receive buffers. */
esp_err_t mcp2515_set_filter(mcp2515_handle_t dev, uint32_t id, uint32_t mask, bool extended);
esp_err_t mcp2515_read_status(mcp2515_handle_t dev, mcp2515_status_t *out);
/* Clears the receive-overflow flags. Bus-off recovers on its own once the bus is quiet. */
esp_err_t mcp2515_clear_errors(mcp2515_handle_t dev);
esp_err_t mcp2515_del(mcp2515_handle_t dev);
#else
#ifdef HAL_SPI_MODULE_ENABLED
int mcp2515_init(const mcp2515_config_t *cfg, mcp2515_handle_t *out);
#endif
int mcp2515_send(mcp2515_handle_t dev, const mcp2515_frame_t *frame, uint32_t timeout_ms);
int mcp2515_receive(mcp2515_handle_t dev, mcp2515_frame_t *frame, uint32_t timeout_ms);
int mcp2515_set_filter(mcp2515_handle_t dev, uint32_t id, uint32_t mask, bool extended);
int mcp2515_read_status(mcp2515_handle_t dev, mcp2515_status_t *out);
int mcp2515_clear_errors(mcp2515_handle_t dev);
int mcp2515_del(mcp2515_handle_t dev);
#endif

/* The handle as a struct can_dev; NULL for a NULL handle. Valid until
 * mcp2515_del(). Filters: up to two, the first on RXB0 and the second on
 * RXB1 (one alone goes on both); recover is a no-op, the chip leaves
 * bus-off on its own (DS p.47); start and stop leave and enter
 * Configuration mode, which clears the error counters (Section 10.1). */
struct can_dev *mcp2515_can(mcp2515_handle_t dev);

#ifdef __cplusplus
}
#endif
