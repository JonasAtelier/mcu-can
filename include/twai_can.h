/*
 * twai_can -- the ESP32's on-chip CAN controller (TWAI) as a struct can_dev,
 * on ESP-IDF's legacy driver: driver/twai.h and its _v2 calls, which take a
 * controller number, so a part with two TWAIs drives both.
 *
 * ESP-IDF only; there is no TWAI on an STM32. Frames, filters, status and
 * the rest go through can.h on twaican_can(); this header is only what the
 * category has no word for: install, transceiver standby, and bus-off
 * recovery on a timer.
 *
 * One owner per controller. The legacy driver refuses a second install on
 * a controller already installed (ESP-IDF twai.c:491, ESP_ERR_INVALID_STATE).
 * So do not enable CAN_TWAI for a controller esp-sn65hvd230-can already
 * drives; the MCP2515 driver of this library sits next to it fine. Nor mix
 * it with ESP-IDF's new esp_twai driver: twai.c:1002-1018 is the legacy
 * driver's start-up check against exactly that.
 *
 * The filter, the operating mode and the bit rate are install-time settings
 * of the legacy driver. They change only while the controller is stopped,
 * after can_stop() -- -EBUSY otherwise -- and take effect at can_start().
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TWAICAN_NO_GPIO (-1)

typedef struct twaican_t *twaican_handle_t;

typedef struct {
    int      controller;    /* 0; 1 as well on a part with two TWAIs */
    int      tx_gpio;
    int      rx_gpio;
    int      rs_gpio;       /* the transceiver's Rs/standby pin, as on an
                             * SN65HVD230; TWAICAN_NO_GPIO if not wired.
                             * Mind that 0 is GPIO0 */
    uint32_t bitrate;       /* 0 selects 500000. 25000, 50000, 100000,
                             * 125000, 250000, 500000, 800000 or 1000000:
                             * ESP-IDF's TWAI_TIMING_CONFIG_* rates */
    uint32_t tx_queue_len;  /* 0 selects 5, ESP-IDF's default */
    uint32_t rx_queue_len;  /* 0 selects 5 */
    uint32_t restart_ms;    /* bus-off: twaican_service() starts recovery
                             * once it has lasted this long; 0 never does */
    enum can_opmode opmode; /* CAN_OPMODE_NORMAL unless set */
} twaican_config_t;

/* Install the driver and start the controller, transceiver awake. */
esp_err_t twaican_init(const twaican_config_t *cfg, twaican_handle_t *out);

/* Stop the controller and uninstall the driver; ESP_ERR_INVALID_STATE
 * while a bus-off recovery runs, which cannot be cut short. */
esp_err_t twaican_del(twaican_handle_t dev);

/*
 * The SN65HVD230's Rs pin: high puts it in its listen-only standby, driver
 * off, receiver on; low brings it back (TI SLOS346O, 10.4.1 and 10.4.3).
 * Sends are refused in standby. ESP_ERR_NOT_SUPPORTED without rs_gpio.
 */
esp_err_t twaican_standby(twaican_handle_t dev);
esp_err_t twaican_wake(twaican_handle_t dev);

/*
 * Call from the app's loop. With restart_ms set: a bus-off that has lasted
 * restart_ms starts recovery; once recovery is done -- which the driver
 * leaves stopped (ESP-IDF twai.c:252-256) -- start the controller again.
 * Without, it does nothing: a bus that keeps failing is usually a wiring
 * fault, and retrying forever hides it.
 */
esp_err_t twaican_service(twaican_handle_t dev);

/* The handle as a struct can_dev; NULL for a NULL handle. Valid until
 * twaican_del(). One filter: the TWAI's single filter mode. */
struct can_dev *twaican_can(twaican_handle_t dev);

#ifdef __cplusplus
}
#endif
