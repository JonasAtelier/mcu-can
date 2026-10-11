/*
 * TWAI through the category: send a counter once a second, print what
 * arrives, and come back from bus-off on its own.
 *
 * Wiring: TX and RX to an SN65HVD230's D and R, its Rs pin to PIN_RS (or
 * TWAICAN_NO_GPIO and Rs tied low), 120 ohm at both ends of the bus.
 */
#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "can.h"
#include "can_log.h"
#include "twai_can.h"

#define PIN_TX      4
#define PIN_RX      5
#define PIN_RS      6

static const char *TAG = "example";

void app_main(void)
{
    twaican_config_t cfg = {
        .controller = 0,
        .tx_gpio = PIN_TX,
        .rx_gpio = PIN_RX,
        .rs_gpio = PIN_RS,
        .bitrate = 500000,
        .restart_ms = 1000,     /* bus-off: recover after a second */
    };
    twaican_handle_t twai;
    struct can_dev *can;
    uint32_t counter = 0;

    ESP_ERROR_CHECK(twaican_init(&cfg, &twai));
    can = twaican_can(twai);

    while (1) {
        struct can_frame tx = {
            .id = 0x321,
            .dlc = 4,
            .data = {
                (uint8_t)(counter >> 24), (uint8_t)(counter >> 16),
                (uint8_t)(counter >> 8),  (uint8_t)counter,
            },
        };
        struct can_frame rx;
        struct can_status st;
        int err;

        err = can_send(can, &tx, 100);
        if (err)
            ESP_LOGW(TAG, "send: %d", err);
        else
            counter++;

        while (can_recv(can, &rx, 100) == 0)
            can_log_frame(can, &rx);

        if (can_get_status(can, &st) == 0 && st.link != CAN_LINK_ACTIVE)
            ESP_LOGW(TAG, "link %d tec %u rec %u lost %u", (int)st.link,
                     st.tec, st.rec, (unsigned)st.rx_lost);
        twaican_service(twai);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
