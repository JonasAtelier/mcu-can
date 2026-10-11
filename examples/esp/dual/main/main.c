/*
 * Two buses, one loop: everything the TWAI hears goes out on the MCP2515,
 * and the other way round -- written once, against struct can_dev.
 *
 * Wiring: the TWAI as in examples/esp/twai, the MCP2515 as in
 * examples/esp/mcp2515, each with its own transceiver. The pins are an
 * ESP32-S3's; set them for your board.
 */
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "can.h"
#include "can_log.h"
#include "mcp2515.h"
#include "twai_can.h"

#define PIN_TX      4
#define PIN_RX      5
#define PIN_MOSI    11
#define PIN_MISO    13
#define PIN_SCLK    12
#define PIN_CS      10
#define PIN_INT     9

static const char *TAG = "example";

/* What waits on from goes out on to; a full to drops it, logged. */
static void forward(struct can_dev *from, struct can_dev *to)
{
    struct can_frame f;

    while (can_recv(from, &f, 0) == 0) {
        int err = can_send(to, &f, 5);

        if (err)
            ESP_LOGW(TAG, "%s -> %s: dropped %03lx: %d", from->name,
                     to->name, (unsigned long)f.id, err);
    }
}

void app_main(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    mcp2515_config_t chip_cfg = {
        .host = SPI2_HOST,
        .cs_gpio = PIN_CS,
        .int_gpio = PIN_INT,
        .crystal_hz = 8000000,  /* read it off the can on the crystal */
        .bitrate = 500000,
    };
    twaican_config_t twai_cfg = {
        .tx_gpio = PIN_TX,
        .rx_gpio = PIN_RX,
        .rs_gpio = TWAICAN_NO_GPIO,
        .bitrate = 500000,
        .rx_queue_len = 32,
    };
    mcp2515_handle_t chip;
    twaican_handle_t twai;
    struct can_dev *a, *b;

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(mcp2515_init(&chip_cfg, &chip));
    ESP_ERROR_CHECK(twaican_init(&twai_cfg, &twai));
    a = twaican_can(twai);
    b = mcp2515_can(chip);

    while (1) {
        forward(a, b);
        forward(b, a);
        /* One tick: frames wait in the TWAI's queue and the MCP2515's two
         * buffers meanwhile. */
        vTaskDelay(1);
    }
}
