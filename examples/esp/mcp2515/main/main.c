/*
 * MCP2515: send a counter once a second and print everything that arrives.
 *
 * Wiring: any SPI bus plus CS, and nINT to a GPIO (or pass GPIO_NUM_NC and the
 * driver polls over SPI instead). A transceiver -- MCP2551, TJA1050,
 * SN65HVD230 -- goes between the chip and the bus, and both ends of the bus
 * need a 120 ohm terminator.
 *
 * CRYSTAL_HZ must match the crystal actually fitted next to the chip. Boards
 * carry 8 MHz and 16 MHz and look identical; getting it wrong gives you half
 * or double the bit rate, and a bus that is silent or corrupted rather than an
 * error you can read.
 *
 * esp-mcp2515-can's example, unchanged but for this note: the pins are an
 * ESP32's; set them for your board.
 */
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mcp2515.h"

#define PIN_MOSI    23
#define PIN_MISO    19
#define PIN_SCLK    18
#define PIN_CS      5
#define PIN_INT     4

#define CRYSTAL_HZ  8000000     /* read it off the can on the crystal */
#define BITRATE     500000

static const char *TAG = "example";

static void print_frame(const mcp2515_frame_t *f)
{
    printf("rx %s%s id 0x%03lx len %u  ",
           f->extended ? "ext " : "std ",
           f->remote ? "rtr " : "    ",
           (unsigned long)f->id, f->len);

    for (uint8_t i = 0; i < f->len; i++)
        printf("%02x ", f->data[i]);

    printf("\n");
}

void app_main(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 16,
    };
    mcp2515_config_t cfg = {
        .host = SPI2_HOST,
        .cs_gpio = PIN_CS,
        .int_gpio = PIN_INT,
        .crystal_hz = CRYSTAL_HZ,
        .bitrate = BITRATE,
    };
    mcp2515_handle_t can;
    uint32_t counter = 0;

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(mcp2515_init(&cfg, &can));

    ESP_LOGI(TAG, "MCP2515 up at %d bit/s on a %d Hz crystal", BITRATE, CRYSTAL_HZ);

    while (1) {
        mcp2515_frame_t tx = {
            .id = 0x123,
            .len = 4,
            .data = {
                (uint8_t)(counter >> 24), (uint8_t)(counter >> 16),
                (uint8_t)(counter >> 8),  (uint8_t)counter,
            },
        };
        mcp2515_frame_t rx;
        esp_err_t err;

        /* Nobody on the bus means nobody acknowledges, and the chip keeps
         * retrying until a transmit buffer frees up. That is what the timeout
         * is for, and why this reports rather than ESP_ERROR_CHECKs. */
        err = mcp2515_send(can, &tx, 100);
        if (err != ESP_OK)
            ESP_LOGW(TAG, "send: %s", esp_err_to_name(err));
        else
            counter++;

        /* Drain everything that arrived in the last second. */
        while (mcp2515_receive(can, &rx, 100) == ESP_OK)
            print_frame(&rx);

        mcp2515_status_t st;
        if (mcp2515_read_status(can, &st) == ESP_OK && (st.warning || st.bus_off)) {
            ESP_LOGW(TAG, "tec %u rec %u%s%s%s", st.tx_errors, st.rx_errors,
                     st.passive ? " passive" : "",
                     st.bus_off ? " bus-off" : "",
                     st.rx_overflow ? " overflow" : "");
            if (st.rx_overflow)
                mcp2515_clear_errors(can);
        }
    }
}
