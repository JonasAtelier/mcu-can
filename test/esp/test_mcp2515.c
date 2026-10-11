/*
 * Host-side check of the mcp2515 driver on ESP-IDF: ../mcp2515_cases.h is
 * the fake chip and the shared cases; this file feeds it from the
 * spi_master calls and the nINT pin from gpio_get_level(), and checks what
 * only ESP-IDF has -- the SPI device added to the app's host and removed
 * again, its clock and mode, the nINT pin's input with pull-up, and the
 * esp_err_t codes callers of esp-mcp2515-can saw.
 */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "can.h"
#include "can_priv.h"
#include "mcp2515.h"

#define SPI_LIVE_UP	1
#define RET(err)	can_esp_err(err)

#define PIN_CS		10
#define PIN_INT		9

static mcp2515_config_t cfg_of(bool int_wired)
{
	return (mcp2515_config_t){
		.host = SPI2_HOST,
		.cs_gpio = PIN_CS,
		.int_gpio = int_wired ? PIN_INT : GPIO_NUM_NC,
		.crystal_hz = 8000000,
		.bitrate = 500000,
	};
}

static int spi_live(void);

static int64_t now_us(void)
{
	return stub_time_us;
}

#include "../mcp2515_cases.h"

/* ------------------------------------------------- the IDF side of it --- */

static struct spi_device_t {
	bool live;
	int hz, mode, cs;
} spi_dev;

static int spi_live(void)
{
	return spi_dev.live;
}

esp_err_t spi_bus_add_device(spi_host_device_t host,
			     const spi_device_interface_config_t *config,
			     spi_device_handle_t *out)
{
	assert(host == SPI2_HOST && !spi_dev.live);
	spi_dev = (struct spi_device_t){
		.live = true,
		.hz = config->clock_speed_hz,
		.mode = config->mode,
		.cs = config->spics_io_num,
	};
	*out = &spi_dev;
	return ESP_OK;
}

esp_err_t spi_bus_remove_device(spi_device_handle_t dev)
{
	assert(dev == &spi_dev && dev->live);
	dev->live = false;
	return ESP_OK;
}

esp_err_t spi_device_polling_transmit(spi_device_handle_t dev,
				      spi_transaction_t *t)
{
	assert(dev->live && t->length % 8 == 0);
	if (spi_fails())
		return ESP_ERR_TIMEOUT;
	fake_spi(t->tx_buffer, t->rx_buffer, t->length / 8);
	return ESP_OK;
}

static gpio_config_t int_cfg;
static int gpio_configs;

esp_err_t gpio_config(const gpio_config_t *config)
{
	int_cfg = *config;
	gpio_configs++;
	return ESP_OK;
}

int gpio_get_level(gpio_num_t gpio)
{
	assert(gpio == PIN_INT);
	return int_low() ? 0 : 1;
}

/* ----------------------------------------------------------- ESP cases --- */

/* Mode 0 at 10 MHz unless asked; nINT an input with its pull-up. */
static void test_esp_device(void)
{
	mcp2515_config_t cfg = cfg_of(true);
	mcp2515_handle_t d;

	chip_power_on();
	gpio_configs = 0;
	assert(mcp2515_init(&cfg, &d) == ESP_OK);
	assert(spi_dev.hz == 10000000 && spi_dev.mode == 0 &&
	       spi_dev.cs == PIN_CS);
	assert(gpio_configs == 1 && int_cfg.pin_bit_mask == 1ULL << PIN_INT &&
	       int_cfg.mode == GPIO_MODE_INPUT && int_cfg.pull_up_en);
	assert(mcp2515_del(d) == ESP_OK && !spi_dev.live);

	cfg = cfg_of(false);
	cfg.spi_hz = 4000000;
	chip_power_on();
	gpio_configs = 0;
	assert(mcp2515_init(&cfg, &d) == ESP_OK);
	assert(spi_dev.hz == 4000000 && gpio_configs == 0);
	mcp2515_del(d);

	/* A zeroed config is 8 MHz, 500 kbit/s. */
	cfg = (mcp2515_config_t){ .host = SPI2_HOST, .cs_gpio = PIN_CS,
				  .int_gpio = GPIO_NUM_NC };
	chip_power_on();
	assert(mcp2515_init(&cfg, &d) == ESP_OK);
	assert(chip.reg[0x29] == 0x90);
	mcp2515_del(d);
}

/* The codes esp-mcp2515-can's callers saw. */
static void test_esp_errors(void)
{
	mcp2515_config_t cfg = cfg_of(false);
	mcp2515_handle_t d;

	chip_power_on();
	chip.present = false;
	assert(mcp2515_init(&cfg, &d) == ESP_ERR_NOT_FOUND);
	chip_power_on();
	chip.mode_stuck = true;
	assert(mcp2515_init(&cfg, &d) == ESP_ERR_INVALID_RESPONSE);
	cfg.bitrate = 333333;
	assert(mcp2515_init(&cfg, &d) == ESP_ERR_INVALID_ARG);
}

int main(void)
{
	run_mcp2515_cases();
	test_esp_device();
	test_esp_errors();
	printf("test_mcp2515 (esp): all passed\n");
	return 0;
}
