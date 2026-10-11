/*
 * Host-side check of the mcp2515 driver on the STM32 HAL: ../mcp2515_cases.h
 * is the fake chip and the shared cases; this file feeds it from
 * HAL_SPI_Transmit/TransmitReceive() under a software chip select and the
 * nINT pin from HAL_GPIO_ReadPin(), and checks what only the HAL side has
 * -- the SPI handle checks and the chip select framing.
 */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "stm32_hal_stub.h"

#include "can.h"
#include "mcp2515.h"

#define SPI_LIVE_UP	0
#define RET(err)	(err)

static SPI_HandleTypeDef hspi = {
	.Init = {
		.CLKPolarity = SPI_POLARITY_LOW,
		.CLKPhase = SPI_PHASE_1EDGE,
		.DataSize = SPI_DATASIZE_8BIT,
	},
};
static GPIO_TypeDef cs_port, int_port;

static mcp2515_config_t cfg_of(bool int_wired)
{
	return (mcp2515_config_t){
		.spi = &hspi,
		.cs_port = &cs_port,
		.cs_pin = GPIO_PIN_4,
		.int_port = int_wired ? &int_port : NULL,
		.int_pin = GPIO_PIN_1,
		.crystal_hz = 8000000,
		.bitrate = 500000,
	};
}

/* The HAL port attaches nothing: nothing to count. */
static int spi_live(void)
{
	return 0;
}

static int64_t now_us(void)
{
	return (int64_t)stub_tick * 1000;
}

#include "../mcp2515_cases.h"

/* ------------------------------------------------- the HAL side of it --- */

static bool cs_low;
static int cs_frames;

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
	assert(port == &cs_port && pin == GPIO_PIN_4);
	if (state == GPIO_PIN_RESET) {
		assert(!cs_low);
		cs_frames++;
	}
	cs_low = state == GPIO_PIN_RESET;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
	assert(port == &int_port && pin == GPIO_PIN_1);
	return int_low() ? GPIO_PIN_RESET : GPIO_PIN_SET;
}

static HAL_StatusTypeDef spi_xfer(SPI_HandleTypeDef *h, const uint8_t *tx,
				  uint8_t *rx, uint16_t n)
{
	assert(h == &hspi && cs_low);
	if (spi_fails())
		return HAL_TIMEOUT;
	fake_spi(tx, rx, n);
	return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *h, const uint8_t *data,
				   uint16_t size, uint32_t timeout)
{
	uint8_t in[64];

	assert(size <= sizeof in && timeout > 0);
	return spi_xfer(h, data, in, size);
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h,
					  const uint8_t *tx, uint8_t *rx,
					  uint16_t size, uint32_t timeout)
{
	assert(timeout > 0);
	return spi_xfer(h, tx, rx, size);
}

/* ----------------------------------------------------------- STM cases --- */

/* One chip select frame per instruction (Section 12.1), CS high after. */
static void test_cs_framing(void)
{
	mcp2515_config_t cfg = cfg_of(false);
	mcp2515_handle_t d;

	chip_power_on();
	cs_frames = 0;
	assert(mcp2515_init(&cfg, &d) == 0);
	assert(cs_frames == chip.xfers && !cs_low);
	mcp2515_del(d);
}

/* Mode 0,0 or 1,1, eight bits; anything else refused before the bus. */
static void test_spi_handle(void)
{
	mcp2515_config_t cfg = cfg_of(false);
	mcp2515_handle_t d;

	chip_power_on();
	hspi.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi.Init.CLKPhase = SPI_PHASE_2EDGE;
	assert(mcp2515_init(&cfg, &d) == 0);
	mcp2515_del(d);

	hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
	chip_power_on();
	assert(mcp2515_init(&cfg, &d) == -EINVAL && chip.xfers == 0);
	hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi.Init.DataSize = SPI_DATASIZE_16BIT;
	assert(mcp2515_init(&cfg, &d) == -EINVAL && chip.xfers == 0);
	hspi.Init.DataSize = SPI_DATASIZE_8BIT;

	cfg.spi = NULL;
	assert(mcp2515_init(&cfg, &d) == -EINVAL);
	cfg = cfg_of(false);
	cfg.cs_port = NULL;
	assert(mcp2515_init(&cfg, &d) == -EINVAL);
}

int main(void)
{
	run_mcp2515_cases();
	test_cs_framing();
	test_spi_handle();
	printf("test_mcp2515 (stm): all passed\n");
	return 0;
}
