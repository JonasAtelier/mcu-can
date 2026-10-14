/*
 * Shared by the chip drivers inside this library. Not public API.
 *
 * Two layers sit under the chips, the way regmap sits under a kernel driver:
 *
 *   - The bus kind is chosen at run time. A chip holds a struct can_bus and
 *     reaches it only through bus->ops, so a driver can sit on I2C, SPI or
 *     a UART. Every category library offers all three kinds, used or not:
 *     the MCP2515 is on SPI; the TWAI and bxCAN controllers are on-chip and
 *     sit outside the bus layer, on ESP-IDF's driver and the HAL's.
 *   - The platform is chosen at link time. can_port_esp.c and
 *     can_port_stm.c each define the ops tables, the attach functions, the
 *     clock and the waits below for their platform; the other one compiles
 *     to nothing.
 *
 * ESP_PLATFORM, which ESP-IDF defines for every component, and the HAL's own
 * module switches are defined or not, never 0 or 1, so they are the one place
 * #ifdef is right; this library's CONFIG_CAN_* options are always tested
 * with #if.
 */
#ifndef CAN_PRIV_H
#define CAN_PRIV_H

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can.h"

/* The most bytes after cmd: tx on I2C and UART, tx and rx together on SPI. */
#define CAN_BUS_MAX_LEN		31

struct can_bus;

enum can_bus_kind {
	CAN_BUS_I2C,
	CAN_BUS_SPI,
	CAN_BUS_UART,
};

struct can_bus_ops {
	enum can_bus_kind kind;
	/*
	 * One transaction: cmd, then tx_len bytes of tx, then rx_len bytes
	 * into rx. cmd is the register address, with any SPI read bit already
	 * set by the chip, and the first byte of a datagram on a UART.
	 *
	 *   I2C:  rx_len ? cmd, repeated start, read : cmd and tx, one write;
	 *         a bare cmd is a one-byte write. Sending tx and then reading
	 *         is -EINVAL.
	 *   SPI:  CS low, cmd, tx, rx, CS high -- high again even when the
	 *         transfer failed.
	 *   UART: drop stale input, send cmd and tx, then wait for rx_len
	 *         bytes; fewer within the timeout is -ETIMEDOUT.
	 *
	 * On SPI more than CAN_BUS_MAX_LEN bytes after cmd is -EINVAL; on
	 * I2C and UART only tx is bounded so, and rx on the HAL by its 16-bit
	 * count. 0 or a negative errno.
	 */
	int (*transfer)(struct can_bus *bus, uint8_t cmd, const uint8_t *tx,
			size_t tx_len, uint8_t *rx, size_t rx_len);
	/*
	 * UART only; NULL on I2C and SPI. Wait for rx_len bytes with nothing
	 * sent, for a part that streams on its own instead of answering a
	 * command. drop_stale first drops whatever is already waiting, to
	 * start on fresh data. Fewer than rx_len within the timeout is
	 * -ETIMEDOUT; rx is then undefined.
	 */
	int (*receive)(struct can_bus *bus, uint8_t *rx, size_t rx_len,
		       bool drop_stale);
	/* Undo the attach. The bus itself is the app's and is left alone. */
	void (*detach)(struct can_bus *bus);
};

#ifdef ESP_PLATFORM
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

/* The I2C argument of every I2C attach on this platform. */
typedef i2c_master_bus_handle_t can_i2c_bus;

/*
 * One device on one bus, as its driver holds it. CAN_HAVE_BUS says the
 * platform has at least one bus kind, so the struct exists.
 */
#define CAN_HAVE_BUS
struct can_bus {
	const struct can_bus_ops *ops;
	uint32_t timeout_ms;
	union {
		i2c_master_dev_handle_t i2c;
		spi_device_handle_t spi;
		uart_port_t uart;
	};
};

/*
 * esp_err_t -> negative errno, at the bus boundary and nowhere else.
 * ESP_ERR_NOT_FOUND is -ENXIO (nothing at that address), not -ENODEV
 * (something answered but it is the wrong chip).
 * ESP_ERR_INVALID_STATE is -EBUSY here, for the TWAI driver's state errors,
 * unlike the other mcu-* libs, where an I2C mid-transfer NACK is -EIO; no
 * CAN chip uses I2C.
 */
static inline int can_errno(esp_err_t err)
{
	switch (err) {
	case ESP_OK:
		return 0;
	case ESP_ERR_INVALID_ARG:
		return -EINVAL;
	case ESP_ERR_TIMEOUT:
		return -ETIMEDOUT;
	case ESP_ERR_NOT_FOUND:
		return -ENXIO;
	case ESP_ERR_INVALID_SIZE:
		return -EMSGSIZE;
	case ESP_ERR_NO_MEM:
		return -ENOMEM;
	case ESP_ERR_NOT_SUPPORTED:
		return -EOPNOTSUPP;
	case ESP_ERR_INVALID_STATE:
		return -EBUSY;
	default:
		return -EIO;
	}
}

/*
 * negative errno -> esp_err_t, for the chips' handle APIs, which return
 * esp_err_t on ESP-IDF as the original driver did. The codes the original
 * returned come back as they were: -ENODEV and -ENXIO as
 * ESP_ERR_NOT_FOUND, -EPROTO (a chip that will not change mode) as
 * ESP_ERR_INVALID_RESPONSE. The bus layer's own mapping is lossy in this
 * direction: an IDF code with no errno of its own came up as -EIO and goes
 * back as ESP_FAIL.
 */
static inline esp_err_t can_esp_err(int err)
{
	switch (err) {
	case 0:
		return ESP_OK;
	case -EINVAL:
		return ESP_ERR_INVALID_ARG;
	case -ENOMEM:
		return ESP_ERR_NO_MEM;
	case -ETIMEDOUT:
		return ESP_ERR_TIMEOUT;
	case -EOPNOTSUPP:
		return ESP_ERR_NOT_SUPPORTED;
	case -ENODEV:
	case -ENXIO:
		return ESP_ERR_NOT_FOUND;
	case -EMSGSIZE:
		return ESP_ERR_INVALID_SIZE;
	case -EBUSY:
		return ESP_ERR_INVALID_STATE;
	case -EPROTO:
		return ESP_ERR_INVALID_RESPONSE;
	default:
		return ESP_FAIL;
	}
}

/*
 * Ticks for at least ms milliseconds, for a FreeRTOS wait: rounded up, plus
 * one for the tick that may be about to fire. Long, never short.
 */
TickType_t can_ticks(uint32_t ms);

#else /* STM32 HAL */
#include CMSIS_device_header

#if !defined(HAL_SPI_MODULE_ENABLED) && CONFIG_CAN_MCP2515
#error "CAN_MCP2515 needs the HAL SPI module: #define HAL_SPI_MODULE_ENABLED in the app's stm32<family>xx_hal_conf.h, with stm32<family>xx_hal_spi.c in the build"
#endif
#if !defined(HAL_CAN_MODULE_ENABLED) && CONFIG_CAN_BXCAN
#error "CAN_BXCAN needs the HAL CAN module: #define HAL_CAN_MODULE_ENABLED in the app's stm32<family>xx_hal_conf.h, with stm32<family>xx_hal_can.c in the build"
#endif
#if !defined(HAL_FDCAN_MODULE_ENABLED) && CONFIG_CAN_FDCAN
#error "CAN_FDCAN needs the HAL FDCAN module: #define HAL_FDCAN_MODULE_ENABLED in the app's stm32<family>xx_hal_conf.h, with stm32<family>xx_hal_fdcan.c in the build"
#endif

#ifdef HAL_I2C_MODULE_ENABLED
typedef I2C_HandleTypeDef *can_i2c_bus;
#endif

#if defined(HAL_I2C_MODULE_ENABLED) || defined(HAL_SPI_MODULE_ENABLED) || \
    defined(HAL_UART_MODULE_ENABLED)
#define CAN_HAVE_BUS
struct can_bus {
	const struct can_bus_ops *ops;
	uint32_t timeout_ms;
	union {
#ifdef HAL_I2C_MODULE_ENABLED
		struct {
			I2C_HandleTypeDef *handle;
			uint8_t address;	/* 7-bit */
		} i2c;
#endif
#ifdef HAL_SPI_MODULE_ENABLED
		struct {
			SPI_HandleTypeDef *handle;
			GPIO_TypeDef *cs_port;	/* app's output, idle high */
			uint16_t cs_pin;
		} spi;
#endif
#ifdef HAL_UART_MODULE_ENABLED
		UART_HandleTypeDef *uart;
#endif
	};
};
#endif

/*
 * HAL_StatusTypeDef -> negative errno, at the bus boundary and nowhere else.
 * HAL_BUSY -- the bus or the handle in use, or a handle never initialised --
 * is -EAGAIN. A NACK in the middle of a transfer is HAL_ERROR like any other
 * fault, so -EIO.
 */
static inline int can_hal_errno(HAL_StatusTypeDef status)
{
	switch (status) {
	case HAL_OK:
		return 0;
	case HAL_TIMEOUT:
		return -ETIMEDOUT;
	case HAL_BUSY:
		return -EAGAIN;
	default:
		return -EIO;
	}
}

#endif /* STM32 HAL */

/*
 * The chips' handle APIs return esp_err_t on ESP-IDF, as the original
 * driver did, and 0 or a negative errno on the HAL. The drivers work in
 * errno and convert once, where each public handle function returns.
 */
#ifdef ESP_PLATFORM
typedef esp_err_t can_handle_ret;
#define CAN_HANDLE_RET(err)	can_esp_err(err)
#else
typedef int can_handle_ret;
#define CAN_HANDLE_RET(err)	(err)
#endif

#ifdef CAN_HAVE_BUS
/* bus->ops->receive(), or -EOPNOTSUPP on a kind that has none. */
static inline int can_bus_receive(struct can_bus *bus, uint8_t *rx,
				  size_t rx_len, bool drop_stale)
{
	if (!bus->ops->receive)
		return -EOPNOTSUPP;
	return bus->ops->receive(bus, rx, rx_len, drop_stale);
}

static inline void can_bus_detach(struct can_bus *bus)
{
	bus->ops->detach(bus);
}
#endif

#if defined(ESP_PLATFORM) || defined(HAL_I2C_MODULE_ENABLED)
/*
 * Is anything at this 7-bit address? -ENXIO when nothing ACKs -- nothing at
 * that address -- not -ENODEV, which means something answered but it is the
 * wrong chip. A NACK in the middle of a transfer is a plain -EIO on both
 * platforms, which is why a driver decides presence here, never by its
 * first transfer.
 */
int can_i2c_probe(can_i2c_bus handle, uint8_t address, uint32_t timeout_ms);

/*
 * Bind bus to the I2C device at address. On ESP-IDF the driver adds the
 * device at scl_hz, 0 for 400 kHz; on the HAL the clock is the handle's and
 * scl_hz is ignored. Undo with can_bus_detach(). Every attach leaves
 * bus->ops NULL when it fails, so a bus with ops is one with something to
 * detach.
 */
int can_i2c_attach(struct can_bus *bus, can_i2c_bus handle,
		   uint8_t address, uint32_t scl_hz, uint32_t timeout_ms);
#endif

#ifdef ESP_PLATFORM
/*
 * Add the SPI device on host's cs_pin, in SPI mode 0-3 at clock_hz. The
 * host is the app's, already spi_bus_initialize()d.
 */
int can_spi_attach(struct can_bus *bus, spi_host_device_t host, int cs_pin,
		   uint32_t clock_hz, uint8_t mode);

/* port's driver is the app's, already installed and configured. */
int can_uart_attach(struct can_bus *bus, uart_port_t port,
		    uint32_t timeout_ms);
#else
#ifdef HAL_SPI_MODULE_ENABLED
/*
 * Bind bus to the SPI device behind cs_port/cs_pin, and deselect it, so the
 * first transfer starts on a falling edge. Mode, clock and frame are the
 * handle's; the chip select is an output the app has set up.
 */
int can_spi_attach(struct can_bus *bus, SPI_HandleTypeDef *handle,
		   GPIO_TypeDef *cs_port, uint16_t cs_pin,
		   uint32_t timeout_ms);
#endif
#ifdef HAL_UART_MODULE_ENABLED
/* handle is the app's, already HAL_UART_Init()ed. */
int can_uart_attach(struct can_bus *bus, UART_HandleTypeDef *handle,
		    uint32_t timeout_ms);
#endif
#endif

/* Block for at least ms milliseconds: long, never short. */
void can_sleep_ms(uint32_t ms);

/*
 * Between two polls: on ESP-IDF a yield of one tick, not a timed wait; on
 * the HAL, which has no scheduler, one millisecond.
 */
void can_yield(void);

/*
 * Whole milliseconds on can_time_us() since start, a value it returned. A
 * deadline is "can_elapsed_ms(start) >= timeout_ms": on the clock, so a
 * wait lasts as long whatever the tick rate and however long a poll takes.
 */
uint32_t can_elapsed_ms(int64_t start);

/*
 * A ring of frames an interrupt hands over: one writer, the ISR, through
 * can_ring_put(); one reader, the driver, through can_ring_get(). head and
 * tail run free and wrap; their difference is what is waiting. A frame that
 * finds the ring full is dropped and counted, never written over one the
 * reader has not taken.
 */
#define CAN_RING_LEN		16	/* a power of two */

struct can_ring {
	struct can_frame slot[CAN_RING_LEN];
	uint32_t head;		/* written by the ISR side only */
	uint32_t tail;		/* written by the reader only */
	uint32_t dropped;	/* frames that found the ring full */
};

/* Empty. */
void can_ring_init(struct can_ring *ring);

/* ISR-safe; never blocks. false: full, the frame dropped and counted. */
bool can_ring_put(struct can_ring *ring, const struct can_frame *frame);

/* The oldest frame into out; false if none waits. */
bool can_ring_get(struct can_ring *ring, struct can_frame *out);

/* Frames waiting. */
uint32_t can_ring_count(const struct can_ring *ring);

#endif /* CAN_PRIV_H */
