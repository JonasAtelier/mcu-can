#ifndef TWAI_H
#define TWAI_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

/*
 * Host stub for ESP-IDF's legacy TWAI driver (driver/twai.h and
 * hal/twai_types_deprecated.h, v5.5.5): only what src/twai/twai_can.c uses.
 * Field names and the initialiser macros' shape match the real ones; the
 * timing macros carry the bit rate in quanta_resolution_hz alone, so a test
 * can tell which one was picked.
 */

typedef struct twai_obj_t *twai_handle_t;

typedef enum {
	TWAI_MODE_NORMAL,
	TWAI_MODE_NO_ACK,
	TWAI_MODE_LISTEN_ONLY,
} twai_mode_t;

typedef enum {
	TWAI_STATE_STOPPED,
	TWAI_STATE_RUNNING,
	TWAI_STATE_BUS_OFF,
	TWAI_STATE_RECOVERING,
} twai_state_t;

typedef struct {
	union {
		struct {
			uint32_t extd: 1;
			uint32_t rtr: 1;
			uint32_t ss: 1;
			uint32_t self: 1;
			uint32_t dlc_non_comp: 1;
			uint32_t reserved: 27;
		};
		uint32_t flags;
	};
	uint32_t identifier;
	uint8_t data_length_code;
	uint8_t data[8];
} twai_message_t;

#define TWAI_IO_UNUSED		((gpio_num_t)-1)
#define TWAI_ALERT_NONE		0x00000000
#define ESP_INTR_FLAG_LEVEL1	(1 << 1)

typedef struct {
	int controller_id;
	twai_mode_t mode;
	gpio_num_t tx_io;
	gpio_num_t rx_io;
	gpio_num_t clkout_io;
	gpio_num_t bus_off_io;
	uint32_t tx_queue_len;
	uint32_t rx_queue_len;
	uint32_t alerts_enabled;
	uint32_t clkout_divider;
	int intr_flags;
	struct {
		uint32_t sleep_allow_pd;
	} general_flags;
} twai_general_config_t;

#define TWAI_GENERAL_CONFIG_DEFAULT_V2(controller_num, tx_io_num, rx_io_num, op_mode) \
	{ .controller_id = controller_num, .mode = op_mode, .tx_io = tx_io_num, \
	  .rx_io = rx_io_num, .clkout_io = TWAI_IO_UNUSED, \
	  .bus_off_io = TWAI_IO_UNUSED, .tx_queue_len = 5, .rx_queue_len = 5, \
	  .alerts_enabled = TWAI_ALERT_NONE, .clkout_divider = 0, \
	  .intr_flags = ESP_INTR_FLAG_LEVEL1, .general_flags = { 0 } }

typedef struct {
	int clk_src;
	uint32_t quanta_resolution_hz;
	uint32_t brp;
	uint8_t prop_seg;
	uint8_t tseg_1;
	uint8_t tseg_2;
	uint8_t sjw;
	uint8_t ssp_offset;
	bool triple_sampling;
} twai_timing_config_t;

#define TWAI_TIMING_CONFIG_25KBITS()	{ .quanta_resolution_hz = 25000 }
#define TWAI_TIMING_CONFIG_50KBITS()	{ .quanta_resolution_hz = 50000 }
#define TWAI_TIMING_CONFIG_100KBITS()	{ .quanta_resolution_hz = 100000 }
#define TWAI_TIMING_CONFIG_125KBITS()	{ .quanta_resolution_hz = 125000 }
#define TWAI_TIMING_CONFIG_250KBITS()	{ .quanta_resolution_hz = 250000 }
#define TWAI_TIMING_CONFIG_500KBITS()	{ .quanta_resolution_hz = 500000 }
#define TWAI_TIMING_CONFIG_800KBITS()	{ .quanta_resolution_hz = 800000 }
#define TWAI_TIMING_CONFIG_1MBITS()	{ .quanta_resolution_hz = 1000000 }

typedef struct {
	uint32_t acceptance_code;
	uint32_t acceptance_mask;
	bool single_filter;
} twai_filter_config_t;

#define TWAI_FILTER_CONFIG_ACCEPT_ALL() \
	{ .acceptance_code = 0, .acceptance_mask = 0xFFFFFFFF, \
	  .single_filter = true }

typedef struct {
	twai_state_t state;
	uint32_t msgs_to_tx;
	uint32_t msgs_to_rx;
	uint32_t tx_error_counter;
	uint32_t rx_error_counter;
	uint32_t tx_failed_count;
	uint32_t rx_missed_count;
	uint32_t rx_overrun_count;
	uint32_t arb_lost_count;
	uint32_t bus_error_count;
} twai_status_info_t;

esp_err_t twai_driver_install_v2(const twai_general_config_t *g_config,
				 const twai_timing_config_t *t_config,
				 const twai_filter_config_t *f_config,
				 twai_handle_t *ret_twai);
esp_err_t twai_driver_uninstall_v2(twai_handle_t handle);
esp_err_t twai_start_v2(twai_handle_t handle);
esp_err_t twai_stop_v2(twai_handle_t handle);
esp_err_t twai_transmit_v2(twai_handle_t handle,
			   const twai_message_t *message,
			   TickType_t ticks_to_wait);
esp_err_t twai_receive_v2(twai_handle_t handle, twai_message_t *message,
			  TickType_t ticks_to_wait);
esp_err_t twai_initiate_recovery_v2(twai_handle_t handle);
esp_err_t twai_get_status_info_v2(twai_handle_t handle,
				  twai_status_info_t *status_info);

#endif
