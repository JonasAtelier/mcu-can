/*
 * Host-side check of the TWAI driver against a fake of ESP-IDF's legacy
 * driver (idf_stubs/driver/twai.h). The fake keeps what the driver relies
 * on, from ESP-IDF v5.5.5 components/driver/twai/twai.c: one install per
 * controller (:491); uninstall from STOPPED or BUS_OFF only (:608); start
 * from STOPPED (:652) and stop from RUNNING (:684); transmit refused in
 * listen-only (:717) and outside RUNNING (:722), waiting its ticks for
 * queue room; recovery from BUS_OFF only (:889), ending STOPPED
 * (:252-256); self-reception for a frame sent with self set (:742).
 */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"

#include "can.h"
#include "can_priv.h"
#include "twai_can.h"

#define PIN_TX	4
#define PIN_RX	5
#define PIN_RS	6

/* ------------------------------------------------------ the fake driver --- */

struct twai_obj_t {
	bool installed;
	twai_state_t state;
	twai_general_config_t g;
	twai_timing_config_t t;
	twai_filter_config_t f;
	twai_message_t rx[8];
	int rx_count;
	twai_message_t tx[8];
	int tx_count;
	bool tx_full;
	twai_status_info_t st;
	TickType_t last_ticks;
};

static struct twai_obj_t ctrl[2];
static int installs, uninstalls, starts;
static esp_err_t install_err, start_err;
static int rs_level = -1, rs_level_at_start = -1;
static gpio_config_t rs_cfg;

esp_err_t twai_driver_install_v2(const twai_general_config_t *g,
				 const twai_timing_config_t *t,
				 const twai_filter_config_t *f,
				 twai_handle_t *out)
{
	struct twai_obj_t *c;

	assert(g->controller_id >= 0 && g->controller_id < 2);
	c = &ctrl[g->controller_id];
	if (install_err)
		return install_err;
	if (c->installed)
		return ESP_ERR_INVALID_STATE;
	memset(c, 0, sizeof(*c));
	c->installed = true;
	c->state = TWAI_STATE_STOPPED;
	c->g = *g;
	c->t = *t;
	c->f = *f;
	installs++;
	*out = c;
	return ESP_OK;
}

esp_err_t twai_driver_uninstall_v2(twai_handle_t h)
{
	assert(h->installed);
	if (h->state != TWAI_STATE_STOPPED && h->state != TWAI_STATE_BUS_OFF)
		return ESP_ERR_INVALID_STATE;
	h->installed = false;
	uninstalls++;
	return ESP_OK;
}

esp_err_t twai_start_v2(twai_handle_t h)
{
	assert(h->installed);
	if (start_err)
		return start_err;
	if (h->state != TWAI_STATE_STOPPED)
		return ESP_ERR_INVALID_STATE;
	h->state = TWAI_STATE_RUNNING;
	rs_level_at_start = rs_level;
	starts++;
	return ESP_OK;
}

esp_err_t twai_stop_v2(twai_handle_t h)
{
	assert(h->installed);
	if (h->state != TWAI_STATE_RUNNING)
		return ESP_ERR_INVALID_STATE;
	h->state = TWAI_STATE_STOPPED;
	return ESP_OK;
}

esp_err_t twai_transmit_v2(twai_handle_t h, const twai_message_t *m,
			   TickType_t ticks)
{
	assert(h->installed);
	h->last_ticks = ticks;
	if (h->g.mode == TWAI_MODE_LISTEN_ONLY)
		return ESP_ERR_NOT_SUPPORTED;
	if (h->state != TWAI_STATE_RUNNING)
		return ESP_ERR_INVALID_STATE;
	if (h->tx_full) {
		vTaskDelay(ticks);
		return ESP_ERR_TIMEOUT;
	}
	h->tx[h->tx_count++ % 8] = *m;
	if (m->self && h->rx_count < 8)
		h->rx[h->rx_count++] = *m;
	return ESP_OK;
}

esp_err_t twai_receive_v2(twai_handle_t h, twai_message_t *m,
			  TickType_t ticks)
{
	assert(h->installed);
	h->last_ticks = ticks;
	if (h->rx_count == 0) {
		vTaskDelay(ticks);
		return ESP_ERR_TIMEOUT;
	}
	*m = h->rx[0];
	memmove(h->rx, h->rx + 1, sizeof(h->rx[0]) * (size_t)--h->rx_count);
	return ESP_OK;
}

esp_err_t twai_initiate_recovery_v2(twai_handle_t h)
{
	assert(h->installed);
	if (h->state != TWAI_STATE_BUS_OFF)
		return ESP_ERR_INVALID_STATE;
	h->state = TWAI_STATE_RECOVERING;
	return ESP_OK;
}

esp_err_t twai_get_status_info_v2(twai_handle_t h, twai_status_info_t *st)
{
	assert(h->installed);
	*st = h->st;
	st->state = h->state;
	st->msgs_to_rx = (uint32_t)h->rx_count;
	return ESP_OK;
}

esp_err_t gpio_config(const gpio_config_t *config)
{
	rs_cfg = *config;
	return ESP_OK;
}

esp_err_t gpio_set_level(gpio_num_t gpio, uint32_t level)
{
	assert(gpio == PIN_RS);
	rs_level = (int)level;
	return ESP_OK;
}

/* A frame off the bus. */
static void bus_frame(int c, uint32_t id, bool ext, bool rtr, uint8_t dlc)
{
	twai_message_t m = { .identifier = id, .data_length_code = dlc };

	m.extd = ext;
	m.rtr = rtr;
	for (int i = 0; i < 8; i++)
		m.data[i] = (uint8_t)(0xA0 + i);
	ctrl[c].rx[ctrl[c].rx_count++] = m;
}

/* -------------------------------------------------------------- cases --- */

static int allocs, frees, calloc_fail;

void *__real_calloc(size_t n, size_t size);
void __real_free(void *p);

void *__wrap_calloc(size_t n, size_t size);
void *__wrap_calloc(size_t n, size_t size)
{
	if (calloc_fail)
		return NULL;
	allocs++;
	return __real_calloc(n, size);
}

void __wrap_free(void *p);
void __wrap_free(void *p)
{
	if (p)
		frees++;
	__real_free(p);
}

static twaican_config_t cfg_of(void)
{
	return (twaican_config_t){
		.tx_gpio = PIN_TX,
		.rx_gpio = PIN_RX,
		.rs_gpio = TWAICAN_NO_GPIO,
	};
}

static twaican_handle_t up(const twaican_config_t *cfg)
{
	twaican_handle_t h = NULL;

	assert(twaican_init(cfg, &h) == ESP_OK && h);
	return h;
}

static void down(twaican_handle_t h)
{
	assert(twaican_del(h) == ESP_OK);
	assert(!ctrl[0].installed && !ctrl[1].installed && allocs == frees);
}

static void test_init(void)
{
	twaican_config_t cfg = cfg_of();
	twaican_handle_t h = up(&cfg);
	struct twai_obj_t *c = &ctrl[0];

	/* Defaults: 500 kbit/s, ESP-IDF's queues, normal, accept all. */
	assert(c->installed && c->state == TWAI_STATE_RUNNING);
	assert(c->g.controller_id == 0 && c->g.tx_io == PIN_TX &&
	       c->g.rx_io == PIN_RX && c->g.mode == TWAI_MODE_NORMAL);
	assert(c->g.tx_queue_len == 5 && c->g.rx_queue_len == 5);
	assert(c->t.quanta_resolution_hz == 500000);
	assert(c->f.acceptance_code == 0 && c->f.acceptance_mask == 0xFFFFFFFF &&
	       c->f.single_filter);
	assert(!strcmp(twaican_can(h)->name, "twai"));
	assert(twaican_can(NULL) == NULL);
	down(h);

	/* The second controller, at 1 Mbit/s, deeper queues, listen-only. */
	cfg.controller = 1;
	cfg.bitrate = 1000000;
	cfg.tx_queue_len = 2;
	cfg.rx_queue_len = 32;
	cfg.opmode = CAN_OPMODE_LISTEN;
	h = up(&cfg);
	assert(ctrl[1].installed && ctrl[1].t.quanta_resolution_hz == 1000000);
	assert(ctrl[1].g.tx_queue_len == 2 && ctrl[1].g.rx_queue_len == 32);
	assert(ctrl[1].g.mode == TWAI_MODE_LISTEN_ONLY);
	down(h);

	/* Every rate the IDF macros name. */
	{
		static const uint32_t rates[] = { 25000, 50000, 100000, 125000,
						  250000, 500000, 800000,
						  1000000 };

		for (size_t i = 0; i < sizeof(rates) / sizeof(rates[0]); i++) {
			cfg = cfg_of();
			cfg.bitrate = rates[i];
			h = up(&cfg);
			assert(ctrl[0].t.quanta_resolution_hz == rates[i]);
			down(h);
		}
	}

	/* The transceiver woken before the controller starts. */
	cfg = cfg_of();
	cfg.rs_gpio = PIN_RS;
	rs_level = 1;
	h = up(&cfg);
	assert(rs_cfg.pin_bit_mask == 1ULL << PIN_RS &&
	       rs_cfg.mode == GPIO_MODE_OUTPUT);
	assert(rs_level_at_start == 0);
	down(h);
}

static void test_init_failures(void)
{
	twaican_config_t cfg = cfg_of();
	twaican_handle_t h, h2;

	assert(twaican_init(NULL, &h) == ESP_ERR_INVALID_ARG);
	assert(twaican_init(&cfg, NULL) == ESP_ERR_INVALID_ARG);
	cfg.bitrate = 333333;
	assert(twaican_init(&cfg, &h) == ESP_ERR_INVALID_ARG);
	cfg = cfg_of();
	cfg.controller = -1;
	assert(twaican_init(&cfg, &h) == ESP_ERR_INVALID_ARG);
	cfg = cfg_of();
	cfg.opmode = (enum can_opmode)9;
	assert(twaican_init(&cfg, &h) == ESP_ERR_INVALID_ARG);
	assert(installs == 0 || !ctrl[0].installed);

	/* One owner per controller: the driver refuses the second. */
	cfg = cfg_of();
	h = up(&cfg);
	assert(twaican_init(&cfg, &h2) == ESP_ERR_INVALID_STATE);
	assert(strstr(stub_log_last, "controller 0 would not start"));
	down(h);

	install_err = ESP_ERR_NO_MEM;
	assert(twaican_init(&cfg, &h) == ESP_ERR_NO_MEM);
	install_err = ESP_OK;
	assert(allocs == frees);

	calloc_fail = 1;
	assert(twaican_init(&cfg, &h) == ESP_ERR_NO_MEM);
	calloc_fail = 0;
	assert(!ctrl[0].installed);
}

static void test_frames(void)
{
	twaican_config_t cfg = cfg_of();
	twaican_handle_t h = up(&cfg);
	struct can_dev *can = twaican_can(h);
	struct can_frame f = { .id = 0x123, .dlc = 3, .data = { 1, 2, 3 } };
	struct twai_obj_t *c = &ctrl[0];
	int64_t t0;

	assert(can_send(can, &f, 0) == 0);
	assert(c->tx[0].identifier == 0x123 && !c->tx[0].extd &&
	       !c->tx[0].rtr && !c->tx[0].self && c->tx[0].data[2] == 3 &&
	       c->tx[0].data_length_code == 3);
	assert(c->last_ticks == 0);		/* 0: no wait */
	f = (struct can_frame){ .id = 0x1ABCDE, .ext = true, .rtr = true,
				.dlc = 8 };
	assert(can_send(can, &f, 25) == 0);
	assert(c->tx[1].extd && c->tx[1].rtr && c->tx[1].data_length_code == 8);
	/* 25 ms at 100 Hz: 3 ticks rounded up, plus one. */
	assert(c->last_ticks == 4);

	c->tx_full = true;
	t0 = esp_timer_get_time();
	assert(can_send(can, &f, 25) == -ETIMEDOUT);
	assert(esp_timer_get_time() - t0 >= 25000);
	c->tx_full = false;

	bus_frame(0, 0x321, false, false, 8);
	bus_frame(0, 0x10, true, true, 2);
	bus_frame(0, 0x11, false, false, 12);	/* dlc_non_comp */
	assert(can_recv(can, &f, 0) == 0);
	assert(f.id == 0x321 && !f.ext && !f.rtr && f.dlc == 8 &&
	       f.data[7] == 0xA7 && f.t_us == esp_timer_get_time());
	assert(can_recv(can, &f, 0) == 0 && f.id == 0x10 && f.ext && f.rtr);
	assert(can_recv(can, &f, 0) == 0 && f.dlc == 8);
	t0 = esp_timer_get_time();
	assert(can_recv(can, &f, 15) == -ETIMEDOUT);
	assert(esp_timer_get_time() - t0 >= 15000);
	down(h);
}

static void test_modes(void)
{
	twaican_config_t cfg = cfg_of();
	twaican_handle_t h;
	struct can_dev *can;
	struct can_frame f = { .id = 0x55, .dlc = 1, .data = { 9 } };

	/* Loopback: NO_ACK, and every frame sent with self-reception. */
	cfg.opmode = CAN_OPMODE_LOOPBACK;
	h = up(&cfg);
	can = twaican_can(h);
	assert(ctrl[0].g.mode == TWAI_MODE_NO_ACK);
	assert(can_send(can, &f, 0) == 0 && ctrl[0].tx[0].self);
	f.id = 0;
	assert(can_recv(can, &f, 0) == 0 && f.id == 0x55 && f.data[0] == 9);
	down(h);

	/* Listen-only: the driver refuses to send. */
	cfg.opmode = CAN_OPMODE_LISTEN;
	h = up(&cfg);
	assert(can_send(twaican_can(h), &f, 0) == -EOPNOTSUPP);
	down(h);
}

/* Filter, bit rate and mode: only while stopped, applied at start. */
static void test_settings(void)
{
	twaican_config_t cfg = cfg_of();
	twaican_handle_t h = up(&cfg);
	struct can_dev *can = twaican_can(h);
	struct can_filter fl[2] = {
		{ .id = 0x120, .mask = 0x7F0 },
		{ .id = 0x18FF0000, .mask = 0x1FFF0000, .ext = true },
	};
	struct can_frame f;
	struct can_status st;

	assert(can_set_filters(can, fl, 1) == -EBUSY);
	assert(can_set_bitrate(can, 125000) == -EBUSY);
	assert(can_set_opmode(can, CAN_OPMODE_LISTEN) == -EBUSY);

	assert(can_stop(can) == 0 && !ctrl[0].installed);
	assert(can_get_status(can, &st) == 0 && st.link == CAN_LINK_STOPPED);
	f = (struct can_frame){ .id = 1 };
	assert(can_send(can, &f, 0) == -ENETDOWN);
	assert(can_recv(can, &f, 0) == -ENETDOWN);
	assert(can_set_filters(can, fl, 2) == -ENOSPC);
	assert(can_set_filters(can, fl, 1) == 0);
	assert(can_set_bitrate(can, 333333) == -EINVAL);
	assert(can_set_bitrate(can, 125000) == 0);
	assert(can_set_opmode(can, CAN_OPMODE_LISTEN) == 0);
	assert(can_start(can) == 0);
	/* ID at bit 21, mask inverted: twai_hal_v1.c:111-122. */
	assert(ctrl[0].f.acceptance_code == 0x120u << 21);
	assert(ctrl[0].f.acceptance_mask == ~(0x7F0u << 21));
	assert(ctrl[0].f.single_filter);
	assert(ctrl[0].t.quanta_resolution_hz == 125000);
	assert(ctrl[0].g.mode == TWAI_MODE_LISTEN_ONLY);

	/* The hardware filter cannot tell an extended frame whose top bits
	 * match; the core drops it. */
	bus_frame(0, 0x120u << 18, true, false, 0);
	bus_frame(0, 0x12F, false, false, 0);
	assert(can_recv(can, &f, 0) == -ETIMEDOUT);
	assert(can_recv(can, &f, 0) == 0 && f.id == 0x12F);

	/* Extended: at bit 3. */
	assert(can_stop(can) == 0);
	assert(can_set_filters(can, &fl[1], 1) == 0);
	assert(can_start(can) == 0);
	assert(ctrl[0].f.acceptance_code == 0x18FF0000u << 3);
	assert(ctrl[0].f.acceptance_mask == ~(0x1FFF0000u << 3));

	/* None: accept all again. */
	assert(can_stop(can) == 0);
	assert(can_set_filters(can, NULL, 0) == 0);
	assert(can_start(can) == 0);
	assert(ctrl[0].f.acceptance_mask == 0xFFFFFFFF);

	/* A failed install leaves it stopped, to try again. */
	assert(can_stop(can) == 0);
	install_err = ESP_ERR_NO_MEM;
	assert(can_start(can) == -ENOMEM);
	install_err = ESP_OK;
	assert(can_start(can) == 0);
	down(h);
}

static void test_status(void)
{
	twaican_config_t cfg = cfg_of();
	twaican_handle_t h = up(&cfg);
	struct can_dev *can = twaican_can(h);
	struct twai_obj_t *c = &ctrl[0];
	struct can_status st;

	c->st = (twai_status_info_t){
		.msgs_to_tx = 2, .tx_error_counter = 95, .rx_error_counter = 3,
		.tx_failed_count = 4, .rx_missed_count = 5,
		.rx_overrun_count = 6, .arb_lost_count = 7,
		.bus_error_count = 8,
	};
	bus_frame(0, 1, false, false, 0);
	assert(can_get_status(can, &st) == 0);
	assert(st.link == CAN_LINK_ACTIVE && st.tec == 95 && st.rec == 3);
	assert(st.tx_failed == 4 && st.rx_lost == 11 && st.arb_lost == 7 &&
	       st.bus_errors == 8 && st.tx_pending == 2 && st.rx_pending == 1);
	c->st.tx_error_counter = 96;
	assert(can_get_status(can, &st) == 0 && st.link == CAN_LINK_WARNING);
	c->st.tx_error_counter = 0;
	c->st.rx_error_counter = 127;
	assert(can_get_status(can, &st) == 0 && st.link == CAN_LINK_WARNING);
	c->st.rx_error_counter = 128;
	assert(can_get_status(can, &st) == 0 && st.link == CAN_LINK_PASSIVE);
	c->state = TWAI_STATE_BUS_OFF;
	c->st.tx_error_counter = 256;
	assert(can_get_status(can, &st) == 0 && st.link == CAN_LINK_BUS_OFF);
	assert(st.tec == 256);

	/* Recovery from bus-off only. */
	assert(can_recover(can) == 0 && c->state == TWAI_STATE_RECOVERING);
	assert(can_get_status(can, &st) == 0 && st.link == CAN_LINK_RECOVERING);
	assert(can_recover(can) == -EBUSY);
	/* Not while it recovers, either. */
	assert(twaican_del(h) == ESP_ERR_INVALID_STATE && c->installed);
	/* Done: the driver leaves it stopped; start again. */
	c->state = TWAI_STATE_STOPPED;
	assert(can_get_status(can, &st) == 0 && st.link == CAN_LINK_STOPPED);
	/* A start that fails there leaves the driver installed. */
	start_err = ESP_FAIL;
	assert(can_start(can) == -EIO && c->installed);
	start_err = ESP_OK;
	assert(can_start(can) == 0 && c->state == TWAI_STATE_RUNNING);
	assert(c->installed && installs > 0);
	down(h);
}

static void test_service(void)
{
	twaican_config_t cfg = cfg_of();
	twaican_handle_t h = up(&cfg);
	struct twai_obj_t *c = &ctrl[0];

	/* No restart_ms: bus-off stays. */
	c->state = TWAI_STATE_BUS_OFF;
	stub_time_us += 10000000;
	assert(twaican_service(h) == ESP_OK && c->state == TWAI_STATE_BUS_OFF);
	down(h);

	cfg.restart_ms = 100;
	h = up(&cfg);
	assert(twaican_service(h) == ESP_OK && c->state == TWAI_STATE_RUNNING);
	c->state = TWAI_STATE_BUS_OFF;
	assert(twaican_service(h) == ESP_OK && c->state == TWAI_STATE_BUS_OFF);
	assert(strstr(stub_log_last, "bus-off; recovery in 100 ms"));
	stub_time_us += 99000;
	assert(twaican_service(h) == ESP_OK && c->state == TWAI_STATE_BUS_OFF);
	stub_time_us += 1000;
	assert(twaican_service(h) == ESP_OK &&
	       c->state == TWAI_STATE_RECOVERING);
	assert(twaican_service(h) == ESP_OK &&
	       c->state == TWAI_STATE_RECOVERING);
	c->state = TWAI_STATE_STOPPED;		/* recovered */
	assert(twaican_service(h) == ESP_OK && c->state == TWAI_STATE_RUNNING);
	assert(strstr(stub_log_last, "recovered from bus-off"));
	/* A second bus-off waits its own 100 ms. */
	c->state = TWAI_STATE_BUS_OFF;
	assert(twaican_service(h) == ESP_OK && c->state == TWAI_STATE_BUS_OFF);
	stub_time_us += 100000;
	assert(twaican_service(h) == ESP_OK &&
	       c->state == TWAI_STATE_RECOVERING);
	c->state = TWAI_STATE_STOPPED;
	assert(twaican_service(h) == ESP_OK);

	/* Stopped by the app: left alone. */
	assert(can_stop(twaican_can(h)) == 0);
	assert(twaican_service(h) == ESP_OK && !c->installed);
	assert(twaican_service(NULL) == ESP_ERR_INVALID_ARG);
	down(h);
}

static void test_standby(void)
{
	twaican_config_t cfg = cfg_of();
	twaican_handle_t h = up(&cfg);
	struct can_frame f = { .id = 1 };

	assert(twaican_standby(h) == ESP_ERR_NOT_SUPPORTED);
	assert(twaican_wake(h) == ESP_ERR_NOT_SUPPORTED);
	down(h);

	cfg.rs_gpio = PIN_RS;
	h = up(&cfg);
	assert(rs_level == 0);
	assert(twaican_standby(h) == ESP_OK && rs_level == 1);
	assert(can_send(twaican_can(h), &f, 0) == -ENETDOWN);
	/* A restart keeps the transceiver as the app left it. */
	assert(can_stop(twaican_can(h)) == 0 && can_start(twaican_can(h)) == 0);
	assert(rs_level == 1);
	assert(twaican_wake(h) == ESP_OK && rs_level == 0);
	assert(can_send(twaican_can(h), &f, 0) == 0);
	assert(twaican_standby(NULL) == ESP_ERR_INVALID_ARG);
	assert(twaican_wake(NULL) == ESP_ERR_INVALID_ARG);
	assert(twaican_del(NULL) == ESP_ERR_INVALID_ARG);
	down(h);
}

int main(void)
{
	test_init();
	test_init_failures();
	test_frames();
	test_modes();
	test_settings();
	test_status();
	test_service();
	test_standby();
	printf("test_twai: all passed\n");
	return 0;
}
