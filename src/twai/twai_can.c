/* First: on ESP-IDF it brings in sdkconfig.h, which the #if below reads. */
#include "can_priv.h"

#if defined(ESP_PLATFORM) && CONFIG_CAN_TWAI

#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/twai.h"

#include "can_log.h"
#include "twai_can.h"

#define DEFAULT_BITRATE 500000
#define DEFAULT_QUEUE   5       /* TWAI_GENERAL_CONFIG_DEFAULT_V2's */

/* The SN65HVD230's Rs pin: high is standby, low is high-speed mode (TI
 * SLOS346O, 10.4.1 and 10.4.3). */
#define RS_STANDBY      1
#define RS_ACTIVE       0

/*
 * ESP-IDF's twai_hal.h counts a controller error passive at 128 (state flag
 * ERR_PASSIVE, "TEC or REC is >= 128", twai_hal.h:44), and the legacy
 * driver sets the error warning limit to 96 (twai_hal_v1.c:17 and :62).
 * The legacy status has the counters but not these two states.
 */
#define WARNING_LIMIT   96
#define PASSIVE_LIMIT   128

/*
 * The acceptance filter in single filter mode: the identifier sits at the
 * top of the 32-bit code, at bit 21 for 11 bits and at bit 3 for 29, and a
 * 1 in the mask register means "don't care" -- so the mask goes in
 * inverted, and the bits below the identifier (RTR, and the first data
 * bytes of a standard frame) are left free. ESP-IDF's own conversion,
 * twai_hal_v1.c:111-122.
 */
#define FILTER_STD_SHIFT 21
#define FILTER_EXT_SHIFT 3

#define TRY(expr) do { int _e = (expr); if (_e) return _e; } while (0)

struct twaican_t {
    struct can_dev   can;       /* the category view; priv points back */
    twai_handle_t    handle;    /* NULL while uninstalled */
    int              controller;
    int              tx_gpio;
    int              rx_gpio;
    int              rs_gpio;
    uint32_t         bitrate;
    uint32_t         tx_queue_len;
    uint32_t         rx_queue_len;
    enum can_opmode  opmode;
    struct can_filter filter;
    bool             filtered;  /* false accepts every frame */
    bool             started;   /* between can_start() and can_stop() */
    bool             standby;
    uint32_t         restart_ms;
    int64_t          bus_off_us; /* when twaican_service() first saw it */
    bool             bus_off_seen;
};

/* ---- install ------------------------------------------------------------ */

static bool timing_of(uint32_t bitrate, twai_timing_config_t *t)
{
    switch (bitrate) {
    case 25000:
        *t = (twai_timing_config_t)TWAI_TIMING_CONFIG_25KBITS();
        break;
    case 50000:
        *t = (twai_timing_config_t)TWAI_TIMING_CONFIG_50KBITS();
        break;
    case 100000:
        *t = (twai_timing_config_t)TWAI_TIMING_CONFIG_100KBITS();
        break;
    case 125000:
        *t = (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS();
        break;
    case 250000:
        *t = (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();
        break;
    case 500000:
        *t = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
        break;
    case 800000:
        *t = (twai_timing_config_t)TWAI_TIMING_CONFIG_800KBITS();
        break;
    case 1000000:
        *t = (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();
        break;
    default:
        return false;
    }
    return true;
}

static twai_filter_config_t filter_of(twaican_handle_t d)
{
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    unsigned int shift;

    if (!d->filtered)
        return f;
    shift = d->filter.ext ? FILTER_EXT_SHIFT : FILTER_STD_SHIFT;
    f.acceptance_code = d->filter.id << shift;
    f.acceptance_mask = ~(d->filter.mask << shift);
    f.single_filter = true;
    return f;
}

/*
 * NORMAL and LISTEN_ONLY are the driver's own; loopback is NO_ACK, which
 * sends without waiting for an acknowledgement, plus self-reception on
 * every frame sent (twai.c:742, message->self).
 */
static twai_mode_t mode_of(enum can_opmode opmode)
{
    switch (opmode) {
    case CAN_OPMODE_LOOPBACK:
        return TWAI_MODE_NO_ACK;
    case CAN_OPMODE_LISTEN:
        return TWAI_MODE_LISTEN_ONLY;
    default:
        return TWAI_MODE_NORMAL;
    }
}

static int install(twaican_handle_t d)
{
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT_V2(
        d->controller, d->tx_gpio, d->rx_gpio, mode_of(d->opmode));
    twai_timing_config_t t;
    twai_filter_config_t f = filter_of(d);

    if (!timing_of(d->bitrate, &t))
        return -EINVAL;
    g.tx_queue_len = d->tx_queue_len;
    g.rx_queue_len = d->rx_queue_len;
    return can_errno(twai_driver_install_v2(&g, &t, &f, &d->handle));
}

static int uninstall(twaican_handle_t d)
{
    TRY(can_errno(twai_driver_uninstall_v2(d->handle)));
    d->handle = NULL;
    return 0;
}

/* Running: stop first. Bus-off: the driver uninstalls from there
 * (twai.c:608). Recovering: -EBUSY, from the driver. */
static int halt(twaican_handle_t d)
{
    twai_status_info_t st;

    if (d->handle == NULL)
        return 0;
    TRY(can_errno(twai_get_status_info_v2(d->handle, &st)));
    if (st.state == TWAI_STATE_RUNNING)
        TRY(can_errno(twai_stop_v2(d->handle)));
    return uninstall(d);
}

static int set_rs(twaican_handle_t d, int level)
{
    return can_errno(gpio_set_level(d->rs_gpio, (uint32_t)level));
}

/* ---- the can_dev ops ---------------------------------------------------- */

static twaican_handle_t of(struct can_dev *can)
{
    return can->priv;
}

static int op_send(struct can_dev *can, const struct can_frame *frame,
                   uint32_t timeout_ms)
{
    twaican_handle_t d = of(can);
    twai_message_t m = { 0 };

    /* Stopped, or the transceiver's driver is off. */
    if (d->handle == NULL || d->standby)
        return -ENETDOWN;
    m.identifier = frame->id;
    m.extd = frame->ext;
    m.rtr = frame->rtr;
    m.self = d->opmode == CAN_OPMODE_LOOPBACK;
    m.data_length_code = frame->dlc;
    if (!frame->rtr)
        memcpy(m.data, frame->data, frame->dlc);
    return can_errno(twai_transmit_v2(d->handle, &m,
                                      timeout_ms ? can_ticks(timeout_ms) : 0));
}

static int op_recv(struct can_dev *can, struct can_frame *frame,
                   uint32_t timeout_ms)
{
    twaican_handle_t d = of(can);
    twai_message_t m;

    if (d->handle == NULL)
        return -ENETDOWN;
    TRY(can_errno(twai_receive_v2(d->handle, &m,
                                  timeout_ms ? can_ticks(timeout_ms) : 0)));
    frame->id = m.identifier;
    frame->ext = m.extd;
    frame->rtr = m.rtr;
    /* The driver hands a DLC over 8 through as it came (dlc_non_comp), with
     * at most eight bytes stored. */
    frame->dlc = m.data_length_code > CAN_FRAME_DATA_MAX ?
                 CAN_FRAME_DATA_MAX : m.data_length_code;
    memcpy(frame->data, m.data, sizeof frame->data);
    return 0;
}

static int op_get_status(struct can_dev *can, struct can_status *status)
{
    twaican_handle_t d = of(can);
    twai_status_info_t st;
    uint32_t worst;

    if (d->handle == NULL) {
        status->link = CAN_LINK_STOPPED;
        return 0;
    }
    TRY(can_errno(twai_get_status_info_v2(d->handle, &st)));

    worst = st.tx_error_counter > st.rx_error_counter ? st.tx_error_counter :
                                                        st.rx_error_counter;
    switch (st.state) {
    case TWAI_STATE_RUNNING:
        status->link = worst >= PASSIVE_LIMIT ? CAN_LINK_PASSIVE :
                       worst >= WARNING_LIMIT ? CAN_LINK_WARNING :
                                                CAN_LINK_ACTIVE;
        break;
    case TWAI_STATE_BUS_OFF:
        status->link = CAN_LINK_BUS_OFF;
        break;
    case TWAI_STATE_RECOVERING:
        status->link = CAN_LINK_RECOVERING;
        break;
    default:
        status->link = CAN_LINK_STOPPED;
        break;
    }
    status->tec = (uint16_t)st.tx_error_counter;
    status->rec = (uint16_t)st.rx_error_counter;
    status->rx_lost = st.rx_missed_count + st.rx_overrun_count;
    status->tx_failed = st.tx_failed_count;
    status->arb_lost = st.arb_lost_count;
    status->bus_errors = st.bus_error_count;
    status->rx_pending = st.msgs_to_rx;
    status->tx_pending = st.msgs_to_tx;
    return 0;
}

static int op_set_filters(struct can_dev *can,
                          const struct can_filter *filters, size_t count)
{
    twaican_handle_t d = of(can);

    if (d->started)
        return -EBUSY;
    if (count > 1)
        return -ENOSPC;
    d->filtered = count == 1;
    if (count == 1)
        d->filter = filters[0];
    return 0;
}

static int op_set_bitrate(struct can_dev *can, uint32_t bitrate)
{
    twaican_handle_t d = of(can);
    twai_timing_config_t t;

    if (d->started)
        return -EBUSY;
    if (!timing_of(bitrate, &t))
        return -EINVAL;
    d->bitrate = bitrate;
    return 0;
}

static int op_set_opmode(struct can_dev *can, enum can_opmode mode)
{
    twaican_handle_t d = of(can);

    if (d->started)
        return -EBUSY;
    d->opmode = mode;
    return 0;
}

/* Only from bus-off; anything else is -EBUSY, from the driver. */
static int op_recover(struct can_dev *can)
{
    twaican_handle_t d = of(can);

    if (d->handle == NULL)
        return -ENETDOWN;
    return can_errno(twai_initiate_recovery_v2(d->handle));
}

/*
 * Install with the settings as they stand now, unless a recovery left the
 * driver installed and stopped; then start. The transceiver is woken
 * between the two, as esp-sn65hvd230-can does.
 */
static int op_start(struct can_dev *can)
{
    twaican_handle_t d = of(can);
    bool fresh = d->handle == NULL;
    int err = 0;

    if (fresh)
        TRY(install(d));
    if (d->rs_gpio != TWAICAN_NO_GPIO && !d->standby)
        err = set_rs(d, RS_ACTIVE);
    if (!err)
        err = can_errno(twai_start_v2(d->handle));
    if (err) {
        if (fresh)
            (void)uninstall(d);
        return err;
    }
    d->started = true;
    d->bus_off_seen = false;
    return 0;
}

/* Stopped and uninstalled, so the next start installs anew -- the only way
 * the legacy driver takes a new filter, mode or bit rate. Frames still
 * queued either way are dropped. */
static int op_stop(struct can_dev *can)
{
    twaican_handle_t d = of(can);

    TRY(halt(d));
    d->started = false;
    return 0;
}

static const struct can_ops twaican_ops = {
    .send = op_send,
    .recv = op_recv,
    .get_status = op_get_status,
    .set_filters = op_set_filters,
    .set_bitrate = op_set_bitrate,
    .set_opmode = op_set_opmode,
    .recover = op_recover,
    .start = op_start,
    .stop = op_stop,
};

/* ---- init --------------------------------------------------------------- */

static int init(const twaican_config_t *cfg, twaican_handle_t *out)
{
    twai_timing_config_t t;
    twaican_handle_t d;
    int err = 0;

    if (cfg == NULL || out == NULL || cfg->controller < 0 ||
        (cfg->opmode != CAN_OPMODE_NORMAL &&
         cfg->opmode != CAN_OPMODE_LOOPBACK &&
         cfg->opmode != CAN_OPMODE_LISTEN))
        return -EINVAL;
    if (!timing_of(cfg->bitrate ? cfg->bitrate : DEFAULT_BITRATE, &t))
        return -EINVAL;

    d = calloc(1, sizeof *d);
    if (d == NULL)
        return -ENOMEM;
    d->can.name = "twai";
    d->can.ops = &twaican_ops;
    d->can.priv = d;
    d->controller = cfg->controller;
    d->tx_gpio = cfg->tx_gpio;
    d->rx_gpio = cfg->rx_gpio;
    d->rs_gpio = cfg->rs_gpio;
    d->bitrate = cfg->bitrate ? cfg->bitrate : DEFAULT_BITRATE;
    d->tx_queue_len = cfg->tx_queue_len ? cfg->tx_queue_len : DEFAULT_QUEUE;
    d->rx_queue_len = cfg->rx_queue_len ? cfg->rx_queue_len : DEFAULT_QUEUE;
    d->restart_ms = cfg->restart_ms;
    d->opmode = cfg->opmode;

    if (d->rs_gpio != TWAICAN_NO_GPIO) {
        gpio_config_t io_cfg = {
            .pin_bit_mask = 1ULL << (unsigned)d->rs_gpio,
            .mode = GPIO_MODE_OUTPUT,
        };

        err = can_errno(gpio_config(&io_cfg));
    }
    if (!err)
        err = op_start(&d->can);
    if (err) {
        can_err(&d->can, "controller %d would not start: %d", d->controller,
                err);
        free(d);
        return err;
    }
    *out = d;
    return 0;
}

static int service(twaican_handle_t d)
{
    twai_status_info_t st;

    if (!d->started || d->handle == NULL || d->restart_ms == 0)
        return 0;
    TRY(can_errno(twai_get_status_info_v2(d->handle, &st)));

    switch (st.state) {
    case TWAI_STATE_BUS_OFF:
        if (!d->bus_off_seen) {
            d->bus_off_seen = true;
            d->bus_off_us = can_time_us();
            can_warn(&d->can, "bus-off; recovery in %u ms",
                     (unsigned)d->restart_ms);
        }
        if (can_elapsed_ms(d->bus_off_us) < d->restart_ms)
            return 0;
        return can_errno(twai_initiate_recovery_v2(d->handle));
    case TWAI_STATE_STOPPED:
        /* Recovered: the driver leaves the controller stopped. */
        d->bus_off_seen = false;
        can_info(&d->can, "recovered from bus-off; restarting");
        return can_errno(twai_start_v2(d->handle));
    default:
        return 0;
    }
}

/* ---- the handle API ----------------------------------------------------- */

esp_err_t twaican_init(const twaican_config_t *cfg, twaican_handle_t *out)
{
    return CAN_HANDLE_RET(init(cfg, out));
}

esp_err_t twaican_del(twaican_handle_t dev)
{
    int err;

    if (dev == NULL)
        return CAN_HANDLE_RET(-EINVAL);
    err = halt(dev);
    if (err)
        return CAN_HANDLE_RET(err);
    free(dev);
    return ESP_OK;
}

esp_err_t twaican_standby(twaican_handle_t dev)
{
    int err;

    if (dev == NULL)
        return CAN_HANDLE_RET(-EINVAL);
    if (dev->rs_gpio == TWAICAN_NO_GPIO)
        return CAN_HANDLE_RET(-EOPNOTSUPP);
    err = set_rs(dev, RS_STANDBY);
    if (!err)
        dev->standby = true;
    return CAN_HANDLE_RET(err);
}

/* t(WAKE) is 1.5 us at most (SLOS346O, 8.10): over before the next frame
 * could be handed to the driver. */
esp_err_t twaican_wake(twaican_handle_t dev)
{
    int err;

    if (dev == NULL)
        return CAN_HANDLE_RET(-EINVAL);
    if (dev->rs_gpio == TWAICAN_NO_GPIO)
        return CAN_HANDLE_RET(-EOPNOTSUPP);
    err = set_rs(dev, RS_ACTIVE);
    if (!err)
        dev->standby = false;
    return CAN_HANDLE_RET(err);
}

esp_err_t twaican_service(twaican_handle_t dev)
{
    if (dev == NULL)
        return CAN_HANDLE_RET(-EINVAL);
    return CAN_HANDLE_RET(service(dev));
}

struct can_dev *twaican_can(twaican_handle_t dev)
{
    return dev ? &dev->can : NULL;
}

#endif /* ESP_PLATFORM && CONFIG_CAN_TWAI */
