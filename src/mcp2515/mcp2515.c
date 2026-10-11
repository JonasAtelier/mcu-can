/* First: on ESP-IDF it brings in sdkconfig.h, which the #if below reads. */
#include "can_priv.h"

#if CONFIG_CAN_MCP2515

#include <stdlib.h>
#include <string.h>

#ifdef ESP_PLATFORM
#include "driver/gpio.h"
#endif

#include "can_log.h"
#include "mcp2515.h"
#include "mcp2515_math.h"
#include "mcp2515_regs.h"

#define DEFAULT_SPI_HZ  MCP2515_SPI_MAX_HZ
#define DEFAULT_XTAL_HZ 8000000
#define DEFAULT_BITRATE 500000

#define RESET_WAIT_MS   10  /* the oscillator start-up timer is 128 TOSC
                             * (Section 10.2.1); 10 ms is generous */
#define MODE_WAIT_MS    10
#define TIMEOUT_MS      100 /* one SPI transfer on the HAL */

#define TRY(expr) do { int _e = (expr); if (_e) return _e; } while (0)

struct mcp2515_t {
    struct can_dev can;     /* the category view; priv points back */
    struct can_bus bus;
#ifdef ESP_PLATFORM
    gpio_num_t     int_gpio;
#else
    GPIO_TypeDef  *int_port;
    uint16_t       int_pin;
#endif
    uint32_t       crystal_hz;
    uint8_t        mode;    /* the running mode to return to after config */
    bool           stopped; /* can_stop(): parked in Configuration mode */
};

/* ---- SPI ---------------------------------------------------------------- */

/*
 * Section 12.1: the first byte after CS falls is always the instruction, and
 * CS must rise before another one. That is one bus transfer per call: the
 * instruction as cmd, then what follows it.
 */
static int xfer(mcp2515_handle_t d, uint8_t cmd, const uint8_t *tx,
                size_t tx_len, uint8_t *rx, size_t rx_len)
{
    return d->bus.ops->transfer(&d->bus, cmd, tx, tx_len, rx, rx_len);
}

/* READ, Section 12.3. The address pointer auto-increments, so n consecutive
 * registers cost one transfer. */
static int reg_read(mcp2515_handle_t d, uint8_t addr, uint8_t *buf, size_t n)
{
    return xfer(d, MCP2515_READ, &addr, 1, buf, n);
}

/* WRITE, Section 12.5, likewise sequential. */
static int reg_write(mcp2515_handle_t d, uint8_t addr, const uint8_t *vals,
                     size_t n)
{
    uint8_t tx[1 + 4];

    if (n > 4)
        return -EINVAL;
    tx[0] = addr;
    memcpy(&tx[1], vals, n);
    return xfer(d, MCP2515_WRITE, tx, 1 + n, NULL, 0);
}

static int reg_write1(mcp2515_handle_t d, uint8_t addr, uint8_t val)
{
    return reg_write(d, addr, &val, 1);
}

/* BIT MODIFY, Section 12.10: a 1 in the mask lets that bit take its value
 * from the data byte. Only some registers accept it -- the shaded ones in
 * Table 11-1 -- and CANCTRL and EFLG, the two used here, both do. */
static int bit_modify(mcp2515_handle_t d, uint8_t addr, uint8_t mask,
                      uint8_t data)
{
    uint8_t tx[3] = { addr, mask, data };

    return xfer(d, MCP2515_BIT_MODIFY, tx, sizeof tx, NULL, 0);
}

/* READ STATUS, Section 12.8: the receive flags and all three TXREQ bits in a
 * single two-byte transfer, instead of one READ per register. */
static int read_status(mcp2515_handle_t d, uint8_t *status)
{
    return xfer(d, MCP2515_READ_STATUS, NULL, 0, status, 1);
}

/* nINT is low while any enabled flag is set (Section 7.0). */
static bool int_asserted(mcp2515_handle_t d)
{
#ifdef ESP_PLATFORM
    return d->int_gpio == GPIO_NUM_NC || gpio_get_level(d->int_gpio) == 0;
#else
    return d->int_port == NULL ||
           HAL_GPIO_ReadPin(d->int_port, d->int_pin) == GPIO_PIN_RESET;
#endif
}

/*
 * Ask for a mode and wait until the chip is actually in it.
 *
 * Section 10.0: "When changing modes, the mode will not actually change until
 * all pending message transmissions are complete. The requested mode must be
 * verified by reading the OPMODE[2:0] bits". A single write-and-check would
 * therefore fail whenever a frame happens to be on the wire, which is exactly
 * when mcp2515_set_filter() is likely to be called. -EPROTO when the chip
 * is not there within MODE_WAIT_MS, on the clock.
 */
static int set_mode(mcp2515_handle_t d, uint8_t mode)
{
    int64_t start = can_time_us();

    TRY(bit_modify(d, MCP2515_CANCTRL, MCP2515_MODE_MASK, mode));

    for (;;) {
        uint8_t stat;

        TRY(reg_read(d, MCP2515_CANSTAT, &stat, 1));
        if ((stat & MCP2515_MODE_MASK) == mode)
            return 0;
        if (can_elapsed_ms(start) >= MODE_WAIT_MS)
            return -EPROTO;

        can_yield();
    }
}

/* Configuration mode for the duration of some writes, then back to where it
 * was -- running, or parked by can_stop(). */
static int leave_config(mcp2515_handle_t d)
{
    return d->stopped ? 0 : set_mode(d, d->mode);
}

/* Back whether or not the writes worked -- leaving the chip parked in
 * Configuration mode would take it off the bus. */
static int end_config(mcp2515_handle_t d, int err)
{
    int back = leave_config(d);

    return err ? err : back;
}

/* ---- the frame paths ---------------------------------------------------- */

static int send_frame(mcp2515_handle_t d, const mcp2515_frame_t *frame,
                      uint32_t timeout_ms)
{
    int64_t start = can_time_us();
    uint8_t id[4];

    if (frame->len > 8 ||
        frame->id > (frame->extended ? MCP2515_EID_MAX : MCP2515_SID_MAX))
        return -EINVAL;

    mcp2515_pack_id(frame->id, frame->extended, id);

    for (;;) {
        uint8_t status;
        int free_buf = -1;

        TRY(read_status(d, &status));

        for (int n = 0; n < 3 && free_buf < 0; n++)
            if ((status & MCP2515_ST_TXREQ(n)) == 0)
                free_buf = n;

        if (free_buf >= 0) {
            uint8_t tx[13];
            size_t n = 5;

            /* LOAD TX BUFFER, Figure 12-5: abc 000, 010 and 100 point at
             * TXB0SIDH, TXB1SIDH and TXB2SIDH. */
            memcpy(&tx[0], id, sizeof id);
            tx[4] = (uint8_t)((frame->remote ? MCP2515_DLC_RTR : 0) |
                              frame->len);

            /* A remote frame carries no data field on the wire; its DLC only
             * says how much the responder should send (Register 3-7). */
            if (!frame->remote) {
                memcpy(&tx[5], frame->data, frame->len);
                n += frame->len;
            }

            TRY(xfer(d, (uint8_t)(MCP2515_LOAD_TX | (free_buf * 2)), tx, n,
                     NULL, 0));

            /* RTS, Figure 12-6: the low three bits pick the buffers to send. */
            return xfer(d, (uint8_t)(MCP2515_RTS | (1u << free_buf)), NULL, 0,
                        NULL, 0);
        }

        if (can_elapsed_ms(start) >= timeout_ms)
            return -ETIMEDOUT;

        can_yield();
    }
}

static int receive_frame(mcp2515_handle_t d, mcp2515_frame_t *frame,
                         uint32_t timeout_ms)
{
    int64_t start = can_time_us();

    for (;;) {
        uint8_t status = 0;

        /* nINT is low while any enabled flag is set, so when it is wired a
         * high pin is a definite "nothing waiting" and the SPI poll can be
         * skipped entirely. */
        if (int_asserted(d))
            TRY(read_status(d, &status));

        if (status & (MCP2515_ST_RX0IF | MCP2515_ST_RX1IF)) {
            uint8_t rx[13];

            /* READ RX BUFFER, Figure 12-3: 0x90 starts at RXB0SIDH, 0x94 at
             * RXB1SIDH. Thirteen bytes follow -- SIDH, SIDL, EID8, EID0, DLC
             * and the eight data registers. RXB0 has the higher priority, so
             * it is drained first. The instruction clears RXnIF itself when
             * CS rises (Section 12.4), so there is nothing to acknowledge. */
            TRY(xfer(d, (status & MCP2515_ST_RX0IF) ?
                        MCP2515_READ_RX :
                        (uint8_t)(MCP2515_READ_RX | MCP2515_READ_RX1),
                     NULL, 0, rx, sizeof rx));

            mcp2515_unpack_id(&rx[0], &frame->id, &frame->extended);

            frame->len = rx[4] & MCP2515_DLC_LEN;
            /* DLC is four bits and the bus can carry 9 to 15 in a malformed
             * frame; the chip only ever stores eight data bytes. */
            if (frame->len > 8)
                frame->len = 8;

            /* Which bit means "remote" depends on the frame format: RXBnDLC
             * bit 6 for an extended frame (Register 4-8), RXBnSIDL bit 4 --
             * SRR -- for a standard one (Register 4-5). */
            frame->remote = frame->extended ? (rx[4] & MCP2515_DLC_RTR) != 0
                                            : (rx[1] & MCP2515_SIDL_SRR) != 0;

            memcpy(frame->data, &rx[5], frame->len);
            memset(frame->data + frame->len, 0,
                   sizeof frame->data - frame->len);

            return 0;
        }

        if (can_elapsed_ms(start) >= timeout_ms)
            return -ETIMEDOUT;

        can_yield();
    }
}

/* ---- filters ------------------------------------------------------------ */

static int accept_all(mcp2515_handle_t d)
{
    /* Everything through, the power-on state (Register 4-1, RXM = 11). */
    TRY(reg_write1(d, MCP2515_RXB0CTRL, MCP2515_RXM_ALL | MCP2515_RXB0_BUKT));
    return reg_write1(d, MCP2515_RXB1CTRL, MCP2515_RXM_ALL);
}

/*
 * f0 on RXB0 (RXF0, RXF1, RXM0), f1 on RXB1 (RXF2..RXF5, RXM1): one filter
 * pair feeds RXB0 and four feed RXB1, each buffer with its own mask
 * (Section 4.5). EXIDE in a filter applies it to extended frames only, or
 * standard frames only (Register 4-11). The chip is in Configuration mode.
 */
static int write_filters(mcp2515_handle_t d, const struct can_filter *f0,
                         const struct can_filter *f1)
{
    static const uint8_t filters[6] = MCP2515_RXF_ADDRS;
    uint8_t id[2][4], mask[2][4];

    mcp2515_pack_id(f0->id, f0->ext, id[0]);
    mcp2515_pack_id(f0->mask, f0->ext, mask[0]);
    mcp2515_pack_id(f1->id, f1->ext, id[1]);
    mcp2515_pack_id(f1->mask, f1->ext, mask[1]);

    for (unsigned i = 0; i < 6; i++)
        TRY(reg_write(d, filters[i], id[i < 2 ? 0 : 1], 4));
    TRY(reg_write(d, MCP2515_RXM0SIDH, mask[0], 4));
    TRY(reg_write(d, MCP2515_RXM1SIDH, mask[1], 4));
    TRY(reg_write1(d, MCP2515_RXB0CTRL,
                   MCP2515_RXM_FILTER | MCP2515_RXB0_BUKT));
    return reg_write1(d, MCP2515_RXB1CTRL, MCP2515_RXM_FILTER);
}

/* As esp-mcp2515-can: one filter for both buffers; mask 0 accepts
 * everything, standard and extended. */
static int set_filter(mcp2515_handle_t d, uint32_t id, uint32_t mask,
                      bool extended)
{
    struct can_filter f = { .id = id, .mask = mask, .ext = extended };

    if (id > (extended ? MCP2515_EID_MAX : MCP2515_SID_MAX))
        return -EINVAL;

    /* Section 4.5.5: "The Mask and Filter registers can only be modified when
     * the MCP2515 is in Configuration mode." */
    TRY(set_mode(d, MCP2515_MODE_CONFIG));
    return end_config(d, mask == 0 ? accept_all(d) : write_filters(d, &f, &f));
}

static int read_chip_status(mcp2515_handle_t d, mcp2515_status_t *out)
{
    uint8_t counters[2], eflg;

    /* TEC (0x1C) and REC (0x1D) are adjacent, so one read gets both. */
    TRY(reg_read(d, MCP2515_TEC, counters, sizeof counters));
    TRY(reg_read(d, MCP2515_EFLG, &eflg, 1));

    out->tx_errors = counters[0];
    out->rx_errors = counters[1];
    out->warning = (eflg & MCP2515_EFLG_EWARN) != 0;
    out->passive = (eflg & (MCP2515_EFLG_TXEP | MCP2515_EFLG_RXEP)) != 0;
    out->bus_off = (eflg & MCP2515_EFLG_TXBO) != 0;
    out->rx_overflow = (eflg & MCP2515_EFLG_OVR) != 0;
    return 0;
}

/* Register 6-3: RX1OVR and RX0OVR are the only writable bits in EFLG and
 * the datasheet says they "must be reset by MCU". The rest track the
 * error counters and clear themselves -- including bus-off, which resets
 * "after a successful bus recovery sequence" the chip runs on its own. */
static int clear_errors(mcp2515_handle_t d)
{
    return bit_modify(d, MCP2515_EFLG, MCP2515_EFLG_OVR, 0x00);
}

/* CNF3, CNF2 and CNF1 are consecutive and in that order (Table 11-1), so
 * the three go out in one write -- descending, hence the shuffle. Then read
 * back: wrong bit timing is the failure this driver can least afford and
 * the bus reports least clearly, so it is confirmed rather than assumed.
 * The chip is in Configuration mode (Section 5.7). */
static int write_timing(mcp2515_handle_t d, const uint8_t cnf[3])
{
    uint8_t written[3] = { cnf[2], cnf[1], cnf[0] }, back[3];

    TRY(reg_write(d, MCP2515_CNF3, written, sizeof written));
    TRY(reg_read(d, MCP2515_CNF3, back, sizeof back));
    if (memcmp(written, back, sizeof back) != 0) {
        can_err(&d->can, "bit timing did not stick: wrote %02x %02x %02x, "
                "read %02x %02x %02x", written[0], written[1], written[2],
                back[0], back[1], back[2]);
        return -ENODEV;
    }
    return 0;
}

/* ---- the can_dev ops ---------------------------------------------------- */

static mcp2515_handle_t of(struct can_dev *can)
{
    return can->priv;
}

static int op_send(struct can_dev *can, const struct can_frame *frame,
                   uint32_t timeout_ms)
{
    mcp2515_frame_t f = {
        .id = frame->id,
        .len = frame->dlc,
        .extended = frame->ext,
        .remote = frame->rtr,
    };

    memcpy(f.data, frame->data, sizeof f.data);
    return send_frame(of(can), &f, timeout_ms);
}

static int op_recv(struct can_dev *can, struct can_frame *frame,
                   uint32_t timeout_ms)
{
    mcp2515_frame_t f;

    TRY(receive_frame(of(can), &f, timeout_ms));
    frame->id = f.id;
    frame->dlc = f.len;
    frame->ext = f.extended;
    frame->rtr = f.remote;
    memcpy(frame->data, f.data, sizeof frame->data);
    return 0;
}

static int op_get_status(struct can_dev *can, struct can_status *status)
{
    mcp2515_handle_t d = of(can);
    mcp2515_status_t st;
    uint8_t flags;

    TRY(read_chip_status(d, &st));
    TRY(read_status(d, &flags));

    status->link = d->stopped ? CAN_LINK_STOPPED :
                   st.bus_off ? CAN_LINK_BUS_OFF :
                   st.passive ? CAN_LINK_PASSIVE :
                   st.warning ? CAN_LINK_WARNING : CAN_LINK_ACTIVE;
    status->tec = st.tx_errors;
    status->rec = st.rx_errors;
    /* The chip has a flag, not a count. */
    status->rx_lost = st.rx_overflow;
    status->rx_pending = !!(flags & MCP2515_ST_RX0IF) +
                         !!(flags & MCP2515_ST_RX1IF);
    for (int n = 0; n < 3; n++)
        status->tx_pending += !!(flags & MCP2515_ST_TXREQ(n));
    return 0;
}

static int op_set_filters(struct can_dev *can,
                          const struct can_filter *filters, size_t count)
{
    mcp2515_handle_t d = of(can);

    TRY(set_mode(d, MCP2515_MODE_CONFIG));
    if (count == 0)
        return end_config(d, accept_all(d));
    return end_config(d, write_filters(d, &filters[0],
                                       &filters[count > 1 ? 1 : 0]));
}

static int op_set_bitrate(struct can_dev *can, uint32_t bitrate)
{
    mcp2515_handle_t d = of(can);
    uint8_t cnf[3];

    if (!mcp2515_timing(d->crystal_hz, bitrate, cnf))
        return -EINVAL;
    TRY(set_mode(d, MCP2515_MODE_CONFIG));
    return end_config(d, write_timing(d, cnf));
}

static int op_set_opmode(struct can_dev *can, enum can_opmode mode)
{
    static const uint8_t modes[] = {
        [CAN_OPMODE_NORMAL] = MCP2515_MODE_NORMAL,
        [CAN_OPMODE_LOOPBACK] = MCP2515_MODE_LOOPBACK,
        [CAN_OPMODE_LISTEN] = MCP2515_MODE_LISTEN,
    };
    mcp2515_handle_t d = of(can);

    d->mode = modes[mode];
    return leave_config(d);
}

/* The chip leaves bus-off on its own after 128 x 11 recessive bits (DS
 * p.47); there is nothing to start. */
static int op_recover(struct can_dev *can)
{
    (void)can;
    return 0;
}

static int op_start(struct can_dev *can)
{
    mcp2515_handle_t d = of(can);

    d->stopped = false;
    return set_mode(d, d->mode);
}

static int op_stop(struct can_dev *can)
{
    mcp2515_handle_t d = of(can);

    TRY(set_mode(d, MCP2515_MODE_CONFIG));
    d->stopped = true;
    return 0;
}

static const struct can_ops mcp2515_ops = {
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

static void release(mcp2515_handle_t d)
{
    if (d->bus.ops)
        can_bus_detach(&d->bus);
    free(d);
}

/* Loopback and listen-only are opposite corners of the same field; asking
 * for both is a caller mistake, not something to silently pick between.
 * The crystal and bit rate must have a row in the table. */
static int check_config(uint32_t crystal_hz, uint32_t bitrate, bool loopback,
                        bool listen_only, uint8_t cnf[3])
{
    static const struct can_dev log = { .name = "mcp2515" };

    if (loopback && listen_only)
        return -EINVAL;
    if (!mcp2515_timing(crystal_hz, bitrate, cnf)) {
        can_err(&log, "no bit timing for a %u Hz crystal at %u bit/s; "
                "supported: 8 MHz with 125000, 250000 or 500000, 16 MHz "
                "with those or 1000000", (unsigned)crystal_hz,
                (unsigned)bitrate);
        return -EINVAL;
    }
    return 0;
}

/*
 * Put the chip in a known state and check it took. The handle's bus is
 * attached.
 */
static int setup(mcp2515_handle_t d, const uint8_t cnf[3], bool has_int)
{
    uint8_t stat;
    int err;

    /* Section 12.2 recommends RESET as part of power-on initialisation. It
     * leaves the chip in configuration mode, which is the only mode where
     * CNF1..3 and the filters can be written (Section 10.1). */
    TRY(xfer(d, MCP2515_RESET, NULL, 0, NULL, 0));
    can_sleep_ms(RESET_WAIT_MS);

    TRY(reg_read(d, MCP2515_CANSTAT, &stat, 1));

    /* Nothing else is worth doing if the chip is not there. A floating MISO
     * reads as all ones or all zeros, and neither is configuration mode. */
    if ((stat & MCP2515_MODE_MASK) != MCP2515_MODE_CONFIG) {
        can_err(&d->can, "no MCP2515: CANSTAT reads 0x%02x, expected "
                "configuration mode after reset", stat);
        return -ENODEV;
    }

    TRY(write_timing(d, cnf));

    /* Accept everything until a filter says otherwise, and let a full RXB0
     * roll over into RXB1 rather than drop the frame (Register 4-1). */
    TRY(accept_all(d));

    /*
     * Polled, so no interrupt is enabled -- unless the caller wired nINT.
     * The pin is only driven low for sources enabled in CANINTE (Section 7.0),
     * so leaving CANINTE at 0 with a wired pin would give a pin that never
     * asserts and a receive that waits for a signal that cannot arrive.
     * Enabling the two receive flags makes the pin mean what the receive
     * path reads it to mean. No ISR is installed either way; the pin is
     * only sampled.
     */
    TRY(reg_write1(d, MCP2515_CANINTE,
                   has_int ? (MCP2515_INT_RX0IF | MCP2515_INT_RX1IF) : 0));

    err = set_mode(d, d->mode);
    if (err) {
        can_err(&d->can, "chip would not leave configuration mode");
        return err;
    }

    d->can.ops = &mcp2515_ops;
    d->can.priv = d;
    return 0;
}

static mcp2515_handle_t alloc(uint32_t crystal_hz, bool loopback,
                              bool listen_only)
{
    mcp2515_handle_t d = calloc(1, sizeof *d);

    if (d == NULL)
        return NULL;
    d->can.name = "mcp2515";
    d->crystal_hz = crystal_hz;
    d->mode = loopback ? MCP2515_MODE_LOOPBACK :
              listen_only ? MCP2515_MODE_LISTEN : MCP2515_MODE_NORMAL;
#ifdef ESP_PLATFORM
    /* calloc leaves this 0, which is a real GPIO on every ESP32. */
    d->int_gpio = GPIO_NUM_NC;
#endif
    return d;
}

#ifdef ESP_PLATFORM
static int init(const mcp2515_config_t *cfg, mcp2515_handle_t *out)
{
    uint32_t crystal_hz, bitrate;
    mcp2515_handle_t d;
    uint8_t cnf[3];
    int err;

    if (cfg == NULL || out == NULL)
        return -EINVAL;
    crystal_hz = cfg->crystal_hz != 0 ? cfg->crystal_hz : DEFAULT_XTAL_HZ;
    bitrate = cfg->bitrate != 0 ? cfg->bitrate : DEFAULT_BITRATE;
    TRY(check_config(crystal_hz, bitrate, cfg->loopback, cfg->listen_only,
                     cnf));

    d = alloc(crystal_hz, cfg->loopback, cfg->listen_only);
    if (d == NULL)
        return -ENOMEM;

    /* Section 12.1: mode 0,0 or 1,1; this is 0,0. */
    err = can_spi_attach(&d->bus, cfg->host, cfg->cs_gpio,
                         cfg->spi_hz != 0 ? (uint32_t)cfg->spi_hz :
                                            DEFAULT_SPI_HZ, 0);
    if (!err && cfg->int_gpio != GPIO_NUM_NC) {
        gpio_config_t io_cfg = {
            .pin_bit_mask = 1ULL << (unsigned)cfg->int_gpio,
            .mode = GPIO_MODE_INPUT,
            /* The pull-up esp-mcp2515-can set: nINT is driven low while
             * a flag is pending (Section 7.0), and a breakout's own pull-up
             * makes this one harmless. */
            .pull_up_en = GPIO_PULLUP_ENABLE,
        };

        err = can_errno(gpio_config(&io_cfg));
        if (!err)
            d->int_gpio = cfg->int_gpio;
    }
    if (!err)
        err = setup(d, cnf, d->int_gpio != GPIO_NUM_NC);
    if (err) {
        release(d);
        return err;
    }
    *out = d;
    return 0;
}
#else
static int init(const mcp2515_config_t *cfg, mcp2515_handle_t *out)
{
    uint32_t crystal_hz, bitrate;
    mcp2515_handle_t d;
    uint8_t cnf[3];
    int err;

    if (cfg == NULL || out == NULL || cfg->spi == NULL ||
        cfg->cs_port == NULL)
        return -EINVAL;
    /* Mode 0,0 or 1,1 (Section 12.1), bytes. The handle is the app's, so
     * refuse a wrong one before the bus is touched. */
    if (cfg->spi->Init.DataSize != SPI_DATASIZE_8BIT ||
        !((cfg->spi->Init.CLKPolarity == SPI_POLARITY_LOW &&
           cfg->spi->Init.CLKPhase == SPI_PHASE_1EDGE) ||
          (cfg->spi->Init.CLKPolarity == SPI_POLARITY_HIGH &&
           cfg->spi->Init.CLKPhase == SPI_PHASE_2EDGE)))
        return -EINVAL;
    crystal_hz = cfg->crystal_hz != 0 ? cfg->crystal_hz : DEFAULT_XTAL_HZ;
    bitrate = cfg->bitrate != 0 ? cfg->bitrate : DEFAULT_BITRATE;
    TRY(check_config(crystal_hz, bitrate, cfg->loopback, cfg->listen_only,
                     cnf));

    d = alloc(crystal_hz, cfg->loopback, cfg->listen_only);
    if (d == NULL)
        return -ENOMEM;
    d->int_port = cfg->int_port;
    d->int_pin = cfg->int_pin;

    err = can_spi_attach(&d->bus, cfg->spi, cfg->cs_port, cfg->cs_pin,
                         TIMEOUT_MS);
    if (!err)
        err = setup(d, cnf, d->int_port != NULL);
    if (err) {
        release(d);
        return err;
    }
    *out = d;
    return 0;
}
#endif

/* ---- the handle API ----------------------------------------------------- */

can_handle_ret mcp2515_init(const mcp2515_config_t *cfg, mcp2515_handle_t *out)
{
    return CAN_HANDLE_RET(init(cfg, out));
}

can_handle_ret mcp2515_send(mcp2515_handle_t dev, const mcp2515_frame_t *frame,
                            uint32_t timeout_ms)
{
    if (dev == NULL || frame == NULL)
        return CAN_HANDLE_RET(-EINVAL);
    return CAN_HANDLE_RET(send_frame(dev, frame, timeout_ms));
}

can_handle_ret mcp2515_receive(mcp2515_handle_t dev, mcp2515_frame_t *frame,
                               uint32_t timeout_ms)
{
    if (dev == NULL || frame == NULL)
        return CAN_HANDLE_RET(-EINVAL);
    return CAN_HANDLE_RET(receive_frame(dev, frame, timeout_ms));
}

can_handle_ret mcp2515_set_filter(mcp2515_handle_t dev, uint32_t id,
                                  uint32_t mask, bool extended)
{
    if (dev == NULL)
        return CAN_HANDLE_RET(-EINVAL);
    return CAN_HANDLE_RET(set_filter(dev, id, mask, extended));
}

can_handle_ret mcp2515_read_status(mcp2515_handle_t dev, mcp2515_status_t *out)
{
    if (dev == NULL || out == NULL)
        return CAN_HANDLE_RET(-EINVAL);
    return CAN_HANDLE_RET(read_chip_status(dev, out));
}

can_handle_ret mcp2515_clear_errors(mcp2515_handle_t dev)
{
    if (dev == NULL)
        return CAN_HANDLE_RET(-EINVAL);
    return CAN_HANDLE_RET(clear_errors(dev));
}

can_handle_ret mcp2515_del(mcp2515_handle_t dev)
{
    if (dev == NULL)
        return CAN_HANDLE_RET(-EINVAL);
    release(dev);
    return CAN_HANDLE_RET(0);
}

struct can_dev *mcp2515_can(mcp2515_handle_t dev)
{
    return dev ? &dev->can : NULL;
}

#endif /* CONFIG_CAN_MCP2515 */
