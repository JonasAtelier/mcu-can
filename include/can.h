/*
 * can -- a small Linux-IIO-style Classic CAN subsystem for ESP-IDF and the
 * STM32 HAL.
 *
 * One struct can_dev per controller. It holds what every CAN 2.0B
 * controller shares: frames out and in, acceptance filters, the operating
 * mode, the error counters and the bus state. Code above this header sees
 * one shape of frame whichever controller carried it.
 *
 * Each chip keeps its own handle API; the handle holds a struct can_dev,
 * and <chip>_can() hands it out. The handle's init fills it and its del
 * ends it: there is no probe or remove of its own.
 *
 * Every int-returning function returns 0 or a negative errno. An optional
 * op a chip lacks is -EOPNOTSUPP.
 *
 * CAN FD rides alongside, not instead: struct can_frame stays Classic CAN's,
 * and a controller that can do FD adds the optional send_fd/recv_fd ops for
 * struct canfd_frame. can_send_fd() and can_recv_fd() work on every
 * controller -- on a Classic one they carry Classic frames and refuse FD
 * ones -- so an app can use the FD calls throughout.
 *
 * Names: everything here is can_* / canfd_* / CAN_FRAME_* / CANFD_FRAME_* /
 * CAN_FILTER_* / CAN_LINK_* / CAN_OPMODE_*, none of which esp-sn65hvd230-
 * can's hvd230can.h or the STM32 HAL's stm32xxxx_hal_can.h and
 * stm32xxxx_hal_fdcan.h uses, so all of those headers can meet in one
 * file. struct canfd_frame shares its name with Linux's <linux/can.h>:
 * do not include both in one translation unit.
 */
#ifndef CAN_H
#define CAN_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_FRAME_DATA_MAX	8		/* Classic CAN, no CAN FD */
#define CAN_FRAME_STD_ID_MAX	0x7FFu		/* 11-bit identifier */
#define CAN_FRAME_EXT_ID_MAX	0x1FFFFFFFu	/* 29-bit identifier */

/* CAN FD: up to 64 bytes, in the lengths canfd_dlc_to_len() lists. */
#define CANFD_FRAME_DATA_MAX	64

/* The most filters can_set_filters() takes, of any chip here. */
#define CAN_FILTER_MAX		2

struct can_frame {
	uint32_t id;		/* up to CAN_FRAME_STD_ID_MAX or _EXT_ID_MAX */
	uint8_t dlc;		/* 0..8; on a remote frame, the length asked */
	bool ext;		/* 29-bit identifier */
	bool rtr;		/* remote frame: a request, no data */
	uint8_t data[CAN_FRAME_DATA_MAX];
	int64_t t_us;		/* received: can_time_us() then; ignored on send */
};

/*
 * A CAN FD frame, or a Classic one carried by the FD calls (fd false).
 * len is the data length in bytes, not the DLC code: 0..8 for Classic,
 * and for FD one of 0..8, 12, 16, 20, 24, 32, 48, 64.
 */
struct canfd_frame {
	uint32_t id;		/* up to CAN_FRAME_STD_ID_MAX or _EXT_ID_MAX */
	uint8_t len;
	bool ext;		/* 29-bit identifier */
	bool rtr;		/* remote frame: Classic only */
	bool fd;		/* FD format; false: a Classic frame */
	bool brs;		/* FD: the data phase at the data bit rate */
	bool esi;		/* FD, received: the sender was error passive;
				 * ignored on send */
	uint8_t data[CANFD_FRAME_DATA_MAX];
	int64_t t_us;		/* received: can_time_us() then; ignored on send */
};

/*
 * The data length of a 4-bit DLC code, and back. ISO 11898-1 FD lengths,
 * as the STM32 HAL's own table has them (stm32h5xx_hal_fdcan.c:246,
 * DLCtoBytes; FDCAN_DLC_BYTES_12..64 at stm32h5xx_hal_fdcan.h:619-625).
 * canfd_len_to_dlc() is -1 for a length no DLC carries.
 */
static inline uint8_t canfd_dlc_to_len(uint8_t dlc)
{
	static const uint8_t len[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8,
					 12, 16, 20, 24, 32, 48, 64 };

	return len[dlc & 0x0F];
}

static inline int canfd_len_to_dlc(uint8_t len)
{
	for (int dlc = 0; dlc < 16; dlc++)
		if (canfd_dlc_to_len((uint8_t)dlc) == len)
			return dlc;
	return -1;
}

/*
 * A frame passes when ext matches and (frame id & mask) == (id & mask): mask
 * 0 passes every frame of that format.
 */
struct can_filter {
	uint32_t id;
	uint32_t mask;
	bool ext;
};

/* Where the controller stands on the bus (ISO 11898-1 error states). */
enum can_link {
	CAN_LINK_STOPPED,	/* off the bus by request */
	CAN_LINK_ACTIVE,	/* both counters under the warning limit */
	CAN_LINK_WARNING,	/* a counter at the warning limit, 96 */
	CAN_LINK_PASSIVE,	/* a counter at 128 or more */
	CAN_LINK_BUS_OFF,	/* TEC past 255; no longer on the bus */
	CAN_LINK_RECOVERING,	/* waiting out the bus-off recovery */
};

enum can_opmode {
	CAN_OPMODE_NORMAL,
	CAN_OPMODE_LOOPBACK,	/* frames sent come back; no ACK needed */
	CAN_OPMODE_LISTEN,	/* never ACKs, never sends */
};

/*
 * Counters a controller does not keep read 0; see the chip's docs for which
 * it has.
 */
struct can_status {
	enum can_link link;
	uint16_t tec;		/* transmit error counter */
	uint16_t rec;		/* receive error counter */
	uint32_t rx_lost;	/* frames lost to a full queue or FIFO */
	uint32_t tx_failed;
	uint32_t arb_lost;
	uint32_t bus_errors;
	uint32_t rx_pending;	/* frames waiting to be read */
	uint32_t tx_pending;	/* frames waiting to go out */
	uint32_t rx_fd_dropped;	/* FD frames can_recv() passed over: it
				 * returns Classic frames only */
};

struct can_dev;

struct can_ops {
	/* Queue one frame, waiting at most timeout_ms for room; -ETIMEDOUT
	 * otherwise. The frame is checked already. */
	int (*send)(struct can_dev *can, const struct can_frame *frame,
		    uint32_t timeout_ms);
	/* The next frame, waiting at most timeout_ms; -ETIMEDOUT otherwise.
	 * t_us may be left 0 for the core to stamp. */
	int (*recv)(struct can_dev *can, struct can_frame *frame,
		    uint32_t timeout_ms);
	int (*get_status)(struct can_dev *can, struct can_status *status);
	/* optional: count is 0..CAN_FILTER_MAX, checked; 0 accepts all. More
	 * than the chip has is -ENOSPC. */
	int (*set_filters)(struct can_dev *can,
			   const struct can_filter *filters, size_t count);
	/* optional */
	int (*set_bitrate)(struct can_dev *can, uint32_t bitrate);
	/* optional */
	int (*set_opmode)(struct can_dev *can, enum can_opmode mode);
	/* optional: start leaving bus-off */
	int (*recover)(struct can_dev *can);
	/* optional: onto the bus and off it */
	int (*start)(struct can_dev *can);
	int (*stop)(struct can_dev *can);
	/* optional, CAN FD: as send and recv, for struct canfd_frame. The
	 * frame is checked already; an FD frame or BRS the controller is not
	 * set up for is -EOPNOTSUPP. recv_fd returns Classic frames too. */
	int (*send_fd)(struct can_dev *can, const struct canfd_frame *frame,
		       uint32_t timeout_ms);
	int (*recv_fd)(struct can_dev *can, struct canfd_frame *frame,
		       uint32_t timeout_ms);
	/* optional, CAN FD: the bit rate of the data phase of a BRS frame */
	int (*set_data_bitrate)(struct can_dev *can, uint32_t bitrate);
};

struct can_dev {
	const char *name;		/* "mcp2515"; log prefix */
	const struct can_ops *ops;
	void *priv;			/* the chip's handle */

	/* The filters can_set_filters() last set; see can_recv(). */
	struct can_filter filters[CAN_FILTER_MAX];
	uint8_t filter_count;

	/* can_irq() and can_irq_pending(); see below */
	volatile uint32_t irq_count;
	uint32_t irq_seen;
};

/* -EINVAL for a dlc over 8, or an id past its format's range. */
int can_send(struct can_dev *can, const struct can_frame *frame,
	     uint32_t timeout_ms);

/*
 * The next frame that passes the filters set through can_set_filters(),
 * stamped with can_time_us() when the chip did not stamp it. A chip's
 * hardware filter may let more through than asked (the TWAI's does not
 * tell standard from extended frames); those are dropped here, and count
 * against timeout_ms. 0 polls once. Classic frames only: an FD controller
 * passes FD frames over and counts them in can_status.rx_fd_dropped.
 */
int can_recv(struct can_dev *can, struct can_frame *frame,
	     uint32_t timeout_ms);

/*
 * Accept only frames that pass at least one of count filters; count 0
 * accepts every frame. -EINVAL for an id or mask past its format's range,
 * -ENOSPC for more filters than the chip has.
 */
int can_set_filters(struct can_dev *can, const struct can_filter *filters,
		    size_t count);

/* -EINVAL for 0; a rate the chip cannot make is -EINVAL from the chip. */
int can_set_bitrate(struct can_dev *can, uint32_t bitrate);

/*
 * Send a CAN FD or a Classic frame. -EINVAL for an id past its format's
 * range; for FD, a len that is not a DLC length or a remote frame; for
 * Classic, a len over 8 or brs. On a controller without send_fd a Classic
 * frame goes through can_send() and an FD frame is -EOPNOTSUPP.
 */
int can_send_fd(struct can_dev *can, const struct canfd_frame *frame,
		uint32_t timeout_ms);

/*
 * The next frame, FD or Classic, that passes the filters; as can_recv()
 * otherwise. On a controller without recv_fd: can_recv()'s frame, fd false.
 */
int can_recv_fd(struct can_dev *can, struct canfd_frame *frame,
		uint32_t timeout_ms);

/* The data-phase bit rate. -EINVAL for 0; -EOPNOTSUPP without FD. */
int can_set_data_bitrate(struct can_dev *can, uint32_t bitrate);

int can_set_opmode(struct can_dev *can, enum can_opmode mode);
int can_get_status(struct can_dev *can, struct can_status *status);
int can_recover(struct can_dev *can);
int can_start(struct can_dev *can);
int can_stop(struct can_dev *can);

/* Does frame pass filters? count 0 passes everything. */
bool can_filter_pass(const struct can_filter *filters, size_t count,
		     const struct can_frame *frame);

/*
 * The INT line, on either platform. Call can_irq() from the app's GPIO ISR
 * or EXTI callback: it only counts, and never touches the bus. In thread
 * context, can_irq_pending() is true once for every batch of counts since
 * it last looked; then call can_recv() there. One caller of
 * can_irq_pending() per device.
 *
 * On ESP-IDF can_irq() is in IRAM and touches only the struct can_dev, in
 * RAM, so it is safe in an ISR registered with ESP_INTR_FLAG_IRAM.
 */
void can_irq(struct can_dev *can);
bool can_irq_pending(struct can_dev *can);

/*
 * The clock frames are stamped with, in microseconds. On ESP-IDF it is
 * esp_timer_get_time(). On the STM32 HAL the default is weak and only
 * HAL_GetTick() * 1000: millisecond steps that wrap after 49.7 days. Define
 * your own can_time_us() for real microseconds.
 */
int64_t can_time_us(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_H */
