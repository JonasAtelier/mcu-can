/*
 * fdcan -- the STM32's FDCAN controller as a struct can_dev, Classic CAN
 * and CAN FD, on the HAL's FDCAN module (stm32xxxx_hal_fdcan.h). Written
 * and checked against the STM32H5 HAL, v1.7.0; another family's FDCAN HAL
 * may differ. The header is fdcan_can.h, not fdcan.h: CubeMX generates an
 * fdcan.h of its own for MX_FDCANx_Init().
 *
 * STM32 HAL only, with HAL_FDCAN_MODULE_ENABLED. The app owns the
 * peripheral's setup: clock, pins and HAL_FDCAN_Init(), CubeMX-style, before
 * fdcan_init(). Its Init.FrameFormat decides what goes on the wire:
 * FDCAN_FRAME_CLASSIC sends Classic frames only, FDCAN_FRAME_FD_NO_BRS adds
 * FD frames, FDCAN_FRAME_FD_BRS adds bit-rate switching.
 *
 * Unlike bxCAN, the driver sets what the HAL lets it set: can_set_bitrate(),
 * can_set_data_bitrate(), can_set_opmode() and can_set_filters() rewrite
 * the handle's Init and run HAL_FDCAN_Init() again, which clears the
 * message RAM, so filters set through the HAL directly are gone after any
 * of them; use can_set_filters(). Nominal bit rates go up to 1 Mbit/s,
 * data bit rates from the nominal up to 8 Mbit/s (RM0481 60.3.4); above
 * 1 Mbit/s the data phase runs with transmitter delay compensation, its
 * offset at the data sample point (see src/fdcan/fdcan_math.h).
 *
 * Receive: Rx FIFO 0. With use_irq, fdcan_irq() empties it into a ring from
 * the app's HAL_FDCAN_RxFifo0Callback(), each frame stamped there; without,
 * can_recv() polls the FIFO, and a FIFO left full drops frames.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include CMSIS_device_header

#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct fdcan_t *fdcan_handle_t;

#ifdef HAL_FDCAN_MODULE_ENABLED
typedef struct {
	FDCAN_HandleTypeDef *hfdcan;	/* HAL_FDCAN_Init()ed by the app */
	bool use_irq;			/* the app calls fdcan_irq() on FIFO 0 */
} fdcan_config_t;

/*
 * Take the controller: unless the app has started it already, set the
 * global filter to take every frame into Rx FIFO 0 and start it
 * (HAL_FDCAN_Start()); with use_irq, turn on the FIFO 0 new-message
 * interrupt (HAL_FDCAN_ActivateNotification()). The NVIC line is the app's.
 * -EINVAL for a handle the HAL has not initialised.
 */
int fdcan_init(const fdcan_config_t *cfg, fdcan_handle_t *out);

/* Stop the controller and the interrupt, and free the handle. */
int fdcan_del(fdcan_handle_t dev);

/*
 * Every frame waiting in Rx FIFO 0 into the ring, stamped with
 * can_time_us(). Call it from HAL_FDCAN_RxFifo0Callback(). A full ring
 * drops the frame and counts it in can_status.rx_lost.
 */
void fdcan_irq(fdcan_handle_t dev);
#endif

/* The handle as a struct can_dev; NULL for a NULL handle. Valid until
 * fdcan_del(). */
struct can_dev *fdcan_can(fdcan_handle_t dev);

#ifdef __cplusplus
}
#endif
