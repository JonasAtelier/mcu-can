/*
 * bxcan -- the STM32's on-chip CAN controller (bxCAN: F1, F2, F4, F7, L4)
 * as a struct can_dev, on the HAL's CAN module (stm32xxxx_hal_can.h).
 *
 * STM32 HAL only, with HAL_CAN_MODULE_ENABLED. The app owns the peripheral:
 * it runs HAL_CAN_Init() -- bit timing, operating mode, automatic bus-off
 * management -- and HAL_CAN_ConfigFilter(), CubeMX-style, before
 * bxcan_init(). The bit layout of the timing and filter registers is in
 * ST's reference manual, which this library does not have, so it does not
 * write them: can_set_bitrate(), can_set_filters(), can_set_opmode() and
 * can_recover() are -EOPNOTSUPP here. With AutoBusOff enabled in the app's
 * Init, the controller leaves bus-off on its own.
 *
 * Receive: FIFO 0. With use_irq, bxcan_irq() empties it into a ring from
 * the app's HAL_CAN_RxFifo0MsgPendingCallback(), each frame stamped there;
 * without, can_recv() polls the FIFO, and a FIFO left full drops frames.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include CMSIS_device_header

#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct bxcan_t *bxcan_handle_t;

#ifdef HAL_CAN_MODULE_ENABLED
typedef struct {
    CAN_HandleTypeDef *hcan;    /* HAL_CAN_Init()ed, filters configured */
    bool               use_irq; /* the app calls bxcan_irq() on FIFO 0 */
} bxcan_config_t;

/*
 * Start the controller (HAL_CAN_Start()) unless the app already has, and
 * with use_irq turn on the FIFO 0 message-pending interrupt
 * (HAL_CAN_ActivateNotification()); the NVIC line is the app's.
 */
int bxcan_init(const bxcan_config_t *cfg, bxcan_handle_t *out);

/* Stop the controller and the interrupt, and free the handle. */
int bxcan_del(bxcan_handle_t dev);

/*
 * Every frame waiting in FIFO 0 into the ring, stamped with can_time_us().
 * Call it from HAL_CAN_RxFifo0MsgPendingCallback(). A full ring drops the
 * frame and counts it in can_status.rx_lost.
 */
void bxcan_irq(bxcan_handle_t dev);
#endif

/* The handle as a struct can_dev; NULL for a NULL handle. Valid until
 * bxcan_del(). */
struct can_dev *bxcan_can(bxcan_handle_t dev);

#ifdef __cplusplus
}
#endif
