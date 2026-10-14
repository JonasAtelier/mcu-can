/*
 * FDCAN on an STM32H563, as the Nexus-S board wires it (hardware/nexus-s,
 * GPIO.md): FDCAN1 on PD0/PD1 is the CAN FD bus, FDCAN2 on PB5/PB6 the
 * Classic 1 Mbit/s bus the ESP32 TWAI nodes share. Each TCAN1044V's STB pin
 * (PD3, PB7) idles high through the transceiver's pull-up, which is
 * standby: drive it low to talk.
 *
 * A snippet, not a project: the clock tree, the printf() retarget, the pins
 * and both peripherals are the app's -- here hfdcan1 and hfdcan2 as CubeMX
 * generates them (MX_FDCAN1_Init(), MX_FDCAN2_Init()), FDCAN1 with
 * FrameFormat FDCAN_FRAME_FD_BRS and FDCAN2 with FDCAN_FRAME_CLASSIC, and
 * FDCAN1_IT0 / FDCAN2_IT0 enabled in the NVIC with HAL_FDCAN_IRQHandler()
 * in their handlers. Needs HAL_FDCAN_MODULE_ENABLED,
 * stm32h5xx_hal_fdcan.c in the build, and CAN_FDCAN = y.
 *
 * FDCAN2 must stay Classic: an FD frame on the ESP bus is an error frame to
 * every TWAI node there (hardware/nexus-s/README.md). The data phase on
 * FDCAN1 is 2 Mbit/s, so the driver turns on transmitter delay
 * compensation. The FDCAN clock has to divide both rates exactly: an 8 MHz
 * HSE makes 1 Mbit/s in 8 quanta and 2 Mbit/s in 4.
 */
#include <stdio.h>

#include CMSIS_device_header

#include "can.h"
#include "can_log.h"
#include "fdcan_can.h"

extern FDCAN_HandleTypeDef hfdcan1;	/* from MX_FDCAN1_Init() */
extern FDCAN_HandleTypeDef hfdcan2;	/* from MX_FDCAN2_Init() */

static fdcan_handle_t fd_bus, esp_bus;

static void wake_transceivers(void)
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);	/* CAN1_STB */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);	/* CAN2_STB */
}

void can_example_init(void)
{
	fdcan_config_t cfg = { .hfdcan = &hfdcan1, .use_irq = true };
	int err;

	wake_transceivers();
	err = fdcan_init(&cfg, &fd_bus);
	if (!err)
		err = can_set_bitrate(fdcan_can(fd_bus), 1000000);
	if (!err)
		err = can_set_data_bitrate(fdcan_can(fd_bus), 2000000);
	if (err)
		printf("fdcan1: %d\n", err);

	cfg.hfdcan = &hfdcan2;
	err = fdcan_init(&cfg, &esp_bus);
	if (!err)
		err = can_set_bitrate(fdcan_can(esp_bus), 1000000);
	if (err)
		printf("fdcan2: %d\n", err);
}

/* The HAL calls it from HAL_FDCAN_IRQHandler(). */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t its)
{
	(void)its;
	if (hfdcan == &hfdcan1)
		fdcan_irq(fd_bus);
	else if (hfdcan == &hfdcan2)
		fdcan_irq(esp_bus);
}

/* From the main loop, once a second. */
void can_example_poll(void)
{
	static uint32_t counter;
	struct canfd_frame fd = { .id = 0x300, .len = 64, .fd = true,
				  .brs = true };
	struct can_frame classic = { .id = 0x200, .dlc = 4 };
	struct canfd_frame rx_fd;
	struct can_frame rx;
	struct can_status st;

	if (!fd_bus || !esp_bus)
		return;
	for (int i = 0; i < 64; i++)
		fd.data[i] = (uint8_t)(counter + (uint32_t)i);
	for (int i = 0; i < 4; i++)
		classic.data[i] = (uint8_t)(counter >> (24 - 8 * i));
	if (can_send_fd(fdcan_can(fd_bus), &fd, 10) == 0 &&
	    can_send(fdcan_can(esp_bus), &classic, 10) == 0)
		counter++;

	while (can_recv_fd(fdcan_can(fd_bus), &rx_fd, 0) == 0)
		printf("fdcan1: %03lx [%u]%s%s\n", (unsigned long)rx_fd.id,
		       rx_fd.len, rx_fd.fd ? " FD" : "",
		       rx_fd.brs ? " BRS" : "");
	while (can_recv(fdcan_can(esp_bus), &rx, 0) == 0)
		can_log_frame(fdcan_can(esp_bus), &rx);

	if (can_get_status(fdcan_can(fd_bus), &st) == 0 &&
	    st.link == CAN_LINK_BUS_OFF)
		can_recover(fdcan_can(fd_bus));
}
