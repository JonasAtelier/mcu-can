/*
 * Kernel dev_err()-style logging for can drivers and their users.
 *
 * ESP-IDF: everything goes out under the one ESP-IDF tag "can", prefixed
 * with the device's name, so one esp_log_level_set("can", ...) controls
 * every chip and a line still says which chip it came from. There is
 * deliberately no level machinery of our own; can_dbg() is also subject
 * to CONFIG_LOG_MAXIMUM_LEVEL, and CONFIG_CAN_DEBUG is not used.
 *
 * STM32 HAL: every line goes out through printf() as "can: <name>: ...",
 * so a line still says which chip it came from. Where printf() ends up is
 * the app's business: with newlib, retarget _write() to a UART. There is
 * deliberately no level machinery; can_dbg() alone is compiled out
 * unless CONFIG_CAN_DEBUG is 1 (stm-forge: CAN_DEBUG = y).
 */
#ifndef CAN_LOG_H
#define CAN_LOG_H

#ifdef ESP_PLATFORM
#include "esp_log.h"
#else
#include <stdio.h>
#endif

#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef ESP_PLATFORM
#define CAN_LOG_TAG	"can"

#define can_err(can, fmt, ...) \
	ESP_LOGE(CAN_LOG_TAG, "%s: " fmt, (can)->name, ##__VA_ARGS__)
#define can_warn(can, fmt, ...) \
	ESP_LOGW(CAN_LOG_TAG, "%s: " fmt, (can)->name, ##__VA_ARGS__)
#define can_info(can, fmt, ...) \
	ESP_LOGI(CAN_LOG_TAG, "%s: " fmt, (can)->name, ##__VA_ARGS__)
#define can_dbg(can, fmt, ...) \
	ESP_LOGD(CAN_LOG_TAG, "%s: " fmt, (can)->name, ##__VA_ARGS__)

#else /* STM32 HAL */
#define can_err(can, fmt, ...) \
	printf("can: %s: " fmt "\n", (can)->name, ##__VA_ARGS__)
#define can_warn(can, fmt, ...) \
	printf("can: %s: " fmt "\n", (can)->name, ##__VA_ARGS__)
#define can_info(can, fmt, ...) \
	printf("can: %s: " fmt "\n", (can)->name, ##__VA_ARGS__)

#if CONFIG_CAN_DEBUG
#define can_dbg(can, fmt, ...) \
	printf("can: %s: " fmt "\n", (can)->name, ##__VA_ARGS__)
#else
/* Still type-checked, so a debug line cannot rot while it is off. */
#define can_dbg(can, fmt, ...) do { \
	if (0) \
		printf("can: %s: " fmt "\n", (can)->name, \
		       ##__VA_ARGS__); \
} while (0)
#endif
#endif /* ESP_PLATFORM */

/* One info line: time, identifier, format, length and data. */
void can_log_frame(const struct can_dev *can, const struct can_frame *frame);

#ifdef __cplusplus
}
#endif

#endif /* CAN_LOG_H */
