#ifndef STM32_HAL_STUB_H
#define STM32_HAL_STUB_H

/*
 * Host stub for the slice of the STM32 HAL this library uses, standing in
 * for "stm32f4xx.h" through CMSIS_device_header (see test/Makefile). Names,
 * signatures and values match the real F4/F7 HAL; the handles carry only the
 * fields the drivers read. HAL_Delay() moves stub_tick; the tests define the
 * transfer functions their fake chips answer.
 */

#include <stdint.h>

typedef enum {
	HAL_OK		= 0x00,
	HAL_ERROR	= 0x01,
	HAL_BUSY	= 0x02,
	HAL_TIMEOUT	= 0x03,
} HAL_StatusTypeDef;

/* A port is a register block on silicon; here it only needs an identity. */
typedef struct {
	uint32_t id;
} GPIO_TypeDef;

typedef enum {
	GPIO_PIN_RESET = 0,
	GPIO_PIN_SET,
} GPIO_PinState;

#define GPIO_PIN_0		((uint16_t)0x0001)
#define GPIO_PIN_1		((uint16_t)0x0002)
#define GPIO_PIN_4		((uint16_t)0x0010)
#define GPIO_PIN_14		((uint16_t)0x4000)
#define GPIO_PIN_15		((uint16_t)0x8000)

typedef struct {
	uint32_t id;
} I2C_HandleTypeDef;

#define I2C_MEMADD_SIZE_8BIT	0x00000001U
#define I2C_MEMADD_SIZE_16BIT	0x00000010U

typedef struct {
	uint32_t DataSize;
	uint32_t CLKPolarity;
	uint32_t CLKPhase;
} SPI_InitTypeDef;

typedef struct {
	SPI_InitTypeDef Init;
} SPI_HandleTypeDef;

typedef struct {
	uint32_t id;
} UART_HandleTypeDef;

/* The F4 values. The drivers compare against the names, never the numbers. */
#define SPI_DATASIZE_8BIT	0x00000000U
#define SPI_DATASIZE_16BIT	0x00000800U
#define SPI_POLARITY_LOW	0x00000000U
#define SPI_POLARITY_HIGH	0x00000002U
#define SPI_PHASE_1EDGE		0x00000000U
#define SPI_PHASE_2EDGE		0x00000001U

/*
 * SysTick, as CMSIS lays it out. Every access through SysTick goes through
 * stub_systick(), which lets time pass first -- stub_systick_step core
 * cycles, VAL counting down and reloading from LOAD -- so a loop that reads
 * VAL sees it move, as on silicon. stub_cycles counts every cycle that
 * passed; SystemCoreClock is the core clock the delay divides by.
 */
typedef struct {
	uint32_t CTRL;
	uint32_t LOAD;
	uint32_t VAL;
	uint32_t CALIB;
} SysTick_Type;

#define SysTick_CTRL_ENABLE_Msk		(1UL << 0)
#define SysTick_CTRL_CLKSOURCE_Msk	(1UL << 2)

extern uint32_t SystemCoreClock;
extern uint32_t stub_systick_step;
extern uint64_t stub_cycles;
extern uint32_t stub_systick_phase;	/* cycles into a count at core / 8 */

SysTick_Type *stub_systick(void);
#define SysTick		(stub_systick())

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t Delay);

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
		       GPIO_PinState PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c,
					uint16_t DevAddress, uint32_t Trials,
					uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c,
				   uint16_t DevAddress, uint16_t MemAddress,
				   uint16_t MemAddSize, uint8_t *pData,
				   uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c,
				    uint16_t DevAddress, uint16_t MemAddress,
				    uint16_t MemAddSize, uint8_t *pData,
				    uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c,
					  uint16_t DevAddress, uint8_t *pData,
					  uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c,
					 uint16_t DevAddress, uint8_t *pData,
					 uint16_t Size, uint32_t Timeout);

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi,
				   const uint8_t *pData, uint16_t Size,
				   uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi,
					  const uint8_t *pTxData,
					  uint8_t *pRxData, uint16_t Size,
					  uint32_t Timeout);

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart,
				    const uint8_t *pData, uint16_t Size,
				    uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData,
				   uint16_t Size, uint32_t Timeout);

/*
 * The UART flag macros the port uses, as functions over a fake receive
 * register: RXNE is "a stale byte is waiting", and the flush reads one.
 */
#define UART_FLAG_RXNE				0x20U
#define __HAL_UART_GET_FLAG(__HANDLE__, __FLAG__) \
	stub_uart_get_flag((__HANDLE__), (__FLAG__))
#define __HAL_UART_CLEAR_OREFLAG(__HANDLE__)	stub_uart_clear_ore(__HANDLE__)
#define __HAL_UART_FLUSH_DRREGISTER(__HANDLE__)	stub_uart_flush(__HANDLE__)

int stub_uart_get_flag(UART_HandleTypeDef *huart, uint32_t flag);
void stub_uart_clear_ore(UART_HandleTypeDef *huart);
void stub_uart_flush(UART_HandleTypeDef *huart);

/*
 * The bxCAN slice: the HAL's CAN module (stm32f4xx_hal_can.h v1.8.5) and the
 * CMSIS register bits bxcan.c reads (stm32f405xx.h), with the F4 values.
 * The register block carries only RF0R and ESR; test/stm/test_bxcan.c
 * defines the HAL_CAN_* functions over a fake controller.
 */
typedef enum {
	DISABLE = 0U,
	ENABLE = !DISABLE,
} FunctionalState;

typedef struct {
	uint32_t RF0R;
	uint32_t ESR;
} CAN_TypeDef;

typedef enum {
	HAL_CAN_STATE_RESET		= 0x00U,
	HAL_CAN_STATE_READY		= 0x01U,
	HAL_CAN_STATE_LISTENING		= 0x02U,
	HAL_CAN_STATE_SLEEP_PENDING	= 0x03U,
	HAL_CAN_STATE_SLEEP_ACTIVE	= 0x04U,
	HAL_CAN_STATE_ERROR		= 0x05U,
} HAL_CAN_StateTypeDef;

typedef struct {
	CAN_TypeDef *Instance;
	HAL_CAN_StateTypeDef State;
	uint32_t ErrorCode;
} CAN_HandleTypeDef;

typedef struct {
	uint32_t StdId;
	uint32_t ExtId;
	uint32_t IDE;
	uint32_t RTR;
	uint32_t DLC;
	FunctionalState TransmitGlobalTime;
} CAN_TxHeaderTypeDef;

typedef struct {
	uint32_t StdId;
	uint32_t ExtId;
	uint32_t IDE;
	uint32_t RTR;
	uint32_t DLC;
	uint32_t Timestamp;
	uint32_t FilterMatchIndex;
} CAN_RxHeaderTypeDef;

#define CAN_ID_STD			(0x00000000U)
#define CAN_ID_EXT			(0x00000004U)
#define CAN_RTR_DATA			(0x00000000U)
#define CAN_RTR_REMOTE			(0x00000002U)
#define CAN_RX_FIFO0			(0x00000000U)
#define CAN_IT_RX_FIFO0_MSG_PENDING	(0x00000002U)	/* CAN_IER_FMPIE0 */

#define CAN_RF0R_FOVR0			(0x1UL << 4U)
#define CAN_ESR_EWGF			(0x1UL << 0U)
#define CAN_ESR_EPVF			(0x1UL << 1U)
#define CAN_ESR_BOFF			(0x1UL << 2U)
#define CAN_ESR_TEC_Pos			(16U)
#define CAN_ESR_TEC_Msk			(0xFFUL << CAN_ESR_TEC_Pos)
#define CAN_ESR_REC_Pos			(24U)
#define CAN_ESR_REC_Msk			(0xFFUL << CAN_ESR_REC_Pos)

HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan,
				       const CAN_TxHeaderTypeDef *pHeader,
				       const uint8_t aData[],
				       uint32_t *pTxMailbox);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan,
				       uint32_t RxFifo,
				       CAN_RxHeaderTypeDef *pHeader,
				       uint8_t aData[]);
uint32_t HAL_CAN_GetRxFifoFillLevel(const CAN_HandleTypeDef *hcan,
				    uint32_t RxFifo);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan,
					       uint32_t ActiveITs);
HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef *hcan,
						 uint32_t InactiveITs);
HAL_CAN_StateTypeDef HAL_CAN_GetState(const CAN_HandleTypeDef *hcan);

/* --- test hooks, defined in hal_stubs.c --------------------------------- */

/* What HAL_GetTick() returns, in ms; tests set it, HAL_Delay() moves it. */
extern uint32_t stub_tick;

/*
 * The last printf() line, without its newline. The tests link with
 * -Wl,--wrap=printf, so every tof_* log line lands here as well as on
 * stdout.
 */
extern char stub_log_last[256];

#endif
