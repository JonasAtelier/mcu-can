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
 * The register block carries only the registers the driver touches, in no
 * particular order; test/stm/test_bxcan.c defines the HAL_CAN_* functions
 * over a fake controller. CAN1 holds the filter banks of both, as on an F4
 * with two CANs.
 */
typedef enum {
	DISABLE = 0U,
	ENABLE = !DISABLE,
} FunctionalState;

typedef struct {
	uint32_t FR1;
	uint32_t FR2;
} CAN_FilterRegister_TypeDef;

typedef struct {
	uint32_t MCR;
	uint32_t MSR;
	uint32_t RF0R;
	uint32_t ESR;
	uint32_t BTR;
	uint32_t FMR;
	uint32_t FM1R;
	uint32_t FS1R;
	uint32_t FFA1R;
	uint32_t FA1R;
	CAN_FilterRegister_TypeDef sFilterRegister[28];
} CAN_TypeDef;

extern CAN_TypeDef stub_can1, stub_can2;
#define CAN1				(&stub_can1)
#define CAN2				(&stub_can2)

typedef struct {
	uint32_t Prescaler;
	uint32_t Mode;
	uint32_t SyncJumpWidth;
	uint32_t TimeSeg1;
	uint32_t TimeSeg2;
} CAN_InitTypeDef;

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
	CAN_InitTypeDef Init;
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
#define CAN_TX_MAILBOX0			(0x00000001U)
#define CAN_TX_MAILBOX1			(0x00000002U)
#define CAN_TX_MAILBOX2			(0x00000004U)

#define CAN_RF0R_FOVR0			(0x1UL << 4U)
#define CAN_ESR_EWGF			(0x1UL << 0U)
#define CAN_ESR_EPVF			(0x1UL << 1U)
#define CAN_ESR_BOFF			(0x1UL << 2U)
#define CAN_ESR_TEC_Pos			(16U)
#define CAN_ESR_TEC_Msk			(0xFFUL << CAN_ESR_TEC_Pos)
#define CAN_ESR_REC_Pos			(24U)
#define CAN_ESR_REC_Msk			(0xFFUL << CAN_ESR_REC_Pos)
#define CAN_MCR_INRQ			(0x1UL << 0U)
#define CAN_MCR_ABOM			(0x1UL << 6U)
#define CAN_MSR_INAK			(0x1UL << 0U)
#define CAN_BTR_TS1_Pos			(16U)
#define CAN_BTR_TS2_Pos			(20U)
#define CAN_BTR_SJW_Pos			(24U)
#define CAN_BTR_LBKM			(0x1UL << 30U)
#define CAN_BTR_SILM			(0x1UL << 31U)
#define CAN_FMR_FINIT			(0x1UL << 0U)
#define CAN_FMR_CAN2SB_Pos		(8U)
#define CAN_FMR_CAN2SB			(0x3FUL << CAN_FMR_CAN2SB_Pos)

#define CAN_MODE_NORMAL			(0x00000000U)
#define CAN_MODE_LOOPBACK		((uint32_t)CAN_BTR_LBKM)
#define CAN_MODE_SILENT			((uint32_t)CAN_BTR_SILM)
#define CAN_MODE_SILENT_LOOPBACK	((uint32_t)(CAN_BTR_LBKM | CAN_BTR_SILM))

HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *hcan,
					 uint32_t TxMailboxes);
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
uint32_t HAL_RCC_GetPCLK1Freq(void);

/*
 * The FDCAN slice: the HAL's FDCAN module (stm32h5xx_hal_fdcan.h, STM32H5
 * HAL v1.7.0) and the CMSIS register bits fdcan.c reads (stm32h563xx.h,
 * cmsis_device_h5 v1.7.0), with the H5 values. Only the fields the driver
 * touches; test/stm/test_fdcan.c defines the HAL_FDCAN_* functions and
 * HAL_RCCEx_GetPeriphCLKFreq() over a fake controller.
 */
typedef struct {
	uint32_t DBTP;
	uint32_t CCCR;
	uint32_t TDCR;
	uint32_t IR;
} FDCAN_GlobalTypeDef;

typedef struct {
	uint32_t CKDIV;
} FDCAN_Config_TypeDef;

/* The one register block every FDCAN instance shares (FDCAN_CONFIG). */
extern FDCAN_Config_TypeDef stub_fdcan_config;
#define FDCAN_CONFIG			(&stub_fdcan_config)

#define FDCAN_CCCR_INIT			(0x1UL << 0U)
#define FDCAN_CKDIV_PDIV		(0xFUL << 0U)
#define FDCAN_IR_RF0L			(0x1UL << 2U)
#define FDCAN_IE_RF0NE			(0x1UL << 0U)
#define FDCAN_DBTP_TDC			(0x1UL << 23U)
#define FDCAN_TDCR_TDCF_Pos		(0U)
#define FDCAN_TDCR_TDCO_Pos		(8U)

typedef enum {
	HAL_FDCAN_STATE_RESET	= 0x00U,
	HAL_FDCAN_STATE_READY	= 0x01U,
	HAL_FDCAN_STATE_BUSY	= 0x02U,
	HAL_FDCAN_STATE_ERROR	= 0x03U,
} HAL_FDCAN_StateTypeDef;

typedef struct {
	uint32_t ClockDivider;
	uint32_t FrameFormat;
	uint32_t Mode;
	FunctionalState AutoRetransmission;
	FunctionalState TransmitPause;
	FunctionalState ProtocolException;
	uint32_t NominalPrescaler;
	uint32_t NominalSyncJumpWidth;
	uint32_t NominalTimeSeg1;
	uint32_t NominalTimeSeg2;
	uint32_t DataPrescaler;
	uint32_t DataSyncJumpWidth;
	uint32_t DataTimeSeg1;
	uint32_t DataTimeSeg2;
	uint32_t StdFiltersNbr;
	uint32_t ExtFiltersNbr;
	uint32_t TxFifoQueueMode;
} FDCAN_InitTypeDef;

typedef struct {
	FDCAN_GlobalTypeDef *Instance;
	FDCAN_InitTypeDef Init;
	HAL_FDCAN_StateTypeDef State;
	uint32_t ErrorCode;
} FDCAN_HandleTypeDef;

typedef struct {
	uint32_t IdType;
	uint32_t FilterIndex;
	uint32_t FilterType;
	uint32_t FilterConfig;
	uint32_t FilterID1;
	uint32_t FilterID2;
} FDCAN_FilterTypeDef;

typedef struct {
	uint32_t Identifier;
	uint32_t IdType;
	uint32_t TxFrameType;
	uint32_t DataLength;
	uint32_t ErrorStateIndicator;
	uint32_t BitRateSwitch;
	uint32_t FDFormat;
	uint32_t TxEventFifoControl;
	uint32_t MessageMarker;
} FDCAN_TxHeaderTypeDef;

typedef struct {
	uint32_t Identifier;
	uint32_t IdType;
	uint32_t RxFrameType;
	uint32_t DataLength;
	uint32_t ErrorStateIndicator;
	uint32_t BitRateSwitch;
	uint32_t FDFormat;
	uint32_t RxTimestamp;
	uint32_t FilterIndex;
	uint32_t IsFilterMatchingFrame;
} FDCAN_RxHeaderTypeDef;

typedef struct {
	uint32_t LastErrorCode;
	uint32_t DataLastErrorCode;
	uint32_t Activity;
	uint32_t ErrorPassive;
	uint32_t Warning;
	uint32_t BusOff;
	uint32_t RxESIflag;
	uint32_t RxBRSflag;
	uint32_t RxFDFflag;
	uint32_t ProtocolException;
	uint32_t TDCvalue;
} FDCAN_ProtocolStatusTypeDef;

typedef struct {
	uint32_t TxErrorCnt;
	uint32_t RxErrorCnt;
	uint32_t RxErrorPassive;
	uint32_t ErrorLogging;
} FDCAN_ErrorCountersTypeDef;

#define FDCAN_FRAME_CLASSIC		(0x00000000U)
#define FDCAN_FRAME_FD_NO_BRS		(0x00000100U)	/* FDCAN_CCCR_FDOE */
#define FDCAN_FRAME_FD_BRS		(0x00000300U)	/* | FDCAN_CCCR_BRSE */
#define FDCAN_MODE_NORMAL		(0x00000000U)
#define FDCAN_MODE_RESTRICTED_OPERATION	(0x00000001U)
#define FDCAN_MODE_BUS_MONITORING	(0x00000002U)
#define FDCAN_MODE_INTERNAL_LOOPBACK	(0x00000003U)
#define FDCAN_MODE_EXTERNAL_LOOPBACK	(0x00000004U)
#define FDCAN_STANDARD_ID		(0x00000000U)
#define FDCAN_EXTENDED_ID		(0x40000000U)
#define FDCAN_DATA_FRAME		(0x00000000U)
#define FDCAN_REMOTE_FRAME		(0x20000000U)
#define FDCAN_ESI_ACTIVE		(0x00000000U)
#define FDCAN_ESI_PASSIVE		(0x80000000U)
#define FDCAN_BRS_OFF			(0x00000000U)
#define FDCAN_BRS_ON			(0x00100000U)
#define FDCAN_CLASSIC_CAN		(0x00000000U)
#define FDCAN_FD_CAN			(0x00200000U)
#define FDCAN_NO_TX_EVENTS		(0x00000000U)
#define FDCAN_FILTER_MASK		(0x00000002U)
#define FDCAN_FILTER_TO_RXFIFO0		(0x00000001U)
#define FDCAN_RX_FIFO0			(0x00000040U)
#define FDCAN_ACCEPT_IN_RX_FIFO0	(0x00000000U)
#define FDCAN_REJECT			(0x00000002U)
#define FDCAN_FILTER_REMOTE		(0x00000000U)
#define FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST	FDCAN_IR_RF0L
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE	FDCAN_IE_RF0NE

#define __HAL_FDCAN_GET_FLAG(__HANDLE__, __FLAG__) \
	((__HANDLE__)->Instance->IR & (__FLAG__))
/* The HAL writes the flag to IR, which clears it on silicon (write 1 to
 * clear); a plain struct needs the clear spelled out. */
#define __HAL_FDCAN_CLEAR_FLAG(__HANDLE__, __FLAG__) \
	((__HANDLE__)->Instance->IR &= ~(__FLAG__))

/* RCC_PERIPHCLK_FDCAN, stm32h5xx_hal_rcc_ex.h:576. */
#define RCC_PERIPHCLK_FDCAN		((uint64_t)0x400000000U)
uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint64_t PeriphClk);

HAL_StatusTypeDef HAL_FDCAN_Init(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *hfdcan,
					 const FDCAN_FilterTypeDef *sFilterConfig);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *hfdcan,
					       uint32_t NonMatchingStd,
					       uint32_t NonMatchingExt,
					       uint32_t RejectRemoteStd,
					       uint32_t RejectRemoteExt);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan,
						const FDCAN_TxHeaderTypeDef *pTxHeader,
						const uint8_t *pTxData);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *hfdcan,
					 uint32_t RxLocation,
					 FDCAN_RxHeaderTypeDef *pRxHeader,
					 uint8_t *pRxData);
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(const FDCAN_HandleTypeDef *hfdcan,
					      FDCAN_ProtocolStatusTypeDef *ProtocolStatus);
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(const FDCAN_HandleTypeDef *hfdcan,
					     FDCAN_ErrorCountersTypeDef *ErrorCounters);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(const FDCAN_HandleTypeDef *hfdcan,
				      uint32_t RxFifo);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *hfdcan,
						 uint32_t ActiveITs,
						 uint32_t BufferIndexes);
HAL_StatusTypeDef HAL_FDCAN_DeactivateNotification(FDCAN_HandleTypeDef *hfdcan,
						   uint32_t InactiveITs);
HAL_FDCAN_StateTypeDef HAL_FDCAN_GetState(const FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_ConfigTxDelayCompensation(FDCAN_HandleTypeDef *hfdcan,
						      uint32_t TdcOffset,
						      uint32_t TdcFilter);
HAL_StatusTypeDef HAL_FDCAN_EnableTxDelayCompensation(FDCAN_HandleTypeDef *hfdcan);

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
