#ifndef DBG_DISP_H
#define DBG_DISP_H

/**
 * @file DbgDisp.h
 * @brief STM32 HAL serial protocol for a 240 x 320 portrait debug display.
 *
 * Every Dbg_t instance uses buffers owned by the application. The library
 * contains no mutable global frame, object, TX or RX buffer.
 */

#include "stm32h7xx_hal.h"
#include <stddef.h>
#include <stdarg.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DBG_W           240U
#define DBG_H           320U
#define DBG_DECIMALS    3U
#define DBG_LINE_H      18U
#define DBG_TIMEOUT_MS  100U

#define DBG_VER         1U
#define DBG_MAX_OBJ     16U
#define DBG_NAME_N      12U
#define DBG_NAME_MAX    (DBG_NAME_N - 1U)

#define DBG_HEAD_N      12U
#define DBG_OBJ_N       20U
#define DBG_CRC_N       2U
#define DBG_DMA_RX_N    64U
#define DBG_PAYLOAD_MAX (DBG_MAX_OBJ * DBG_OBJ_N)
#define DBG_FRAME_MAX   (DBG_HEAD_N + DBG_PAYLOAD_MAX + DBG_CRC_N)
#define DBG_EVENT_RX_N  (DBG_FRAME_MAX + DBG_DMA_RX_N)

/* Header offsets. Magic is the ASCII string "RDBG". */
#define DBG_H_MAGIC     0U
#define DBG_H_VER       4U
#define DBG_H_TYPE      5U
#define DBG_H_FLAGS     6U
#define DBG_H_COUNT     7U
#define DBG_H_SEQ       8U
#define DBG_H_LEN       10U

#define DBG_MAGIC_0     0x52U
#define DBG_MAGIC_1     0x44U
#define DBG_MAGIC_2     0x42U
#define DBG_MAGIC_3     0x47U

/* Object offsets: name[12], float32 value, uint16 x and uint16 y. */
#define DBG_O_NAME      0U
#define DBG_O_VAL       12U
#define DBG_O_X         16U
#define DBG_O_Y         18U

#define DBG_CRC_POLY    0x1021U
#define DBG_CRC_INIT    0xFFFFU

typedef enum
{
    DBG_TYPE_SNAPSHOT = 1,
} DbgType;

typedef enum
{
    DBG_FLAG_NONE = 0x00,
    DBG_FLAG_CLEAR = 0x01,
} DbgFlag;

typedef enum
{
    DBG_OK = 0,
    DBG_FRAME_READY = 1,
    DBG_NEED_MORE = 2,
    DBG_BAD_ARG = -1,
    DBG_BAD_FRAME = -2,
    DBG_OVERFLOW = -3,
    DBG_BUSY = -4,
    DBG_IO_ERR = -5,
} DbgRet;

typedef enum
{
    DBG_TX_BLOCKING = 0,
    DBG_TX_IT = 1,
    DBG_TX_DMA = 2,
} DbgTxMode;

typedef struct
{
    char name[DBG_NAME_N];
    float val;
    uint16_t x;
    uint16_t y;
} DbgObj_t;

/** A frame references an application-owned object array. */
typedef struct
{
    uint16_t seq;
    uint8_t flags;
    uint8_t count;
    DbgObj_t *objects;
} DbgFrame_t;

/** Immutable resources and policy selected by the application. */
typedef struct
{
    UART_HandleTypeDef *huart;
    DbgTxMode tx_mode;
    uint32_t timeout_ms;

    uint8_t *tx_buf;
    uint16_t tx_cap;
    uint8_t *rx_buf;
    uint16_t rx_cap;
    uint8_t *dma_rx_buf;
    uint16_t dma_rx_cap;
    uint8_t *event_rx_buf;
    uint16_t event_rx_cap;
    DbgObj_t *obj_buf;
    uint8_t obj_cap;
} DbgConfig_t;

/** Mutable protocol and transport state for one instance. */
typedef struct
{
    uint16_t rx_n;
    uint16_t dma_pos;
    volatile uint16_t event_head;
    volatile uint16_t event_tail;
    DbgFrame_t frame;
    uint8_t ready;
    uint8_t rx_active;
    volatile uint8_t rx_error_pending;
    uint8_t has_seq;
    uint16_t last_seq;

    uint16_t tx_seq;
    volatile uint8_t tx_busy;

    uint32_t ok_n;
    uint32_t crc_err_n;
    uint32_t fmt_err_n;
    uint32_t lost_n;
    uint32_t rx_io_err_n;
} DbgState_t;

typedef struct
{
    DbgConfig_t config;
    DbgState_t state;
} Dbg_t;

/*
 * Declare one static Dbg_t together with its private TX, parser RX, DMA RX,
 * ISR-to-main RX queue and object arrays.
 * name_ must be a plain C identifier. Use this macro at file scope so the
 * instance is directly available to HAL callbacks in the same source file.
 */
#define DBG_CREATE_T(name_, huart_, mode_, timeout_)                 \
    static uint8_t name_##_tx[DBG_FRAME_MAX];                        \
    static uint8_t name_##_rx[DBG_FRAME_MAX];                        \
    static uint8_t name_##_dma_rx[DBG_DMA_RX_N];                     \
    static uint8_t name_##_event_rx[DBG_EVENT_RX_N];                 \
    static DbgObj_t name_##_objects[DBG_MAX_OBJ];                    \
    static Dbg_t name_ = {                                           \
        {                                                            \
            (huart_),                                                \
            (mode_),                                                 \
            (uint32_t)(timeout_),                                    \
            name_##_tx,                                              \
            (uint16_t)sizeof(name_##_tx),                            \
            name_##_rx,                                              \
            (uint16_t)sizeof(name_##_rx),                            \
            name_##_dma_rx,                                         \
            (uint16_t)sizeof(name_##_dma_rx),                        \
            name_##_event_rx,                                       \
            (uint16_t)sizeof(name_##_event_rx),                      \
            name_##_objects,                                        \
            (uint8_t)(sizeof(name_##_objects) /                      \
                      sizeof(name_##_objects[0])),                   \
        },                                                           \
        {0},                                                         \
    }

#define DBG_CREATE(name_, huart_, mode_) \
    DBG_CREATE_T(name_, huart_, mode_, DBG_TIMEOUT_MS)

/** Feed any UART byte span into this instance's streaming parser. */
DbgRet Dbg_Push(Dbg_t *dbg, const uint8_t *data, uint16_t len);

/**
 * Start circular Receive-to-Idle DMA using the instance DMA RX array.
 * The configured UART must have an RX DMA handle in DMA_CIRCULAR mode.
 */
DbgRet Dbg_StartRx(Dbg_t *dbg);

/**
 * Abort circular reception and discard any partial or pending debug frame.
 * Reception remains stopped until Dbg_StartRx() is called explicitly.
 */
DbgRet Dbg_StopRx(Dbg_t *dbg);

/**
 * Copy only the bytes added since the previous circular-DMA RX event into the
 * instance queue. Protocol parsing is deliberately deferred to Dbg_ProcessRx()
 * so the UART ISR stays bounded. Circular DMA remains active across IDLE and
 * transfer-complete events. Forward the HAL callback exactly as shown.
 *
 * @code
 * void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
 * {
 *     (void)Dbg_RxEvent(&debug, huart, size);
 * }
 * @endcode
 */
DbgRet Dbg_RxEvent(Dbg_t *dbg,
                   UART_HandleTypeDef *huart,
                   uint16_t size);

/**
 * Drain queued DMA bytes and parse complete frames in main-loop context.
 * Call this frequently while reception is active.
 */
DbgRet Dbg_ProcessRx(Dbg_t *dbg);

/**
 * Defer recovery after a HAL UART error to Dbg_ProcessRx(). Forward the error
 * callback exactly as shown. Dbg_TxDone also releases the TX array if the UART
 * error terminated an asynchronous transmission.
 *
 * @code
 * void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
 * {
 *     Dbg_TxDone(&debug, huart);
 *     Dbg_RxError(&debug, huart);
 * }
 * @endcode
 */
void Dbg_RxError(Dbg_t *dbg, UART_HandleTypeDef *huart);

/** Return the newest unconsumed frame, or NULL. */
const DbgFrame_t *Dbg_Get(const Dbg_t *dbg);

/** Mark the newest received frame as consumed by the display. */
void Dbg_Done(Dbg_t *dbg);

/** Encode one explicit frame into a caller-provided byte array. */
DbgRet Dbg_Encode(const DbgFrame_t *frame,
                  uint8_t *out,
                  uint16_t cap,
                  uint16_t *out_n);

/** Send name/value pairs using this instance's HAL UART and TX array. */
DbgRet Dbg_SendVArgs(Dbg_t *dbg,
                     const char *name,
                     double value,
                     va_list arguments);

/** Send name/value pairs using this instance's HAL UART and TX array. */
DbgRet Dbg_SendArgs(Dbg_t *dbg, const char *name, double value, ...);

#define Dbg_Send(dbg_, ...) \
    Dbg_SendArgs((dbg_), __VA_ARGS__, (const char *)NULL)

/**
 * Release an IT/DMA TX array after HAL reports transmission completion.
 * The huart check is performed inside Dbg_TxDone, so unrelated UART callbacks
 * are ignored.
 *
 * @code
 * void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
 * {
 *     Dbg_TxDone(&debug, huart);
 * }
 * @endcode
 */
void Dbg_TxDone(Dbg_t *dbg, UART_HandleTypeDef *huart);

/** Return nonzero while IT/DMA is using this instance's TX array. */
uint8_t Dbg_TxBusy(const Dbg_t *dbg);

/** Calculate CRC-16/CCITT-FALSE. */
uint16_t Dbg_Crc16(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* DBG_DISP_H */
