/**
 * @file DbgDisp.c
 * @brief Instance-based debug display protocol and STM32 HAL transport.
 */

#include "DbgDisp.h"

#include <stdarg.h>
#include <string.h>

#define DBG_FLAG_MASK ((uint8_t)DBG_FLAG_CLEAR)

static const uint8_t g_dbg_magic[4] = {
    DBG_MAGIC_0,
    DBG_MAGIC_1,
    DBG_MAGIC_2,
    DBG_MAGIC_3,
};

/** Read one little-endian uint16 field. */
static uint16_t Dbg_R16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] |
                      ((uint16_t)data[1] << 8U));
}

/** Read one little-endian uint32 field. */
static uint32_t Dbg_R32(const uint8_t *data)
{
    return (uint32_t)((uint32_t)data[0] |
                      ((uint32_t)data[1] << 8U) |
                      ((uint32_t)data[2] << 16U) |
                      ((uint32_t)data[3] << 24U));
}

/** Write one little-endian uint16 field. */
static void Dbg_W16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

/** Write one little-endian uint32 field. */
static void Dbg_W32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

/** Check that one float bit pattern is neither infinity nor NaN. */
static uint8_t Dbg_Finite(uint32_t bits)
{
    return ((bits & 0x7F800000UL) == 0x7F800000UL) ? 0U : 1U;
}

/** Check whether a configured TX mode is supported. */
static uint8_t Dbg_ModeOk(DbgTxMode mode)
{
    return ((mode == DBG_TX_BLOCKING) ||
            (mode == DBG_TX_IT) ||
            (mode == DBG_TX_DMA))
               ? 1U
               : 0U;
}

/** Validate the caller-owned object workspace. */
static uint8_t Dbg_ObjCfgOk(const Dbg_t *dbg)
{
    return ((dbg != NULL) &&
            (dbg->config.obj_buf != NULL) &&
            (dbg->config.obj_cap != 0U))
               ? 1U
               : 0U;
}

/** Validate resources needed by the streaming receiver. */
static uint8_t Dbg_RxCfgOk(const Dbg_t *dbg)
{
    return ((Dbg_ObjCfgOk(dbg) != 0U) &&
            (dbg->config.rx_buf != NULL) &&
            (dbg->config.rx_cap >= (DBG_HEAD_N + DBG_CRC_N)))
               ? 1U
               : 0U;
}

/** Validate resources needed by Receive-to-Idle DMA. */
static uint8_t Dbg_DmaCfgOk(const Dbg_t *dbg)
{
    return ((Dbg_RxCfgOk(dbg) != 0U) &&
            (dbg->config.huart != NULL) &&
            (dbg->config.huart->hdmarx != NULL) &&
            (dbg->config.huart->hdmarx->Init.Mode == DMA_CIRCULAR) &&
            (dbg->config.dma_rx_buf != NULL) &&
            (dbg->config.dma_rx_cap != 0U) &&
            (dbg->config.event_rx_buf != NULL) &&
            (dbg->config.event_rx_cap > dbg->config.dma_rx_cap))
               ? 1U
               : 0U;
}

/** Copy one DMA span into the single-producer/single-consumer byte queue. */
static DbgRet Dbg_QueueRxSpan(Dbg_t *dbg, const uint8_t *data, uint16_t len)
{
    uint16_t index = 0U;
    uint16_t head = dbg->state.event_head;

    for (index = 0U; index < len; index++)
    {
        uint16_t next = (uint16_t)(head + 1U);
        if (next >= dbg->config.event_rx_cap)
        {
            next = 0U;
        }
        if (next == dbg->state.event_tail)
        {
            dbg->state.lost_n++;
            return DBG_OVERFLOW;
        }

        dbg->config.event_rx_buf[head] = data[index];
        head = next;
    }

    dbg->state.event_head = head;
    return DBG_OK;
}

/** Validate resources needed by the HAL sender. */
static uint8_t Dbg_TxCfgOk(const Dbg_t *dbg)
{
    return ((Dbg_ObjCfgOk(dbg) != 0U) &&
            (dbg->config.huart != NULL) &&
            (Dbg_ModeOk(dbg->config.tx_mode) != 0U) &&
            (dbg->config.tx_buf != NULL) &&
            (dbg->config.tx_cap >= (DBG_HEAD_N + DBG_CRC_N)))
               ? 1U
               : 0U;
}

/** Validate one decoded name and optionally return its length. */
static uint8_t Dbg_NameOk(const char name[DBG_NAME_N], uint8_t *name_n)
{
    uint8_t index = 0U;

    for (index = 0U; index < DBG_NAME_N; ++index)
    {
        const uint8_t value = (uint8_t)name[index];

        if (value == 0U)
        {
            if (index == 0U)
            {
                return 0U;
            }
            if (name_n != NULL)
            {
                *name_n = index;
            }
            return 1U;
        }

        if ((value < 0x20U) || (value > 0x7EU))
        {
            return 0U;
        }
    }

    return 0U;
}

/** Copy one convenience-API name without reading past its 12-byte limit. */
static uint8_t Dbg_CopyName(char destination[DBG_NAME_N], const char *source)
{
    uint8_t index = 0U;

    if ((destination == NULL) || (source == NULL))
    {
        return 0U;
    }

    memset(destination, 0, DBG_NAME_N);
    for (index = 0U; index < DBG_NAME_N; ++index)
    {
        const uint8_t value = (uint8_t)source[index];

        if (value == 0U)
        {
            return (index == 0U) ? 0U : 1U;
        }
        if ((value < 0x20U) || (value > 0x7EU))
        {
            return 0U;
        }

        destination[index] = (char)value;
    }

    return 0U;
}

/** Validate all fields in one decoded frame before encoding. */
static uint8_t Dbg_FrameOk(const DbgFrame_t *frame)
{
    uint8_t index = 0U;
    uint8_t other = 0U;

    if ((frame == NULL) ||
        (sizeof(float) != 4U) ||
        ((frame->flags & (uint8_t)(~DBG_FLAG_MASK)) != 0U) ||
        (frame->count > DBG_MAX_OBJ) ||
        ((frame->count != 0U) && (frame->objects == NULL)))
    {
        return 0U;
    }

    for (index = 0U; index < frame->count; ++index)
    {
        uint32_t value_bits = 0U;

        if ((Dbg_NameOk(frame->objects[index].name, NULL) == 0U) ||
            (frame->objects[index].x >= DBG_W) ||
            (frame->objects[index].y >= DBG_H))
        {
            return 0U;
        }

        memcpy(&value_bits, &frame->objects[index].val, sizeof(value_bits));
        if (Dbg_Finite(value_bits) == 0U)
        {
            return 0U;
        }

        for (other = 0U; other < index; ++other)
        {
            if (strncmp(frame->objects[index].name,
                        frame->objects[other].name,
                        DBG_NAME_N) == 0)
            {
                return 0U;
            }
        }
    }

    return 1U;
}

/** Validate one fixed-width wire name and its zero padding. */
static uint8_t Dbg_WireNameOk(const uint8_t *name)
{
    uint8_t index = 0U;
    uint8_t zero_seen = 0U;

    for (index = 0U; index < DBG_NAME_N; ++index)
    {
        const uint8_t value = name[index];

        if (value == 0U)
        {
            if (index == 0U)
            {
                return 0U;
            }
            zero_seen = 1U;
        }
        else if ((zero_seen != 0U) || (value < 0x20U) || (value > 0x7EU))
        {
            return 0U;
        }
    }

    return zero_seen;
}

/** Validate every object directly in the instance's RX byte array. */
static uint8_t Dbg_WireOk(const Dbg_t *dbg, uint8_t count)
{
    const uint8_t *data = dbg->config.rx_buf;
    uint8_t index = 0U;
    uint8_t other = 0U;

    for (index = 0U; index < count; ++index)
    {
        const uint8_t *object = &data[DBG_HEAD_N + ((uint16_t)index * DBG_OBJ_N)];
        const uint16_t x = Dbg_R16(&object[DBG_O_X]);
        const uint16_t y = Dbg_R16(&object[DBG_O_Y]);

        if ((Dbg_WireNameOk(&object[DBG_O_NAME]) == 0U) ||
            (Dbg_Finite(Dbg_R32(&object[DBG_O_VAL])) == 0U) ||
            (x >= DBG_W) ||
            (y >= DBG_H))
        {
            return 0U;
        }

        for (other = 0U; other < index; ++other)
        {
            const uint8_t *previous = &data[DBG_HEAD_N + ((uint16_t)other * DBG_OBJ_N)];

            if (memcmp(&object[DBG_O_NAME],
                       &previous[DBG_O_NAME],
                       DBG_NAME_N) == 0)
            {
                return 0U;
            }
        }
    }

    return 1U;
}

/** Keep the next full or partial magic prefix after a malformed frame. */
static void Dbg_Resync(Dbg_t *dbg)
{
    uint8_t *rx = dbg->config.rx_buf;
    uint16_t start = 0U;

    for (start = 1U; start < dbg->state.rx_n; ++start)
    {
        const uint16_t keep_n = (uint16_t)(dbg->state.rx_n - start);
        const uint16_t compare_n = (keep_n < sizeof(g_dbg_magic))
                                       ? keep_n
                                       : (uint16_t)sizeof(g_dbg_magic);

        if (memcmp(&rx[start], g_dbg_magic, compare_n) == 0)
        {
            memmove(rx, &rx[start], keep_n);
            dbg->state.rx_n = keep_n;
            return;
        }
    }

    dbg->state.rx_n = 0U;
}

/** Decode a validated wire frame into the configured object array. */
static void Dbg_Decode(Dbg_t *dbg)
{
    const uint8_t *rx = dbg->config.rx_buf;
    const uint8_t count = rx[DBG_H_COUNT];
    const uint16_t sequence = Dbg_R16(&rx[DBG_H_SEQ]);
    uint8_t index = 0U;

    if (dbg->state.has_seq != 0U)
    {
        const uint16_t delta = (uint16_t)(sequence - dbg->state.last_seq);

        if ((delta > 1U) && (delta < 0x8000U))
        {
            dbg->state.lost_n += (uint32_t)(delta - 1U);
        }
    }

    dbg->state.has_seq = 1U;
    dbg->state.last_seq = sequence;
    memset(dbg->config.obj_buf,
           0,
           (size_t)dbg->config.obj_cap * sizeof(dbg->config.obj_buf[0]));

    dbg->state.frame.seq = sequence;
    dbg->state.frame.flags = rx[DBG_H_FLAGS];
    dbg->state.frame.count = count;
    dbg->state.frame.objects = dbg->config.obj_buf;

    for (index = 0U; index < count; ++index)
    {
        const uint8_t *object = &rx[DBG_HEAD_N + ((uint16_t)index * DBG_OBJ_N)];
        uint32_t value_bits = Dbg_R32(&object[DBG_O_VAL]);

        memcpy(dbg->config.obj_buf[index].name,
               &object[DBG_O_NAME],
               DBG_NAME_N);
        memcpy(&dbg->config.obj_buf[index].val,
               &value_bits,
               sizeof(value_bits));
        dbg->config.obj_buf[index].x = Dbg_R16(&object[DBG_O_X]);
        dbg->config.obj_buf[index].y = Dbg_R16(&object[DBG_O_Y]);
    }

    dbg->state.ready = 1U;
    dbg->state.ok_n++;
}

/**
 * Try to consume the current RX array.
 * Return 1 for valid, -1 for rejected, or 0 when incomplete.
 */
static int8_t Dbg_TryFrame(Dbg_t *dbg)
{
    uint8_t *rx = dbg->config.rx_buf;
    uint8_t count = 0U;
    uint16_t payload_n = 0U;
    uint16_t frame_n = 0U;
    uint16_t received_crc = 0U;
    uint16_t calculated_crc = 0U;

    if (dbg->state.rx_n < DBG_HEAD_N)
    {
        return 0;
    }

    count = rx[DBG_H_COUNT];
    payload_n = Dbg_R16(&rx[DBG_H_LEN]);
    frame_n = (uint16_t)(DBG_HEAD_N + payload_n + DBG_CRC_N);

    if ((memcmp(rx, g_dbg_magic, sizeof(g_dbg_magic)) != 0) ||
        (rx[DBG_H_VER] != DBG_VER) ||
        (rx[DBG_H_TYPE] != (uint8_t)DBG_TYPE_SNAPSHOT) ||
        ((rx[DBG_H_FLAGS] & (uint8_t)(~DBG_FLAG_MASK)) != 0U) ||
        (count > DBG_MAX_OBJ) ||
        (count > dbg->config.obj_cap) ||
        (payload_n != ((uint16_t)count * DBG_OBJ_N)) ||
        (frame_n > dbg->config.rx_cap))
    {
        dbg->state.fmt_err_n++;
        Dbg_Resync(dbg);
        return -1;
    }

    if (dbg->state.rx_n < frame_n)
    {
        return 0;
    }

    received_crc = Dbg_R16(&rx[frame_n - DBG_CRC_N]);
    calculated_crc = Dbg_Crc16(rx, (uint16_t)(frame_n - DBG_CRC_N));
    if (received_crc != calculated_crc)
    {
        dbg->state.crc_err_n++;
        Dbg_Resync(dbg);
        return -1;
    }

    if (Dbg_WireOk(dbg, count) == 0U)
    {
        dbg->state.fmt_err_n++;
        Dbg_Resync(dbg);
        return -1;
    }

    Dbg_Decode(dbg);

    if (dbg->state.rx_n > frame_n)
    {
        const uint16_t remain_n = (uint16_t)(dbg->state.rx_n - frame_n);
        memmove(rx, &rx[frame_n], remain_n);
        dbg->state.rx_n = remain_n;
    }
    else
    {
        dbg->state.rx_n = 0U;
    }

    return 1;
}

DbgRet Dbg_Push(Dbg_t *dbg, const uint8_t *data, uint16_t len)
{
    uint16_t index = 0U;
    uint8_t frame_seen = 0U;
    uint8_t error_seen = 0U;

    if ((Dbg_RxCfgOk(dbg) == 0U) || ((data == NULL) && (len != 0U)))
    {
        return DBG_BAD_ARG;
    }

    if (dbg->state.rx_n > dbg->config.rx_cap)
    {
        dbg->state.rx_n = 0U;
        dbg->state.fmt_err_n++;
    }

    for (index = 0U; index < len; ++index)
    {
        const uint8_t value = data[index];
        uint8_t retry = 1U;

        while (retry != 0U)
        {
            int8_t parse_result = 0;
            retry = 0U;

            if (dbg->state.rx_n < sizeof(g_dbg_magic))
            {
                if (value != g_dbg_magic[dbg->state.rx_n])
                {
                    if (value == g_dbg_magic[0])
                    {
                        dbg->config.rx_buf[0] = value;
                        dbg->state.rx_n = 1U;
                    }
                    else
                    {
                        dbg->state.rx_n = 0U;
                    }
                    break;
                }
            }

            if (dbg->state.rx_n >= dbg->config.rx_cap)
            {
                dbg->state.fmt_err_n++;
                Dbg_Resync(dbg);
                error_seen = 1U;
                retry = 1U;
                continue;
            }

            dbg->config.rx_buf[dbg->state.rx_n] = value;
            dbg->state.rx_n++;

            do
            {
                parse_result = Dbg_TryFrame(dbg);
                if (parse_result > 0)
                {
                    frame_seen = 1U;
                }
                else if (parse_result < 0)
                {
                    error_seen = 1U;
                }
            } while ((parse_result != 0) && (dbg->state.rx_n >= DBG_HEAD_N));
        }
    }

    if (frame_seen != 0U)
    {
        return DBG_FRAME_READY;
    }
    if (error_seen != 0U)
    {
        return DBG_BAD_FRAME;
    }
    return DBG_NEED_MORE;
}

DbgRet Dbg_StartRx(Dbg_t *dbg)
{
    HAL_StatusTypeDef hal_status = HAL_ERROR;

    if (Dbg_DmaCfgOk(dbg) == 0U)
    {
        return DBG_BAD_ARG;
    }
    if (dbg->state.rx_active != 0U)
    {
        return DBG_BUSY;
    }

    hal_status = HAL_UARTEx_ReceiveToIdle_DMA(dbg->config.huart,
                                              dbg->config.dma_rx_buf,
                                              dbg->config.dma_rx_cap);
    if (hal_status == HAL_OK)
    {
        dbg->state.dma_pos = 0U;
        dbg->state.event_head = 0U;
        dbg->state.event_tail = 0U;
        dbg->state.rx_error_pending = 0U;
        dbg->state.rx_active = 1U;
        __HAL_DMA_DISABLE_IT(dbg->config.huart->hdmarx, DMA_IT_HT);
        return DBG_OK;
    }

    dbg->state.rx_io_err_n++;
    return (hal_status == HAL_BUSY) ? DBG_BUSY : DBG_IO_ERR;
}

DbgRet Dbg_StopRx(Dbg_t *dbg)
{
    HAL_StatusTypeDef hal_status = HAL_OK;

    if (Dbg_DmaCfgOk(dbg) == 0U)
    {
        return DBG_BAD_ARG;
    }

    if (dbg->state.rx_active != 0U)
    {
        /* Clear first so an abort-related callback cannot consume stale bytes. */
        dbg->state.rx_active = 0U;
        hal_status = HAL_UART_AbortReceive(dbg->config.huart);
    }
    dbg->state.dma_pos = 0U;
    dbg->state.event_head = 0U;
    dbg->state.event_tail = 0U;
    dbg->state.rx_n = 0U;
    dbg->state.ready = 0U;
    dbg->state.rx_error_pending = 0U;

    if (hal_status != HAL_OK)
    {
        dbg->state.rx_io_err_n++;
        return DBG_IO_ERR;
    }
    return DBG_OK;
}

DbgRet Dbg_RxEvent(Dbg_t *dbg,
                   UART_HandleTypeDef *huart,
                   uint16_t size)
{
    DbgRet queue_result = DBG_OK;
    DbgRet part_result = DBG_OK;
    uint16_t previous_pos = 0U;
    uint16_t capacity = 0U;

    if ((Dbg_DmaCfgOk(dbg) == 0U) ||
        (huart == NULL) ||
        (huart != dbg->config.huart))
    {
        return DBG_BAD_ARG;
    }
    if (dbg->state.rx_active == 0U)
    {
        return DBG_NEED_MORE;
    }

    capacity = dbg->config.dma_rx_cap;
    previous_pos = dbg->state.dma_pos;
    if ((size > capacity) || (previous_pos >= capacity))
    {
        dbg->state.fmt_err_n++;
        dbg->state.dma_pos = 0U;
        return DBG_BAD_FRAME;
    }

    if (size > previous_pos)
    {
        queue_result = Dbg_QueueRxSpan(dbg,
                                      &dbg->config.dma_rx_buf[previous_pos],
                                      (uint16_t)(size - previous_pos));
    }
    else if (size < previous_pos)
    {
        /* DMA wrapped before this event: queue the tail, then the head. */
        queue_result = Dbg_QueueRxSpan(dbg,
                                      &dbg->config.dma_rx_buf[previous_pos],
                                      (uint16_t)(capacity - previous_pos));
        if (size != 0U)
        {
            part_result = Dbg_QueueRxSpan(dbg,
                                         dbg->config.dma_rx_buf,
                                         size);
            if (part_result != DBG_OK)
            {
                queue_result = part_result;
            }
        }
    }

    /* HAL reports capacity at TC; DMA has already wrapped to position zero. */
    dbg->state.dma_pos = (size == capacity) ? 0U : size;

    return queue_result;
}

DbgRet Dbg_ProcessRx(Dbg_t *dbg)
{
    DbgRet result = DBG_NEED_MORE;

    if (Dbg_DmaCfgOk(dbg) == 0U)
    {
        return DBG_BAD_ARG;
    }

    if (dbg->state.rx_error_pending != 0U)
    {
        dbg->state.rx_error_pending = 0U;
        dbg->state.dma_pos = 0U;
        dbg->state.event_head = 0U;
        dbg->state.event_tail = 0U;
        dbg->state.rx_n = 0U;
        (void)HAL_UART_AbortReceive(dbg->config.huart);
        if (Dbg_StartRx(dbg) != DBG_OK)
        {
            dbg->state.rx_io_err_n++;
            return DBG_IO_ERR;
        }
    }

    while (dbg->state.event_tail != dbg->state.event_head)
    {
        uint16_t tail = dbg->state.event_tail;
        uint16_t head = dbg->state.event_head;
        uint16_t span = (head > tail)
                            ? (uint16_t)(head - tail)
                            : (uint16_t)(dbg->config.event_rx_cap - tail);
        const DbgRet part = Dbg_Push(dbg, &dbg->config.event_rx_buf[tail], span);

        tail = (uint16_t)(tail + span);
        if (tail >= dbg->config.event_rx_cap)
        {
            tail = 0U;
        }
        dbg->state.event_tail = tail;

        if (part == DBG_FRAME_READY)
        {
            result = DBG_FRAME_READY;
        }
        else if ((part != DBG_NEED_MORE) && (result == DBG_NEED_MORE))
        {
            result = part;
        }
    }

    return result;
}

void Dbg_RxError(Dbg_t *dbg, UART_HandleTypeDef *huart)
{
    if ((Dbg_DmaCfgOk(dbg) == 0U) ||
        (huart == NULL) ||
        (huart != dbg->config.huart))
    {
        return;
    }

    dbg->state.rx_active = 0U;
    dbg->state.rx_error_pending = 1U;
}

const DbgFrame_t *Dbg_Get(const Dbg_t *dbg)
{
    if ((dbg == NULL) || (dbg->state.ready == 0U))
    {
        return NULL;
    }

    return &dbg->state.frame;
}

void Dbg_Done(Dbg_t *dbg)
{
    if (dbg != NULL)
    {
        dbg->state.ready = 0U;
    }
}

DbgRet Dbg_Encode(const DbgFrame_t *frame,
                  uint8_t *out,
                  uint16_t cap,
                  uint16_t *out_n)
{
    uint16_t payload_n = 0U;
    uint16_t frame_n = 0U;
    uint16_t crc = 0U;
    uint8_t index = 0U;

    if (out_n != NULL)
    {
        *out_n = 0U;
    }

    if ((frame == NULL) || (out == NULL) || (out_n == NULL))
    {
        return DBG_BAD_ARG;
    }
    if (Dbg_FrameOk(frame) == 0U)
    {
        return DBG_BAD_FRAME;
    }

    payload_n = (uint16_t)((uint16_t)frame->count * DBG_OBJ_N);
    frame_n = (uint16_t)(DBG_HEAD_N + payload_n + DBG_CRC_N);
    if (cap < frame_n)
    {
        return DBG_OVERFLOW;
    }

    memset(out, 0, frame_n);
    memcpy(&out[DBG_H_MAGIC], g_dbg_magic, sizeof(g_dbg_magic));
    out[DBG_H_VER] = DBG_VER;
    out[DBG_H_TYPE] = (uint8_t)DBG_TYPE_SNAPSHOT;
    out[DBG_H_FLAGS] = frame->flags;
    out[DBG_H_COUNT] = frame->count;
    Dbg_W16(&out[DBG_H_SEQ], frame->seq);
    Dbg_W16(&out[DBG_H_LEN], payload_n);

    for (index = 0U; index < frame->count; ++index)
    {
        uint8_t *object = &out[DBG_HEAD_N + ((uint16_t)index * DBG_OBJ_N)];
        uint8_t name_n = 0U;
        uint32_t value_bits = 0U;

        (void)Dbg_NameOk(frame->objects[index].name, &name_n);
        memcpy(&object[DBG_O_NAME], frame->objects[index].name, name_n);
        memcpy(&value_bits, &frame->objects[index].val, sizeof(value_bits));
        Dbg_W32(&object[DBG_O_VAL], value_bits);
        Dbg_W16(&object[DBG_O_X], frame->objects[index].x);
        Dbg_W16(&object[DBG_O_Y], frame->objects[index].y);
    }

    crc = Dbg_Crc16(out, (uint16_t)(frame_n - DBG_CRC_N));
    Dbg_W16(&out[frame_n - DBG_CRC_N], crc);
    *out_n = frame_n;

    return DBG_OK;
}

DbgRet Dbg_SendVArgs(Dbg_t *dbg,
                     const char *name,
                     double value,
                     va_list arguments)
{
    const char *current_name = name;
    double current_value = value;
    DbgFrame_t frame;
    uint16_t encoded_n = 0U;
    DbgRet result = DBG_OK;
    HAL_StatusTypeDef hal_status = HAL_ERROR;
    if ((Dbg_TxCfgOk(dbg) == 0U) || (name == NULL))
    {
        return DBG_BAD_ARG;
    }
    if (dbg->state.tx_busy != 0U)
    {
        return DBG_BUSY;
    }

    dbg->state.tx_busy = 1U;
    memset(dbg->config.obj_buf,
           0,
           (size_t)dbg->config.obj_cap * sizeof(dbg->config.obj_buf[0]));
    frame.seq = dbg->state.tx_seq;
    frame.flags = DBG_FLAG_NONE;
    frame.count = 0U;
    frame.objects = dbg->config.obj_buf;

    while (current_name != NULL)
    {
        DbgObj_t *object = NULL;

        if ((frame.count >= DBG_MAX_OBJ) ||
            (frame.count >= dbg->config.obj_cap))
        {
            result = DBG_OVERFLOW;
            break;
        }

        object = &frame.objects[frame.count];
        if (Dbg_CopyName(object->name, current_name) == 0U)
        {
            result = DBG_BAD_FRAME;
            break;
        }

        object->val = (float)current_value;
        object->x = 0U;
        object->y = (uint16_t)frame.count * DBG_LINE_H;
        frame.count++;

        current_name = va_arg(arguments, const char *);
        if (current_name != NULL)
        {
            current_value = va_arg(arguments, double);
        }
    }
    if (result == DBG_OK)
    {
        result = Dbg_Encode(&frame,
                            dbg->config.tx_buf,
                            dbg->config.tx_cap,
                            &encoded_n);
    }
    if (result != DBG_OK)
    {
        dbg->state.tx_busy = 0U;
        return result;
    }

    if (dbg->config.tx_mode == DBG_TX_BLOCKING)
    {
        hal_status = HAL_UART_Transmit(dbg->config.huart,
                                       dbg->config.tx_buf,
                                       encoded_n,
                                       dbg->config.timeout_ms);
    }
    else if (dbg->config.tx_mode == DBG_TX_IT)
    {
        hal_status = HAL_UART_Transmit_IT(dbg->config.huart,
                                          dbg->config.tx_buf,
                                          encoded_n);
    }
    else
    {
        hal_status = HAL_UART_Transmit_DMA(dbg->config.huart,
                                           dbg->config.tx_buf,
                                           encoded_n);
    }

    if (hal_status == HAL_OK)
    {
        dbg->state.tx_seq++;
        if (dbg->config.tx_mode == DBG_TX_BLOCKING)
        {
            dbg->state.tx_busy = 0U;
        }
        return DBG_OK;
    }

    dbg->state.tx_busy = 0U;
    return (hal_status == HAL_BUSY) ? DBG_BUSY : DBG_IO_ERR;
}

DbgRet Dbg_SendArgs(Dbg_t *dbg, const char *name, double value, ...)
{
    DbgRet result = DBG_OK;
    va_list arguments;

    va_start(arguments, value);
    result = Dbg_SendVArgs(dbg, name, value, arguments);
    va_end(arguments);

    return result;
}

void Dbg_TxDone(Dbg_t *dbg, UART_HandleTypeDef *huart)
{
    if ((dbg != NULL) &&
        (huart != NULL) &&
        (huart == dbg->config.huart))
    {
        dbg->state.tx_busy = 0U;
    }
}

uint8_t Dbg_TxBusy(const Dbg_t *dbg)
{
    return (dbg == NULL) ? 0U : dbg->state.tx_busy;
}

uint16_t Dbg_Crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = DBG_CRC_INIT;
    uint16_t index = 0U;

    if ((data == NULL) && (len != 0U))
    {
        return 0U;
    }

    for (index = 0U; index < len; ++index)
    {
        uint8_t bit = 0U;

        crc ^= (uint16_t)((uint16_t)data[index] << 8U);
        for (bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1U) ^ DBG_CRC_POLY);
            }
            else
            {
                crc = (uint16_t)(crc << 1U);
            }
        }
    }

    return crc;
}
