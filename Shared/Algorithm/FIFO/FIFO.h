/**
 ******************************************************************************
 * @file    FIFO.h
 * @version V1.0.0
 * @date    2026.03.05
 * @brief   FIFO环形缓冲区功能声明
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __FIFO_H
#define __FIFO_H

/* Includes ----------------------------------------------------------------- */
#include "stm32h7xx.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"

/* Defines ------------------------------------------------------------------ */
#define ASSERT(x) do {while(!(x));} while(0)		// 断言宏
/* 等待互斥锁 */
#define MUTEX_WAIT() 			                        \
	do {                                            \
		if(pfifo->mutex != NULL)                      \
		xSemaphoreTake(pfifo->mutex, portMAX_DELAY);  \
	} while(0)			
/* 释放互斥锁 */
#define MUTEX_RELEASE() 							    \
	do{ 											              \
		if(pfifo->mutex != NULL) 						  \
		xSemaphoreGive(pfifo->mutex);					\
	} while(0)			

#define MUTEX_DELETE()  vQueueDelete(pfifo->mutex)	// 删除互斥锁

/* Enums -------------------------------------------------------------------- */

/* Structs ------------------------------------------------------------------ */
/**
  * @brief FIFO内存模型结构体 (单字节模式)
  */
typedef struct
{
  uint8_t   *start_addr;		// 起始地址
  uint8_t   *end_addr;		// 结束地址
  uint32_t  free;				// FIFO容量
  uint32_t  buf_size;			// 缓冲区大小
  int32_t   used;				// FIFO中元素数量
  uint32_t  read_index;		// 读指针
  uint32_t  write_index;		// 写指针
  SemaphoreHandle_t mutex;	// 互斥锁句柄
} fifo_s_t;

/* Externs ------------------------------------------------------------------ */

/* Functions ---------------------------------------------------------------- */
fifo_s_t* fifo_s_create(uint32_t unit_cnt, SemaphoreHandle_t mutex);
void      fifo_s_destory(fifo_s_t* pfifo);
int32_t   fifo_s_init(fifo_s_t* pfifo, void* base_addr, uint32_t unit_cnt, SemaphoreHandle_t mutex);
int32_t   fifo_s_put(fifo_s_t* pfifo, uint8_t element);
uint8_t   fifo_s_puts(fifo_s_t *pfifo, uint8_t *psource, uint16_t number);
uint8_t   fifo_s_get(fifo_s_t* pfifo);
uint16_t  fifo_s_gets(fifo_s_t* pfifo, uint8_t* source, uint32_t len);
uint8_t   fifo_s_pre_read(fifo_s_t* pfifo, uint8_t offset);
uint8_t   fifo_is_empty(fifo_s_t* pfifo);
uint8_t   fifo_is_full(fifo_s_t* pfifo);
uint32_t  fifo_used_count(fifo_s_t* pfifo);
uint32_t  fifo_free_count(fifo_s_t* pfifo);
uint8_t   fifo_flush(fifo_s_t* pfifo);

/* -------------------------------------------------------------------------- */
#endif /* __FIFO_H */
