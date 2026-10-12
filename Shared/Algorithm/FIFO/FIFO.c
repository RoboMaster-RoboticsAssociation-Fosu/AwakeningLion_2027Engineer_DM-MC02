/**
 ******************************************************************************
 * @file    FIFO.c
 * @version V1.0.0
 * @date    2026.03.05
 * @brief   FIFO环形缓冲区功能实现
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Includes ---------------------------------------------------------------- */
#include "FIFO.h"
#include "stdlib.h"
#include "stdio.h"

/* Defines ----------------------------------------------------------------- */

/* Global variable --------------------------------------------------------- */

/* Static Fun -------------------------------------------------------------- */

/* Functions --------------------------------------------------------------- */
/**
  * @brief  关闭全局中断
  * @param  无
  * @return 无
  * @note   无
  */
void __disable_IRQ(void)
{
	__set_PRIMASK(1);
}

/**
  * @brief  开启全局中断
  * @param  无
  * @return 无
  * @note   无
  */
void __enable_IRQ(void)
{
	__set_PRIMASK(0);
}

/**
  * @brief  创建FIFO实例
  * @param  unit_cnt: FIFO单元数量
  * @param  mutex: 互斥锁句柄
  * @return fifo_s_t*: 创建的FIFO实例指针
  * @note   动态分配内存
  */
fifo_s_t* fifo_s_create(uint32_t unit_cnt, SemaphoreHandle_t mutex)
{
  fifo_s_t *pfifo     = NULL;
  uint8_t  *base_addr = NULL;
  
  //! Check input parameters.
  ASSERT(0 != unit_cnt);

  //! Allocate Memory for pointer of new FIFO Control Block.
  pfifo = (fifo_s_t*) malloc(sizeof(fifo_s_t));
  if(NULL == pfifo)
  {
    //! Allocate Failure, exit now.
    return (NULL);
  }

  //! Allocate memory for FIFO.
  base_addr = malloc(unit_cnt);
  if(NULL == base_addr)
  {
    //! Allocate Failure, exit now.
    return (NULL);
  }

  fifo_s_init(pfifo, base_addr, unit_cnt, mutex);

  return (pfifo);
}

/**
  * @brief  销毁FIFO实例
  * @param  pfifo: 指向FIFO实例的指针
  * @return 无
  * @note   释放动态分配的内存
  */
void fifo_s_destory(fifo_s_t* pfifo)
{
  //! Check input parameters.
  ASSERT(NULL != pfifo);
  ASSERT(NULL != pfifo->start_addr);

  //! free FIFO memory
  free(pfifo->start_addr);
  
  //! free FIFO Control Block memory.
  free(pfifo);

  return;
}

/**
  * @brief  初始化FIFO
  * @param  pfifo: 指向FIFO实例的指针
  * @param  base_addr: 缓冲区基地址
  * @param  unit_cnt: FIFO单元数量
  * @param  mutex: 互斥锁句柄
  * @return int32_t: 0表示成功
  * @note   无
  */
int32_t fifo_s_init(fifo_s_t* pfifo, void* base_addr, uint32_t unit_cnt, SemaphoreHandle_t mutex)
{
	//! Check input parameters.
	ASSERT(NULL != pfifo);
	ASSERT(NULL != base_addr);
	ASSERT(0    != unit_cnt);
	pfifo->mutex = mutex;
    //! Initialize FIFO Control Block.
    pfifo->start_addr  = (uint8_t*) base_addr;
    pfifo->end_addr    = (uint8_t*) base_addr + unit_cnt - 1;
    pfifo->buf_size    = unit_cnt;
    pfifo->free        = unit_cnt;
    pfifo->used        = 0;
    pfifo->read_index  = 0;
    pfifo->write_index = 0;
    
    return 0;
}

/**
  * @brief  向FIFO中放入一个字节
  * @param  pfifo: 指向FIFO实例的指针
  * @param  element: 要放入的字节
  * @return int32_t: 0表示成功, -1表示失败
  * @note   无
  */
int32_t fifo_s_put(fifo_s_t* pfifo, uint8_t element)
{
  //! Check input parameters.
  ASSERT(NULL != pfifo);

  if(0 >= pfifo->free)
  {
    //! Error, FIFO is full!
    return -1;
  }
  
  MUTEX_WAIT();
  pfifo->start_addr[pfifo->write_index++] = element;
  pfifo->write_index %= pfifo->buf_size;
  pfifo->free--;
  pfifo->used++;
  MUTEX_RELEASE();
  return 0;
}

/**
  * @brief  向FIFO中写入指定数量的数据
  * @param  pfifo: 指向FIFO实例的指针
  * @param  psource: 源数据缓冲区指针
  * @param  number: 写入的数量
  * @return uint8_t: 1表示成功, 0表示失败
  * @note   无
  */
uint8_t fifo_s_puts(fifo_s_t *pfifo, uint8_t *psource, uint16_t number)
{
  
  //! Check input parameters.
  ASSERT(NULL != pfifo);
  
  if (psource == NULL)
      return 0;
  if (pfifo->free < number)
	  return 0;
   MUTEX_WAIT();
  for(uint16_t i = 0; (i < number) && (pfifo->free > 0); i++)
  {
    pfifo->start_addr[pfifo->write_index++] = psource[i];
    pfifo->write_index %= pfifo->buf_size;
    pfifo->free--;
    pfifo->used++;
  }
	MUTEX_RELEASE();
  return 1;
}

/**
  * @brief  从FIFO中获取一个字节
  * @param  pfifo: 指向FIFO实例的指针
  * @return uint8_t: 读取到的字节
  * @note   无
  */
uint8_t fifo_s_get(fifo_s_t* pfifo)
{
	uint8_t retval = 0;
  
	//! Check input parameters.
	ASSERT(NULL != pfifo);
  
	MUTEX_WAIT();
	retval = pfifo->start_addr[pfifo->read_index++];
	pfifo->read_index %= pfifo->buf_size;
	pfifo->free++;
	pfifo->used--;
	MUTEX_RELEASE();
  return retval;
}

/**
  * @brief  从FIFO中读取指定长度的数据
  * @param  pfifo: 指向FIFO实例的指针
  * @param  source: 目标数据缓冲区指针
  * @param  len: 读取的长度
  * @return uint16_t: 实际读取的长度
  * @note   无
  */
uint16_t fifo_s_gets(fifo_s_t* pfifo, uint8_t* source, uint32_t len)
{
	uint8_t retval = 0;
  
	//! Check input parameters.
	ASSERT(NULL != pfifo);
	MUTEX_WAIT();
	for (int i = 0; (i < len) && (pfifo->used > 0); i++)
	{
		source[i] = pfifo->start_addr[pfifo->read_index++];
		pfifo->read_index %= pfifo->buf_size;
		pfifo->free++;
		pfifo->used--;
		retval++;
	}
	MUTEX_WAIT();
	return retval;
}

/**
  * @brief  预读取FIFO中的一个元素
  * @param  pfifo: 指向FIFO实例的指针
  * @param  offset: 距离当前读取指针的偏移量
  * @return uint8_t: 预读取的字节
  * @note   不改变读指针
  */
uint8_t fifo_s_pre_read(fifo_s_t* pfifo, uint8_t offset)
{
	uint32_t index;
	
	//! Check input parameters.
	ASSERT(NULL != pfifo);
	
	if(offset > pfifo->used)
	{        
		return 0x00;
	}
	else
	{
		index = ((pfifo->read_index + offset) % pfifo->buf_size);
		// Move Read Pointer to right position   
		return pfifo->start_addr[index];
	}
}

/**
  * @brief  检查FIFO是否为空
  * @param  pfifo: 指向FIFO实例的指针
  * @return uint8_t: 1表示空, 0表示非空
  * @note   无
  */
uint8_t fifo_is_empty(fifo_s_t* pfifo)
{
	//! Check input parameter.
	ASSERT(NULL != pfifo);

	return (0 == pfifo->used);
}

/**
  * @brief  检查FIFO是否已满
  * @param  pfifo: 指向FIFO实例的指针
  * @return uint8_t: 1表示满, 0表示未满
  * @note   无
  */
uint8_t fifo_is_full(fifo_s_t* pfifo)
{
	//! Check input parameter.
	ASSERT(NULL != pfifo);

	return (0 == pfifo->free);
}

/**
  * @brief  获取FIFO中已用空间大小
  * @param  pfifo: 指向FIFO实例的指针
  * @return uint32_t: 已用字节数
  * @note   无
  */
uint32_t fifo_used_count(fifo_s_t* pfifo)
{
	//! Check input parameter.
	ASSERT(NULL != pfifo);

	return (pfifo->used);
}

/**
  * @brief  获取FIFO中剩余空间大小
  * @param  pfifo: 指向FIFO实例的指针
  * @return uint32_t: 剩余字节数
  * @note   无
  */
uint32_t fifo_free_count(fifo_s_t* pfifo)
{
	//! Check input parameter.
	ASSERT(NULL != pfifo);

	return (pfifo->free);
}

/**
  * @brief  清空FIFO
  * @param  pfifo: 指向FIFO实例的指针
  * @return uint8_t: 0表示成功
  * @note   无
  */
uint8_t fifo_flush(fifo_s_t* pfifo)
{
  //! Check input parameters.
  ASSERT(NULL != pfifo);

  //! Initialize FIFO Control Block.
  pfifo->free        = pfifo->buf_size;
  pfifo->used        = 0;
  pfifo->read_index  = 0;
  pfifo->write_index = 0;

  return 0;
}

/* Private functions ------------------------------------------------------- */

/* Interrupt functions ----------------------------------------------------- */

/* ------------------------------------------------------------------------- */
