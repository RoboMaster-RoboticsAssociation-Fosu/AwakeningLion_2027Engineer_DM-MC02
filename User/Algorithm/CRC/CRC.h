/**
 ******************************************************************************
 * @file    CRC.h
 * @version V1.0.0
 * @date    2026.03.05
 * @brief   CRC校验算法功能声明
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef CRC_H
#define CRC_H

/* Includes ----------------------------------------------------------------- */
#include "stdint.h"
#include "stdbool.h"

/* Defines ------------------------------------------------------------------ */

/* Enums -------------------------------------------------------------------- */

/* Structs ------------------------------------------------------------------ */

/* Externs ------------------------------------------------------------------ */

/* Functions ---------------------------------------------------------------- */
uint8_t Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
bool Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
bool Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

/* -------------------------------------------------------------------------- */
#endif /* CRC_H */
