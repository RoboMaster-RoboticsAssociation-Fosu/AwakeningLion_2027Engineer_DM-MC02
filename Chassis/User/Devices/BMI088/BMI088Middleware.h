/**
 ******************************************************************************
 * @file    BMI088Middleware.h
 * @version V1.0.0
 * @date    2026.03.05
 * @brief   BMI088中间件功能声明
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

/* Includes ----------------------------------------------------------------- */
#include "main.h"

/* Defines ------------------------------------------------------------------ */
/* 通信方式选择 */
#define BMI088_USE_SPI			// 使用SPI通信
//#define BMI088_USE_IIC		// 使用IIC通信

/*
#define CS1_ACCEL_GPIO_Port ACCEL_NSS_GPIO_Port
#define CS1_ACCEL_Pin ACCEL_NSS_Pin
#define CS1_GYRO_GPIO_Port GYRO_NSS_GPIO_Port
#define CS1_GYRO_Pin GYRO_NSS_Pin
*/

/* Enums -------------------------------------------------------------------- */

/* Structs ------------------------------------------------------------------ */

/* Externs ------------------------------------------------------------------ */
#if defined(BMI088_USE_SPI)
extern SPI_HandleTypeDef *BMI088_SPI;	// BMI088 SPI句柄
#endif

/* Functions ---------------------------------------------------------------- */
void BMI088_GPIO_init(void);
void BMI088_com_init(void);
void BMI088_delay_ms(uint16_t ms);
void BMI088_delay_us(uint16_t us);

#if defined(BMI088_USE_SPI)
void BMI088_ACCEL_NS_L(void);
void BMI088_ACCEL_NS_H(void);
void BMI088_GYRO_NS_L(void);
void BMI088_GYRO_NS_H(void);
uint8_t BMI088_read_write_byte(uint8_t reg);

#elif defined(BMI088_USE_IIC)

#endif

/* -------------------------------------------------------------------------- */
#endif /* BMI088MIDDLEWARE_H */
