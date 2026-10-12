/**
 ******************************************************************************
 * @file    BMI088Middleware.c
 * @version V1.0.0
 * @date    2026.03.05
 * @brief   BMI088中间件功能实现
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Includes ---------------------------------------------------------------- */
#include "BMI088Middleware.h"
#include "main.h"

/* Defines ----------------------------------------------------------------- */

/* Global variable --------------------------------------------------------- */
SPI_HandleTypeDef *BMI088_SPI;	// BMI088 SPI句柄

/* Static Fun -------------------------------------------------------------- */

/* Functions --------------------------------------------------------------- */
/**
  * @brief  BMI088加速度计片选引脚拉低
  * @param  无
  * @return 无
  * @note   无
  */
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  BMI088加速度计片选引脚拉高
  * @param  无
  * @return 无
  * @note   无
  */
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
}

/**
  * @brief  BMI088陀螺仪片选引脚拉低
  * @param  无
  * @return 无
  * @note   无
  */
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  BMI088陀螺仪片选引脚拉高
  * @param  无
  * @return 无
  * @note   无
  */
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
}

/**
  * @brief  SPI读写一个字节
  * @param  txdata: 发送的数据
  * @return uint8_t: 接收的数据
  * @note   无
  */
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(BMI088_SPI, &txdata, &rx_data, 1, 1000);
    return rx_data;
}

/* Private functions ------------------------------------------------------- */

/* Interrupt functions ----------------------------------------------------- */

/* ------------------------------------------------------------------------- */
