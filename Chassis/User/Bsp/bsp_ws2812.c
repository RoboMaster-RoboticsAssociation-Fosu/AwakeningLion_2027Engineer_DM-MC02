/**
 ******************************************************************************
 * @file    bsp_ws2812.c
 * @version V1.0.0
 * @date    2026.08.16
 * @brief   板载单颗WS2812B指示灯驱动
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * WS2812数据位使用5个SPI位编码：0为11000，1为11100。
 * * 仅应在MX_SPI6_Init()执行完成后调用公开接口。
 ******************************************************************************
 */

/* Includes ----------------------------------------------------------------- */
#include "bsp_ws2812.h"

#include "spi.h"

/* Defines ------------------------------------------------------------------ */
#define BSP_WS2812_SPI_KERNEL_CLOCK_HZ        130000000U
#define BSP_WS2812_ENCODED_BYTES_PER_CHANNEL  5U
#define BSP_WS2812_COLOR_DATA_BYTES            15U
#define BSP_WS2812_RESET_BYTES                 32U
#define BSP_WS2812_FRAME_BYTES                 (BSP_WS2812_COLOR_DATA_BYTES + BSP_WS2812_RESET_BYTES)
#define BSP_WS2812_SYMBOL_ZERO                 0x18U
#define BSP_WS2812_SYMBOL_ONE                  0x1CU
#define BSP_WS2812_SPI_TIMEOUT_MS              2U

/* Global variable ---------------------------------------------------------- */

/* Static Fun --------------------------------------------------------------- */
/**
 * @brief  检查SPI6配置是否满足本驱动的固定时序契约
 * @return 1表示配置正确，0表示配置不匹配或SPI6尚未初始化
 */
static uint8_t bsp_ws2812_spi_config_is_valid(void)
{
    if ((hspi6.Instance != SPI6) ||
        (hspi6.Init.Mode != SPI_MODE_MASTER) ||
        (hspi6.Init.Direction != SPI_DIRECTION_2LINES_TXONLY) ||
        (hspi6.Init.DataSize != SPI_DATASIZE_8BIT) ||
        (hspi6.Init.CLKPolarity != SPI_POLARITY_LOW) ||
        (hspi6.Init.CLKPhase != SPI_PHASE_1EDGE) ||
        (hspi6.Init.NSS != SPI_NSS_SOFT) ||
        (hspi6.Init.BaudRatePrescaler != SPI_BAUDRATEPRESCALER_32) ||
        (hspi6.Init.FirstBit != SPI_FIRSTBIT_MSB) ||
        (hspi6.Init.MasterKeepIOState != SPI_MASTER_KEEP_IO_STATE_ENABLE) ||
        (HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI6) != BSP_WS2812_SPI_KERNEL_CLOCK_HZ))
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief  将一个颜色通道编码为连续的5字节WS2812波形数据
 * @param  value: 待编码的8位颜色值
 * @param  output: 5字节输出缓冲区
 */
static void bsp_ws2812_encode_channel(uint8_t value,
                                      uint8_t output[BSP_WS2812_ENCODED_BYTES_PER_CHANNEL])
{
    uint64_t encoded = 0U;
    uint8_t bit_index;

    for (bit_index = 0U; bit_index < 8U; bit_index++)
    {
        encoded <<= 5U;
        if ((value & (uint8_t)(0x80U >> bit_index)) != 0U)
        {
            encoded |= (uint64_t)BSP_WS2812_SYMBOL_ONE;
        }
        else
        {
            encoded |= (uint64_t)BSP_WS2812_SYMBOL_ZERO;
        }
    }

    output[0] = (uint8_t)(encoded >> 32U);
    output[1] = (uint8_t)(encoded >> 24U);
    output[2] = (uint8_t)(encoded >> 16U);
    output[3] = (uint8_t)(encoded >> 8U);
    output[4] = (uint8_t)encoded;
}

/**
 * @brief  将HAL SPI状态转换为板级驱动状态
 * @param  hal_status: HAL SPI发送结果
 * @return 板级驱动状态
 */
static BspWs2812Status_e bsp_ws2812_status_from_hal(HAL_StatusTypeDef hal_status)
{
    switch (hal_status)
    {
        case HAL_OK:
            return BSP_WS2812_STATUS_OK;

        case HAL_BUSY:
            return BSP_WS2812_STATUS_BUSY;

        case HAL_TIMEOUT:
            return BSP_WS2812_STATUS_TIMEOUT;

        case HAL_ERROR:
        default:
            return BSP_WS2812_STATUS_ERROR;
    }
}

/* Functions ---------------------------------------------------------------- */
/**
 * @brief  设置板载WS2812B颜色
 * @param  red: 红色亮度，范围0~255
 * @param  green: 绿色亮度，范围0~255
 * @param  blue: 蓝色亮度，范围0~255
 * @return WS2812发送结果
 * @note   WS2812线序为GRB。帧尾32个零字节产生约63 us低电平复位时间。
 */
BspWs2812Status_e BSP_WS2812_SetColor(uint8_t red, uint8_t green, uint8_t blue)
{
    uint8_t frame[BSP_WS2812_FRAME_BYTES] = {0U};
    HAL_StatusTypeDef hal_status;

    if (bsp_ws2812_spi_config_is_valid() == 0U)
    {
        return BSP_WS2812_STATUS_CONFIG_ERROR;
    }

    bsp_ws2812_encode_channel(green, &frame[0]);
    bsp_ws2812_encode_channel(red, &frame[BSP_WS2812_ENCODED_BYTES_PER_CHANNEL]);
    bsp_ws2812_encode_channel(blue, &frame[2U * BSP_WS2812_ENCODED_BYTES_PER_CHANNEL]);

    hal_status = HAL_SPI_Transmit(&hspi6,
                                  frame,
                                  (uint16_t)sizeof(frame),
                                  BSP_WS2812_SPI_TIMEOUT_MS);
    return bsp_ws2812_status_from_hal(hal_status);
}

/* Private functions ------------------------------------------------------- */

/* Interrupt functions ----------------------------------------------------- */

/* ------------------------------------------------------------------------- */
