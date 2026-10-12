/**
 ******************************************************************************
 * @file    bsp_usb.c
 * @version V1.0.0
 * @date    2026.03.04
 * @brief   USB通信功能函数
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Includes ---------------------------------------------------------------- */
#include "bsp_usb.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "PC_Comm.h"
/* Defines ----------------------------------------------------------------- */

/* Global variable --------------------------------------------------------- */
static uint8_t *USB_Curruct_Buf;
extern USBD_HandleTypeDef hUsbDeviceHS;
extern uint8_t UserRxBufferHS[APP_RX_DATA_SIZE];

__attribute__((section (".AXI_SRAM"))) uint8_t USB_RxBuf[2][USB_BUFF_SIZE];

/* Static Fun -------------------------------------------------------------- */

/* Functions --------------------------------------------------------------- */
/**
 * @brief  USB发送数据
 * @param  Data: 数据指针
 * @param  Len: 数据长度
 * @return 无
 * @note   无
 */
void USB_Transmit_Data(uint8_t* Data, uint16_t Len)
{
    /* 发送数据 */
    CDC_Transmit_HS(Data,Len);
}

/* Private functions ------------------------------------------------------- */

/* Interrupt functions ----------------------------------------------------- */
/**
 * @brief  USB接收回调函数
 * @param  Len: 接收数据长度
 * @return 无
 * @note   使用双缓冲机制
 */
void USB_ReceiveCallback(uint16_t Len)
{
    uint8_t *pDataBuf;
    
    pDataBuf = USB_Curruct_Buf;  // 这个缓冲区刚接收完数据
    
    if(USB_Curruct_Buf == USB_RxBuf[0]) {
        USB_Curruct_Buf = USB_RxBuf[1];
    } else {
        USB_Curruct_Buf = USB_RxBuf[0];
    }
    
    USBD_CDC_SetRxBuffer(&hUsbDeviceHS, USB_Curruct_Buf);
    USBD_CDC_ReceivePacket(&hUsbDeviceHS);
    
    if (pDataBuf != NULL) {
        PC_Info_Update(pDataBuf, &PC_RxInfo); 
    }
}

/* ------------------------------------------------------------------------- */
