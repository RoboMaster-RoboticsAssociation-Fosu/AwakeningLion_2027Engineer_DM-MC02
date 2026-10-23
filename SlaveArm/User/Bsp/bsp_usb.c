/** @file bsp_usb.c @brief USB CDC 持久发送缓冲与忙状态处理，不承担业务解析。 */
#include "bsp_usb.h"

#include "FreeRTOS.h"
#include "task.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceHS;
extern uint8_t UserTxBufferHS[APP_TX_DATA_SIZE];

bool USB_Transmit_Data(const uint8_t *data, uint16_t length)
{
    USBD_CDC_HandleTypeDef *cdc;
    bool submitted = false;

    if ((data == NULL) || (length == 0U) || (length > APP_TX_DATA_SIZE))
    {
        return false;
    }
    /* 检查、复制、提交在同一临界区完成；USB 复位 ISR 不会使 cdc 指针失效。 */
    taskENTER_CRITICAL();
    cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
    if ((hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED) &&
        (cdc != NULL) && (cdc->TxState == 0U))
    {
        memcpy(UserTxBufferHS, data, length);
        submitted = CDC_Transmit_HS(UserTxBufferHS, length) == USBD_OK;
    }
    taskEXIT_CRITICAL();
    return submitted;
}
