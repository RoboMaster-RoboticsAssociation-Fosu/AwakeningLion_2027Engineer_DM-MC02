#ifndef __PS2_DEVICE_H__
#define __PS2_DEVICE_H__

#include "cmsis_os.h"
#include "usart.h"
#include "gpio.h"
#include "bsp_dwt.h"

#define CLK_PORT(INFO)      (((PS2_Info_Typedef *)(INFO))->Set.CLK_PORT)
#define CLK_PIN(INFO)       (((PS2_Info_Typedef *)(INFO))->Set.CLK_PIN)
#define CMD_PORT(INFO)      (((PS2_Info_Typedef *)(INFO))->Set.CMD_PORT)
#define CMD_PIN(INFO)       (((PS2_Info_Typedef *)(INFO))->Set.CMD_PIN)
#define DI_PORT(INFO)       (((PS2_Info_Typedef *)(INFO))->Set.DI_PORT)
#define DI_PIN(INFO)        (((PS2_Info_Typedef *)(INFO))->Set.DI_PIN)
#define CS_PORT(INFO)       (((PS2_Info_Typedef *)(INFO))->Set.CS_PORT)
#define CS_PIN(INFO)        (((PS2_Info_Typedef *)(INFO))->Set.CS_PIN)

#define InPort(PORT, PIN)       HAL_GPIO_ReadPin(PORT, PIN)
#define OutPort_H(PORT, PIN)    HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_SET)
#define OutPort_L(PORT, PIN)    HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_RESET)

#define PS2_MODE_RED    0x73
#define PS2_MODE_GREEN  0x41

#define PS2_UP_MID   0x7F
#define PS2_LR_MID   0x80

#define PS2_RC_MAX   127
#define PS2_RC_MIN   -128

#define PS2_LRC_LR(x)       (((PS2_Info_Typedef *)(x))->Data.LJoy_LR)
#define PS2_LRC_UD(x)       (((PS2_Info_Typedef *)(x))->Data.LJoy_UD)
#define PS2_RRC_LR(x)       (((PS2_Info_Typedef *)(x))->Data.RJoy_LR)
#define PS2_RRC_UD(x)       (((PS2_Info_Typedef *)(x))->Data.RJoy_UD)

typedef struct 
{
    uint8_t mode;		    /* 手柄的工作模式 */

    union {         
        /* btn1 位域解析 (PS2_buff[3] 取反后) */                    
        struct {                            
            uint8_t PS2_SELECT : 1;         /* B0: SELECT   */
            uint8_t PS2_L3     : 1;         /* B1: 左摇杆按下  */
            uint8_t PS2_R3     : 1;         /* B2: 右摇杆按下  */
            uint8_t PS2_START  : 1;         /* B3: START    */
            uint8_t PS2_UP     : 1;         /* B4: 方向键上   */
            uint8_t PS2_RIGHT  : 1;         /* B5: 方向键右   */
            uint8_t PS2_DOWN   : 1;         /* B6: 方向键下   */
            uint8_t PS2_LEFT   : 1;         /* B7: 方向键左   */ 
        };
        uint8_t btn1;                       /* 整字节访问 btn1 */
    };
    union { 
        /* btn2 位域解析 (PS2_buff[4] 取反后) */                    
        struct {                            
            uint8_t PS2_L2     : 1;         /* B0: L2       */
            uint8_t PS2_R2     : 1;         /* B1: R2       */
            uint8_t PS2_L1     : 1;         /* B2: L1       */
            uint8_t PS2_R1     : 1;         /* B3: R1       */
            uint8_t PS2_TRI    : 1;         /* B4: 三角      */
            uint8_t PS2_CIRC   : 1;         /* B5: 圆圈      */
            uint8_t PS2_CROSS  : 1;         /* B6: 叉叉      */
            uint8_t PS2_SQUA   : 1;         /* B7: 方块      */
        };
        uint8_t btn2;                       /* 整字节访问 btn2 */
    };
    int16_t RJoy_LR;        /* 右边摇杆  0x00 = 左    0xff = 右   */

    int16_t RJoy_UD;        /* 右边摇杆  0x00 = 上    0xff = 下   */

    int16_t LJoy_LR;        /* 左边摇杆  0x00 = 左    0xff = 右   */

    int16_t LJoy_UD;        /* 左边摇杆  0x00 = 上    0xff = 下   */
} PS2_Data_Typedef;

typedef struct {
    GPIO_TypeDef *DI_PORT;      // 接收数据引脚
	uint16_t DI_PIN;
    GPIO_TypeDef *CMD_PORT;     // 发送命令引脚
	uint16_t CMD_PIN;
    GPIO_TypeDef *CS_PORT;      // 使能引脚,通信期间低电平
	uint16_t CS_PIN;
	GPIO_TypeDef *CLK_PORT;     // 时钟引脚
	uint16_t CLK_PIN;
} PS2_Set_Typedef;

typedef struct {
    PS2_Set_Typedef Set;
    PS2_Data_Typedef Data;
    const uint8_t PS2_cmnd[9];   //请求获取数据命令                         
    uint8_t PS2_buff[9];         //接收的数据
} PS2_Info_Typedef;

extern PS2_Info_Typedef PS2_Info;

void PS2_Init(void);

void PS2_ScanKey(PS2_Info_Typedef *PS2);






#endif
