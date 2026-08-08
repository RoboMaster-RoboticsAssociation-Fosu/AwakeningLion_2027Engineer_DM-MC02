#include "PS2_Device.h"
#include "FreeRTOS.h"
#include "task.h"

PS2_Info_t PS2_Info = {
    .Set = {
        .DI_PORT = GPIOE,
        .DI_PIN = GPIO_PIN_13,
        .CMD_PORT = GPIOE,
        .CMD_PIN = GPIO_PIN_9,
        .CS_PORT = GPIOA,
        .CS_PIN = GPIO_PIN_2,
        .CLK_PORT = GPIOA,
        .CLK_PIN = GPIO_PIN_0,
    },
    .PS2_cmnd = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    .PS2_buff = {0},
};

#define DI()    InPort(DI_PORT(&PS2_Info), DI_PIN(&PS2_Info))

#define CLK_H() OutPort_H(CLK_PORT(&PS2_Info), CLK_PIN(&PS2_Info))
#define CLK_L() OutPort_L(CLK_PORT(&PS2_Info), CLK_PIN(&PS2_Info))

#define CMD_H() OutPort_H(CMD_PORT(&PS2_Info), CMD_PIN(&PS2_Info))
#define CMD_L() OutPort_L(CMD_PORT(&PS2_Info), CMD_PIN(&PS2_Info))

#define CS_H()  OutPort_H(CS_PORT(&PS2_Info), CS_PIN(&PS2_Info))
#define CS_L()  OutPort_L(CS_PORT(&PS2_Info), CS_PIN(&PS2_Info)) 

void PS2_Init(void)
{
    CS_H();
    CLK_L();
    CMD_H();
}

static uint8_t PS2_ReadWriteData(uint8_t data)
{
    uint8_t ref, res = 0;
	for(ref = 0x01; ref > 0x00; ref <<= 1)
	{
		CLK_L();
		if(ref&data)
			CMD_H();
		else
			CMD_L();	
		
		DWT_Delay_us(16);
		
		CLK_H();
		if(DI())
			res |= ref; 
		
		DWT_Delay_us(16);		
	}
	CMD_H();
	//返回读出数据
    return res;	
}

/**
  * @简  述  PS2获取按键及摇杆数值。
  * @参  数  *JoystickStruct 手柄键值结构体
  * @返回值  无
  */
void PS2_ScanKey(PS2_Info_t *PS2)
{
	uint8_t i;

	taskENTER_CRITICAL();

	CS_L();
	for(i=0; i<9; i++)
	{
		PS2->PS2_buff[i] = PS2_ReadWriteData(PS2->PS2_cmnd[i]);
		DWT_Delay_us(16);
	}
	CS_H();

	taskEXIT_CRITICAL();

	PS2->Data.mode = PS2->PS2_buff[1];
	PS2->Data.btn1 = ~PS2->PS2_buff[3];
	PS2->Data.btn2 = ~PS2->PS2_buff[4];
	if(PS2->Data.mode == PS2_MODE_RED)
	{
		PS2->Data.RJoy_LR = (PS2->PS2_buff[5] - PS2_LR_MID);
		PS2->Data.RJoy_UD = -(PS2->PS2_buff[6] - PS2_UP_MID);
		PS2->Data.LJoy_LR = (PS2->PS2_buff[7] - PS2_LR_MID);
		PS2->Data.LJoy_UD = -(PS2->PS2_buff[8] - PS2_UP_MID);
	}
	else{
		PS2->Data.RJoy_LR = PS2->PS2_buff[5];
		PS2->Data.RJoy_UD = PS2->PS2_buff[6];
		PS2->Data.LJoy_LR = PS2->PS2_buff[7];
		PS2->Data.LJoy_UD = PS2->PS2_buff[8];
	}
}
