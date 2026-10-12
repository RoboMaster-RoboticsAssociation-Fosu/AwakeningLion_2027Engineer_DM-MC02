# 公共源码库

SlaveArm、Chassis 和后续 MasterArm 的公共源码只在本目录维护。
各固件工程通过相对路径引用需要的源文件和头文件，分别编译、分别保存输出。
本目录没有板型选择宏；需要哪套固件，直接打开对应工程。

## 目录职责

| 目录 | 内容 |
| --- | --- |
| Algorithm | CRC、PID、EKF、Kalman、Mahony、FIFO 等可复用算法 |
| Controller、Lib | 恢复的控制工具和通用工具库 |
| Devices | 电机、传感器、遥控、按钮和 PC 通信驱动 |
| Protocol | VOFA、调试显示、自定义控制器协议、板间协议及姿态数据定义 |
| Bsp | 共用 DWT 时间工具、恢复的句柄式 PWM 工具 |
| ThirdParty | ARM Compiler 6 的 FreeRTOS Cortex-M 端口 |
| Drivers | 共用 STM32 HAL 和 CMSIS 源码、头文件及 DSP 库 |
| Middlewares | 共用 FreeRTOS 内核和 USB 设备协议栈 |
| tools | 共用 CubeMX 工程修复脚本和恢复的串口监视工具 |

原有库中的接口和实现保持原版本。库存在不代表已经加入某个固件工程的编译。
暂未使用的库保留在这里，按实际需要再加入对应 Keil/EIDE 工程。
部分旧驱动依赖工程提供的 main.h、GPIO、UART 或定时器配置，调用前仍需核对硬件绑定。
例如旧步进电机驱动使用 USART10，而当前两板的 USART10 属于板间链路，不能直接同时启用。

## 当前引用

| 工程 | 当前主要公共库 |
| --- | --- |
| SlaveArm | DM、RS05、Remote_Control、PC_Comm、CRC、DbgDisp、VOFA、BoardLink、DWT、HAL/CMSIS、FreeRTOS、USB |
| Chassis | DJI、BMI088、PC_Comm、PID、Mahony、CRC、VOFA、BoardLink、DWT、HAL/CMSIS、FreeRTOS、USB |
| MasterArm | 当前仅保留目录说明；完成业务设计后引用需要的公共库 |

BoardLink 的帧格式和两种负载编解码各保存一份。Remote_Info_t 和 INS_t 也使用共同定义。
接收/发送哪种消息、如何更新快照和如何通知控制任务，仍由各板的 board_link.c 和 input_task.c 决定。
从臂机械臂专用算法恢复在 SlaveArm/User/Algorithm/Arm，底盘腿部 VMC 算法恢复在 Chassis/User/Algorithm/VMC；当前均未加入固件编译。

## 工程保留项

各板独立保留 Core、USB_DEVICE、.ioc、任务、CAN/UART 回调、状态灯策略、启动文件、链接配置和构建输出。
stm32h7xx_hal_conf.h、FreeRTOSConfig.h、usbd_conf.h、main.h 等配置从当前工程读取，不放入共享库。
Shared 中的源码修正后，需要重新验证所有实际使用该模块的工程。

## CubeMX 重生成

各工程的 tools/cubemx/post_generate.cmd 调用这里的共用修复脚本。
脚本保持 ARM Compiler 6 端口，并把生成工程的公共源码与头文件引用重新指向 Shared。
脚本只更新工程引用，不自动覆盖 Shared；CubeMX 新生成的本地 Drivers/Middlewares 也不会替代共享版本。
升级 STM32 或 RTOS 库时应单独核对版本，统一更新 Shared，再分别编译两板。
当前验证涵盖工程修复脚本和已有固件构建，尚未重新运行 CubeMX 图形界面的完整生成流程。
