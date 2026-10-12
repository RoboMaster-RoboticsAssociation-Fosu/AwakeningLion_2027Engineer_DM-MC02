# 从臂工程

本目录是独立的 SlaveArm 固件工程，直接编译自己的任务和驱动，不再通过板型宏切换。
打开 `MDK-ARM/CtrlBoard-H7_IMU.uvprojx` 使用 ARM Compiler 6；EIDE 配置在 `.eide/eide.yml`。
本工程源码、头文件和构建输出都位于本目录，不引用另一工程的文件。

## 当前控制链

UART5 DBUS → RcSystem → InputTask → ArmTask → CAN1/CAN2 达妙、CAN3 RS05。

固定任务：InputTask、ArmTask、ServiceTask。InputTask 周期 1 ms，ServiceTask 周期 10 ms。
从臂保持六台达妙、两台 RS05 的现有 MIT 控制、Keil 目标调试和电机故障灯。

## 板间通信

USART10 使用现有循环 DMA 和流帧协议。发送完整遥控快照，接收底盘姿态快照；INS 仅是接收数据，不创建本地采样任务。
遥控负载仍为 44 字节，姿态负载仍为 149 字节，字段、小端编码、CRC 和 5 ms 发送周期保持一致。
`ins_types.h` 只定义完整姿态快照，避免接收端依赖本地 IMU 驱动。
两工程各自保存协议代码；协议变更需要同时核对两端兼容性。

## 保留的行为

上电锁定、SW2 解锁/锁定、遥控失联冻结目标及原电机失能/反馈故障处理均沿用原分支。
底盘板间超时和重连策略尚未加入，本次拆分不改变这些语义。
硬件时钟、GPIO、DMA、UART/SPI/USB 配置及三路 CAN 初始化沿用现有工程。
控制任务和入口位于 CubeMX USER CODE 保留区；重生成后执行现有 FreeRTOS AC6 端口修复脚本。
UART7 仅承担 VOFA TX；`vofa_system.c` 中的观测视图宏只选择通道，不切换臂/底盘固件。

## 清理范围

已移除另一板控制任务、板型选择头文件及当前调用链未使用的旧驱动、旧算法、自定义控制器和串口监视工具。
公共 HAL、CMSIS、FreeRTOS、USB 中间件和仍由 USB 回调使用的 PC_Comm 保留。
MasterArm 目录不属于本次拆分。
