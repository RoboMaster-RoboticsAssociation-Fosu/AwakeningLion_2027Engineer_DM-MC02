# 从臂工程

本目录是独立的 SlaveArm 固件工程，直接编译自己的任务和驱动，不再通过板型宏切换。
打开 `MDK-ARM/CtrlBoard-H7_IMU.uvprojx` 使用 ARM Compiler 6；EIDE 配置在 `.eide/eide.yml`。
本工程业务源码、硬件配置和构建输出位于本目录；通用库通过相对路径引用 ../Shared，不引用另一板的业务代码。

## 当前控制链

UART5 DBUS → RcSystem → InputTask → ArmTask → CAN1/CAN2 达妙、CAN3 RS05。

固定任务：InputTask、ArmTask、ServiceTask。InputTask 周期 1 ms，ServiceTask 周期 10 ms。
从臂保持六台达妙、两台 RS05 的现有 MIT 控制、Keil 目标调试和电机故障灯。

## 板间通信

USART10 使用现有循环 DMA 和流帧协议。发送完整遥控快照，接收底盘姿态快照；INS 仅是接收数据，不创建本地采样任务。
遥控负载仍为 44 字节，姿态负载仍为 149 字节，字段、小端编码、CRC 和 5 ms 发送周期保持一致。
`ins_types.h` 只定义完整姿态快照，避免接收端依赖本地 IMU 驱动。
帧格式、负载编解码和数据定义统一在 Shared/Protocol/BoardLink 维护；两端只保留自己的收发职责。

## 保留的行为

上电锁定、SW2 解锁/锁定、遥控失联冻结目标及原电机失能/反馈故障处理均沿用原分支。
底盘板间超时和重连策略尚未加入，本次拆分不改变这些语义。
硬件时钟、GPIO、DMA、UART/SPI/USB 配置及三路 CAN 初始化沿用现有工程。
控制任务和入口位于 CubeMX USER CODE 保留区；重生成后由本地入口执行 Shared 中的共用工程修复脚本，保留 AC6 端口和共享源码引用。
UART7 仅承担 VOFA TX；`vofa_system.c` 中的观测视图宏只选择通道，不切换臂/底盘固件。

## 公共库与保留源码

公共算法、设备驱动、协议、DWT、HAL/CMSIS、FreeRTOS 和 USB 库统一在 ../Shared 维护。
之前移除的通用库已从源码备份恢复，当前没有使用的库保留源码但不加入编译。
机械臂专用算法恢复在 SlaveArm/User/Algorithm/Arm，腿部 VMC 恢复在 Chassis/User/Algorithm/VMC。
板间收发、输入许可、控制任务和硬件回调仍由本工程直接实现，没有臂/底盘切换宏。
MasterArm 当前仅保留目录说明，尚无可编译业务工程。共享目录职责和维护方式见 ../Shared/README.md。
