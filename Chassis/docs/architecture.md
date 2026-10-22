# 底盘工程

本目录是独立的 Chassis 固件工程，直接编译自己的任务和驱动，不再通过板型宏切换。
打开 `MDK-ARM/CtrlBoard-H7_IMU.uvprojx` 使用 ARM Compiler 6；EIDE 配置在 `.eide/eide.yml`。
本工程业务源码、硬件配置和构建输出位于本目录；通用库通过相对路径引用 ../Shared，不引用另一板的业务代码。

## 当前控制链

USART10 遥控快照 → InputTask 发布输出许可 → ChassisTask → 所选电机闭环 → 对应 CAN。

当前只接调试器逐台测试：CAN1 四台麦轮 M3508，CAN2 四台腿部 DM-J4340，CAN3 两台履带 M3508。
六台 M3508 沿用速度 PID 输出 C620 电流；J4340 使用 H7 位置 PD 输出力矩，MIT 报文 kp、kd 为零。
调试器通过 `g_chassis.command` 选择电机与目标，每次只运行一台；启动、锁定和切换电机会清除测试 run。
J4340 确认使能后以当前绝对角度初始化目标；原腿确认失能前不启动下一台电机。
本版不增加软件机械行程限位、姿态预设或轮履联动。现有驱动协议编码量程保持原义。
完整电机编号、调参单位、停止确认与测试步骤见 [motor_bringup.md](motor_bringup.md)。

固定任务：InputTask、ChassisTask、InsTask、ServiceTask。InputTask 周期 1 ms，ServiceTask 周期 10 ms。
麦轮解算、遥控摇杆到整车速度的映射和四轮联动暂不接入，也不提供模式切换开关。调试器直接给所选 M3508 目标转速，遥控器只提供原有总输出许可。
逐台模式只检查所选电机的反馈；反馈超时、异常值或发送失败会撤销测试 run。
电机反馈检查在同一临界区内取当前时间并读取快照，避免 CAN 中断更新下一毫秒反馈后发生无符号时间差回绕；原 50 ms 超时边界不变。
本地 BMI088/Mahony 和原绿灯指示仍由原任务运行，不新增 RTOS 任务。腿状态管理每 20 ms 请求一次，控制仍由原 1 ms 通知驱动。

## 板间通信

USART10 使用现有循环 DMA 和流帧协议。接收完整遥控快照，发送本地姿态快照；不启动 UART5 DBUS 接收模块。
遥控负载仍为 44 字节，姿态负载仍为 149 字节，字段、小端编码、CRC 和 5 ms 发送周期保持一致。
`ins_types.h` 只定义完整姿态快照，避免接收端依赖本地 IMU 驱动。
帧格式、负载编解码和数据定义统一在 Shared/Protocol/BoardLink 维护；两端只保留自己的收发职责。

## 保留的行为

上电锁定、SW2 解锁/锁定、遥控许可与任务通知门控保持原实现。测试模式默认不选电机且 run=0；遥控恢复后还需手动重新设置 run。
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
