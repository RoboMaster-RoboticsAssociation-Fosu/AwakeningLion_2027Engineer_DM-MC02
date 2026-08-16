# 2027 工程机器人控制架构

## 1. 当前决策

本文记录 `new_robot_code` 向本工程逐步再实现时采用的当前架构。旧工程只作为功能、参数和协议参考，不整体复制目录、任务或通用电机模型。

- 不建立统一大电机抽象层，底盘和机械臂直接使用各自厂商驱动。
- 不再保留 `RobotControlTask`、`ChassisSystem` 和 `ArmSystem`。
- `InputTask` 处理遥控输入、整车锁定状态，并通过 CMSIS-RTOS2 线程标志唤醒 `ChassisTask` 与 `ArmTask`。
- `ChassisTask` 和 `ArmTask` 分别拥有各自的控制生命周期、跨周期状态和电机发送路径。
- 上电默认锁定；SW2 进入 UP 的边沿解锁，进入 DOWN 的边沿锁定，同周期同时出现时锁定优先。
- 遥控器未就绪时，即使锁定状态已经解开，也不允许底盘或机械臂输出。
- 云台、发射和 P1010B 后腿电机不属于当前复现范围；机械臂的
  `roll3/GM6020` 与 `pitch2/GO8010` 已进入当前装配范围。

## 2. 任务模型

| 任务 | 当前调度方式 | 主要职责 |
| --- | --- | --- |
| `InsTask` | 1 ms 周期 | IMU 采样与姿态解算 |
| `InputTask` | 1 ms 周期 | 推进 `RcSystem`、读取 SW2 边沿、维护锁定状态、通知两个控制任务 |
| `ChassisTask` | 输入通知驱动，2 tick 安全超时 | 遥控快照、反馈检查、麦轮解算、四轮 PID、CAN1 电流发送 |
| `ArmTask` | 输入通知驱动，2 tick 等待超时后保持状态 | 机械臂装配、安全门控、CAN2 Damiao/GM6020 与 USART10 GO8010 安全命令 |
| `ServiceTask` | 10 ms 周期 | LED 状态、VOFA 调参视图上传和 USB 后台服务 |

`InputTask` 每次完成输入处理后，向两个控制任务发布当前输出许可。通知只负责唤醒。`ChassisTask` 在等待错误或连续两个输入周期未收到通知时按失能处理；`ArmTask` 仅在收到有效通知时更新使能状态，等待超时时保持机械臂上一状态，上电初值仍为失能。

两个控制任务相互独立：一个任务发生正常阻塞不会直接阻塞另一个任务。任务拆分不能防止高优先级死循环导致的 CPU 饥饿，因此控制路径仍禁止无限循环、忙等待和带延时的驱动发送。

## 3. 实时链路

```text
UART5 DBUS / DMA
        │
        ▼
    RcSystem
        │
        ▼
InputTask（1 ms）
  SW2边沿 → locked
  enabled = !locked && RcSystem_IsReady()
        │
        ├──线程标志──► ChassisTask ─► DJI M3508 ─► CAN1
        │
        └──线程标志──► ArmTask ─┬──► Damiao ─────► CAN2
                                ├──► DJI GM6020 ─► CAN2
                                └──► GO8010 ─────► USART10
```

底盘任务保持一屏可见的主链：

```text
读取遥控快照 → 检查四轮反馈 → 安全门控 → 麦轮解算 → 速度 PID → CAN1 电流帧
```

机械臂当前只实现安全基线：

```text
读取输出许可 → 恢复 USART10 接收 → 下发 GM6020/GO8010 安全输出
             → 每次最多发送一个 Damiao 使能/失能帧 → 记录返回值
```

## 4. 硬件通信归属

| 物理端口 | 协议/设备 | 发送所有者 | 配置 |
| --- | --- | --- | --- |
| CAN1 | DJI M3508 底盘 | `ChassisTask` | 经典 CAN，1 Mbit/s |
| CAN2 | Damiao 机械臂与 roll3 GM6020 | `ArmTask` | 经典 CAN，1 Mbit/s；按反馈 ID 分发 |
| CAN3 | 保留 | 暂无业务发送所有者 | 保持启用 |
| USART10 | GO8010 | 后续归 `ArmTask` | 4 Mbit/s，DMA 接收与非阻塞发送 |
| UART7 | VOFA+ JustFloat | `VofaSystem` / `ServiceTask` | 115200 bit/s，中断发送，忙时丢弃本帧 |

VOFA 第一版只保留直接调参量，不上传视图 ID、在线标志或统计值：

| 视图 | 值 | 通道顺序 |
| --- | --- | --- |
| `ARM_JOINT_ANGLES` | 0 | 8 个机构关节角，顺序与 `ArmJoint_e` 一致，单位 rad |
| `CHASSIS_SPEED_PID` | 1 | 每个车轮的目标 rpm、反馈 rpm、C620 电流指令值依次相邻排列，车轮顺序与 `ChassisWheel_e` 一致 |
| `REMOTE_INPUT` | 2 | CH0、CH1、CH2、CH3、iw |
| `INS_ATTITUDE` | 3 | Roll、Pitch、Yaw、Gyro XYZ、Accel XYZ |

默认为 `ARM_JOINT_ANGLES`。调试器可改写
`g_vofa_system.requested_view` 请求切换；`active_view` 和
`previous_view` 用于确认当前视图并返回上一视图。

FDCAN/UART/DMA 中断只接收并更新对应任务拥有的反馈实例，不进行机构控制计算，也不发送新的控制目标。

## 5. APP 文件基线

```text
User/APP/
├── input_task.c
├── input_task.h
├── chassis_task.c
├── chassis_task.h
├── arm_task.c
├── arm_task.h
├── ins_task.c
├── ins_task.h
├── rc_system.c
└── rc_system.h
```

- `input_task` 是输入任务，`rc_system` 是 UART5 DBUS 协议和 DMA 生命周期所有者，二者不合并。
- `chassis_task` 直接持有四个 M3508 实例和四个速度 PID。
- `arm_task` 直接持有六个 Damiao、一个 roll3 GM6020 与一个 pitch2 GO8010
  实例，不增加通用 `Motor_t` 或厂商联合体。
- CAN1 回调只投递底盘 M3508 反馈；CAN2 回调按 ID 投递 Damiao 与
  roll3 GM6020 反馈；USART10 回调只投递完整 GO8010 反馈帧。

## 6. 安全语义

以下任一条件失败时，控制任务必须发送对应的安全命令并继续运行：

- 整车未解锁；
- 遥控器未就绪；
- 输入任务通知超时；
- 当前参与闭环的电机反馈无效或超时；
- PID 或控制计算失败；
- CAN 发送入队失败时，不得等待队列，而是在后续周期继续处理。

底盘失能时复位速度 PID、清零轮速目标并发送零电流。机械臂失能时逐个发送
Damiao 失能帧、给 roll3 发送零电流并把 pitch2 切到 LOCK。当前每次任务运行
最多发送一帧 Damiao 状态命令，所有发送结果均保留供调试观察。

## 7. 后续再实现顺序

1. 实机验证 `InputTask → ChassisTask/ArmTask` 通知、默认锁定和遥控失联门控。
2. 验证 CAN1 四轮反馈、麦轮方向、原地自旋和速度 PID。
3. 实机验证 roll3 GM6020 的 CAN2 ID 5 反馈和 pitch2 GO8010 的 USART10
   ID 1 收发、方向与零位。
4. 完成 `ArmTask` 的机构状态、目标生成、运动学和补偿；两个 P1010B
   后腿电机继续跳过。
5. 控制链稳定后再加入 DWT 最坏执行时间和栈余量观测。

每一阶段都必须能够单独构建、烧录和回退；未通过当前阶段验证前不进入下一协议或机构。

## 8. 明确不采用

- 不恢复 `RobotControlTask`、`ChassisSystem` 或 `ArmSystem`；
- 不建立统一 `MotorIo`、厂商 union、字符串查找或运行时电机注册表；
- 不让 BSP 中断执行机构控制；
- 不在控制任务中加入阻塞式等待、电机应答轮询或非必要延时；
- 不把 USB、VOFA 和日志发送放入实时控制任务。
