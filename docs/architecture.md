# 2027 工程机器人控制架构

## 1. 决策状态

本文记录 `new_robot_code` 向本工程逐步再实现时采用的架构基线。它不是源代码目录的整体复制方案。

- 采用“单控制 owner + 独立算法系统 + 集中 MotorIo”的混合结构。
- `RobotControlTask` 是机器人控制生命周期的唯一 owner，基准周期固定为 **1 ms（1 kHz）**。
- `RobotControlTask` 内不再为底盘、机械臂或电机通信设置 3 ms、5 ms、6 ms 等额外软件分频周期。
- 当前 `configTICK_RATE_HZ = 1000`，实现时使用绝对节拍等待，使每次唤醒基于上一目标时刻，而不是本次执行结束时刻。
- 云台与发射不属于当前目标系统，不保留对应任务、控制状态和电机输出链路。

## 2. 设计原则

1. 算法与硬件解耦：PID、运动学、轨迹、重力补偿只处理带单位的输入与输出，不包含 HAL、FreeRTOS 或厂商协议。
2. 总线只有一个发送 owner：CAN1、CAN2 和 USART10 的电机收发都归 `MotorIo`，应用系统不能直接发送电机帧。
3. 中断只搬运数据：ISR/DMA 回调只接收、记录和唤醒，不调用底盘、机械臂等应用控制函数。
4. 连续量使用最新值快照：遥控器、IMU、模式和电机反馈允许覆盖旧样本；只有不能丢失的离散动作才使用队列。
5. 使用明确的执行器角色和物理单位，不建立包含所有厂商字段、字符串查找和运行时注册的通用 `Motor` 大对象。
6. 主控制链必须在一个任务入口中可见：输入快照 → 安全门控 → 目标/状态机 → 控制计算 → 电机输出 → 周期等待。
7. 迁移按可构建、可观测、可回退的小步进行，每一步只引入当时真实需要的接口。

## 3. 硬件通信归属

| 物理端口 | 电机协议 | 唯一 owner | 目标配置 |
| --- | --- | --- | --- |
| CAN1 | DJI、P1010B | `MotorIo` | 经典 CAN，1 Mbit/s |
| CAN2 | Damiao | `MotorIo` | 经典 CAN，1 Mbit/s |
| USART10 | GO8010 | `MotorIo` | 4 Mbit/s，DMA 接收与非阻塞发送 |

协议解析仍分别放在 `driver/dji`、`driver/p1010b`、`driver/damiao` 和 `driver/go8010`。驱动负责帧格式、量程换算和协议生命周期；它们不知道底盘、机械臂和遥控模式。

`MotorIo` 隐藏以下跨协议复杂度：

- CAN1 的 DJI 分组发送和 P1010B 帧调度；
- CAN2 的 Damiao MIT 编码、使能与反馈解析；
- USART10 的 GO8010 字节流缓存、帧头重同步和发送时序；
- 反馈时间戳、在线状态和安全零输出；
- 一个控制周期内三条物理总线的接收刷新与发送提交。

## 4. 模块与依赖方向

```text
FreeRTOS / HAL
    │
    ├── InputTask / InsTask ──发布最新值──┐
    │                                    │
    └── RobotControlTask (1 ms owner)    │
            │                            │
            ├── RobotSystem              │
            │     ├── ChassisSystem      │
            │     └── ArmSystem          │
            │                            │
            └── MotorIo ─────────────────┘
                   ├── DJI + P1010B / CAN1
                   ├── Damiao / CAN2
                   └── GO8010 / USART10
```

依赖只能向下：

- task 层负责调度和快照，不包含厂商协议；
- system 层负责机器人策略和控制算法，不调用 HAL；
- `MotorIo` 是 system 与电机驱动之间的 seam；
- driver 层负责一种协议；
- BSP/HAL adapter 负责具体 FDCAN、UART、DMA 和时间源。

`RobotSystem` 只协调底盘与机械臂的共享模式和互锁。`ChassisSystem` 与 `ArmSystem` 保持独立实现和状态，合并的是调度 owner，不是把两套算法写进一个大函数。

## 5. 1 ms 控制时序

`RobotControlTask` 只保留一个 1 ms 外层节拍，不创建独立 MCT 通信任务，也不在 system 内再次等待。

```c
void RobotControlTask(void const *argument)
{
    TickType_t last_wake_tick = xTaskGetTickCount();

    MotorIo_Init();
    RobotSystem_Init();

    for (;;)
    {
        MotorIo_BeginCycle(&feedback);
        InputSnapshot_Copy(&input);
        SensorSnapshot_Copy(&sensor);

        operational = RobotSafety_Evaluate(&input, &sensor, &feedback);
        RobotSystem_Step(&input, &sensor, &feedback, operational, &command);
        MotorIo_EndCycle(&command, operational);

        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(1U));
    }
}
```

约束：

- `BeginCycle` 在本周期开始时消费已到达的 CAN/UART 数据并生成一致反馈快照。
- safety 失效时，各 system 复位控制器状态，`EndCycle` 必须提交安全零输出。
- `EndCycle` 是所有电机命令的唯一提交点。
- 任何 system 函数都不得阻塞、调用 `osDelay` 或自行维护另一个控制周期。
- 如果单周期最坏执行时间接近 1 ms，先测量并削减工作量；不能通过静默降低部分 system 的运行频率掩盖超时。

## 6. 最小接口草案

```c
MotorIoStatus_e MotorIo_Init(void);
MotorIoStatus_e MotorIo_BeginCycle(RobotActuatorFeedback_t *feedback);
MotorIoStatus_e MotorIo_EndCycle(const RobotActuatorCommand_t *command,
                                 bool operational);
void MotorIo_CopyDiagnostic(MotorIoDiagnostic_t *diagnostic);
```

`RobotActuatorFeedback_t` 和 `RobotActuatorCommand_t` 使用固定的机器人执行器角色字段。每个字段名或注释必须给出实际单位，例如 `rad`、`rad_s`、`rpm`、`Nm` 或 DJI 原始电流指令。不同协议不共享含义模糊的 `target`、`value`、`output` 字段。

上述接口是设计方向，不要求第一阶段一次创建所有类型。只有首个真实执行器链路需要时才落代码，并用第二条链路验证接口是否确实稳定。

## 7. 并发与数据传递

- RC 与 INS 各自采集并发布最新快照；`RobotControlTask` 每周期复制一次，计算期间不再读取可变全局量。
- CAN FIFO 中断应循环取尽当前 FIFO 帧，交给 `MotorIo_OnCanRxFromISR` 或等价的固定容量接收缓存。
- USART10 DMA/空闲中断只提交本次收到的字节片段。GO8010 parser 在 owner 上下文按字节消费，并基于帧头重同步，不假设一次 DMA 事件必然等于一整帧。
- 模式切换、回零开始等不可丢失的离散命令才进入事件队列；摇杆、姿态、速度和电机反馈不排队。
- task 之间不互相调用 `Submit...()` 形成网状依赖。

## 8. 安全语义

最小安全门控只使用已经存在且有真实消费者的条件：

- 遥控器是否就绪；
- 当前参与闭环的执行器反馈是否有效且新鲜；
- `MotorIo` 是否完成必要初始化；
- system 控制计算是否成功。

任一条件失败时，本周期输出安全零命令并复位相关 PID。暂不预建复杂故障树、自动恢复状态机或多层健康管理；当具体电机协议的 late-power-on 等问题在接入阶段出现时，再把恢复策略放进拥有该协议生命周期的 driver/`MotorIo` 实现。

## 9. 分阶段再实现

1. **基线清理**：删除已取消的云台、发射和 Stepper 链路，保留可构建的 INS、RC、底盘基线与 USART10 外设资源。
2. **MotorIo 骨架**：先接入 CAN1 DJI 轮组，建立反馈快照和集中发送；验证 1 ms 循环及零输出。
3. **底盘再实现**：把运动学和 PID 移入 `ChassisSystem`，删除旧 `chassis_task` 对 HAL、遥控全局量和中断回调的直接依赖。
4. **其他协议接入**：依次加入 CAN1 P1010B、CAN2 Damiao、USART10 GO8010；每次只接一条闭环并验证单位、反馈新鲜度与安全失能。
5. **机械臂再实现**：将机构状态机和控制算法移入 `ArmSystem`，由 `RobotSystem` 处理与底盘共享的模式和互锁。
6. **观测与诊断**：控制链稳定后再加入执行时间、总线错误和反馈年龄观测，不改变 owner 关系。

## 10. 明确不采用的结构

- 不采用“算法对象永久绑定一个厂商电机对象并直接发总线”的传统结构；它会把控制策略、协议和物理端口耦合在一起。
- 不照搬 `new_robot_code` 当前的通用电机注册表、字符串查找、厂商 union 和 task 间直接 submit 网络。
- 不设置独立 `mct`、`chassis_task`、`arm_task` 的异步控制周期；本工程用一个 1 ms owner 消除跨周期相位漂移。
- 不让 BSP 中断回调直接调用 APP/system。
- 不为尚未接入的执行器、故障或平台预留空接口。

## 11. 参考设计

- [PX4 uORB](https://docs.px4.io/main/en/middleware/uorb)：连续状态默认读取最新样本，只有确实不能丢失的数据才配置队列。
- [PX4 Device Drivers](https://docs.px4.io/main/en/middleware/drivers)：驱动向上发布标准数据，不承载飞行控制策略。
- [ArduPilot Threading](https://ardupilot.org/dev/docs/learning-ardupilot-threading.html)：调度表明确任务频率和最坏执行时间预算。
- [ArduPilot Sensor Drivers](https://ardupilot.org/dev/docs/code-overview-sensor-drivers.html)：backend 采集硬件，frontend 提供一致的最新状态。
- [ros2_control Controller Manager](https://control.ros.org/rolling/doc/getting_started/getting_started.html)：单周期采用 read → update → write，硬件接口与控制器分离。
- [Zephyr Device Driver Model](https://docs.zephyrproject.org/latest/kernel/drivers/index.html)：设备配置、运行状态和按类别组织的驱动接口分离。
- [Betaflight 4.3 Tuning Notes](https://betaflight.com/docs/wiki/tuning/4-3-Tuning-Notes)：高频控制链需要明确调度优先级和执行时间预算。
