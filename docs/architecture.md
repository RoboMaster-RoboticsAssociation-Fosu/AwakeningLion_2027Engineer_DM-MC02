# 2027 工程机器人控制架构

## 1. 决策状态

本文记录 `new_robot_code` 向本工程逐步再实现时采用的架构基线。旧工程只作为功能和协议参考，不整体复制目录、任务和通用电机模型。

- 采用“单控制 owner + 独立控制系统 + 直接使用厂商电机驱动”的结构。
- `RobotControlTask` 是机器人控制生命周期和电机发送的唯一 owner，基准周期固定为 **1 ms（1 kHz）**。
- `ChassisSystem` 与 `ArmSystem` 是独立控制模块，但不是独立 FreeRTOS 任务；二者由 `RobotControlTask` 在同一周期依次调用。
- 不建立统一 `MotorIo`、通用 `Motor_t`、厂商联合体、字符串查找或运行时电机注册表。
- 当前 `configTICK_RATE_HZ = 1000`，控制任务使用绝对节拍等待，不使用本次执行结束后再相对延时的方式。
- 云台与发射不属于当前目标系统，不保留对应任务、控制状态和电机输出链路。

## 2. 设计原则

1. 算法与硬件解耦：PID、运动学、轨迹和重力补偿只处理带单位的输入与输出，不包含 HAL、FreeRTOS 或厂商协议。
2. 驱动保持独立：DJI、P1010B、Damiao、GO8010 分别维护自己的协议类型、反馈、命令、初始化和在线状态，不强行统一成一种电机对象。
3. 发送 owner 唯一：只有 `RobotControlTask` 能调用电机驱动的发送接口；其他 task、system 和 ISR 都不能直接发送电机命令。
4. 中断只搬运数据：FDCAN/UART/DMA 回调只取走硬件数据并交给对应 BSP/driver 接收路径，不调用底盘、机械臂和安全策略。
5. 连续量使用最新快照：遥控器、IMU 和电机反馈允许覆盖旧样本；只有不能丢失的离散动作才使用队列。
6. 主控制链在一个任务入口中可见：输入快照 → 驱动反馈 → 安全门控 → 底盘/机械臂计算 → 厂商驱动发送 → 周期等待。
7. 使用明确的机构角色和物理单位；不同电机输出保持各自真实单位，不建立含义模糊的统一 `target`、`value` 或 `output`。
8. 按可构建、可观测、可回退的小步再实现；不为尚未接入的执行器、故障和平台预建接口。

## 3. 任务模型

最终保留四个常驻应用任务，其中前三个组成实时数据链，`ServiceTask` 只承担非实时工作。

| 任务 | 调度方式 | 主要职责 | 禁止事项 |
| --- | --- | --- | --- |
| `InsTask` | 1 ms 绝对节拍 | IMU 采样、姿态解算、发布传感器快照 | 不发送电机，不决定机器人模式 |
| `RobotControlTask` | 1 ms 绝对节拍 | 复制快照、读取电机反馈、安全门控、底盘和机械臂计算、直接发送所有电机命令 | 不阻塞，不使用相对 `osDelay(1)` 维持周期 |
| `InputTask` | DMA 事件唤醒或短周期 | 解析遥控器/自定义控制器，发布输入快照和离散边沿 | 不运行机构控制，不发送电机 |
| `ServiceTask` | 10～20 ms 低优先级 | USB、VOFA、LED、非实时诊断和执行时间统计 | 不进入安全闭环，不发送电机 |

`ChassisSystem`、`ArmSystem`、`RobotSystem`、模式状态机和安全门控都是普通控制模块，不因模块独立而自动创建线程。

只有出现下列实测证据时，才考虑新增独立控制或通信任务：

- 某个驱动存在无法消除的阻塞等待；
- USART10 接收缓存持续积压或溢出；
- `RobotControlTask` 最坏执行时间长期超过 1 ms 预算的 70%～80%；
- 某项规划计算明确需要不同频率，并且不能在 1 ms 周期内完成；
- 某机构需要独立暂停、重启或故障隔离生命周期。

即使将来把机械臂规划拆为低频任务，机械臂低层闭环和电机发送仍留在 `RobotControlTask`。

## 4. 硬件通信归属

| 物理端口 | 协议 | 发送 owner | 目标配置 |
| --- | --- | --- | --- |
| CAN1 | DJI、P1010B | `RobotControlTask` 直接调用两个驱动 | 经典 CAN，1 Mbit/s |
| CAN2 | Damiao | `RobotControlTask` 直接调用 Damiao 驱动 | 经典 CAN，1 Mbit/s |
| CAN3 | 保留，具体用途待定 | 暂无业务发送 owner | 保持启用，不提前绑定设备或控制逻辑 |
| USART10 | GO8010 | `RobotControlTask` 直接调用 GO8010 驱动 | 4 Mbit/s，DMA 接收与非阻塞发送 |

CAN3 在当前复现阶段只保留 CubeMX 初始化和底层资源，不接入 `RobotControlTask` 的业务链路；确认真实设备和协议后再补充分配。

驱动分别负责：

- 帧格式、校验、量程换算和协议状态；
- 接收缓存、帧头重同步和反馈时间戳；
- 电机使能、模式设置和该协议真实需要的启动步骤；
- 本协议内部的分组发送或命令编码。

驱动不知道底盘、机械臂和遥控模式。`RobotControlTask` 只做明确的机器人角色接线，例如将 DJI 轮速反馈填入 `ChassisFeedback_t`，将 `ChassisCommand_t` 的四轮电流分别交给 DJI 驱动；这种显式接线不是通用电机抽象层。

## 5. 模块与依赖方向

```text
FreeRTOS / HAL
    │
    ├── InsTask ─────发布 SensorSnapshot────┐
    ├── InputTask ───发布 InputSnapshot─────┤
    ├── ServiceTask ─只读诊断快照           │
    │                                      │
    └── RobotControlTask（1 ms owner）◄─────┘
            │
            ├── RobotSafety
            ├── RobotSystem
            │     ├── ChassisSystem
            │     └── ArmSystem
            │
            ├── DJI + P1010B driver / CAN1
            ├── Damiao driver / CAN2
            └── GO8010 driver / USART10
```

依赖规则：

- task 层负责调度、快照和 owner 生命周期；
- system 层负责机器人策略、机构状态和控制计算，不调用 HAL 或厂商驱动；
- `RobotControlTask` 是 system 与厂商驱动之间的显式接线位置；
- driver 层只负责一种协议；
- BSP/HAL adapter 负责具体 FDCAN、UART、DMA 和时间源；
- `ServiceTask` 只能读取诊断快照，不能反向参与电机控制。

`RobotSystem` 只协调底盘和机械臂的共享模式与互锁。`ChassisSystem` 和 `ArmSystem` 保持各自实现和跨周期状态；合并的是调度 owner，不是把两套控制算法写进一个大函数。

## 6. 1 ms 控制时序

以下代码表达调用顺序，不预先规定尚未接入驱动的最终函数名。

```c
void RobotControlTask(void *argument)
{
    TickType_t last_wake_tick = xTaskGetTickCount();

    VendorDrivers_Init();
    RobotSystem_Init();

    for (;;)
    {
        InputSnapshot_Copy(&input);
        SensorSnapshot_Copy(&sensor);

        DjiDriver_CopyFeedback(&dji_feedback);
        P1010BDriver_CopyFeedback(&p1010b_feedback);
        DamiaoDriver_CopyFeedback(&damiao_feedback);
        Go8010Driver_ProcessRxAndCopyFeedback(&go8010_feedback);

        ChassisFeedback_Build(&chassis_feedback, &dji_feedback);
        ArmFeedback_Build(&arm_feedback,
                          &p1010b_feedback,
                          &damiao_feedback,
                          &go8010_feedback);

        operational = RobotSafety_Evaluate(&input,
                                           &sensor,
                                           &chassis_feedback,
                                           &arm_feedback);

        RobotSystem_Step(&input,
                         &sensor,
                         &chassis_feedback,
                         &arm_feedback,
                         operational,
                         &chassis_command,
                         &arm_command);

        if (!operational)
        {
            ChassisSystem_Reset();
            ArmSystem_Reset();
            ChassisCommand_Clear(&chassis_command);
            ArmCommand_Clear(&arm_command);
        }

        DjiDriver_Send(&chassis_command);
        P1010BDriver_Send(&arm_command);
        DamiaoDriver_Send(&arm_command);
        Go8010Driver_Send(&arm_command);

        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(1U));
    }
}
```

约束：

- 每周期只复制一次输入、姿态和反馈，计算期间不再次读取可变全局量；
- safety 失效时，同一周期复位相关控制器并通过每个厂商驱动提交安全命令；
- system 函数不得阻塞、调用 `osDelay` 或自行维护另一个控制周期；
- 驱动发送必须非阻塞，初始化/查询/恢复过程应拆成单步状态机；
- 如果周期超时，先用 DWT 测量每段最坏执行时间，再决定优化或拆分，不能静默降低某个 system 的频率。

## 7. APP 目标目录

```text
User/APP/
├── Task/
│   ├── robot_control_task.c
│   ├── robot_control_task.h
│   ├── input_task.c
│   ├── input_task.h
│   ├── ins_task.c
│   ├── ins_task.h
│   ├── service_task.c
│   └── service_task.h
│
└── System/
    ├── robot_system.c
    ├── robot_system.h
    ├── robot_safety.c
    ├── robot_safety.h
    ├── chassis_system.c
    ├── chassis_system.h
    ├── arm_system.c
    └── arm_system.h
```

类型放在真正拥有它们的头文件中：

- `InputSnapshot_t` 归 `input_task.h`；
- `SensorSnapshot_t` 归 `ins_task.h`；
- `ChassisFeedback_t`、`ChassisCommand_t` 归 `chassis_system.h`；
- `ArmFeedback_t`、`ArmCommand_t` 归 `arm_system.h`。

不创建通用 `robot_control_types.h` 类型垃圾桶，也不为每个厂商在 APP 再包一层只转发调用的适配文件。

## 8. 并发与数据传递

- RC 和 INS 各自发布最新快照；`RobotControlTask` 每周期复制一次。
- FDCAN FIFO 中断循环取尽当前 FIFO 帧，交给对应 DJI、P1010B 或 Damiao 驱动接收入口；ISR 不进行机构控制计算。
- USART10 DMA/空闲中断只提交本次收到的字节片段。GO8010 驱动负责缓存和帧头重同步，不假设一次 DMA 事件等于一整帧。
- 驱动反馈若由 ISR 更新，任务侧使用短临界区、双缓冲或驱动已经提供的原子快照方法复制，不能在控制计算期间持锁。
- 模式切换、回零开始等不可丢失的离散命令才进入事件队列；摇杆、姿态、速度和电机反馈不排队。
- `ChassisSystem` 与 `ArmSystem` 不互相调用，也不直接访问对方内部状态；共享模式和互锁由 `RobotSystem` 协调。

## 9. 安全语义

最小安全门控只使用已经存在且有真实消费者的条件：

- 遥控器是否就绪；
- INS 是否已发布有效且新鲜的姿态；
- 当前参与闭环的执行器反馈是否有效且新鲜；
- 对应厂商驱动是否完成必要初始化；
- system 控制计算是否成功。

任一条件失败时，本周期必须：

1. 停止生成新的机构目标；
2. 复位受影响的 PID 和跨周期控制状态；
3. 通过各厂商驱动发送其协议定义的安全命令；
4. 保持周期运行并等待输入/反馈恢复，不能永久阻塞在等待循环中。

暂不预建复杂故障树和统一自动恢复框架。具体协议出现 late-power-on 等真实问题时，将恢复状态放在拥有该协议生命周期的驱动中，由 `RobotControlTask` 每周期推进一步。

## 10. 分阶段再实现

1. **文档与 CubeMX 任务基线**：固化四任务模型，将旧 `CHASSIS_TASK`、`rcTask` 重命名为 `robotControlTask`、`inputTask`，把 `defaultTask` 明确为 `serviceTask`，保持 CMSIS-RTOS2 和 1 kHz tick。
2. **任务骨架与安全零输出**：建立 `RobotControlTask` 1 ms 绝对节拍，直接初始化现有厂商驱动，只读取反馈并持续发送安全命令；用 DWT 测量最坏执行时间，不接入 PID 和运动学。
3. **CAN1 DJI 底盘闭环**：将麦轮运动学和 PID 重写为 `ChassisSystem`，完成遥控输入 → 轮速目标 → DJI 反馈 → 电流命令的第一条真实闭环。
4. **逐协议接入**：依次接入 CAN1 P1010B、CAN2 Damiao、USART10 GO8010；每次只增加一条执行器链路并验证单位、反馈新鲜度、失能零输出和 1 ms 预算。
5. **机械臂再实现**：将机构状态机、目标生成、运动学和补偿重写为 `ArmSystem`，由 `RobotSystem` 处理与底盘共享的模式和互锁。
6. **输入与观测收口**：将旧 `rc_task` 的应用语义收敛到 `InputTask`，将 UART/DMA 细节留在 BSP/driver；控制链稳定后再由 `ServiceTask` 加入 VOFA、USB 和执行时间观测。
7. **残留清理**：删除旧 `chassis_task`、旧任务入口、Stepper、云台、发射以及不再使用的工程登记，最后执行 EIDE 和 Keil 全量构建。

每一阶段都必须能够单独构建、烧录和回退；未通过当前阶段验证前不进入下一协议或机构。

## 11. 明确不采用的结构

- 不建立统一 `MotorIo` 或包含全部厂商字段的通用电机对象；
- 不让算法对象永久绑定厂商驱动或直接发送总线；显式驱动接线只存在于 `RobotControlTask`；
- 不照搬 `new_robot_code` 的通用电机注册表、字符串查找、厂商 union、DataPool/event bus 和 task 间 submit 网络；
- 不设置独立 `mct`、`chassis_task`、`arm_task` 控制周期；除非后续有明确的执行时间或阻塞证据；
- 不让 BSP 中断回调直接调用 APP/system；
- 不为尚未接入的执行器、故障或平台预留空接口；
- 不把 USB、VOFA 和日志放入 1 ms 控制任务。

## 12. 参考设计

- [ArduPilot Copter scheduler](https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Copter.cpp)：同一调度器按确定顺序运行 INS、控制器和电机输出，并为低速功能声明频率与执行时间预算。
- [ros2_control Controller Manager](https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/userdoc.html)：默认采用单周期 `read → update → write`，只有阻塞或超预算控制器才异步化。
- [Betaflight task table](https://github.com/betaflight/betaflight/blob/master/src/main/fc/tasks.c)：按频率和优先级拆调度项，而不是按源码模块机械创建线程。
- [PX4 multicopter rate control](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_rate_control/MulticopterRateControl.cpp)：数据驱动的 WorkItem 适用于具有不同频率和独立数据源的复杂系统。
- [RoboMaster standard robot FreeRTOS setup](https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/20.standard_robot/Src/freertos.c)：底盘、云台和 INS 分任务是传统 RoboMaster 模式，适用于不同周期和独立机构，但不是本工程的默认选择。
