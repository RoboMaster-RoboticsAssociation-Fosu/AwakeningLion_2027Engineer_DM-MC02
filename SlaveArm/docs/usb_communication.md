# 从臂与 Pi 的 USB 通信

从臂作为 USB CDC Device，Pi 作为 USB Host。当前只接通通信：收到的角度保存在
`g_arm_usb.received_command`，不写入 `g_arm_task.control.joint_target_rad`，不触发电机动作。
电机使能、现有 Keil 调试入口和 CAN 控制链路保持原状。

## 业务数据

八轴顺序固定为 Yaw、Pitch1、Roll1、Pitch2、Roll2、Pitch3、Roll3、夹爪。
角度、角速度和力矩均以对应输出轴为准；夹爪当前表示驱动轴角度/力矩。

Pi 下发 32 字节业务载荷：

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `target_angle_rad[8]` | 8 个 float32 | 目标角度，rad |

从臂上传 97 字节业务载荷：

| 载荷偏移 | 字段 | 含义 |
| --- | --- | --- |
| 0 | `angle_rad[8]` | 反馈角度，rad |
| 32 | `velocity_rad_s[8]` | 反馈角速度，rad/s |
| 64 | `load_torque_nm[8]` | 扣除本体和固定工具重力、保留物体重量的负载力矩，N.m |
| 96 | `load_valid_mask` | uint8，bit0~7 对应八轴负载力矩是否有效 |

浮点数使用 IEEE-754 binary32、小端，不发送 C 结构体的对齐填充。
没有有效电机反馈时，角度和角速度上传 NaN，Pi 工具显示为 `null`。
当前未接入重力/负载估算，八轴负载力矩均上传 NaN、有效掩码为 0；它们不是有效的零负载。

## 帧格式

复用现有 `BoardLinkProtocol` 的组帧和 CRC；USB 与 USART10 各有独立解析实例。

| 帧偏移 | 长度 | 字段 |
| --- | --- | --- |
| 0 | 2 | 帧头 `A5 5A` |
| 2 | 1 | 版本 `01` |
| 3 | 1 | 消息类型：下发角度 `10`，上传反馈 `11`（十六进制） |
| 4 | 2 | 载荷长度，uint16 小端；下行 32，上行 97 |
| 6 | 2 | 命令序号，uint16 小端 |
| 8 | 32 或 97 | 业务载荷 |
| 载荷之后 | 2 | CRC16，小端，覆盖帧头至载荷末尾 |

CRC 使用反射多项式 `0x8408`、初值 `0xFFFF`、无最终异或。
校验向量：ASCII `123456789` 的 CRC 为 `0x6F91`。
完整下行帧 42 字节，上行帧 107 字节。

上行序号回显最近一次通过 CRC、类型、长度和有限值检查的命令序号；新 USB 会话从 0 开始。
回显只表示通信模块已接受命令，不表示电机已执行。
当前不对命令序号排序或去重，不实现断线后的运动策略。

## 运行与检查

`ServiceTask` 每 10 ms 消费 USB 接收队列并尝试上传一次反馈，正常约 100 Hz。
USB 忙时跳过本次上传，下一周期发送最新快照，不覆盖正在发送的缓冲。
接收支持任意分包、连续多帧、CRC 错误后的重同步；队列溢出会丢弃旧流并重新组帧。

Pi 端工具 `tools/arm_usb.py` 只依赖 Python 3 标准库。已部署位置：
`/home/xuyan/arm_usb/arm_usb.py`。

只接收，不发送目标：

```sh
python3 /home/xuyan/arm_usb/arm_usb.py --duration 5
```

双向通信检查（当前接收端只存储目标，不驱动电机）：

```sh
python3 /home/xuyan/arm_usb/arm_usb.py --duration 5 --rate 100 \
  --angles 0 0.01 -0.02 0.03 -0.04 0.05 -0.06 0.07
```

存在多个 STM32 串口时必须用 `--port` 指定端口，禁止自动选择其中一个。
本次实机发现的稳定路径为：
`/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_306235523231-if00`。

Keil 可观察 `g_arm_usb` 的接收角度、最近命令序号、接收数量、拒绝数量、溢出数量与上传计数。

## Pi 上电自动打开串口

`tools/arm-usb.service` 是当前泰山派的 systemd 服务，使用 xuyan 用户运行，
固定打开上述 H7 的稳定设备路径。程序使用 `--continuous` 常驻，只接收反馈，不发送目标。
USB 晚插入或断开导致程序退出时，服务每 2 秒重新尝试，恢复后继续接收。
常驻模式每 5 秒向系统日志输出一次反馈和接收计数，不创建持续增长的数据文件。

服务文件安装到 `/etc/systemd/system/arm-usb.service`，启用后随系统启动。
查看状态与日志：

```sh
systemctl status arm-usb.service --no-pager
journalctl -u arm-usb.service -f
```

需要手工运行串口检查工具时，先停止后台服务，避免两个进程同时读取同一个端口：

```sh
sudo systemctl stop arm-usb.service
python3 /home/xuyan/arm_usb/arm_usb.py --duration 5
sudo systemctl start arm-usb.service
```

本地主机回归入口：

```sh
python SlaveArm/tests/run_usb_tests.py
```

测试覆盖实际编解码、USB owner 和发送缓冲函数；原生测试只替代硬件与临界区。
实机枚举和 USB 收发验证与原生测试分别记录；目前没有验证电机动作、重力补偿或力反馈效果。

## 本次验证记录（2026-10-23）

- 设备：泰山派 RK3566，Ubuntu 24.04，Python 3.12；STM32 USB 串号 `306235523231`。
- 从臂 Keil 构建：0 Error(s)、0 Warning(s)；GCC/Clang 严格 C99 回归与 6 项 Python 测试通过。
- 只接收检查：3 秒内解析 300 帧有效反馈。
- 双向检查：5.017 秒内发送 484 帧命令、收到 500 帧反馈；500 帧的回显序号均指向本轮发送的命令。
- 实机边界检查：正常回执、错误 CRC、NaN、Inf、错误长度、未知消息类型、42 次逐字节写入、噪声加连续双帧、序号回绕，共 9 项通过。
- 当前电机角度/速度没有有效观测，工具显示 `null`；负载估算尚未接入，`load_valid_mask=0`。
- 接收的目标没有接入电机动作；这些结果只证明通信链路。
