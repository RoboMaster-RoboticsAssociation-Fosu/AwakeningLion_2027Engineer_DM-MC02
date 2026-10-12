# HWT6033-TTL 驱动

本目录接入维特智能官方 WIT 标准串口协议 SDK，并提供适用于本工程 STM32H7 HAL 的薄适配层。

## 上游来源

- 官方仓库：<https://github.com/WITMOTION/WitStandardProtocol_JY901>
- 固定提交：`e8f4e3afc02263e2ef15052dc758f45efaebe12f`
- 上游路径：`Linux_C/normal`
- 原样引入：`REG.h`、`wit_c_sdk.h`、`wit_c_sdk.c`
- 工程适配：`HWT6033.h`、`HWT6033.c`

采用 `Linux_C/normal` 是因为它是官方仓库中完整的可移植 C 实现；同仓库旧 STM32F1 副本包含已知源码缺陷，不作为本工程基线。

> 上游仓库在上述提交中未提供 LICENSE 文件。若需要对外分发这些源文件，请先向维特智能确认授权范围。

## 串口边界

驱动不会擅自占用工程中的 UART。当前工程 UART5、USART2、USART10 已分别用于 SBUS、宇树电机和步进电机，因此必须由应用层明确选择可用串口或调整现有设备分配后再启用 HWT6033。

串口应配置为：

- 8 数据位、无校验、1 停止位；
- 波特率与传感器当前设置一致，WIT 标准协议默认值为 9600 bit/s；
- 传感器 TX 接 MCU RX，传感器 RX 接 MCU TX，双方共地；
- 供电与 TTL 电平必须以具体 HWT6033-TTL 硬件手册为准。

## 最小接入

在串口初始化完成后绑定句柄：

```c
#include "HWT6033.h"

HWT6033_Init(&huartX);
```

在对应串口的 DMA/空闲接收回调中投递原始字节：

```c
HWT6033_FeedData(rx_buffer, received_length);
```

在任务中读取一致的数据快照：

```c
HWT6033_Data_Typedef imu_data;

if (HWT6033_CopyData(&imu_data) == HWT6033_STATUS_OK)
{
    /* imu_data.acceleration_g / angular_velocity_dps / angle_deg */
}
```

如需修改输出速率、输出内容、校准或波特率，可在 `HWT6033_Init` 之后调用 `wit_c_sdk.h` 提供的官方接口。相关接口包含阻塞发送和延时，只能在任务上下文调用；随后用 `HWT6033_GetLastTransmitStatus()` 检查底层 HAL 发送状态。
