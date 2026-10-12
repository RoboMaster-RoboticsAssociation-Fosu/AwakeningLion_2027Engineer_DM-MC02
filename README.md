# H7 机器人固件

公共库在 Shared 中维护一份，各板的业务和硬件配置独立。
可以打开 CtrlBoard-H7.code-workspace，同时查看 SlaveArm、Chassis、MasterArm 和 Shared。

- SlaveArm：从臂，打开 SlaveArm/MDK-ARM/CtrlBoard-H7_IMU.uvprojx。
- Chassis：底盘，打开 Chassis/MDK-ARM/CtrlBoard-H7_IMU.uvprojx。
- MasterArm：保留目录，目前没有业务工程。
- Shared：公共算法、驱动、协议和第三方源码，具体职责见 Shared/README.md。

Keil 使用 ARM Compiler 6，各板也保留自己的 EIDE 配置。
需要编译的公共模块由各工程的文件列表指定，不遍历并编译整个 Shared。
构建结果保存在各板目录内，整套代码应连同 Shared 一起使用。

## Git 协作

main 保存 Shared 和各板的完整工程，板型由目录区分。
开发具体功能时从 main 创建工作分支，完成后合回 main；公共库修改应验证所有实际引用它的工程。
从仓库根目录提交与拉取，构建输出、迁移备份和临时验证文件不进入版本管理。
