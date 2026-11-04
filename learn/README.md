# PX4 学习入口

**主线只读一本：[PX4 从一次真实运行到独立开发](WORKBOOK.md)。** 它从你“已经能启动仿真、能读数据”的起点出发，把实际启动、参与代码、调试、修改和验证连成完整过程。

先按第 1～6 章推进，完成一个闭环；后续再沿同一系统深入估计、控制、通信、仿真和硬件。无需先通读全部参考资料。

## 先解决当前最困惑的问题

- **每次启动哪些代码参与？** [第 2 章](WORKBOOK.md#c02)追脚本与模块，[第 5 章](WORKBOOK.md#c05)提供真实位置控制调用栈、变量、条件分析和两个终端的复现步骤。
- **几个软件各做什么，怎样调试？** [第 1 章](WORKBOOK.md#c01)解释职责；[第 6 章调试过程](WORKBOOK.md#debug-lab)明确每个提示符该输入什么、在哪里停、看什么变量、怎样退出。
- **怎样开始改代码？** [第 6 章](WORKBOOK.md#c06)给出完整可编译模块，贯通参数、消息、停止重启、日志、测试和独立修改。
- **怎样证明学会了？** 完成书中的[三项独立验收](WORKBOOK.md#assessment)，以自己的源码差异、诊断和结果评价。

历史 `px4-sih-debug.code-workspace` 已删除，不需要恢复。它是可选 VS Code 配置，删除影响见[说明](DEBUGGING.md#workspace-explained)；主文档使用明确的终端命令。

## 其他材料什么时候用

| 材料 | 用途 |
|---|---|
| [系统导读](SYSTEM_GUIDE.md) | 查整机关系、术语、[全部模块职责](SYSTEM_GUIDE.md#module-index)、[接口和配置契约](SYSTEM_GUIDE.md#interface-contracts) |
| [工程全景](PROJECT_GUIDE.md)、[开发参考](DEVELOPMENT.md) | 查源码布局、构建机制与扩展方式 |
| [技术原理](TECHNICAL.md)、[硬件架构](ARCHITECTURE.md) | 深入数学、物理、平台和设备约束 |
| [调试参考](DEBUGGING.md)、[操作参考](OPERATIONS.md) | 查 GDB、日志、性能、外部应用和板端步骤 |
| [C++ 参考](CPP_GUIDE.md) | 按当前源码补语法，不作为开工前必须读完的前置课 |
| [十阶段路线](ROADMAP.md) | 需要检查能力范围时查阅；不另起一套阅读顺序 |
| [作者验证记录](records/IMPLEMENTATION.md) | 查哪些构建/测试/仿真/断点已实测、[工程覆盖与缺项](records/IMPLEMENTATION.md#coverage) |

`labs/` 只保存可执行实验代码和检查工具，操作步骤放在主文档。学习记录模板和能力评价也在书内，不要求维护另一堆空白文件。建设规范保留在[MASTERY_PLAN](MASTERY_PLAN.md)，无需把它当作课程阅读。

本轮基线：`310e5c30ee64784b13c94de430c463ebe4656024`，2026-11-04。作者验证与个人掌握分开记录；硬件、外部系统和不同机型需要对应条件下的证据。已有历史环境说明不能代替当前运行身份。
