# PX4 系统学习文档

面向希望从零认识 PX4，逐步具备配置使用、原理分析、代码开发、应用集成和验证能力的读者。学习主线是理解完整载具怎样工作，并能独立解决问题。

## 现在从这里开始

打开 **[第一课：读懂一个 PX4 模块的工程结构](ROADMAP.md#lesson-1-1)**。课程从一个实际文件开始，给出代码、逐项解释、完整关系图和参考答案。第一步只需打开 `src/examples/work_item/CMakeLists.txt`，找到 `px4_add_module(...)`，跟着课程理解它的内容。

第一课可以只读源码完成，不需要先整理环境报告、运行仿真或通读下面全部文档。学完后再进入路线 1.2，逐步展开刚认识的结构。

## 工程全景与六篇专题文档

| 文档 | 回答的问题 | 主要内容 |
|---|---|---|
| **[当前 PX4 工程全景导读](PROJECT_GUIDE.md)** | 当前这份工程怎样组织、运行和开发，各部分怎样连成整机 | 25 章系统串讲：版本与目录、构建启动、运行与接口、完整控制链、其他机型、通信、个人 lab Docker、仿真、调试测试和开发学习顺序 |
| **[PX4 技术架构详解](ARCHITECTURE.md)** | 支持哪些硬件，需要哪些外设，技术怎样实现，有哪些仿真 | 18 章：飞控/伴随计算机/外设节点、板型与目标、器件组合、接口电气、测量与动力、算法与通信、仿真实现和参考系统 |
| **[PX4 系统架构与技术原理](TECHNICAL.md)** | PX4 是什么，各部分为什么这样协同 | 系统边界、物理与硬件、运行平台、测量、估计、控制、导航、安全、通信、机型、配置与验证 |
| **[PX4 代码架构与开发指南](DEVELOPMENT.md)** | 上述能力怎样由代码实现，从哪里读、在哪里改 | 工程与子模块、构建、启动、调度、uORB/参数、核心模块地图、贯通实现路径、完整开发练习 |
| **[PX4 开发调试实战](DEBUGGING.md)** | 怎样修改、编译、观察和定位问题，怎样证明改动有效 | 18 章：lab/VS Code、源码练习、终端与图形 GDB、uORB/参数、日志、测试、性能、应用集成、烧录、SWD/HardFault 与故障速查 |
| **[PX4 系统学习路线](ROADMAP.md)** | 当前学什么、怎样做、达到什么条件再继续 | 十个阶段的具体学习单元，逐项指定阅读、源码入口、实验、验收与补足方法 |
| **[PX4 操作与调试手册](OPERATIONS.md)** | 每一步怎样操作，怎样观察与判断结果 | 主题实践、编译、编辑器/GDB、仿真、QGC、日志、回放、测试、性能、通信与板上调试 |

## 读完第一课怎样继续

1. 进入[1.2 构建选择过程](ROADMAP.md#lesson-1-2)，沿第一课的关系图研究 Kconfig 和根构建文件；接着做 1.3 的产物观察与 1.4 的小改动实验。
2. 用[最初六次学习安排](ROADMAP.md#first-sessions)查看接下来几次的顺序；按[基础补足表](ROADMAP.md#entry)处理当前遇到的概念，不需要一次完成所有自测。
3. 每次只做当前单元：读指定内容，做一个实验，解释结果。若术语或代码看不懂，先补相应讲解，再继续，验收用于检验已经学过的内容。

学习顺序由路线安排；技术说明解释原理，开发指南解释实现，操作手册提供步骤。操作手册按主题组织，章节号与路线阶段号不要求相同，以任务中的直达链接为准。

随当前任务查阅[工程全景导读](PROJECT_GUIDE.md)建立模块关系，阅读[技术架构详解](ARCHITECTURE.md)理解硬件、外设和仿真；需要源码修改、GDB 或板端定位的完整步骤时，使用[开发调试实战](DEBUGGING.md)。

没有运行过仿真、没有飞控硬件也能开始。没有设备时可以完成软件和模型学习，但硬件时序、安装、动力和实际飞行能力需要对应实测。

## 已有问题时直接查这里

| 想解决的问题 | 入口 |
|---|---|
| 想用一篇文档系统认识当前工程及自己的 lab 操作 | [工程全景导读](PROJECT_GUIDE.md)、[本机环境与操作](PROJECT_GUIDE.md#environment) |
| 支持哪些飞控，怎样选择传感器、电调、定位与伴随计算机 | [硬件目标](ARCHITECTURE.md#hardware)、[外设需求](ARCHITECTURE.md#peripherals)、[参考系统](ARCHITECTURE.md#systems) |
| SIH、Gazebo、jMAVSim、JSBSim、HITL 有什么区别 | [仿真体系](ARCHITECTURE.md#simulation)、[仿真实现](ARCHITECTURE.md#simulation-internals) |
| 从第一次源码修改开始学习开发调试 | [源码练习](DEBUGGING.md#first-change)、[终端 GDB](DEBUGGING.md#gdb)、[VS Code 调试](DEBUGGING.md#vscode-debug) |
| 需要真机烧录、设备透传、SWD 或故障分析 | [固件部署](DEBUGGING.md#deployment)、[板端调试](DEBUGGING.md#hardware-debug)、[故障速查](DEBUGGING.md#triage) |
| 看不懂 PX4 全局，功能为什么分成这些层 | [整体架构](TECHNICAL.md#c1)与[任务生命周期](TECHNICAL.md#c8) |
| 子仓库是什么、版本怎样选择 | [目录与依赖](DEVELOPMENT.md#d2) |
| 源码怎样变成程序，又怎样运行 | [构建](DEVELOPMENT.md#d3)、[启动](DEVELOPMENT.md#d4)、[执行模型](DEVELOPMENT.md#d5) |
| 一个需求对应哪些模块，怎样沿代码追踪 | [核心模块地图](DEVELOPMENT.md#d7)、[三条实现路径](DEVELOPMENT.md#d8) |
| 第一次修改 PX4，怎样完成开发闭环 | [参数驱动模块练习](DEVELOPMENT.md#d9)、[自定义消息](DEVELOPMENT.md#message-exercise)、[算法库与测试](DEVELOPMENT.md#algorithm-exercise) |
| 怎样选择调试方法、查看变量或分析崩溃 | [开发与调试方法](OPERATIONS.md#methods) |
| 怎样分析估计、控制与连续运行数据 | [估计实践](OPERATIONS.md#s5)、[控制与日志](OPERATIONS.md#s6)、[Replay](OPERATIONS.md#e4) |
| 怎样规划任务、接入程序、部署设备 | [任务实践](OPERATIONS.md#s7)、[应用实践](OPERATIONS.md#s8)、[硬件实践](OPERATIONS.md#s9) |
| 今天从哪里开始 | [第一课：跟着代码逐项理解工程结构](ROADMAP.md#lesson-1-1) |
| 怎样确定是否学会、下一步做什么 | [学习路线](ROADMAP.md)中各单元验收，以及[三次综合验收](ROADMAP.md#milestones) |

## 资料与仓库约定

原理说明按 PX4 的系统职责组织；具体接口、参数、路径和命令以本仓库源码核对。全景导读记录了[本次核对基准](PROJECT_GUIDE.md#identity)和[当前 lab 布局](PROJECT_GUIDE.md#environment)，环境的详细维护操作见[操作手册附录 A](OPERATIONS.md#environment)。升级源码或迁移环境后，应重新核对这些记录。

个人仓库为 [orbits-yiyi/px4-system](https://github.com/orbits-yiyi/px4-system)。远程、分支和提交方式见[Git 操作](OPERATIONS.md#git)。正文代码和命令是待读者按步骤实施的实验，不代表已经执行或在所有平台验证通过。
