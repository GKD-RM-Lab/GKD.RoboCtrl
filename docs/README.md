# GKD.Roboctrl Agent 文档

这组文档服务于阅读和修改仓库的开发者与编码 Agent。目标不是重复 Doxygen API，而是说明模块边界、依赖、运行时数据流、设计取舍、当前成熟度以及改动时必须保持的约束。

## 推荐阅读顺序

1. [`architecture.md`](architecture.md)：先理解分层、实例生命周期和主数据流。
2. 根据任务阅读下表中的模块文档。
3. 修改前阅读 [`development.md`](development.md)，确认验证方式和文档同步要求。
4. 最后回到源码核实；源码中的当前实现是事实依据。

## 模块索引

| 模块 | 主要路径 | 职责 | 文档 |
| --- | --- | --- | --- |
| 入口与构建 | `src/main.cpp`、`xmake.lua`、`start.sh` | 选择机器人类型、解析参数、初始化、启动事件循环 | [`modules/config-build.md`](modules/config-build.md) |
| 配置 | `include/config/` | 按机器人类型声明 IO、设备与控制器实例参数 | [`modules/config-build.md`](modules/config-build.md) |
| 核心 | `include/core/`、`src/core/` | 协程调度、实例管理、日志 | [`modules/core.md`](modules/core.md) |
| IO | `include/io/`、`src/io/` | 字节传输、回调分发、CAN/串口/网络适配 | [`modules/io.md`](modules/io.md) |
| 设备 | `include/device/`、`src/device/` | 协议解析、物理量、离线检测、电机输出 | [`modules/device.md`](modules/device.md) |
| 控制 | `include/ctrl/`、`src/ctrl/` | 底盘、云台、发射、整机状态与功率管理 | [`modules/control.md`](modules/control.md) |
| 工具 | `include/utils/` | PID、斜坡、回调、矩阵/RLS、字节与类型工具 | [`modules/utils.md`](modules/utils.md) |

旧工程 `GKD_Control` 的迁移范围、已落地行为和明确未迁移项见
[`migration-gkd-control.md`](migration-gkd-control.md)。

## 成熟度术语

本文档统一使用以下标签，避免把接口数量误认为完成度：

- **已接入**：入口会初始化或现有主流程会实际使用，且实现包含主要行为。
- **部分实现**：已有可执行逻辑，但功能、配置、错误处理或集成仍不完整。
- **接口骨架**：主要是声明、空任务或占位实现，不能当作可用功能。
- **未接入**：源码存在，但当前 `src/main.cpp` 的正常启动流程没有初始化它。

## 当前基线摘要

- 默认构建类型是 `infantry`；可选 `hero`、`sentry`、`project`。
- `src/main.cpp` 先整体验证配置，再构造 CAN、串口、DJI 电机、遥控器和 IMU，连接依赖后初始化 `robot`。
- CAN、串口和 DJI 电机采用“构造 → 连接 → 启动”阶段，不在构造函数中启动长期协程。
- Robot 默认进入 `NoForce`，DJI 电机默认禁用；遥控器完成双开关加滚轮解锁手势后进入 `FollowGimbal`，遥控失联会退回 `NoForce`。
- Infantry/Hero 启用底盘、云台和发射，Sentry/Project 当前只启用底盘；“启用”不等于功能已经完整。
- `gimbal` 已接入 IMU 角度外环和电机速度目标，`power_manager`、`referee`、M9025 等仍有明显骨架或未完成部分，详见模块文档。
- `tests/unit_tests.cpp` 覆盖配置校验、multiton 重复键、组合解析器、底盘限速、遥控映射、发射互锁和 `motor_ref`；新增 CI 目标是覆盖四车型的 Debug/Release 组合，但当前工作流命令仍有已知问题，见构建文档。
- 运行主程序仍需要 Linux SocketCAN、串口和真实/仿真硬件；单元测试通过不等于实车安全。

## 按改动类型定位文档

| 准备修改 | 先读 | 同步更新重点 |
| --- | --- | --- |
| 初始化、对象生命周期、协程异常 | `architecture.md`、`modules/core.md` | 启动顺序、失败传播、任务所有权 |
| CAN/串口/TCP/UDP 或报文分发 | `modules/io.md`、`modules/utils.md` | 帧格式、长度、字节序、回调顺序 |
| 电机、IMU、遥控器、超级电容 | `modules/device.md` | 物理单位、ID、离线策略、安全输出 |
| 底盘、云台、发射、整机状态 | `modules/control.md` | 控制状态机、符号、限幅、NoForce 行为 |
| 车型、硬件 key、PID、构建/CI | `modules/config-build.md` | 四车型一致性、配置预检和验证矩阵 |
| 公共模板、PID/Ramp、矩阵/RLS | `modules/utils.md` | concept、数值语义、协议边界 |

## 文档维护原则

- 文档描述“当前代码是什么”和“改动应保持什么”，不把计划写成已完成事实。
- 新模块必须加入本索引；模块行为、接口、协议或成熟度变化时必须同步更新对应文档。
- 精确 API 以头文件和生成的 Doxygen 文档为准；这里重点记录跨文件语义与开发理念。
- 文档与源码不一致时，先确认变更意图，再同时修正二者；不能只把文档改成听起来合理的状态。
