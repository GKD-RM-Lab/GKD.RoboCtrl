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
| 入口与构建 | `src/main.cpp`、`xmake.lua`、`start.sh` | 选择运行时配置、解析参数、初始化、启动事件循环 | [`modules/config-build.md`](modules/config-build.md) |
| 配置 | `configs/*.yaml`、`include/config/runtime.hpp`、`include/config/validate.hpp` | 运行时 YAML/JSON 组装与语义校验 | [`modules/config-build.md`](modules/config-build.md) |
| 核心 | `include/core/`、`src/core/` | 协程调度、实例管理、日志 | [`modules/core.md`](modules/core.md) |
| IO | `include/io/`、`src/io/` | 字节传输、回调分发、CAN/串口/网络适配 | [`modules/io.md`](modules/io.md) |
| 设备 | `include/device/`、`src/device/` | 协议解析、物理量、离线检测、电机输出 | [`modules/device.md`](modules/device.md) |
| 控制 | `include/ctrl/`、`src/ctrl/` | 后台控制任务、整机状态与行为分发 | [`modules/control.md`](modules/control.md) |
| 工具 | `include/utils/` | PID、斜坡、回调、矩阵/RLS、字节与类型工具 | [`modules/utils.md`](modules/utils.md) |
| 运动迁移 | `ctrl/motion_control`、`device/gimbal` | 坐标/回中/IMU 内环/多云台/搜索 | [motion-migration](modules/motion-migration.md) |
| 电机与超容协议 | `device/motor`、`device/super_cap` | DJI/J6006/M9025 codec 与安全输出 | [motor-protocols](modules/motor-protocols.md) |
| 网络控制 | `io/udp_server`、`device/aim_link`、`device/remote_logger` | 视觉/导航/遥测协议与新鲜度 | [network-control](modules/network-control.md) |
| 裁判与 UI | `device/referee`、`ctrl/shoot` | CRC/帧/状态、图元与发射许可 | [referee](modules/referee.md) |
| 功率控制 | `ctrl/power_manager`、`ctrl/power_feedback` | 模型/分配/能量/RLS/实际电流门 | [power-control](modules/power-control.md) |
| 弹道与工具 | `utils/ballistics`、`tools/` | 离线解算/预测、日志接收与绘图 | [ballistics-tools](modules/ballistics-tools.md) |

旧工程 `GKD_Control` 的迁移范围、已落地行为和明确未迁移项见
[`migration-gkd-control.md`](migration-gkd-control.md)。

## 成熟度术语

本文档统一使用以下标签，避免把接口数量误认为完成度：

- **已接入**：入口会初始化或现有主流程会实际使用，且实现包含主要行为。
- **部分实现**：已有可执行逻辑，但功能、配置、错误处理或集成仍不完整。
- **接口骨架**：主要是声明、空任务或占位实现，不能当作可用功能。
- **未接入**：源码存在，但当前 `src/main.cpp` 的正常启动流程没有初始化它。

## 当前基线摘要

- 同一二进制运行时选择 YAML/JSON，默认 infantry；配置直接组成既有组件 info_type，硬件访问前做语义预检。
- 电机/IO 按 construct/connect/start 接入；主程序默认 NoForce，只使能实际绑定执行器。云台解锁先受控回中，全部就绪才允许底盘/发射；遥控失联退回 NoForce。
- 坐标变换、IMU 串级、视觉/导航、裁判/UI、超容、实际功率输出限制和辅助工具的软件迁移已接入，详细状态见 [迁移清单](migration-gkd-control.md)。
- 哨兵真实拓扑是 J6006 大 yaw 加 DJI 小头/两 IMU；旧 CAN 映射冲突被预检拒绝，按用户要求暂不改硬件 ID。回中零位也未确认。
- 无硬件测试覆盖配置/协议、真实控制协程配模拟设备、功率限流和算法；localhost 测试覆盖网络与回调所有权。运行主程序仍需要 Linux/硬件授权，单测通过不等于实车验证。
- 本轮按用户要求未继续执行 xmake；最终证据见 [验证记录](migration-verification.md)。

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
