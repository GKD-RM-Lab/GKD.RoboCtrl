# 总体架构

## 项目目标

GKD.Roboctrl 将机器人电控拆成可复用的通信、设备和控制层。核心理念是：硬件通信细节停留在底层，物理设备对上暴露统一状态与命令，机器人行为在控制层组合；所有持续任务由一个 Asio 事件循环协作调度，从而减少传统多线程电控中的锁与共享状态问题。

## 分层与依赖方向

```text
src/main.cpp
    │ 选择 BUILD_TYPE、解析 CLI、读取 YAML/JSON、按顺序初始化
    ▼
configs/*.yaml + include/config/runtime.hpp
    │ 直接组合既有组件的 info_type，解析和校验连接关系
    ▼
ctrl  ──────────────── 后台控制任务、机器人行为与状态编排
    ▼
device ─────────────── 协议解释、物理量、离线状态、底层控制输出
    ▼
io ────────────────── 字节收发、按 key/原始数据分发
    ▼
core + utils ───────── 事件循环、实例管理、日志、控制与数据工具
    ▼
Linux SocketCAN / serial / TCP / UDP
```

允许同层协作和上层依赖下层，不应让 `io` 依赖设备或控制业务，也不应让设备驱动直接决定机器人模式。ControlPad 在 Device 层只发布解析后的输入，由 Robot 在 Control 层完成映射和状态切换。

## 启动生命周期

1. xmake 通过 `type` 选项生成 `BUILD_TYPE`，`include/config/base.hpp` 将其映射为 `TYPE_*` 和 `TYPE_STR`，并决定默认路径 `configs/<type>.yaml`。
2. `src/main.cpp` 解析 `--help`、`--log`、`--filter`、`--config`，打印所选配置目录中的全部 YAML/JSON 文本，然后读取指定配置文件。
3. `load_configuration()` 用 reflect-cpp 将文件直接反序列化为既有组件的 `info_type`；`validate_configuration()` 在访问硬件前检查 key、依赖、DJI ID/指令槽和控制模块必需电机。
4. `roboctrl::init` 先构造 CAN、串口、DJI 电机、遥控器和 IMU；DJI 电机此时不注册回调或启动任务。
5. `connect_all<dji_motor>()` 连接 CAN 回调和电机组，再初始化 `robot`；Robot 通过底盘/云台注册表选择设备，启动 `motion_control` 后台任务并默认保持 `NoForce`。
6. 依次 `start_all<can>()`、`start_all<serial>()`、`start_all<dji_motor_group>()` 和 `start_all<dji_motor>()`；各 `start()` 是幂等的。
7. `async::run()` 启动唯一的 `asio::io_context`，所有 IO 接收、周期控制和回调协程在同一线程协作运行。

运行后，双开关加俯仰拨轮解锁手势是从 `NoForce` 进入 `FollowGimbal` 的入口；100 ms 内无有效 ControlPad 报文时 Robot 会重新进入 `NoForce`。后台控制任务负责把输入分发给抽象底盘、云台和发射设备。

初始化顺序是隐式依赖注入的一部分。遥控器和串口 IMU 在构造时通过串口名称注册回调；DJI 电机刻意把构造与 `connect()` 分开，保证同批对象全部注册后才建立跨对象关系。顺序错误会由配置预检或 `get()` 明确报错。

```text
validate → construct → connect/register callbacks → init controllers/NoForce
         → start IO/group/device tasks → run event loop
```

新增具有长期任务的多例类型时，优先沿用这套阶段化生命周期：构造函数只取得自身资源，跨对象绑定放在幂等 `connect()`，协程启动放在幂等 `start()`。不需要跨对象关系且启动时机明确的单例控制器，可在 `init()` 末尾启动自身任务。

## 实例与所有权

项目用两种长生命周期对象模型：

- **多例**：CAN、串口、网络端点、电机等同类多实例对象。`info_type` 提供 key，由 `multiton_impl<T>` 的静态 map 持有 `unique_ptr<T>`。
- **单例**：异步上下文、整机、底盘、云台、发射器、超级电容等全局唯一对象。类继承 `singleton_base<T>`，由 `T::instance()` 持有，并通过 `init(info)` 完成显式初始化。

批量多例初始化会先检查当前表和本批次内的重复 key，再构造整批对象；`for_each_instance`、`connect_all`、`start_all` 提供阶段化批处理。`instance_ref<T>` 保存 key 并延迟查找具体多例。

对象间可以保存配置/key，也可以在初始化后保存指向 `device::motor_base` 的非拥有型指针。初始化阶段通过具体电机类型查找一次，控制循环通过基类虚函数访问稳定的 multiton 对象。这些指针不拥有实例，因此依赖“进程内实例不删除”的当前生命周期。

## 数据流

以 DJI 电机为例：

```text
配置预检并构造电机
  → connect() 注册 CAN 反馈回调与电机组槽位
  → Robot 设为 NoForce、电机保持 disabled
  → 显式启动 CAN/电机组/电机任务
SocketCAN 帧
  → io::can::task 读取 can_frame
  → keyed_io_base 按 CAN ID 分发
  → dji_motor 回调解析编码器/转速/电流并 tick()
  → PID 根据目标速度和反馈更新 current_
  → dji_motor_group 每 1 ms 聚合同一总线上的电流命令；禁用或离线电机贡献 0
  → io::can::send 下发 0x1ff / 0x200 / 0x2ff
```

以串口设备为例：串口读取 `0xAA55` 头、1 字节 key，再依据该 key 注册回调时记录的固定长度读取 payload；随后设备回调把 payload 转成遥控器或 IMU 数据。新增协议必须明确帧边界、长度、字节序与恢复策略。

## 单线程异步理念

单线程并不等于所有操作都可直接执行：任何阻塞系统调用、长计算或不含 `co_await` 的长循环都会阻塞全车任务。周期任务应使用 `wait_for`，即时让出使用 `yield`；IO 应使用 Asio 异步接口。共享状态通常无需互斥，但回调与周期任务仍可能在不同挂起点交错，修改状态时要保持单次操作的一致性。

`task_context::spawn` 的 completion handler 会记录未捕获异常并停止事件循环。停止事件循环不等于硬件已经完成安全卸载，因此执行器仍必须依靠 `NoForce`、离线置零和显式禁用建立安全门。

## 配置驱动理念

不同机器人共用驱动和控制实现，差异集中在 `configs/<type>.yaml`：总线名、设备路径、电机 ID、轮半径、PID 和控制参数都以组件原有 `info_type` 的字段名直接表达。加载后保存的是拥有字符串的 `info_type`，因此配置文件的文本寿命不会影响 multiton key。`include/config/runtime.hpp` 负责反序列化，`validate.hpp` 负责启动前语义校验；不再维护车型硬编码配置副本。

配置不是“能解析即可”。`validate_configuration` 把跨表引用和槽位冲突提前到硬件打开之前；新增 `info_type` 字段时，应同时确定 YAML 表达、默认值/缺失策略、可在无硬件环境执行的校验和正例/负例测试。`robot.chassis_type` 和 `robot.gimbal_type` 只作为工厂选择键，由 `chassis_registry`/`gimbal_registry` 在初始化时查找并派发具体设备，配置层不复制注册表白名单。

## 安全边界

- 构建可以在无硬件环境执行；运行会打开 CAN/串口，并可能在事件循环开始后持续发送电机命令。
- 启动失败可能来自设备文件、网络接口、权限、配置 key 或协议不匹配。不要通过吞掉异常来“让程序启动”。
- 控制器的 `NoForce`、离线状态和功率限制应形成统一安全门；当前实现尚未完整闭环，因此新增执行器逻辑必须明确失联和禁用时输出什么。
- 任何改变输出符号、单位、减速比、轮半径、CAN ID 或字节序的改动都属于硬件行为变更，必须同步文档并进行台架验证。
- 当前 CI 和单元测试只覆盖无硬件逻辑。Linux 构建成功、测试通过、台架通过和实车通过必须分别报告。
