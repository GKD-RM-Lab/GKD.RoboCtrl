# Core 模块

路径：`include/core/`、`src/core/`

Core 提供所有上层模块共享的运行时基础：单线程异步上下文、长生命周期实例注册表和统一日志。其设计目标是让设备与控制模块只描述自己的任务，不各自创建线程或管理全局对象。

## `async`：协程调度

`task_context` 是 `asio::io_context` 的单例包装，主要接口包括：

- `spawn(awaitable<void>)`：通过 `asio::co_spawn` 注册协程，未捕获异常会记录并停止事件循环。
- `post(fn, args...)`：把普通可调用对象放入事件队列。
- `run()` / `stop()`：启动或停止全局事件循环。
- `wait_for(duration)`：使用 `steady_timer` 挂起当前协程。
- `yield()`：把执行权交回调度器。
- `executor()` / `io_context()`：供 IO 对象绑定 Asio executor。

开发约束：

- 周期循环必须包含可挂起操作，且频率/单位要可解释。
- 不要在其他模块创建私有 `io_context` 或后台线程，除非先形成明确的跨线程所有权与同步设计。
- 构造阶段调用 `spawn` 只会登记任务；任务在 `async::run()` 后才开始执行。
- completion handler 统一收集任务边界异常；模块内部仍应为可恢复错误提供明确状态和上下文。

成熟度：**已接入**。CAN、串口、网络、电机和控制循环都依赖它。

## `multiton`：多例与统一初始化

多例适合“同一类型、多个具名硬件”的对象。合规类型应提供：

```cpp
struct info_type {
    using owner_type = my_device;
    using key_type = std::string_view;
    std::string_view name;
    std::string_view key() const { return name; }
};

explicit my_device(const info_type& info);
```

`roboctrl::init(info)` 创建实例，`roboctrl::get<T>(key)` 取得引用；`get(info)` 可按需创建。单例的 `info_type` 只需 `owner_type`，统一的 `init` 会转调该单例的 `init(info)`。

设计理念：配置只保存可复制的构造信息，实例注册表负责进程级生命周期，上层以稳定 key 建立关系。代价是依赖图不由类型系统完全表达，必须显式维护初始化顺序。

注意事项：

- 单个和批量初始化都会拒绝重复 key；批量初始化在构造首个对象前完成整组重复检查。
- `get()` 返回长期引用，依赖实例不被删除；目前没有卸载/重建生命周期。
- map 有互斥保护，`for_each_instance` 调用回调期间也持锁。回调不能重入同一 owner 类型的 multiton 操作，否则有死锁风险；项目整体也不能据此推断所有对象线程安全。
- `connect_all<T>()` 与 `start_all<T>()` 分别用于连接依赖和启动任务，仅对实现对应接口的多例类型可用。
- `instance_ref<T>` 是按 key 延迟取得具体 multiton 类型的轻量引用；跨电机类型的统一接口使用 `device::motor_ref`。

成熟度：主多例/单例路径、分阶段启动辅助和 `instance_ref` **已接入**；卸载和初始化回滚仍未实现。

## `logger`：统一可观测性

`logger` 单例支持 `Debug / Info / Warn / Error` 等级和字符串过滤。`logable<T>` 使用 CRTP，要求派生类提供 `desc()`，从而让日志自动携带实例角色。也提供 `LOG_DEBUG` 等宏，用文件、行号和函数组成 role。

开发理念：底层对象日志应能回答“哪个物理/逻辑实例产生了消息”，所以设备、IO 和控制类优先继承 `logable` 并提供稳定、可搜索的 `desc()`。高频 1 ms 循环中的 Debug 日志可能显著增加开销，新增日志要区分状态变化与周期采样。

成熟度：**已接入**。命令行的 `--log` 和 `--filter` 控制其行为。

## 修改 Core 时的检查清单

- 所有使用 `roboctrl::get/init/spawn` 的模块是否仍满足 concept。
- 初始化失败是否能在进入事件循环前被观察。
- `connect()` / `start()` 是否幂等，且调用顺序是否由入口明确保证。
- key 类型和生命周期是否安全，尤其是 `string_view` 是否引用静态/长期存储。
- 是否改变任务调度、异常、停止或对象生命周期语义。
- 同步更新 `docs/architecture.md` 和依赖 Core 的模块文档。
