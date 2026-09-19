# Utils 模块

路径：`include/utils/`

Utils 提供 PID、Ramp、回调、字节转换、数学和类型约束。

设计原则是把与具体硬件无关、可在无硬件环境验证的机制放在这里；但“通用”不等于可以忽略单位、生命周期或数值边界。公共模板变化会影响几乎所有模块，必须构建 Debug/Release 主目标并加载校验四份运行时车型配置。

- `from_bytes<T>()` 要求输入长度恰好等于 `sizeof(T)`，不匹配时抛出 `invalid_argument`。
- `to_bytes()` 提供返回固定数组和写入既有容器两种形式，容器不足时抛出异常。
- `callback` 将同步函数包装为协程；一次调用中的回调按注册顺序执行。
- PID 与 Ramp 支持显式 `dt`；控制任务应传入配置或调度器测得的采样周期，不应让控制行为隐式依赖调用次数。

## 字节与数值工具

`utils/utils.hpp` 定义 `fp32/fp64`、二维 `vector<T>`、角度归一化、启动后单调时间、大小端字节组合和字节序列互转。`package` 只约束 `is_trivially_copyable`，因此仍可能含 padding，并使用主机端序；它适合内部快照，不自动构成稳定外部协议。

`from_bytes<T>` 要求长度恰好相等，`to_bytes` 的写入型重载要求容器至少足够大。新协议优先逐字段 codec；若确需 packed 结构，增加 `static_assert(sizeof(...))`、端序说明和已知字节向量测试。

二维向量的 `normalized()` 当前没有零范数保护；调用者必须先保证非零。

## 回调

`callback<Args...>` 把同步函数包装为 `awaitable<void>`。触发时复制当前回调列表和参数到具名协程函数的 frame，避免临时 capturing coroutine lambda 在首次恢复前已析构；随后按注册顺序逐个 `co_await`。因此：

- 同一次 dispatch 内回调有稳定顺序。
- 两次独立 dispatch 各自 spawn，可能在挂起点交错。
- 触发后新增/删除（当前无删除 API）不会影响已复制的本次列表。
- 参数或其内部引用必须在异步执行期间有效；IO 层通过共享缓冲解决字节生命周期。

## PID、Ramp 与控制器 concept

`utils::controller` 要求 `input_type`、`state_type`、`params_type`、参数构造、`update()` 和 `state()`。`control_chain::state()` 按值返回末级状态，以兼容按值返回状态的 PID/Ramp 控制器，避免返回临时对象引用；新增组合用法前应补模板实例化测试。

`pid_base` 计算比例、积分和误差差分，显式 `update(current, dt)` 使用秒为单位的时间步，`update(target, current, dt)` 可在一次调用中设置目标并更新输出，同时保留单参数调用作为旧行为兼容；先限制积分再限制总输出。`clean()` 会清空目标、积分、上次误差和输出。`linear_pid` 使用普通目标差，`rad_pid` 把误差包裹到 `[-π, π]`。

`ramp<T>` 的 `update(target, dt)` 使用显式时间步限制变化率，并保留基于 `steady_clock` 的兼容重载。`reset()` 清输出但不重置 `last_update_`，若行为需要完全重新计时必须显式设计。

`runtime_control_chain<T>` 以 `control_stage<T>` 虚接口串联同类型控制阶段，阶段可由配置或注册表动态组装；PID、Ramp、限幅器和执行器适配器可以分别实现为阶段。数据源不由 PID 持有，控制任务每周期提供 reference、feedback 和 dt。

## Singleton、concept 与作用域工具

- `singleton_base<T>` 提供进程级静态实例并禁止复制/移动；具体单例仍需合规 `info_type` 和 `init(info)`。
- `concepts.hpp` 定义 `package`、不可复制/移动基类、模板实例识别等编译期约束。`utils::pair` 的 `first/left` 与 `second/right` 始终别名到自身的底层 `std::pair`，复制和移动后不会指向源对象。
- `function_arg_t` 用于提取一元可调用对象的参数类型，支持自由函数/函数指针、`const` 或 `mutable` lambda、`noexcept` 可调用对象及它们的转发引用。
- `defer` 宏构造作用域退出回调，捕获当前作用域引用；不得让捕获对象早于 defer 析构。

## Matrix 与 RLS

`Matrix.hpp` 是固定尺寸矩阵模板，默认构造会清零内部数据，转置、分块和对角矩阵操作保留元素类型。`inv()` 使用带部分主元的 Gauss-Jordan 消元；某列无非零主元时保持历史契约，返回全零矩阵。`RLS.hpp` 在其上实现递归最小二乘，`update()` 保存更新后参数对当前样本的估计输出，`reset()` 同时清空输出和更新计数。当前 RLS 主要为未接入的功率管理草稿服务，仍缺少条件数/发散监控和与主流程的集成。修改时需要构造已知解、边界和数值稳定性测试。

成熟度：字节长度检查与回调顺序 **已接入**；协议端序和控制时间语义仍需完善。

## 修改 Utils 时的检查清单

- concept 变化是否让所有现有类型继续实例化。
- 数值函数对零、符号、饱和、溢出、NaN 和周期边界的行为是否明确。
- 异步回调参数与缓冲生命周期是否安全。
- 字节工具是否误把主机内存布局当成外部协议。
- 增加无硬件单测，构建 Debug/Release 主目标，并在单测中加载校验四份车型配置。
