# IO 模块

路径：`include/io/`、`src/io/`

IO 层只负责字节传输和回调分发。typed callback 在 payload 长度不匹配时不会被调用，`from_bytes()` 对长度错误抛出异常而不是依赖 `assert`。

## 抽象层

`include/io/base.hpp` 定义两类 IO：

- `bare_io_base`：一段数据对应一次无 key 分发，适用于 UDP、TCP 等字节/报文通道。
- `keyed_io_base<Key>`：先按 key 找回调，再分发 payload，适用于 CAN ID 和串口逻辑通道。

二者都允许为同一通道注册多个同步或协程回调。底层收到数据后先复制到只读共享缓冲，保证分发协程执行期间缓冲仍有效；typed callback 会用 `utils::from_bytes<T>` 转换成平凡可复制结构体。keyed typed callback 还会记录 `sizeof(T)`，分发前先做严格长度匹配；同一个 key 注册冲突长度会立即抛出异常。单个回调抛出的异常会被记录并隔离，不会停止整个事件循环。

这层的开发理念是“传输与解释分离”：IO 只知道字节、通道 key 和帧边界，不知道 IMU、遥控器或电机的业务含义。协议字段、单位和设备状态应留在 Device 层。

## 生命周期

CAN、Serial、UDP 和主动 TCP 连接的构造函数只建立资源；长期接收协程由显式 `start()` 启动。初始化代码应先注册所有设备回调，再统一启动 IO。

`start()` 都通过 `started_` 保证幂等。主入口当前只构造并启动 CAN、Serial；UDP/TCP 类型可用但没有出现在车型配置和启动清单中。`tcp_server` 同样提供幂等 `start()`，使用者需要显式启动它。

所有 IO 的发送都经过对象内的串行写队列。`send()` 会先复制数据并在入队后返回，实际写入由唯一 writer 协程按顺序完成；队列超过 64 KiB 时丢弃新报文并记录告警，写失败时清空剩余队列并记录错误。

## 传输边界

- CAN：接收只接受完整 `can_frame`，并拒绝 DLC 超限、RTR 和错误帧；发送只接受带 ID 的 payload，不再暴露任意裸帧发送。
- TCP：只分发 `async_read_some` 返回的有效长度。
- UDP：分发实际接收长度，接收缓冲可容纳标准 UDP 最大数据报。
- Serial：发送和接收均使用当前兼容格式 `55 AA + key + 固定长度 payload`。接收采用增量缓冲和找帧头状态机，可处理分片、噪声和未知 key 的重同步；协议仍无显式 payload 长度、校验和及版本字段，属于**部分实现**。

### CAN / SocketCAN

`io::can` 是以接口名（如 `can0`、`CAN_CHASSIS`）为 multiton key 的 Linux SocketCAN 封装。构造函数同步执行 `socket/ioctl/bind` 并把 fd 交给 `asio::posix::stream_descriptor`；失败时抛异常，入口不会进入事件循环。接收循环只接受 `sizeof(can_frame)`，检查 DLC 和帧标志；发送 payload 上限是 8 字节，并通过串行写队列写入整帧。

注意：回调 key 当前直接使用 `can_frame.can_id`，扩展帧/RTR/错误标志是否需要 mask 必须由协议设计明确。此实现依赖 `<linux/can.h>`，不是 macOS 原生可运行实现。

成熟度：主流程 **已接入**；CAN-FD、错误帧恢复、总线重连和硬件状态监控尚未实现。

### Serial

`io::serial` 以逻辑名称为 multiton key，构造时同步打开设备文件并配置 8N1、无流控。接收协议为线序 `55 AA`、1 字节 key 和由回调注册推导出的固定 payload 长度；发送端会补齐同样的头和 key。接收采用增量缓冲，不假设一次 `async_read_some` 就对应一帧。

已知边界：

- 帧内没有 payload 长度、版本或校验和。
- 遇到未知 key 时无法知道 payload 长度，状态机会丢弃一个字节并继续寻找帧头；没有长度字段时无法做到完全可靠的恢复。
- 当前线序固定为小端兼容的 `55 AA`，后续协议升级应增加版本/长度字段。
- 同一 key 注册不同 typed 长度会立即报错，不再静默覆盖。

成熟度：当前 IMU/遥控器路径 **已接入**，协议健壮性 **部分实现**。

### UDP

`io::udp` 构造时创建并 `connect` 到固定远端，`start()` 后保留数据报边界，每次只分发实际接收长度。当前没有源地址分流、重连或丢包策略。

成熟度：实现存在、主流程 **未接入**。

### TCP 与 TCP Server

主动 `io::tcp` 构造时同步连接固定端点，`start()` 后用 `async_read_some` 分发数据块。TCP 只提供字节流，不保证一次 dispatch 对应一个应用层消息；设备协议必须自行做累计、定界和粘包/拆包处理。

`tcp_server` 接受连接后创建 `shared_ptr<tcp>`、保存在连接表中、启动连接接收任务并触发 `on_connect`；连接断开后会从连接表移除。当前没有断线重连或应用层协议解析。

成熟度：基础实现存在、主流程 **未接入**。

`combined_parser` 的组件统一实现 `parse(byte_span)`；组合器使用 `std::apply` 顺序消费 tuple 中的解析器。单次分发中的多个回调按注册顺序逐个 `co_await`，不同分发事件仍可能交错。

内置解析单元：`fixed_data<...>` 匹配固定魔数，`nbytes<N>` 复制定长字节，`struct_data<T>` 复制成平凡类型，`other_all` 引用剩余 span。组合器返回总消费长度，任一单元失败则返回 0。`other_all` 保存的 span 只在原共享缓冲生命周期内有效，不要长期缓存。

外部协议不得只依靠“平凡可复制”保证兼容；稳定协议还需要逐字段端序、宽度、对齐和版本定义。

## 修改 IO 时的检查清单

- 构造、注册回调、`connect/start` 的先后关系是否仍明确。
- 每次异步写使用的缓冲是否活到完成点，是否会被并发写覆盖。
- dispatch 使用的是实际有效长度，而不是整个固定缓冲。
- 未知 key、短帧、超长帧、断线和异常是否有可观测行为。
- typed 结构的 packing、大小、字节序和协议版本是否有文档与测试。
- 同步更新使用该协议的 Device 文档和无硬件解析测试。
