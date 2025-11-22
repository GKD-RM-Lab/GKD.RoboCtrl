# GKD.Roboctrl
这是新一代的电控代码。相比于老代码，这一版代码的结构更加清晰，更容易上手。

[文档](https://gkd-rm-lab.github.io/GKD.RoboCtrl/)

## 多例
在机器人的电控中，有很多类是会在初始化时创建多个对象，并且生命周期几乎是从程序开始到结束，例如马达对象，Can对象等等 ，我们称这种对象为多例。我们希望有一个简单的拿到各个实例的方法。

在代码中，我们抽象出了一个多例类的管理模式：每个多例类有一个 `info_type`，用于存放用来初始化这个类的相关信息。`info_type` 中有一个 叫做 `key_type` 的类型，这是用于区分各个多例对象的“标识符”。例如，can对象的 `key_type` 是一个字符串，表示这个can的名称；那么如果有一 个 can 叫"can0"，在初始化后，就可以用 `roboctrl::get<can_io>("can0")` 来获取到这个can对象的引用。

为了满足这个管理模式，所有的多例类都应该有一个接受 `const info_type&` 参数的构造函数，用于初始化这个多例对象。在程序的初始化阶段，用户会调用 `roboctrl::init()` 函数，并传入 `info_type` 来初始化。

多例模块文档 : @ref roboctrl::multiton

##单例 

除了多例，我们也支持单例模式。要编写一个单例类 T，你需要让 T 继承自 `roboctrl::utils::singleton_base<T>` ,并和多例模式一样添加一个用于初始化的 `info_type` 类型，并实现一个名为 init() 的成员函数，该函数接受 info_type 类型的参数。

 在初始化时，init() 函数会被调用，你应该通过其返回值是 true 还是 false 来表明是否成功初始化。

对于一个单例类T，你可以通过 T::instance() 或者 roboctrl::get<T>() 来得到它的实例。

单例文档： @ref roboctrl::utils::singleton_base

## 异步

我们在这版电控中把之前的基于多线程的逻辑改为了 **单线程异步** 。这样做有几个好处：一是不用考虑线程同步和线程安全相关的问题，二是一些复杂的逻辑可以通过异步函数更方便地写出。我们使用ZZ大人极力推崇的 `think-asio` 来提供异步功能。如果你没用过异步编程，建议先学学隔壁python的异步编程是怎么个事。

众所周知，异步代码需要一个上下文来运行，在我们的项目中，异步上下文是 `roboctrl::task_context` 。`task_context` 是一个单例类，所以你可以用 `roboctrl::get<roboctrl::task_context>()` 来获取到它的实例。

`task_context` 封装了异步代码的几个核心操作：

- `roboctrl::async::task_context::spawn` : 提交一个异步函数到任务队列。
- `roboctrl::async::task_context::post ` : 提交一个普通函数到任务队列，并可以传递参数。
- `roboctrl::async::task_context::run` : 开始运行异步任务。

由于 `task_context` 是单例，每次都用 `roboctrl::get` 获取实例不免麻烦，因此我们也提供几个函数直接提交任务：

- `roboctrl::spawn`
- `roboctrl::post`
- `roboctrl::run`

这几个函数的功能和前面几个完全一样。

### 使用异步函数

在我们的项目中，异步函数的返回值是被 `roboctrl::awaitable<T>` 包裹的，例如原来返回 `void` 的函数，异步时应该返回 `awaitable<void>` 。这个`awaitable` 实际上是 `asio::awaitable` 的别名。

除此之外，相比于正常函数，异步函数中还需要注意在调用另一个异步函数时，需要加上 `co_await` ，以及返回时应该使用 `co_return` 而不是 `return` 。例如：

```cpp 
roboctrl::awaitable<int> add(int a,int b){
  co_await roboctrl::wait_for(1s);
  co_return a+b;
}

roboctrl::awaitable<void> task(){
  std::printf("1 + 1 = {}",co_await add(1,1));
  co_return;
}
```

### 工具函数

我们提供几个常用的异步函数方便调用：

- `roboctrl::wait_for` : 等待指定时间
- `roboctrl::yield` : 让出当前函数的控制权

异步模块文档： @ref roboctrl::async


## IO

本项目提供统一的 IO 抽象，根据接收到信息后回调的方式分为两类：

- 裸 IO：直接分发消息，例如总线，见 `roboctrl::io::bare_io_base`。
- 带键值 IO：根据信息携带的“键”分发消息，例如CAN，见 `roboctrl::io::keyed_io_base`。

两类 IO 都可以通过 `on_data` 函数注册回调，并且支持重复注册回调多个回调函数；也支持通过 `send` 函数发送字节数组或 `utils::package` 。

### 裸 IO（bare_io_base）

派生类只需实现：

- `awaitable<void> send(byte_span)`：发送字节数据
- `awaitable<void> task()`：接收循环，接收到数据后调用 `dispatch()` 分发

注册回调：

```cpp
udp u{ udp::info_type{.key_ = "dbg", .address = "127.0.0.1", .port = 9000, .context = roboctrl::get<roboctrl::task_context>() } };
u.on_data([](roboctrl::io::byte_span bytes) -> roboctrl::awaitable<void> {
    // 处理收到的数据
    co_return;
});
```

发送数据（支持平凡类型自动序列化，见 `utils::package` 与 `utils::to_bytes/from_bytes`）：

```cpp
struct Pkg { int a; float b; };
static_assert(roboctrl::utils::package<Pkg>);

Pkg p{1, 2.0f};
co_await u.send(roboctrl::utils::to_bytes(p));
// 或使用泛型辅助：
co_await roboctrl::io::send(u, p);
```

### 带键值 IO（keyed_io_base）

和 `roboctrl::io::bare_io` 类似，但根据 key 分发数据，例如在 CAN 总线中用 CAN ID 作为键：

```cpp
can c{ can::info_type{ .can_name = "can0", .context = roboctrl::get<roboctrl::task_context>() } };

c.on_data(0x201u, [](roboctrl::io::byte_span buf) -> roboctrl::awaitable<void> {
    // 只处理 ID 为 0x201 的帧
    co_return;
});

co_await c.send(0x201u, std::span{some_data});
```

### 解析器（combined_parser 与内置单元）

在 `include/io/base.hpp` 中提供可组合的解析器基元：

- `nbytes<N>`：读取固定 N 字节
- `struct_data<T>`：将字节直接反序列化为平凡类型 T
- `fixed_data<bytes...>`：匹配固定字节序列
- `other_all`：消费剩余全部字节

可用 `combined_parser<P1,P2,...>` 串联：

```cpp
using parser_t = roboctrl::io::combined_parser<
    roboctrl::io::fixed_data<0xAA_b, 0x55_b>,
    roboctrl::io::nbytes<2>,
    roboctrl::io::struct_data<MyHeader>
>;
```

> 其中 `0xAA_b` 是在 `utils::byte_literals` 定义的字节字面量。

### 具体 IO 实现

- UDP：@ref roboctrl::io::udp
- TCP 客户端/服务端：@ref roboctrl::io::tcp 与 @ref roboctrl::io::tcp_server 。注意
- 串口：@ref roboctrl::io::serial
- CAN（SocketCAN）：@ref roboctrl::io::can

这些类均派生自上述基类，提供 `send()` 和 `task()` 协程接口，并可通过 `desc()` 输出简要描述。

### 回调与协程

所有回调既支持同步函数 `void(Args...)`，也支持协程函数 `awaitable<void>(Args...)`，内部统一包装为协程在 `task_context` 中调度，见 @ref roboctrl::callback。

## 设备

设备是对 IO 的进一步封装，代表了机器人上的某个物理实体，例如马达，传感器等等。设备对象通过 IO 对象与对应的实体进行通讯，解析上报的报文，并按格式封装并下发指令报文。

设备最基础的功能是判断是否掉线（offline）。我们认为，一个设备在一段时间内没有上报信息则认定为掉线。这些功能由 `roboctrl::device::device_base` 基类提供，因此所有设备类都应该继承自这个基类，在初始化时设定超时时间，并在接收到上报的报文时调用 `roboctrl::device::device_base::tick` 函数：

```cpp
xxx_io.on_data([&](const xxx_pkg& pkg){
  xxx; // 处理上报的信息
  tick();
})
```

可以通过 `roboctrl::device::device_base::offline` 或 `roboctrl::device::is_offline` 来判断一个设备是否掉线。

设备基类文档： @ref roboctrl::device::device_base

### 马达

马达设备有相似的功能，因此可以被进一步抽象。
