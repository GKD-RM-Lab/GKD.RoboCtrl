# 网络控制与调参遥测

路径：`include/device/network_protocol.hpp`、`device/aim_link.hpp`、
`device/remote_logger.hpp`、`io/udp_server.hpp` 及对应实现。

## 传输、生命周期与安全边界

`io::udp_server` 是共享 UDP 监听端点。`info_type` 的 `key_`、`address`、`port`
直接用于运行时 `udp_servers` 配置；构造只检查地址，`connect()` 打开并绑定 socket，
设备 `connect()` 注册接收回调，最后显式 `start()`。默认监听 `0.0.0.0:11451`，
测试可以使用端口 0 由系统分配端口。没有网络配置时不创建网络端点。

IO 层仅按配置的远端 IP 和源端口分发原始数据报，设备层再按 header 解释。
同一来源可以承载多个云台和导航 header；未知来源、未知 header、长度不匹配、
非有限数值和非法 bool/旧模式编号不刷新目标时间。源地址过滤不是身份认证；
该旧协议没有序号、发送时间戳、校验和或重放检测，需部署在受控网络。

发送参数及目标 endpoint 入队时复制，由每个 socket 唯一 writer 协程依序发送，
保证异步写期间缓冲有效且不互相覆盖。等待队列上限 64 KiB，超限丢弃新包并记录；
发送失败记录并继续处理后续数据报。UDP 不承诺送达、次序或重传。

`aim_link`、`navigation_link`、`remote_logger` 各自拥有字符串 `info_type`，
通过 `udp_name` 引用监听端点，通过 `address/port` 固定其收发对端。
网络设备只发布目标/状态，不改变 Robot 模式，不使能电机，也不自行发射。
控制层决定手动/视觉目标切换、导航许可、搜索和发射，仍受 `NoForce`、
遥控失联、设备在线、回中完成、裁判与发射器使能等现有门控约束。

## 视觉协议

旧来源为 `GKD_Control/include/robot.hpp` 的 packed `Auto_aim_control`
和 `SendAutoAimInfo`。线序明确为 IEEE-754 小端，不再把报文直接转换为结构体。
角度单位为 rad，绝对 yaw/pitch 的方向须与配置后的 IMU 坐标一致。

| 方向 | 字节 | 内容 |
| --- | --- | --- |
| 接收 | 0 | 配置的 `header`，不可为导航保留值 `0x37` |
| 接收 | 1–4、5–8 | yaw、pitch，float32 LE |
| 接收 | 9 | fire，严格 0/1 |
| 接收 `legacy_14` | 10–13 | 旧 `ROBOT_MODE`，uint32 LE，范围 0–5 |
| 发送 | 0、1–4、5–8、9 | header、yaw、pitch、red，固定 10 字节 |

`wire_format: legacy_14` 严格要求 14 字节，依据旧工程普通 Linux 枚举 ABI 的
四字节 `ROBOT_MODE`；旧头文件未显式固定枚举宽度。
`wire_format: compact_10` 是显式的 10 字节对端兼容选项，不自动猜测长度。
部署前必须确认真实视觉端格式。旧 mode 字段仅供诊断保留，绝不把网络包中的
mode 当作解锁或整机状态切换命令。

`target()` 返回 `optional<aim_target>`；收到完整有效包之前、超过
`target_timeout_ms`（默认 100 ms，边界时刻即失效）或时间倒退时返回空。
无效包不能延长前一 fire 请求寿命。`send_posture(yaw,pitch,red)` 编码发送反馈。
两个云台用不同 `aim_link` key/header 独立接收与维护新鲜度。

## 导航协议

旧 `ReceiveNavigationInfo` **只有** `0x37 + vx(float32 LE) + vy(float32 LE)`，
固定 9 字节；没有角速度、搜索或整机模式字段，不为旧报文虚构这些字段。
`command()` 发布平移速度 m/s，默认 `target_timeout_ms: 200`；控制层负责其坐标
变换和速度限幅，过期时停止采用导航目标。

`send_status(yaw,hp_fraction,match_started)` 固定发送 10 字节：`0x37`、yaw rad、
HP 比例 `[0,1]`、比赛开始 bool。无裁判有效数据时由调用方发送保守状态，
不得进行除零。旧哨兵正常搜索/导航曾被调试无限循环阻断；迁移不复制该阻塞，
新控制状态接入也不构成旧实车功能已验证的证据。

## RemoteLogger

`remote_logger` 保留旧 UDP 二进制格式：每条记录以 `uint16 LE 总长度` 和
`uint8 type` 开头，一份 UDP 数据报可串联多条记录。

| type | payload |
| --- | --- |
| 0 | FNV-1a uint32 LE 名称 ID、uint8 名称字节数、名称（1–255 字节） |
| 1 | uint32 LE ID、IEEE-754 float64 LE 数值 |
| 2、3 | uint16 LE 文本字节数、文本；分别为控制台和消息框 |

`push_value(name,value)` 在**每个**样本数据报内先发送名称注册，再发送数值，
避免接收器晚启动或丢失首包后无法识别后续样本。拒绝非有限数值和超长字段；
logger 只出站，不执行任何远程调参或执行器命令。`push_console_message()` 和
`push_message_box()` 提供原文本通道；抓取/绘图工具见
[`ballistics-tools.md`](ballistics-tools.md)。

## 软件验证与剩余限制

`tests/network_protocol_tests.cpp` 检查已知小端向量、精确帧长、非法 header/bool/
mode/NaN/Inf、导航结构、超时边界、FNV-1a 和日志编码边界。
`tests/network_transport_tests.cpp` 是独立 loopback 测试，不打开 CAN、串口，
验证来源过滤、共享来源按 header 分流、双向视觉/导航、两次排队发送的缓冲所有权、
RemoteLogger 注册加样本、无效包不刷新 fire 及异步回调生命周期。

旧报文没有校验和/序号，无法区分合法旧包与新包；新鲜度只依据本机接收时间。
对端端口、视觉角度方向、导航坐标和真实链路延迟仍需部署联调。
本模块的单测和 loopback 通过不能代表 Linux 总线构建、台架或实车验证通过。
