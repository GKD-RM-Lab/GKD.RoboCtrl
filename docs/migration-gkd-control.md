# GKD_Control 逻辑迁移记录

来源目录：`/Users/junity/code/GKD_Control`。本记录区分“旧源码存在”和“旧主流程实际使用”，避免把休眠或被旁路代码直接包装成当前可用功能。

## 已迁移并接入

- 遥控器：52 字节输入布局、解锁手势、键鼠与遥控通道映射、R/F 边沿切换、摩擦轮与开火输入。
- Robot 安全门：默认 `NoForce`，解锁进入 `FollowGimbal`，ControlPad 100 ms 失联回到 `NoForce`。
- 底盘：云台坐标变换、麦轮分解、统一比例限速、停止自旋后的方向保持和跟随回中。
- 云台：IMU yaw/pitch 角度外环、DJI 电机速度目标、相对 yaw 向底盘同步、依赖离线时零输出。
- 发射：摩擦轮斜坡、实际转速就绪门、连续拨弹命令、过流低速堵转检测与 50 ms 暂停。

迁移采用当前仓库的单线程 Asio、multiton/singleton、阶段化启动和 `motor_base` 运行时多态，没有复制旧工程的线程、硬件管理器或全局 `Robot_set`。

## 配置迁移到 YAML

`GKD_Control/include/configs/config_<type>.hpp` 是 YAML 配置的硬件拓扑参考：CAN 名、串口、底盘轮序、电机型号/ID、半径、PID 和控制周期被映射到 `configs/<type>.yaml` 的现有 `info_type` 字段。新格式不再另建 `*Spec`；例如旧 `DJIMotorConfig` 映射为 `dji_motors` 中一项，旧 `ChassisConfig`/`GimbalConfig`/`ShootConfig` 中当前控制层已经拥有的字段分别映射到 `robot.*_info`。

这不是旧文件的逐字段机械复制。旧工程含有目前 `info_type` 未表达的 `YawOffSet`、相对角 PID、视觉端口、超级电容、裁判、双云台/M9025 与搜索策略；这些字段不会悄悄写入 YAML，也不会因文件存在而生效。Sentry YAML 仍仅配置已接入的底盘，保留旧双云台配置作为后续具体 `gimbal` 子类的设计输入。旧 PID、方向和速度单位必须以当前控制循环的单位和安全台架重新标定，不能把“旧工程参数存在”当作实车验证。

## 明确未迁移

- 裁判系统与 UI：旧 `Robot_ctrl` 中启动代码被注释；当前也缺少完整串口帧定界、CRC、协议版本和接入配置。
- 超级电容：旧主流程的初始化和发送被注释；当前设备实现仍未接入入口。
- 功率管理：旧底盘会计算 `getControlledOutput()`，但随后实际写入电机的是原始 `wheels_pid[i].out`，计算结果没有生效；在补齐模型验证和统一安全门前不宣称迁移完成。
- 弹道解算：`BulletSolver` 在旧工程中没有调用点。
- 视觉/导航 Socket：旧协议依赖车型宏、固定本机端口和隐含 packed 布局；当前 TCP/UDP 抽象尚未配置对应消息边界，因此未直接搬运。
- Sentry 双云台、J6006/M9025：当前 Sentry 配置只启用底盘，M9025 仍是设备骨架，不能安全承接旧哨兵云台逻辑。

## 验证边界

控制映射、发射互锁、配置校验与 YAML 解析由无硬件单元测试覆盖。macOS 上无法完整构建 Linux SocketCAN 主目标，且本次没有运行主程序、台架或实车验证。CAN ID、电机方向、PID、IMU 角速度缩放和摩擦轮/拨弹阈值必须在目标 Linux 主机与安全台架上复核后才能上车。
