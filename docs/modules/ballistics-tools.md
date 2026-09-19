# 弹道预测与离线调参工具

路径：`include/utils/ballistics.hpp`、`src/utils/ballistics.cpp`、`tools/`、`examples/tuning.csv`。

成熟度：弹道算法、离线命令行入口和日志解析/绘图 **已实现**；弹道算法 **未接入主控制链**。旧 `BulletSolver` 也没有调用点，所以独立接口不自动驱动云台或生成发射许可。远程日志发送端由 [网络模块](network-control.md) 管理，采集工具只接收；示例数据是人工构造的阶跃响应，不是实车验证结果。

## 旧代码对应关系

| 来源 | 新接口/入口 | 保留与变更 |
| --- | --- | --- |
| `include/shoot/bullet_solver.hpp`、`src/shoot/bullet_solver.cc` | `utils::solve_ballistics()`、`ballistic_position()` | 保留线性阻力、平移预测、相邻装甲切换及快速旋转时瞄准前侧表面的策略；替换隐式状态和无效对数域为有限时间搜索与显式状态 |
| `BulletSolver::getResistanceCoefficient()` | `legacy_ballistic_drag(speed)` | 保留旧速度分段系数供显式选择，不自动当作实测标定参数 |
| `draw/draw.py` | `tools/plot_tuning.py` 的带时间戳文本解析 | 原脚本启动会清空源日志；新工具只读输入，离线生成图表 |
| `scripts/draw.py` | `tools/plot_tuning.py` 的摩擦轮日志解析 | 支持整数、小数和科学计数；左轮取反必须显式 `--invert left`，无时间戳必须声明采样周期 |
| `RemoteLogger` 调参消息 | `tools/tuning_log.py` | 接收旧兼容 little-endian 协议，保存统一 CSV，工作站侧显式运行 |
| 哨兵 `test_yaw_speed_pid()` / `test_yaw_position_pid()` CSV | `--columns time_ms,command,feedback` | 可读取历史 `speed.csv` / `pos.csv`；不迁移阻塞无限循环、硬编码写入路径或调试电流指令 |

旧 `log/yaw_log.csv/.txt` 的列含义并无可靠表头。工具要求操作者用 `--columns` 声明含义和时间单位，不根据文件名猜测。旧 `speed.csv` 中写入的 `output_current` 曾与实际 `1 >> yaw_motor` 命令不一致，不能用该日志证明执行器按记录值运行。

## 弹道模型与契约

所有输入采用米、秒、弧度：x 向前、y 向左、z 向上，yaw 绕 z 轴，pitch 正值表示抬高。旧 `getPitch()` 返回数学俯仰的负值；新接口明确返回数学俯仰，实际云台正负号应由调用方依据已标定坐标转换，不能直接复用旧负号。

`ballistic_config.drag = k` 是速度线性衰减系数，单位 `1/s`，采用 `dv/dt = -k*v - g*e_z`。令 `A(t) = (1-exp(-k*t))/k`，则弹丸位置为：

```text
x = speed*cos(pitch)*cos(yaw)*A(t)
y = speed*cos(pitch)*sin(yaw)*A(t)
z = speed*sin(pitch)*A(t) - gravity*(t-A(t))/k
```

`k=0` 使用真实无阻力极限 `A=t`、重力下降 `g*t²/2`；小 `k*t` 使用级数避免相减消失。旧实现把零阻力换成 `0.001`，新实现不再需要这一替代。

`ballistic_target.position/velocity` 描述目标中心及其匀速速度，`armor_yaw/yaw_speed` 描述装甲方位与角速度。`radius`、`alternate_radius` 为米，`alternate_height` 是四装甲相邻板的高度差。只在 `armor_count=4` 且选中相邻板时使用备用半径和高度；当前接受 1–16 块装甲。初始目标中心水平距离必须大于装甲半径。

保留旧策略：角速度绝对值低于 `max_tracking_yaw_speed`（默认 5 rad/s）时跟踪预测装甲角度；超过阈值时瞄准预测中心的前侧表面。后者是等待装甲经过的瞄准策略，**成功求解不保证实际装甲恰在弹丸到达时经过**，不能直接作为开火条件。相邻装甲切换使用包裹到 `[-π,π]` 的角差，避免旧代码跨角度边界误判。

新算法搜索 `norm(target(t)+gravity_drop*e_z)-speed*A(t)=0`，默认在 5 s 内扫描 512 个时间区间，随后最多二分 64 次，以 1 mm 空间误差为停止条件。与旧 20 次修正角度算法相比，数值求解方法变更，阻力轨迹与目标预测模型保持一致。返回状态：

- `success`：结果、目标位置和残差有限且满足指定精度。
- `invalid_config` / `invalid_input`：非有限数、非法速度/半径/装甲数量或参数范围。
- `no_intercept`：指定时间范围与扫描精度内没有括住交点，包含距离过远、目标过快和扫描遗漏切触解等情况；不构成全局不可达证明。
- `not_converged`：括住交点但迭代预算不足。

旧阻力表依次为 `<12.5 → 0.45`、`<15.5 → 1.0`、`<17 → 0.7`、`<24 → 0.55`、其余 `5.0`。没有源码证据证明这些数值适用于新机器人，尤其 30 m/s 档位应独立标定。API 默认无阻力供离线分析；CLI 必须显式提供阻力。纯函数不建立后台任务，若将来接入实时控制，应控制调用频率并计量最坏耗时，不能假定每 1 ms 调用合适。

## 无硬件入口

直接编译与运行只涉及独立算法，不打开总线、串口或网络：

```sh
clang++ -std=c++23 -Iinclude src/utils/ballistics.cpp tools/ballistics_cli.cpp -o /tmp/ballistics-cli
/tmp/ballistics-cli 10 0 0 20 0
/tmp/ballistics-cli 10 0 0 20 0 2 0 0
```

参数顺序为 `x_m y_m z_m speed_m_s drag_1_s [vx_m_s vy_m_s vz_m_s]`。输出 CSV 含 yaw、向上为正的 pitch、飞行时间、预测目标位置和残差；不可求解时返回非零。`ballistics-cli` 同时有 xmake 构建声明，不属于机器人启动入口。

绘图依赖 Python 3 标准库，无需 matplotlib/NumPy，也不联网。结果包含序列开关、窗口缩放与位置滑块：

```sh
python3 tools/plot_tuning.py examples/tuning.csv --output /tmp/tuning.html
python3 tools/plot_tuning.py /path/to/fric_log.txt --sample-period 0.002 --invert left --output /tmp/friction.html
python3 tools/plot_tuning.py /path/to/speed.csv --columns time_ms,command,feedback --output /tmp/yaw-speed.html
```

`0.002` 只是调用示例，必须换为生成该日志的实际采样周期。标准 CSV 支持 `time_s,name,value` 长表，以及 `time_s,target,feedback,...` / `time_ms,...` 宽表；带时间戳文本支持 `[YYYY-MM-DD HH:MM:SS] name: value`。工具拒绝非有限值和不一致列数，默认最多 100000 个样本。原始文件与输出路径必须不同，已有输出文件不会被覆盖。

工作站显式采集示例（本次没有执行此联网命令）：

```sh
python3 tools/tuning_log.py --bind 192.168.1.53 --peer 192.168.1.10 --port 8080 --duration 30 --output /tmp/captured.csv
```

地址是示例，应与实际网络配置一致。默认只监听 `127.0.0.1`。捕获时间来自工作站 `monotonic()`，是接收时间而非机器人采样时间；UDP 丢包和网络抖动仍会影响时序分析。注册名与更新消息同一报文重复发送，使稍晚启动的接收器也可解析。解码器校验总长度、字段长度、FNV-1a 名称 ID、UTF-8、浮点有限性，整包失败不提交部分注册；最多保存 4096 个名字。文本消息打印到 stderr，数值写入 CSV。注册冲突、短帧和未知消息会计入拒绝数。

## 验证与限制

```sh
clang++ -std=c++23 -Wall -Wextra -Werror -Iinclude -DROBOCTRL_BALLISTICS_TEST_MAIN src/utils/ballistics.cpp tests/ballistics_tests.cpp -o /tmp/ballistics-tests
/tmp/ballistics-tests
python3 -m unittest discover -s tests -p test_tuning_tools.py -v
```

C++ 测试覆盖无阻力解析低弹道、已知线性阻力轨迹、零阻力连续极限、匀速目标解析拦截、旋转/高自旋预测、不可达、非有限输入及迭代失败。Python 测试覆盖已知协议字节向量、重复注册、整包拒绝、CSV/旧日志单位、符号和 HTML 转义。

这些是软件数值与格式测试，不代表阻力已标定、云台坐标已确认、弹丸命中或真实 UDP 链路通过。没有运行机器人二进制或硬件调参循环。相关边界见 [开发验证规则](../development.md) 与 [Utils 模块](utils.md)。
