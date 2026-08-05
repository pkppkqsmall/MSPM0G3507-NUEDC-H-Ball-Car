# MSPM0G3507 H 题地图底盘测试工程

## 项目概览

当前入口程序是 `App/MapRun`。上电默认 `M1`，对应 H 题第 2 问：
小车从 A 点朝 B 点放置，短按 B21 后按题图顺时针连续循迹；离开 A 点并
完成一圈后，重新检测到 A 点横线时自动停机并保留用时。`M3` 对应
第 4 问，以 `88 RPM` 从 A 行驶到 B，并用右二灰度与编码器组合停车。
`M4/M5` 分别对应第 5、6 问，两者共用带球模式的平滑起步、误差
整形、`85 RPM` 一圈循迹和自动滚球闭环；M4 固定保持中心，M5 使用
赛前设置的指定位置。`M2` 对应第 3 问，底盘保持停止，
滚球自动执行 `0 cm -> +5 cm -> -5 cm` 并在终点保持。
SW1/SW2 在停车时切换 `M1~M5`。完整操作见
[`README_MAP_RUN.md`](README_MAP_RUN.md)。

当前构建包含 `App/MapRun`、直流电机 PWM、双编码器、BNO085、步进电机、感为灰度串行读取、OLED 和基础中断驱动。PA25/PB5 步进平台在地图底盘测试阶段只初始化并保持停止，不会再自动正反转。

旧状态机、完整 UART 命令解析、MPU6050 和 `StepperDemo` 源码仍作为参考保留，但不由当前主入口调用。旧直角转弯和按转弯次数计圈模块已经移除。地图程序保留 UART0 VOFA 测速输出能力但默认关闭自动推送，并内置 `STOP/TARGET/PID/START/CAR?/MAP?/MODE?/MAP` 精简速度环测试和诊断命令。

## 文档索引

| 文档 | 作用 |
| --- | --- |
| `README_MAP_RUN.md` | 当前题目地图底盘程序的操作、状态流程、终点判定和调参入口。 |
| `README_STEPPER_BNO085.md` | 步进电机、BNO085、感为灰度串行接口和接线说明。 |
| `KnowledgeBase/stepper_ball_beam_kinematics.md` | 记录 V12 齿轮齿条摆杆的 STEP 实测尺寸、角度与 16 细分脉冲换算及安全限位。 |
| `KnowledgeBase/code_logic.md` | 按当前代码真实执行顺序整理地图巡线、速度环、蓝牙、OLED 和中断。 |
| `KnowledgeBase/car_parameters.md` | 记录小车硬件、传感器、电机、速度环和巡线参数。 |
| `KnowledgeBase/tuning_log.md` | 记录调车过程、现象、已验证参数和后续调参方向。 |
| `KnowledgeBase/CompetitionDB/` | 记录队伍能力、成员贡献、项目证据和电赛题目 AI 匹配提示词。 |

## 目录结构

```text
.
├── App/                    上层应用逻辑
│   ├── MapRun/             当前地图连续循迹和一圈停车应用
│   ├── StepperDemo/        旧步进往返演示，当前不调用
│   ├── StateMachine/       主状态机调度
│   ├── LineFollow/         巡线外环
│   ├── AngleLoop/          角度环算法
│   ├── AngleTest/          角度环测试模式
│   ├── SensorTask/         灰度传感器采样滤波
│   ├── SpeedTask/          测速、速度环和串口速度输出
│   └── OLEDView/           OLED 显示页面
├── Drivers/                底层驱动和硬件相关模块
├── KnowledgeBase/          参数记录、调试日志和经验文档
├── targetConfigs/          CCS 调试目标配置
├── timx_timer_mode_pwm_edge_sleep.c
└── timx_timer_mode_pwm_edge_sleep.syscfg
```

## 应用层 App

| 目录 | 作用 |
| --- | --- |
| `App/MapRun/` | 当前运行入口，负责题目模式选择、第三问滚球时序、B21 启停、连续循迹、速度环调度、停车、计时和 OLED。 |
| `App/StepperDemo/` | 独立步进往返和灰度校准演示，源码保留但当前主入口不调用。 |
| `App/StateMachine/` | 主状态机，只负责调度按键、蓝牙、传感器、运动控制、速度环和 OLED。 |
| `App/LineFollow/` | 巡线误差计算和左右轮目标速度修正。 |
| `App/AngleLoop/` | 角度环，把 MPU6050 的 yaw 误差转换成转弯轮目标速度。 |
| `App/AngleTest/` | 蓝牙 `ANGLE` 命令对应的角度环测试模式。 |
| `App/SensorTask/` | 参考状态机的旧灰度任务，使用三次多数表决；当前 MapRun 不调用。 |
| `App/SpeedTask/` | 按固定周期更新编码器测速、左右轮速度环，并输出 VOFA/AI 调参数据。 |
| `App/OLEDView/` | OLED 页面显示和灰度传感器状态格式化。 |

## 底层驱动 Drivers

| 目录 | 作用 |
| --- | --- |
| `Drivers/Button/` | 按键扫描、电机启停状态和目标速度入口。 |
| `Drivers/motor/` | PWM 输出、编码器测速、速度环和电机方向适配。 |
| `Drivers/StepperMotor/` | PA25 STEP、PB5 DIR 的非阻塞步进脉冲驱动；地图测试时保持停止。 |
| `Drivers/BNO085/` | I2C 读取 BNO085，并输出 yaw、pitch、roll。 |
| `Drivers/UART/` | 串口/蓝牙收发、命令解析、VOFA/AI 调参数据输出。 |
| `Drivers/Sensor/` | 感为 8 路灰度模块的 CLK/DAT 串行读取和 KEY 校准控制。 |
| `Drivers/MPU6050/` | MPU6050、DMP、I2C 和姿态角数据。 |
| `Drivers/OLED_Hardware_I2C/` | OLED I2C 显示驱动。 |
| `Drivers/MSPM0/` | 时钟、SysTick、中断等芯片平台辅助代码。 |

## 硬件约定

| 项目 | 当前约定 |
| --- | --- |
| 主控 | `LP_MSPM0G3507` |
| 电机驱动 | AT8236 或同类 H 桥驱动 |
| 当前底盘 | WHEELTEC R3X 加大版三轮差速底盘 |
| 底板尺寸 | `320 x 240 x 5 mm` |
| 左右驱动轮中心距 | 模型标注约 `214.2 mm`，装车后还需实测复核 |
| 驱动轮轴至万向轮轴距 | 模型标注约 `204.7 mm` |
| 左轮 | `AIN1/AIN2` |
| 右轮 | `BIN1/BIN2` |
| 轮胎直径 | `65 mm` |
| 电机减速比 | `1:28` |
| 编码器 | `13 PPR`，当前程序实测等效 `2` 倍频，电机轴约 `26` 计数/圈 |
| 车轮输出轴每圈计数 | `13 * 2 * 28 = 728`，以实测一圈约 `728` 为测速换算基准 |
| 速度正方向 | 正速度表示小车前进 |
| 右轮方向 | 右轮硬件方向与左轮相反，代码中已用方向符号统一 |

## 历史蓝牙命令

当前地图构建已排除 `Drivers/UART` 的旧完整命令模块。地图程序在自身内部实现了 `STOP/TARGET/PID/START/CAR?/LINE?/MAP?/MODE?/MAP/HELP` 速度环测试和巡线诊断命令。`TARGET` 后发送 `START` 进入速度测试，`MAP` 后发送 `START` 进入当前题目模式；M1/M3/M4/M5 执行灰度位置、BNO085 yaw 角速度和轮速 PI 串级巡线，M2 执行静止滚球时序。UART0 自动速度帧和摄像头原始帧转发当前均已关闭，可通过 `CAR?`、`BALL?`、`CAM?` 按需查询。下面其余 AI 调参和分轮 PID 命令仅供恢复旧状态机时参考。

| 项目 | 当前值 |
| --- | --- |
| UART | `UART0` |
| 波特率 | `115200` |
| 命令结束 | 换行符 |

```text
PID,0.15,0.2,0
PID?
TARGET,50
TARGET,50,48
TARGET?
LPID,0.15,0.2
RPID,0.15,0.2
ANGLE,65
ANGLE,-65
ANGLE?
ENC?
CAR?
AITUNE,ON
AITUNE,OFF
STOP
START
```

普通速度回传兼容 VOFA+ FireWater：

```text
speed:left_rpm,right_rpm\n
```

AI 调参模式使用 CSV 数据，供 `llm-pid-tuner` 解析。

## 按键和 OLED

| 按键 | 作用 |
| --- | --- |
| `B21` 短按 | 停止状态下起跑；M5/Q6 会先采集当前球位为目标再起跑；运行状态下一旦按下便立即手动停机。 |
| `B21` 长按 | 停止状态下长按至少 1.5 秒，进入灰度模块 KEY 校准流程。 |
| `SW1` | 停止状态下选择上一个题目模式，`M1` 再按会循环到 `M5`。 |
| `SW2` | 停止状态下选择下一个题目模式，`M5` 再按会循环到 `M1`。 |

| OLED 页面 | 内容 |
| --- | --- |
| 第 1 行 | 当前 `M1~M5`、对应题号、`READY/RUN/DONE/NOT READY` 和运行用时。 |
| 第 2 行 | 8 路灰度 `G:` 和巡线误差 `E:`。 |
| 第 3 行 | 左右轮实测 RPM 和当前黑线探头数量。 |
| 第 4 行 | 本次平均编码器路程计数和 BNO085 yaw。 |

M2/Q3 使用专用页面：第 1 行显示 `TO+5/TO-5/PASS/LATE` 和计时，
其余三行显示小球位置/目标、球速/误差、摆杆目标角/I 项输出。

## 地图循迹和停车

| 项目 | 当前约定 |
| --- | --- |
| 黑线 | `1` |
| 白底 | `0` |
| `INPUT1` / bit0 | 最左侧，单独命中时 OLED 显示 `00000001` |
| `INPUT8` / bit7 | 最右侧，单独命中时 OLED 显示 `10000000` |
| 起跑方向 | A 点朝 B 点，沿题图顺时针运行。 |
| 比赛速度 | M1 `120 RPM`；M3 `88 RPM`；M4/M5 静止预置 `200 ms` 后从 `30 RPM` 在 `1400 ms` 内升至 `85 RPM`。 |
| 巡线控制 | 灰度位置生成目标 yaw 角速度，BNO085 角速度环生成差速，左右轮速度 PI 跟踪目标。 |
| M4/M5 平滑 | 中心误差减半、外侧提前转向，内轮最低 `25 RPM`、外轮最高 `95 RPM`，单轮目标每 `10 ms` 最多增加 `3 RPM`。 |
| M4/M5 弯道 | 右三探头提前确认入弯；累计 `170~175°` 时渐减最低右转量，出弯再用 `50+200 ms` 平滑恢复灰度控制。 |
| M4/M5 滚球 | 起跑前检查目标 `±10 mm`；使用 `20 mm/s、0.75/s` 回目标包络、Q4 同类起步预置/前馈和终点缓停前馈。 |
| M5 指定位置 | 把球放到指定位置后短按 B21，程序采集当前摄像头 `X` 为本轮目标并立即起跑；不需要蓝牙。 |
| M4/M5 蓝牙 | 运行中每 `250 ms` 自动发送精简 `Q5BALL/Q6BALL`；完整 `BALLLOG` 仍可手动开启。 |
| BNO085 回退 | 连续 `150 ms` 无有效角速度时自动使用原灰度 P 内轮减速。 |
| 运行中丢线 | M4/M5 保持最后有效灰度方向并继续角速度环；其他模式保持最后轮速。 |
| 离开起点 | 平均编码器超过 `120 count` 且黑线探头不多于 3 路，连续确认 3 次。 |
| 终点解锁 | 已离开起点且平均编码器距离达到 `21000 count`。 |
| A 点停车 | 解锁后至少 3 路探头连续 3 次检测到黑线；M1 主动刹车 `100 ms`，M4/M5 用 `500 ms` S 曲线缓停并继续保持滚球目标。 |
| B 点停车 | M3 锁存右二探头黑线后达到 `5350 count`，或兜底达到 `5600 count`。 |

## 构建

在 CCS Theia 中直接构建本工程即可。命令行构建可在工程目录执行：

```powershell
& 'C:\ti\ccs2100\ccs\utils\bin\gmake.exe' -C '.\Debug' all -j 4
```

如果移动过源码目录但 CCS 没有自动刷新，请在 CCS 中刷新项目并重新构建；必要时重新生成 Debug 构建文件。

## 移植建议

优先整体复制 `App/`、`Drivers/` 和 `timx_timer_mode_pwm_edge_sleep.c`。移植到新芯片时，通常先替换 `Drivers/MSPM0/` 和 SysConfig 相关配置，再逐步验证 UART、OLED、灰度传感器、编码器、PWM、状态机、巡线和速度环。
