# App

## 目录职责

本目录存放上层应用逻辑，尽量不直接写底层寄存器操作，方便后续移植到其他工程。

| 目录 | 作用 |
| --- | --- |
| `MapRun/` | 当前题目地图入口：M1~M5、灰度/BNO085 串级、滚球摄像头 PD、速度环、计时和停车。 |
| `StepperDemo/` | 旧步进往返与灰度校准演示，当前主入口不调用。 |
| `StateMachine/` | 主状态机，只负责决定每一步调度哪个任务，不再堆放具体算法。 |
| `LineFollow/` | 巡线误差计算和左右轮目标速度修正。 |
| `AngleLoop/` | 角度环，把 yaw 误差转换成转弯轮目标速度。 |
| `AngleTest/` | 蓝牙 `ANGLE` 命令对应的角度环测试模式。 |
| `SensorTask/` | 参考状态机的旧灰度采样任务，当前 MapRun 不调用。 |
| `SpeedTask/` | 周期性测速、更新左右轮速度环，并输出 VOFA/AI 调参数据。 |
| `OLEDView/` | OLED 状态显示和灰度传感器格式化。 |

## 调度关系

```text
当前构建：
MapRun
  -> Sensor + BNO085
  -> grayscale P
  -> motor speed loop
  -> camera x/dt -> ball PD -> stepper target position
  -> A-line finish detection
  -> OLED

参考状态机构建：
StateMachine -> SensorTask -> UART -> LineFollow/AngleTest -> SpeedTask -> OLEDView
```

## 修改建议

| 想调整的功能 | 优先修改 |
| --- | --- |
| 当前地图巡线回中、抖动、丢线 | `MapRun/map_line_controller.c` |
| 当前地图基础速度和安全停机 | `MapRun/map_run.c` |
| 当前地图计时、停车和速度调度 | `MapRun/map_run.c` |
| 摄像头滚球位置 PD 和齿轮齿条脉冲换算 | `MapRun/ball_beam_controller.c` |
| OLED 页面内容 | `OLEDView/` |
| 角度环单独测试 | `AngleLoop/` 和 `AngleTest/` |

移植时优先复制本目录和 `Drivers/`。如果只想改某一块逻辑，优先改对应子模块。
