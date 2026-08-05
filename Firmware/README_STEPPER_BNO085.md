# 步进电机、BNO085 与感为灰度 OLED 示例

> 本文描述 `App/StepperDemo` 独立演示。当前主入口已经切换为题目地图
> `App/MapRun`，步进电机只初始化并保持停止；当前运行方法见
> [`README_MAP_RUN.md`](README_MAP_RUN.md)。

## StepperDemo 功能

上电后程序按以下顺序循环：

1. STEP 和 DIR 保持低电平，应用初始化 OLED、感为灰度与 BNO085。
2. 再等待 500 ms。
3. DIR 置低，等待 20 us，以 8000 Hz 输出设定圈数对应的 STEP 脉冲。
4. 停止 1 秒。
5. DIR 置高，等待 20 us，再输出相同圈数对应的 STEP 脉冲。
6. 停止 1 秒后重复。

步进脉冲由 TIMG12 硬件 PWM 输出；定时器中断只累计脉冲和软件位置。
BNO085 的 I2C/SH-2 解析、灰度串行读取、按键校准和 OLED 刷新都在主循环执行。

## 引脚

| 功能 | 外设/工程宏 | MSPM0G3507 引脚 |
| --- | --- | --- |
| STEP | `PWM_STEPPER` / TIMG12 CCP1 | PA25 |
| DIR | `GPIO_STEPPER_DIR_PIN` | PB5 |
| OLED SDA/SCL | I2C1 | PB3 / PB2 |
| BNO085 SDA/SCL | I2C0 | PA28 / PA31 |
| BNO085 INT | `GPIO_BNO085_PIN_BNO085_INT_PIN` | PB1 |
| 灰度 CLK | `Sensor_CLK`，串行时钟输出 | PA15 |
| 灰度 DAT | `Sensor_DAT`，3.3V 上拉输入 | PA16 |
| 灰度 KEY | `Sensor_KEY`，只拉低/高阻释放 | PA13 |
| 板载校准按键 | `GPIO_Button_PIN_Button_PIN` | PB21 |

BNO085 的 PS0、PS1 必须在上电时保持低电平以选择 I2C 模式。驱动会依次
尝试地址 `0x4A` 和 `0x4B`。

## OLED 页面

```text
Turn:    +xxx
Step: +xxx.x
Yaw : +xxx.x
Gray:xxxxxxxx SR
```

- `Turn`：相对上电位置累计完成的整数圈数。
- `Step`：不足一整圈部分的指令角度，范围小于一圈。
- `Yaw`：BNO085 Game Rotation Vector 输出的 yaw。
- `Gray`：从 8 号到 1 号探头显示，约定 `1=黑线、0=白底`。
- `SR`：灰度驱动处于串行模式；单向串行接口无法判断模块是否真正在线。

`Step` 不是 ZDT 内部编码器反馈的真实机械角度。若需要真实角度，必须再接入
ZDT 驱动器的通信反馈协议。

感为灰度模块的串行接法、电平注意事项和 B21 校准流程见
`Drivers/Sensor/README.md`。使用串行 DAT 时需要安装模块的“开漏模式”
跳线帽并断电重启，但不能安装 SCL、SDA 的 5V 上拉跳线帽。

切换回 `StepperDemo` 入口后，正常页面按一次 B21 会进入灰度校准。程序先模拟长按 KEY，随后 OLED
依次提示放置黑场和白场；每次放稳后按一次 B21 确认。校准请求会在当前
STEP 高脉冲结束后停机，完成后恢复演示。

## 细分设置

角度换算参数位于 `Drivers/StepperMotor/stepper_motor.h`：

```c
#define STEPPER_FULL_STEPS_PER_REVOLUTION    (200UL)
#define STEPPER_MICROSTEP_DIVISOR            (16UL)
```

当前默认按 16 细分计算，即 `3200 pulse/rev`。ZDT 上位机也必须设置为 16，
否则软件显示的圈数和角度将与实际机械运动不一致。

每次运动的圈数位于 `App/StepperDemo/stepper_demo.c`：

```c
#define STEPPER_DEMO_MOVE_REVOLUTIONS       (1UL)
```

例如改为 `10UL`，程序就会正转 10 圈、停止 1 秒、反转 10 圈，再循环。
程序会结合细分参数自动换算脉冲数，无需手工计算。
