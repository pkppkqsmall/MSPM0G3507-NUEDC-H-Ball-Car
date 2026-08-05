# BNO085 I2C 驱动

本目录将 BNO085 接到原 MPU6050 使用的硬件资源：

| 信号 | MSPM0G3507 引脚 | 说明 |
|---|---|---|
| SDA | PA28 | I2C0，400 kHz |
| SCL | PA31 | I2C0，400 kHz |
| H_INTN | PB1 | 低有效，下降沿中断 |
| PS0、PS1 | 模块侧拉低 | 上电时选择 I2C 模式 |
| SA0 | 模块侧配置 | 驱动依次尝试 `0x4A`、`0x4B` |

## 软件结构

- `bno085.c/.h`：初始化 Game Rotation Vector、解析四元数并更新 `yaw/pitch/roll`。
- `bno085_i2c.c/.h`：MSPM0 DriverLib I2C HAL 和两阶段 SHTP 读取。
- `sh2/`：CEVA 官方 SH-2/SHTP C 库，来源提交
  `b514b1e2586ddc195e553dac89fc94c637b25298`，按 Apache-2.0
  许可证保留原文件和 `NOTICE.txt`。

GPIO 中断只调用 `BNO085_NotifyDataReadyFromIsr()` 置位。阻塞 I2C、
SHTP 解析和浮点姿态解算都由状态机在主循环中调用
`BNO085_UpdateIfDataReady()` 完成。

当前使用 `SH2_GAME_ROTATION_VECTOR`，50 Hz。它不依赖磁力计，适合电机
磁场较强的小车；仍需通过实车确认模块安装方向。项目约定正向旋转时 yaw 增大，
若方向相反，只修改 `bno085.c` 中的 `BNO085_YAW_DIRECTION`。
