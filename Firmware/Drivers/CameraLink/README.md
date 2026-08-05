# 摄像头 BLE 串口链路

## 接口

| 项目 | 当前配置 |
| --- | --- |
| 外设 | `UART2` / `UART_CAMERA` |
| MCU TX | `PB17` |
| MCU RX | `PB18` |
| 波特率 | `115200` |
| 数据格式 | `8N1`，无流控 |
| 电平 | `3.3V UART` |
| 消息格式 | ASCII，以 `\r\n` 结束 |

## 接线

| BLE 从机 | MSPM0G3507 |
| --- | --- |
| TX | PB18 / `UART_CAMERA RX` |
| RX | PB17 / `UART_CAMERA TX` |
| GND | GND |

BLE 模块的供电电压按模块规格连接，但 UART 信号必须是 3.3V 电平。
当前数据方向是 MaixCAM 经 BLE 主从模块发送到 MSPM0，因此只接 BLE TX、
PB18 和共地也能完成单向接收；PB17 留作后续应答。

## 软件行为

- UART2 ISR 只把字节写入 512 字节环形缓冲。
- 主循环每轮排空当前积压字节，接受 CR、LF 或 CRLF，并始终保存最近一条
  不超过 127 字节的完整消息。应用控制优先使用最新帧，避免摄像头帧率高于
  主循环频率时旧消息持续积压并导致环形缓冲溢出。
- 摄像头消息仍会被后台解析；当前
  `MAP_RUN_AUTO_CAMERA_FORWARD_ENABLED=0U`，UART0 调试蓝牙不再自动输出
  `[CAM RX] 摄像头原始消息`。
- 原有 UART0 的蓝牙调试口发送 `CAM?`，可查看接收字节数、完整帧数、
  溢出次数、消息年龄和最近一帧内容。
- `App/MapRun/ball_beam_controller.c` 解析以下滚球控制帧：

  ```text
  BALL,frame_id,timestamp_ms,x_mm,valid\r\n
  [BALL] POS:+11.5cm\r\n
  ```

- `frame_id` 必须随新图递增，`timestamp_ms` 使用摄像头采集时间，
  `x_mm` 约定左负右正，找到小球时 `valid=1`，丢球时发送 `x_mm=0,valid=0`。
- 偶发 `valid=0` 或超过 `40 ms` 没有新帧时不会立即清空控制：MSPM0 最多
  用最后有效位置和球速预测 `100 ms`，预测位移限制 `±12 mm`；持续丢球
  仍由原失联保护停机。
- 简化的 `[BALL] POS:` 格式会自动把 cm 换算为 mm，并使用 MSPM0
  收到完整行的时间计算 `dt`。该方式可直接测试，但时间精度低于标准协议。
- 只有严格匹配该格式的行才会进入滚球 PD；如需临时查看所有原始行，
  可将 `MAP_RUN_AUTO_CAMERA_FORWARD_ENABLED` 改为 `1U`。
- 摄像头只负责位置测量和时间戳，PD、失联保护、齿轮齿条脉冲换算均在
  MSPM0 主循环执行。
