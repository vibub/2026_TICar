# HI219M / MSPM0G3507 联调

## 硬件链路

原理图确认的信号路径：

| HI219M | H745 底板网名 | M0 转接板 | MSPM0G3507 |
| --- | --- | --- | --- |
| TXD | IIC2_SDA / PB11 | M0_SDA | PB3 / UART3_RX |
| RXD | IIC2_SCL / PB10 | M0_SCL | PB2 / UART3_TX |
| VCC | VCC_3V3 | VCC_3V3 | 3.3 V |
| GND | GND | GND | GND |

底板沿用了 `IIC2_*` 网名，但 HI219M 器件符号上的引脚是 `RXD/TXD`，此处必须按 UART 使用，不能配置成 I2C。

## SysConfig 配置（唯一待完成项）

当前自动化会话没有 CCS SysConfig 接口，因此未直接修改 `TIcar.syscfg`。在 CCS 中打开 `TIcar.syscfg`，添加一个 UART 实例：

- Name: `UART_IMU`
- Peripheral: `UART3`
- TX: `PB2`
- RX: `PB3`
- Baud: `115200`
- Data bits: `8`
- Parity: `None`
- Stop bits: `1`
- FIFO: Enable
- RX FIFO threshold: One entry
- Interrupts: `RX`、`OVERRUN_ERROR`、`BREAK_ERROR`、`PARITY_ERROR`、`FRAMING_ERROR`、`NOISE_ERROR`

保存并重新构建后，`UART_IMU_INST` 宏出现，`bsp_imu.c` 会自动启用 UART3 ISR；无需再改业务代码。

`Bsp_Imu_Init()` 会在 UART 初始化后等待 100 ms，并自动发送老工程已验证的
`AT+MODE=0` / `AT+RST` 启动序列。`g_imu_legacy_start_count == 1` 且
`g_imu_uart_tx_count == 19` 表示两条命令已写入 UART3。

## CCS Watch 验收

观察全局变量 `g_imu`：

1. `hardware_ready == 1`：SysConfig 名称和构建已生效。
2. `g_imu_legacy_start_count == 1`、`g_imu_uart_tx_count == 19`：启动命令已发送。
3. `byte_count` 持续增加：UART 接收链路通。
4. `sync_count` 持续增加：波特率、8N1 和 TX/RX 方向正确。
5. `valid_frame_count`、`data_frame_count` 持续增加且 `crc_error_count` 基本为 0：协议通。
6. 转动车体，`euler_deg[2]`（yaw）变化；抬头/侧倾时 `euler_deg[1]` / `[0]` 变化。
7. `protocol == 1` 表示老 HI219 TLV，`protocol == 2` 表示当前 HI91。

若 `byte_count` 为 0，先查 3.3 V、共地和转接板插接方向；若有字节但 `sync_count` 为 0，优先查串口参数；若同步正常但 CRC 持续错误，查信号完整性和是否误设了偶校验。
