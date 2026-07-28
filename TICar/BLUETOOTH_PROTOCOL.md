# 双车蓝牙位姿速度协议

当前版本只传输两车普遍需要的位姿和速度，不包含任何比赛题目相关参数。

## SysConfig

主工程需要一个名为 `UART_HC05` 的 UART 实例：

- Peripheral：UART3
- TX：PB2
- RX：PB3
- 115200 baud、8 data bits、no parity、1 stop bit
- Enable FIFO
- RX FIFO threshold：one entry
- Interrupts：RX、overrun、break、parity、framing、noise error

实例名必须保持为 `UART_HC05`，源码根据该名称使用
`UART_HC05_INST` 和 `UART_HC05_INST_IRQHandler`。

## 数据单位

| 字段 | 类型 | 单位 |
| --- | --- | --- |
| `x_mm` | `int32_t` | mm |
| `y_mm` | `int32_t` | mm |
| `yaw_cdeg` | `int16_t` | 0.01° |
| `linear_mm_s` | `int16_t` | mm/s |
| `angular_cdeg_s` | `int16_t` | 0.01°/s |

所有多字节整数均使用小端序。正负方向由具体赛题建立统一坐标系后确定。

## 帧格式

每帧固定 23 字节：

| 偏移 | 长度 | 内容 |
| ---: | ---: | --- |
| 0 | 1 | 帧头 `0xA5` |
| 1 | 1 | 帧头 `0x5A` |
| 2 | 1 | 协议版本 `0x01` |
| 3 | 1 | 消息类型 `0x01`（位姿速度） |
| 4 | 1 | 载荷长度 `14` |
| 5 | 2 | 帧序号 `sequence` |
| 7 | 4 | `x_mm` |
| 11 | 4 | `y_mm` |
| 15 | 2 | `yaw_cdeg` |
| 17 | 2 | `linear_mm_s` |
| 19 | 2 | `angular_cdeg_s` |
| 21 | 2 | CRC16，小端序 |

CRC 使用 CRC-16/CCITT-FALSE，多项式 `0x1021`、初值 `0xFFFF`，
计算范围为偏移 2～20，不包含帧头和 CRC 字段。

## 调用方式

发送本车数据：

```c
BtPoseVelocity local = {
    .x_mm = 1200,
    .y_mm = -350,
    .yaw_cdeg = 9000,
    .linear_mm_s = 420,
    .angular_cdeg_s = 0
};

(void) Protocol_Bt_SendPoseVelocity(&local);
```

主循环中的 `Protocol_Bt_Task()` 自动解析接收字节。读取一帧新数据：

```c
BtPoseVelocity peer;
uint16_t sequence;

if (Protocol_Bt_TakeLatestPoseVelocity(&peer, &sequence) != 0U) {
    /* 使用 peer.x_mm 等字段。 */
}
```

`Protocol_Bt_IsRxFresh(max_age_ms)` 可判断对车数据是否仍然新鲜。当前版本没有
自动发送周期，调用方应根据题目需要设置发送频率，建议从 20 Hz（50 ms）开始。
