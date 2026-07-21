# LSM6DSV16X IMU 联调交接

## 当前结论

- LSM6DSV16X 已通过 MSPM0 的 I2C1 正常通信。
- `WHO_AM_I = 0x70`，当前模块地址为 `0x6A`。
- 加速度、角速度、SFLP 四元数和欧拉角均可连续更新。
- 陀螺仪静止校准完成时，`g_imu_calibration_status` 从 1 变为 2。
- 相对航向已验证：车体顺时针向右转为负角度，逆时针向左转为正角度。
- 已提供低速 IMU 航向保持测试模式 8，以及无需触摸屏的 CCS Watch
  模式命令邮箱。

## 硬件连接

| LSM6DSV16X | MSPM0 |
| --- | --- |
| VCC | 3.3 V |
| GND | GND |
| AD0/MISO | GND |
| SDA/MOSI | PB3 / I2C1 SDA |
| SCL/SCLK | PB2 / I2C1 SCL |
| CS | 3.3 V |
| INT1、INT2 | 不连接 |

SysConfig 实例名必须保持为 `I2C_LSM6DSV16X`，使用 I2C1 Fast Mode
（400 kHz）。详细配置和 Watch 变量见 `IMU_BRINGUP.md`。

## 上电检查

复位后保持车辆静止约 3 秒，然后检查：

| Watch 变量 | 正常值 |
| --- | --- |
| `g_imu_init_status` | 3 |
| `g_imu_who_am_i` | 112 / `0x70` |
| `g_imu_i2c_address` | 106 / `0x6A` |
| `g_imu_i2c_error_count` | 0 |
| `g_imu_sample_count` | 持续增长 |
| `g_imu_sflp_count` | 持续增长 |
| `g_imu_calibration_status` | 最终变为 2 |
| `g_imu_heading_ready` | 最终变为 1 |

## 无触摸屏进入模式 8

1. 先架空驱动轮。
2. 在 CCS Watch 中添加 `g_app_debug_request_mode`、
   `g_app_debug_request_result`、`g_app_current_mode` 和
   `g_app_switch_state`。
3. 暂停 CPU，把 `g_app_debug_request_mode` 写为 8，再继续运行。
4. 正常情况下邮箱恢复为 255，`g_app_debug_request_result = 1`，
   `g_app_current_mode = 8`，`g_app_switch_state = 0`。
5. 停车时暂停 CPU，把 `g_app_debug_request_mode` 写为 0，再继续运行。

不要直接修改 `g_app_current_mode` 或 `g_app_debug_mode`。

## 当前硬件限制与待办

- 当前车上的 CCD 已拆下，模式 8 仍把 CCD 有效线作为运行门控。ADC 输入悬空时
  可能偶发误判一帧有效线，因此可能只出现一帧电机控制量；这不是 IMU 丢帧。
- CCD 装回车尾后，可以继续使用现有门控。若需要在 CCD 未安装时测试电机，应先
  单独实现一个有明确启停保护的临时门控，不要依赖悬空 ADC。
- K230 目前没有接入模式 8 的航向目标；模式 8 当前只保持进入时捕获的零航向。
- 触摸屏在最近一次测试后停止工作，原因尚未诊断。CCS Watch 命令邮箱可临时绕过
  触摸屏完成模式切换。

## 相关源文件

- `bsp_imu.c` / `bsp_imu.h`：I2C 驱动、FIFO、SFLP、校准和相对航向。
- `app_main.c`：模式 8 航向保持和 CCS Watch 命令邮箱。
- `app_config.h`：模式编号。
- `TIcar.syscfg`：I2C1、PB2/PB3 配置。
- `IMU_BRINGUP.md`：完整接线、配置和调试说明。
