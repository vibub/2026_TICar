# LSM6DSV16X I2C bring-up

## Wiring (right-side 7-pin header shown in the module photo)

Connect the module directly to the MSPM0 for the first test. Do not route it
through the base board until the direct connection works.

| LSM6DSV16X module | MSPM0 | Purpose |
| --- | --- | --- |
| VCC | 3.3V | Module power |
| GND | GND | Common ground |
| AD0/MISO | GND | Select I2C address 0x6A |
| SDA/MOSI | PB3 | I2C1 SDA |
| SCL/SCLK | PB2 | I2C1 SCL |
| CS | 3.3V | Select I2C interface |
| INT1 | Not connected | Polling test does not use interrupts |

Remove the PB2/PB3 UART loopback jumper before powering the board. Power the
sensor from 3.3 V; do not use 5 V for this bring-up.

## SysConfig

1. Remove the `UART_IMU` UART instance so PB2/PB3 become available.
2. Add one **I2C** instance.
3. Set **Name** to `I2C_LSM6DSV16X` (the spelling must match exactly).
4. Enable **Controller Mode**.
5. Select **I2C1**.
6. Set bus speed to **400 kHz / Fast Mode**.
7. Assign **SCL = PB2** and **SDA = PB3**.
8. Do not enable I2C interrupts and do not add a GPIO interrupt yet.
9. Save SysConfig, then run **Project > Clean Project** and rebuild.

The module normally provides I2C pull-ups. If both SCL and SDA are not near
3.3 V while idle, add 4.7 kohm pull-ups to 3.3 V or inspect the module solder
jumpers.

## CCS Watch

Add these global variables:

- `g_imu_init_status`: 3 means initialization completed.
- `g_imu_who_am_i`: must be `0x70`.
- `g_imu_i2c_address`: normally `0x6A` with AD0 tied to GND; the code also
  probes `0x6B` automatically.
- `g_imu_i2c_error_count`: should remain zero after initialization.
- `g_imu_sample_count`: must continuously increase.
- `g_imu_sflp_count`: must continuously increase when sensor fusion is running.
- `g_imu_calibration_status`: keep the vehicle still until this changes from 1 to 2.
- `g_imu_calibration_sample_count`: increases to 180 during gyro calibration.
- `g_imu_gyro_bias_dps[0..2]`: measured gyro bias removed from later samples.
- `g_imu_fifo_overrun_count`: normally remains zero while running; stopping at
  a breakpoint can make it increase.
- `g_imu.acc_g[0]`, `[1]`, `[2]`: acceleration in g.
- `g_imu.gyro_dps[0]`, `[1]`, `[2]`: angular rate in degrees per second.
- `g_imu.quat[0..3]`: SFLP quaternion in x, y, z, w order.
- `g_imu.euler_deg[0..2]`: roll, pitch and relative yaw in degrees.
- `g_imu_heading_ready`: changes to 1 after calibration and automatic yaw zero.
- `g_imu_yaw_zero_deg`: raw SFLP yaw captured as the current forward direction.
- `g_imu_heading_deg`: zero-referenced heading wrapped to -180..+180 degrees.
- `g_imu_heading_zero_count`: 1 after automatic zero; increases after each
  successful manual `Bsp_Imu_ZeroYaw()` call.

Initialization status values:

| Value | Meaning |
| --- | --- |
| 0 | SysConfig did not generate `I2C_LSM6DSV16X` |
| 1 | Neither address returned WHO_AM_I 0x70; inspect power and wiring |
| 2 | Device was found but register configuration failed |
| 3 | Sensor is running |

The sensor uses 60 Hz output data rate, +/-4 g accelerometer range and +/-2000
dps gyroscope range. SFLP game rotation, accelerometer and gyroscope samples are
read from the FIFO without an external interrupt, so INT1/INT2 remain unconnected.

Keep the vehicle completely still for about three seconds after every reset.
Calibration restarts whenever any raw gyro axis exceeds 5 dps. SFLP is a six-axis
game rotation vector: roll and pitch are gravity referenced, but yaw is relative
and will drift slowly because this module has no magnetometer.

After gyro calibration finishes, the next SFLP sample is automatically captured
as heading zero. Control code should first check `Bsp_Imu_IsHeadingReady()` and
then read `Bsp_Imu_GetHeadingDeg()`. Call `Bsp_Imu_ZeroYaw()` while stopped to
redefine the current vehicle direction as zero; it returns 0 before calibration
or before the first SFLP sample is available.
