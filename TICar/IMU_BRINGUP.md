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
- `g_imu.acc_g[0]`, `[1]`, `[2]`: acceleration in g.
- `g_imu.gyro_dps[0]`, `[1]`, `[2]`: angular rate in degrees per second.

Initialization status values:

| Value | Meaning |
| --- | --- |
| 0 | SysConfig did not generate `I2C_LSM6DSV16X` |
| 1 | Neither address returned WHO_AM_I 0x70; inspect power and wiring |
| 2 | Device was found but register configuration failed |
| 3 | Sensor is running |

The initial test uses 60 Hz output data rate, +/-4 g accelerometer range and
/-2000 dps gyroscope range. INT1/INT2 and the sensor-fusion FIFO can be enabled
after the basic link is verified.
