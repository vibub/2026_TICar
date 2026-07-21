/**
 * @file bsp_imu.c
 * @brief LSM6DSV16X I2C polling bring-up and raw data acquisition.
 */
#include "bsp_imu.h"

#include "bsp_time.h"
#include "protocol_imu.h"
#include "ti_msp_dl_config.h"

#include <math.h>

#define LSM6DSV16X_ADDRESS_LOW        (0x6AU)
#define LSM6DSV16X_ADDRESS_HIGH       (0x6BU)
#define LSM6DSV16X_REG_WHO_AM_I       (0x0FU)
#define LSM6DSV16X_REG_CTRL1          (0x10U)
#define LSM6DSV16X_REG_CTRL2          (0x11U)
#define LSM6DSV16X_REG_CTRL3          (0x12U)
#define LSM6DSV16X_REG_CTRL6          (0x15U)
#define LSM6DSV16X_REG_CTRL8          (0x17U)
#define LSM6DSV16X_REG_FUNC_CFG_ACCESS (0x01U)
#define LSM6DSV16X_REG_FIFO_CTRL3     (0x09U)
#define LSM6DSV16X_REG_FIFO_CTRL4     (0x0AU)
#define LSM6DSV16X_REG_FIFO_STATUS1   (0x1BU)
#define LSM6DSV16X_REG_FIFO_DATA_TAG  (0x78U)
#define LSM6DSV16X_REG_EMB_FUNC_EN_A  (0x04U)
#define LSM6DSV16X_REG_EMB_FIFO_EN_A  (0x44U)
#define LSM6DSV16X_REG_SFLP_ODR       (0x5EU)
#define LSM6DSV16X_WHO_AM_I_VALUE     (0x70U)

#define LSM6DSV16X_CTRL3_SW_RESET     (0x01U)
#define LSM6DSV16X_CTRL3_IF_INC_BDU   (0x44U)
#define LSM6DSV16X_ODR_60_HZ          (0x05U)
#define LSM6DSV16X_FS_2000_DPS        (0x04U)
#define LSM6DSV16X_FS_4_G             (0x01U)
#define LSM6DSV16X_FIFO_BATCH_60_HZ   (0x55U)
#define LSM6DSV16X_FIFO_STREAM_MODE   (0x06U)
#define LSM6DSV16X_EMBED_BANK         (0x80U)
#define LSM6DSV16X_MAIN_BANK          (0x00U)
#define LSM6DSV16X_SFLP_GAME_ENABLE   (0x02U)
#define LSM6DSV16X_SFLP_FIFO_ENABLE   (0x02U)
#define LSM6DSV16X_SFLP_ODR_60_HZ     (0x10U)

#define LSM6DSV16X_FIFO_TAG_GYRO      (0x01U)
#define LSM6DSV16X_FIFO_TAG_ACCEL     (0x02U)
#define LSM6DSV16X_FIFO_TAG_SFLP_GAME (0x13U)

#define BSP_IMU_POLL_PERIOD_MS        (10U)
#define BSP_IMU_I2C_TIMEOUT_LOOPS     (200000U)
#define BSP_IMU_MAX_FIFO_DRAIN        (32U)
#define BSP_IMU_CALIBRATION_SAMPLES   (180U)
#define BSP_IMU_STILL_LIMIT_DPS       (5.0f)
#define BSP_IMU_RAD_TO_DEG            (57.2957795131f)

volatile uint8_t g_imu_i2c_address;
volatile uint8_t g_imu_who_am_i;
volatile uint8_t g_imu_init_status;
volatile uint8_t g_imu_last_status;
volatile uint32_t g_imu_i2c_read_count;
volatile uint32_t g_imu_i2c_error_count;
volatile uint32_t g_imu_sample_count;
volatile uint16_t g_imu_fifo_level;
volatile uint8_t g_imu_last_fifo_tag;
volatile uint8_t g_imu_calibration_status;
volatile uint16_t g_imu_calibration_sample_count;
volatile uint32_t g_imu_sflp_count;
volatile uint32_t g_imu_fifo_overrun_count;
volatile float g_imu_gyro_bias_dps[3];
volatile float g_imu_yaw_zero_deg;
volatile float g_imu_heading_deg;
volatile uint8_t g_imu_heading_ready;
volatile uint32_t g_imu_heading_zero_count;
volatile uint32_t g_imu_last_sflp_ms;

#if defined(I2C_LSM6DSV16X_INST)
static uint32_t g_imu_last_poll_ms;
static float g_imu_gyro_bias_sum[3];

static float Bsp_Imu_WrapAngleDeg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static void Bsp_Imu_ApplyCurrentYawZero(void)
{
    g_imu_yaw_zero_deg = g_imu.euler_deg[2];
    g_imu_heading_deg = 0.0f;
    g_imu_heading_ready = 1U;
    g_imu_heading_zero_count++;
}

static int16_t Bsp_Imu_ReadI16(const uint8_t *data)
{
    return (int16_t) ((uint16_t) data[0] | ((uint16_t) data[1] << 8));
}

static float Bsp_Imu_HalfToFloat(uint16_t half)
{
    uint16_t half_exp = half & 0x7C00U;
    uint32_t float_sign = ((uint32_t) half & 0x8000U) << 16;
    uint32_t float_bits;
    union {
        uint32_t bits;
        float value;
    } result;

    if (half_exp == 0U) {
        uint16_t half_significand = half & 0x03FFU;
        if (half_significand == 0U) {
            float_bits = float_sign;
        } else {
            half_significand <<= 1;
            while ((half_significand & 0x0400U) == 0U) {
                half_significand <<= 1;
                half_exp++;
            }
            float_bits = float_sign +
                ((uint32_t) (127U - 15U - half_exp) << 23) +
                ((uint32_t) (half_significand & 0x03FFU) << 13);
        }
    } else if (half_exp == 0x7C00U) {
        float_bits = float_sign + 0x7F800000U +
            ((uint32_t) (half & 0x03FFU) << 13);
    } else {
        float_bits = float_sign +
            (((uint32_t) (half & 0x7FFFU) + 0x1C000U) << 13);
    }

    result.bits = float_bits;
    return result.value;
}

static void Bsp_Imu_UpdateEulerFromSflp(const uint8_t *data)
{
    float sum_squared = 0.0f;
    float sin_pitch;
    uint8_t axis;

    for (axis = 0U; axis < 3U; axis++) {
        uint16_t half = (uint16_t) data[axis * 2U] |
            ((uint16_t) data[axis * 2U + 1U] << 8);
        g_imu.quat[axis] = Bsp_Imu_HalfToFloat(half);
        sum_squared += g_imu.quat[axis] * g_imu.quat[axis];
    }

    if (sum_squared > 1.0f) {
        float norm = sqrtf(sum_squared);
        for (axis = 0U; axis < 3U; axis++) {
            g_imu.quat[axis] /= norm;
        }
        sum_squared = 1.0f;
    }
    g_imu.quat[3] = sqrtf(1.0f - sum_squared);

    g_imu.euler_deg[0] = atan2f(
        2.0f * (g_imu.quat[3] * g_imu.quat[0] +
                g_imu.quat[1] * g_imu.quat[2]),
        1.0f - 2.0f * (g_imu.quat[0] * g_imu.quat[0] +
                       g_imu.quat[1] * g_imu.quat[1])) * BSP_IMU_RAD_TO_DEG;

    sin_pitch = 2.0f * (g_imu.quat[3] * g_imu.quat[1] -
                        g_imu.quat[2] * g_imu.quat[0]);
    if (sin_pitch > 1.0f) {
        sin_pitch = 1.0f;
    } else if (sin_pitch < -1.0f) {
        sin_pitch = -1.0f;
    }
    g_imu.euler_deg[1] = asinf(sin_pitch) * BSP_IMU_RAD_TO_DEG;

    g_imu.euler_deg[2] = atan2f(
        2.0f * (g_imu.quat[3] * g_imu.quat[2] +
                g_imu.quat[0] * g_imu.quat[1]),
        1.0f - 2.0f * (g_imu.quat[1] * g_imu.quat[1] +
                       g_imu.quat[2] * g_imu.quat[2])) * BSP_IMU_RAD_TO_DEG;

    if ((g_imu_calibration_status == BSP_IMU_CALIBRATED) &&
        (g_imu_heading_ready == 0U)) {
        Bsp_Imu_ApplyCurrentYawZero();
    } else if (g_imu_heading_ready != 0U) {
        g_imu_heading_deg = Bsp_Imu_WrapAngleDeg(
            g_imu.euler_deg[2] - g_imu_yaw_zero_deg);
    }

    g_imu_sflp_count++;
    g_imu_last_sflp_ms = Bsp_Time_GetMilliseconds();
    g_imu.data_frame_count++;
    g_imu.valid_frame_count++;
    g_imu.frame_updated = 1U;
}

static void Bsp_Imu_UpdateGyro(const uint8_t *data)
{
    float uncorrected[3];
    uint8_t axis;
    uint8_t still = 1U;

    for (axis = 0U; axis < 3U; axis++) {
        int16_t raw = Bsp_Imu_ReadI16(&data[axis * 2U]);
        g_imu.legacy_gyro_raw[axis] = raw;
        uncorrected[axis] = (float) raw * 0.070f;
        if (fabsf(uncorrected[axis]) > BSP_IMU_STILL_LIMIT_DPS) {
            still = 0U;
        }
    }

    if (g_imu_calibration_status == BSP_IMU_CALIBRATING) {
        if (still == 0U) {
            g_imu_calibration_sample_count = 0U;
            for (axis = 0U; axis < 3U; axis++) {
                g_imu_gyro_bias_sum[axis] = 0.0f;
            }
        } else {
            for (axis = 0U; axis < 3U; axis++) {
                g_imu_gyro_bias_sum[axis] += uncorrected[axis];
            }
            g_imu_calibration_sample_count++;
            if (g_imu_calibration_sample_count >= BSP_IMU_CALIBRATION_SAMPLES) {
                for (axis = 0U; axis < 3U; axis++) {
                    g_imu_gyro_bias_dps[axis] =
                        g_imu_gyro_bias_sum[axis] /
                        (float) g_imu_calibration_sample_count;
                }
                g_imu_calibration_status = BSP_IMU_CALIBRATED;
            }
        }
    }

    for (axis = 0U; axis < 3U; axis++) {
        g_imu.gyro_dps[axis] = uncorrected[axis];
        if (g_imu_calibration_status == BSP_IMU_CALIBRATED) {
            g_imu.gyro_dps[axis] -= g_imu_gyro_bias_dps[axis];
        }
    }
}

static void Bsp_Imu_UpdateAccel(const uint8_t *data)
{
    uint8_t axis;

    for (axis = 0U; axis < 3U; axis++) {
        int16_t raw = Bsp_Imu_ReadI16(&data[axis * 2U]);
        g_imu.legacy_acc_raw[axis] = raw;
        g_imu.acc_g[axis] = (float) raw * 0.000122f;
    }
    g_imu.sample_count = ++g_imu_sample_count;
}

static uint8_t Bsp_Imu_WaitForIdle(void)
{
    uint32_t timeout = BSP_IMU_I2C_TIMEOUT_LOOPS;

    while ((DL_I2C_getControllerStatus(I2C_LSM6DSV16X_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (--timeout == 0U) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t Bsp_Imu_HasBusError(void)
{
    uint32_t status = DL_I2C_getControllerStatus(I2C_LSM6DSV16X_INST);
    return (uint8_t) ((status & (DL_I2C_CONTROLLER_STATUS_ERROR |
                                DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST)) != 0U);
}

static uint8_t Bsp_Imu_ReadRegisters(
    uint8_t address, uint8_t reg, uint8_t *data, uint16_t length)
{
    uint16_t received = 0U;
    uint32_t timeout = BSP_IMU_I2C_TIMEOUT_LOOPS;
    uint8_t transfer_done = 0U;

    if ((length == 0U) || (Bsp_Imu_WaitForIdle() == 0U)) {
        g_imu_i2c_error_count++;
        return 0U;
    }

    DL_I2C_flushControllerTXFIFO(I2C_LSM6DSV16X_INST);
    DL_I2C_flushControllerRXFIFO(I2C_LSM6DSV16X_INST);
    DL_I2C_transmitControllerData(I2C_LSM6DSV16X_INST, reg);
    I2C_LSM6DSV16X_INST->MASTER.MCTR = I2C_MCTR_RD_ON_TXEMPTY_ENABLE;
    DL_I2C_clearInterruptStatus(I2C_LSM6DSV16X_INST,
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
        DL_I2C_INTERRUPT_CONTROLLER_NACK |
        DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);
    DL_I2C_startControllerTransfer(I2C_LSM6DSV16X_INST, address,
        DL_I2C_CONTROLLER_DIRECTION_RX, length);

    while (transfer_done == 0U) {
        while ((!DL_I2C_isControllerRXFIFOEmpty(I2C_LSM6DSV16X_INST)) &&
               (received < length)) {
            data[received++] =
                DL_I2C_receiveControllerData(I2C_LSM6DSV16X_INST);
        }
        if (DL_I2C_getRawInterruptStatus(I2C_LSM6DSV16X_INST,
                DL_I2C_INTERRUPT_CONTROLLER_RX_DONE) != 0U) {
            transfer_done = 1U;
            continue;
        }
        if (Bsp_Imu_HasBusError() != 0U) {
            break;
        }
        if (--timeout == 0U) {
            break;
        }
    }

    while ((!DL_I2C_isControllerRXFIFOEmpty(I2C_LSM6DSV16X_INST)) &&
           (received < length)) {
        data[received++] = DL_I2C_receiveControllerData(I2C_LSM6DSV16X_INST);
    }

    DL_I2C_resetControllerTransfer(I2C_LSM6DSV16X_INST);
    DL_I2C_flushControllerTXFIFO(I2C_LSM6DSV16X_INST);
    if ((transfer_done == 0U) || (received != length)) {
        g_imu_i2c_error_count++;
        return 0U;
    }

    g_imu_i2c_read_count++;
    return 1U;
}

static uint8_t Bsp_Imu_WriteRegister(uint8_t address, uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    uint32_t timeout = BSP_IMU_I2C_TIMEOUT_LOOPS;

    if (Bsp_Imu_WaitForIdle() == 0U) {
        g_imu_i2c_error_count++;
        return 0U;
    }

    DL_I2C_flushControllerTXFIFO(I2C_LSM6DSV16X_INST);
    (void) DL_I2C_fillControllerTXFIFO(I2C_LSM6DSV16X_INST, data, 2U);
    DL_I2C_clearInterruptStatus(I2C_LSM6DSV16X_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
        DL_I2C_INTERRUPT_CONTROLLER_NACK |
        DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);
    DL_I2C_startControllerTransfer(I2C_LSM6DSV16X_INST, address,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2U);

    while (DL_I2C_getRawInterruptStatus(I2C_LSM6DSV16X_INST,
               DL_I2C_INTERRUPT_CONTROLLER_TX_DONE) == 0U) {
        if ((Bsp_Imu_HasBusError() != 0U) || (--timeout == 0U)) {
            DL_I2C_resetControllerTransfer(I2C_LSM6DSV16X_INST);
            g_imu_i2c_error_count++;
            return 0U;
        }
    }
    return 1U;
}

static uint8_t Bsp_Imu_ProbeAddress(uint8_t address)
{
    uint8_t id = 0U;

    if ((Bsp_Imu_ReadRegisters(address, LSM6DSV16X_REG_WHO_AM_I, &id, 1U) != 0U) &&
        (id == LSM6DSV16X_WHO_AM_I_VALUE)) {
        g_imu_i2c_address = address;
        g_imu_who_am_i = id;
        return 1U;
    }
    return 0U;
}

static uint8_t Bsp_Imu_ConfigureSflp(void)
{
    uint8_t success;

    if (Bsp_Imu_WriteRegister(g_imu_i2c_address,
            LSM6DSV16X_REG_FUNC_CFG_ACCESS, LSM6DSV16X_EMBED_BANK) == 0U) {
        return 0U;
    }

    success = (uint8_t) (
        Bsp_Imu_WriteRegister(g_imu_i2c_address,
            LSM6DSV16X_REG_EMB_FIFO_EN_A, LSM6DSV16X_SFLP_FIFO_ENABLE) &&
        Bsp_Imu_WriteRegister(g_imu_i2c_address,
            LSM6DSV16X_REG_SFLP_ODR, LSM6DSV16X_SFLP_ODR_60_HZ) &&
        Bsp_Imu_WriteRegister(g_imu_i2c_address,
            LSM6DSV16X_REG_EMB_FUNC_EN_A, LSM6DSV16X_SFLP_GAME_ENABLE));

    if (Bsp_Imu_WriteRegister(g_imu_i2c_address,
            LSM6DSV16X_REG_FUNC_CFG_ACCESS, LSM6DSV16X_MAIN_BANK) == 0U) {
        return 0U;
    }
    return success;
}

static uint8_t Bsp_Imu_ConfigureSensor(void)
{
    uint8_t ctrl3 = LSM6DSV16X_CTRL3_SW_RESET;
    uint32_t timeout = 100U;

    if (Bsp_Imu_WriteRegister(g_imu_i2c_address, LSM6DSV16X_REG_CTRL3,
            LSM6DSV16X_CTRL3_SW_RESET) == 0U) {
        return 0U;
    }

    do {
        delay_cycles(CPUCLK_FREQ / 1000U);
        if (Bsp_Imu_ReadRegisters(g_imu_i2c_address, LSM6DSV16X_REG_CTRL3,
                &ctrl3, 1U) == 0U) {
            return 0U;
        }
    } while (((ctrl3 & LSM6DSV16X_CTRL3_SW_RESET) != 0U) && (--timeout != 0U));

    if ((ctrl3 & LSM6DSV16X_CTRL3_SW_RESET) != 0U) {
        return 0U;
    }

    if (!(Bsp_Imu_WriteRegister(g_imu_i2c_address, LSM6DSV16X_REG_CTRL3,
              LSM6DSV16X_CTRL3_IF_INC_BDU) &&
          Bsp_Imu_WriteRegister(g_imu_i2c_address, LSM6DSV16X_REG_CTRL8,
              LSM6DSV16X_FS_4_G) &&
          Bsp_Imu_WriteRegister(g_imu_i2c_address, LSM6DSV16X_REG_CTRL6,
              LSM6DSV16X_FS_2000_DPS) &&
          Bsp_Imu_WriteRegister(g_imu_i2c_address, LSM6DSV16X_REG_FIFO_CTRL4,
              0U) &&
          Bsp_Imu_WriteRegister(g_imu_i2c_address, LSM6DSV16X_REG_FIFO_CTRL3,
              LSM6DSV16X_FIFO_BATCH_60_HZ))) {
        return 0U;
    }

    if (Bsp_Imu_ConfigureSflp() == 0U) {
        return 0U;
    }

    return (uint8_t) (
        Bsp_Imu_WriteRegister(g_imu_i2c_address, LSM6DSV16X_REG_CTRL1,
            LSM6DSV16X_ODR_60_HZ) &&
        Bsp_Imu_WriteRegister(g_imu_i2c_address, LSM6DSV16X_REG_CTRL2,
            LSM6DSV16X_ODR_60_HZ) &&
        Bsp_Imu_WriteRegister(g_imu_i2c_address, LSM6DSV16X_REG_FIFO_CTRL4,
            LSM6DSV16X_FIFO_STREAM_MODE));
}
#endif

void Bsp_Imu_Init(void)
{
    Protocol_Imu_Init();
    g_imu_i2c_address = 0U;
    g_imu_who_am_i = 0U;
    g_imu_init_status = BSP_IMU_INIT_NO_I2C;
    g_imu_last_status = 0U;
    g_imu_i2c_read_count = 0U;
    g_imu_i2c_error_count = 0U;
    g_imu_sample_count = 0U;
    g_imu_fifo_level = 0U;
    g_imu_last_fifo_tag = 0U;
    g_imu_calibration_status = BSP_IMU_CALIBRATION_IDLE;
    g_imu_calibration_sample_count = 0U;
    g_imu_sflp_count = 0U;
    g_imu_fifo_overrun_count = 0U;
    g_imu_gyro_bias_dps[0] = 0.0f;
    g_imu_gyro_bias_dps[1] = 0.0f;
    g_imu_gyro_bias_dps[2] = 0.0f;
    g_imu_yaw_zero_deg = 0.0f;
    g_imu_heading_deg = 0.0f;
    g_imu_heading_ready = 0U;
    g_imu_heading_zero_count = 0U;
    g_imu_last_sflp_ms = 0U;

#if defined(I2C_LSM6DSV16X_INST)
    g_imu_gyro_bias_sum[0] = 0.0f;
    g_imu_gyro_bias_sum[1] = 0.0f;
    g_imu_gyro_bias_sum[2] = 0.0f;
    g_imu_init_status = BSP_IMU_INIT_NOT_FOUND;
    delay_cycles(CPUCLK_FREQ / 100U);

    if ((Bsp_Imu_ProbeAddress(LSM6DSV16X_ADDRESS_LOW) == 0U) &&
        (Bsp_Imu_ProbeAddress(LSM6DSV16X_ADDRESS_HIGH) == 0U)) {
        return;
    }

    g_imu_init_status = BSP_IMU_INIT_CONFIG_ERROR;
    if (Bsp_Imu_ConfigureSensor() == 0U) {
        return;
    }

    g_imu.hardware_ready = 1U;
    g_imu.protocol = (uint8_t) IMU_PROTOCOL_LSM6DSV16X;
    g_imu_init_status = BSP_IMU_INIT_OK;
    g_imu_calibration_status = BSP_IMU_CALIBRATING;
    g_imu_last_poll_ms = 0U;
#endif
}

void Bsp_Imu_Task(void)
{
#if defined(I2C_LSM6DSV16X_INST)
    uint8_t status[2];
    uint8_t raw[7];
    uint32_t now_ms = Bsp_Time_GetMilliseconds();
    uint16_t drain_count;

    if ((g_imu.hardware_ready == 0U) ||
        ((uint32_t) (now_ms - g_imu_last_poll_ms) < BSP_IMU_POLL_PERIOD_MS)) {
        return;
    }
    g_imu_last_poll_ms = now_ms;

    if (Bsp_Imu_ReadRegisters(g_imu_i2c_address, LSM6DSV16X_REG_FIFO_STATUS1,
            status, sizeof(status)) == 0U) {
        return;
    }
    g_imu_last_status = status[1];
    g_imu_fifo_level =
        (uint16_t) status[0] | ((uint16_t) (status[1] & 0x01U) << 8);
    if ((status[1] & 0x40U) != 0U) {
        g_imu_fifo_overrun_count++;
    }

    drain_count = g_imu_fifo_level;
    if (drain_count > BSP_IMU_MAX_FIFO_DRAIN) {
        drain_count = BSP_IMU_MAX_FIFO_DRAIN;
    }

    while (drain_count-- != 0U) {
        uint8_t tag;

        if (Bsp_Imu_ReadRegisters(g_imu_i2c_address,
                LSM6DSV16X_REG_FIFO_DATA_TAG, raw, sizeof(raw)) == 0U) {
            return;
        }
        tag = (raw[0] >> 3) & 0x1FU;
        g_imu_last_fifo_tag = tag;

        switch (tag) {
            case LSM6DSV16X_FIFO_TAG_GYRO:
                Bsp_Imu_UpdateGyro(&raw[1]);
                break;

            case LSM6DSV16X_FIFO_TAG_ACCEL:
                Bsp_Imu_UpdateAccel(&raw[1]);
                break;

            case LSM6DSV16X_FIFO_TAG_SFLP_GAME:
                Bsp_Imu_UpdateEulerFromSflp(&raw[1]);
                break;

            default:
                g_imu.unknown_item_count++;
                break;
        }
    }
#endif
}

uint8_t Bsp_Imu_IsHardwareReady(void)
{
    return g_imu.hardware_ready;
}

uint8_t Bsp_Imu_ZeroYaw(void)
{
#if defined(I2C_LSM6DSV16X_INST)
    if ((g_imu_calibration_status != BSP_IMU_CALIBRATED) ||
        (g_imu_sflp_count == 0U)) {
        return 0U;
    }
    Bsp_Imu_ApplyCurrentYawZero();
    return 1U;
#else
    return 0U;
#endif
}

uint8_t Bsp_Imu_IsHeadingReady(void)
{
    return g_imu_heading_ready;
}

float Bsp_Imu_GetHeadingDeg(void)
{
    return g_imu_heading_deg;
}

uint8_t Bsp_Imu_IsHeadingFresh(uint32_t max_age_ms)
{
    if (g_imu_heading_ready == 0U) {
        return 0U;
    }
    return ((uint32_t) (Bsp_Time_GetMilliseconds() - g_imu_last_sflp_ms) <=
            max_age_ms) ? 1U : 0U;
}
