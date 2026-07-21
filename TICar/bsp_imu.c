/**
 * @file bsp_imu.c
 * @brief LSM6DSV16X I2C polling bring-up and raw data acquisition.
 */
#include "bsp_imu.h"

#include "bsp_time.h"
#include "protocol_imu.h"
#include "ti_msp_dl_config.h"

#define LSM6DSV16X_ADDRESS_LOW        (0x6AU)
#define LSM6DSV16X_ADDRESS_HIGH       (0x6BU)
#define LSM6DSV16X_REG_WHO_AM_I       (0x0FU)
#define LSM6DSV16X_REG_CTRL1          (0x10U)
#define LSM6DSV16X_REG_CTRL2          (0x11U)
#define LSM6DSV16X_REG_CTRL3          (0x12U)
#define LSM6DSV16X_REG_CTRL6          (0x15U)
#define LSM6DSV16X_REG_CTRL8          (0x17U)
#define LSM6DSV16X_REG_STATUS         (0x1EU)
#define LSM6DSV16X_REG_OUTX_L_G       (0x22U)
#define LSM6DSV16X_WHO_AM_I_VALUE     (0x70U)

#define LSM6DSV16X_CTRL3_SW_RESET     (0x01U)
#define LSM6DSV16X_CTRL3_IF_INC_BDU   (0x44U)
#define LSM6DSV16X_ODR_60_HZ          (0x05U)
#define LSM6DSV16X_FS_2000_DPS        (0x04U)
#define LSM6DSV16X_FS_4_G             (0x01U)
#define LSM6DSV16X_STATUS_XL_G_READY  (0x03U)

#define BSP_IMU_POLL_PERIOD_MS        (10U)
#define BSP_IMU_I2C_TIMEOUT_LOOPS     (200000U)

volatile uint8_t g_imu_i2c_address;
volatile uint8_t g_imu_who_am_i;
volatile uint8_t g_imu_init_status;
volatile uint8_t g_imu_last_status;
volatile uint32_t g_imu_i2c_read_count;
volatile uint32_t g_imu_i2c_error_count;
volatile uint32_t g_imu_sample_count;

#if defined(I2C_LSM6DSV16X_INST)
static uint32_t g_imu_last_poll_ms;

static int16_t Bsp_Imu_ReadI16(const uint8_t *data)
{
    return (int16_t) ((uint16_t) data[0] | ((uint16_t) data[1] << 8));
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

    return (uint8_t) (
        Bsp_Imu_WriteRegister(g_imu_i2c_address, LSM6DSV16X_REG_CTRL3,
            LSM6DSV16X_CTRL3_IF_INC_BDU) &&
        Bsp_Imu_WriteRegister(g_imu_i2c_address, LSM6DSV16X_REG_CTRL8,
            LSM6DSV16X_FS_4_G) &&
        Bsp_Imu_WriteRegister(g_imu_i2c_address, LSM6DSV16X_REG_CTRL6,
            LSM6DSV16X_FS_2000_DPS) &&
        Bsp_Imu_WriteRegister(g_imu_i2c_address, LSM6DSV16X_REG_CTRL1,
            LSM6DSV16X_ODR_60_HZ) &&
        Bsp_Imu_WriteRegister(g_imu_i2c_address, LSM6DSV16X_REG_CTRL2,
            LSM6DSV16X_ODR_60_HZ));
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

#if defined(I2C_LSM6DSV16X_INST)
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
    g_imu_last_poll_ms = 0U;
#endif
}

void Bsp_Imu_Task(void)
{
#if defined(I2C_LSM6DSV16X_INST)
    uint8_t status;
    uint8_t raw[12];
    uint32_t now_ms = Bsp_Time_GetMilliseconds();
    uint8_t axis;

    if ((g_imu.hardware_ready == 0U) ||
        ((uint32_t) (now_ms - g_imu_last_poll_ms) < BSP_IMU_POLL_PERIOD_MS)) {
        return;
    }
    g_imu_last_poll_ms = now_ms;

    if (Bsp_Imu_ReadRegisters(g_imu_i2c_address, LSM6DSV16X_REG_STATUS,
            &status, 1U) == 0U) {
        return;
    }
    g_imu_last_status = status;
    if ((status & LSM6DSV16X_STATUS_XL_G_READY) !=
        LSM6DSV16X_STATUS_XL_G_READY) {
        return;
    }
    if (Bsp_Imu_ReadRegisters(g_imu_i2c_address, LSM6DSV16X_REG_OUTX_L_G,
            raw, sizeof(raw)) == 0U) {
        return;
    }

    for (axis = 0U; axis < 3U; axis++) {
        int16_t gyro_raw = Bsp_Imu_ReadI16(&raw[axis * 2U]);
        int16_t accel_raw = Bsp_Imu_ReadI16(&raw[6U + axis * 2U]);
        g_imu.legacy_gyro_raw[axis] = gyro_raw;
        g_imu.legacy_acc_raw[axis] = accel_raw;
        g_imu.gyro_dps[axis] = (float) gyro_raw * 0.070f;
        g_imu.acc_g[axis] = (float) accel_raw * 0.000122f;
    }
    g_imu.sample_count = ++g_imu_sample_count;
    g_imu.data_frame_count++;
    g_imu.valid_frame_count++;
    g_imu.frame_updated = 1U;
#endif
}

uint8_t Bsp_Imu_IsHardwareReady(void)
{
    return g_imu.hardware_ready;
}
