/**
 * @file protocol_imu.c
 * @brief 逐字节解析 HiPNUC 5A A5 帧，兼容 HI219 老 TLV 与当前 HI91。
 */
#include "protocol_imu.h"

#include <stddef.h>
#include <string.h>

#define IMU_SYNC_1 0x5AU
#define IMU_SYNC_2 0xA5U
#define IMU_MAX_PAYLOAD_SIZE 512U

#define IMU_LEGACY_ID 0x90U
#define IMU_LEGACY_ACC_RAW 0xA0U
#define IMU_LEGACY_ACC_CAL 0xA1U
#define IMU_LEGACY_ACC_FILTERED 0xA2U
#define IMU_LEGACY_ACC_LINEAR 0xA5U
#define IMU_LEGACY_GYRO_RAW 0xB0U
#define IMU_LEGACY_GYRO_CAL 0xB1U
#define IMU_LEGACY_GYRO_FILTERED 0xB2U
#define IMU_LEGACY_MAG_RAW 0xC0U
#define IMU_LEGACY_MAG_CAL 0xC1U
#define IMU_LEGACY_MAG_FILTERED 0xC2U
#define IMU_LEGACY_EULER_I16 0xD0U
#define IMU_LEGACY_QUAT_F32 0xD1U
#define IMU_LEGACY_EULER_F32 0xD9U
#define IMU_LEGACY_TEMPERATURE 0xE0U
#define IMU_LEGACY_PRESSURE 0xF0U
#define IMU_HI91_TAG 0x91U
#define IMU_HI91_PAYLOAD_SIZE 76U

typedef enum {
    IMU_RX_SYNC_1 = 0,
    IMU_RX_SYNC_2,
    IMU_RX_LEN_LOW,
    IMU_RX_LEN_HIGH,
    IMU_RX_CRC_LOW,
    IMU_RX_CRC_HIGH,
    IMU_RX_PAYLOAD
} ImuRxState;

typedef struct {
    uint8_t payload[IMU_MAX_PAYLOAD_SIZE];
    uint16_t payload_len;
    uint16_t payload_offset;
    uint16_t received_crc;
    uint8_t len_low;
    ImuRxState state;
} ImuReceiver;

volatile ImuState g_imu;
static ImuReceiver g_imu_receiver;

static uint16_t Imu_ReadU16LE(const uint8_t *data)
{
    return (uint16_t) data[0] | ((uint16_t) data[1] << 8);
}

static uint32_t Imu_ReadU32LE(const uint8_t *data)
{
    return (uint32_t) data[0] | ((uint32_t) data[1] << 8) |
           ((uint32_t) data[2] << 16) | ((uint32_t) data[3] << 24);
}

static int16_t Imu_ReadI16LE(const uint8_t *data)
{
    return (int16_t) Imu_ReadU16LE(data);
}

static float Imu_ReadF32LE(const uint8_t *data)
{
    uint32_t bits = Imu_ReadU32LE(data);
    float value;

    memcpy(&value, &bits, sizeof(value));
    return value;
}

/* HiPNUC 二进制协议使用 CRC-16/CCITT，poly=0x1021，init=0。 */
static uint16_t Imu_Crc16Update(uint16_t crc, const uint8_t *data, uint16_t length)
{
    uint16_t index;

    for (index = 0U; index < length; index++) {
        uint8_t bit;
        crc ^= (uint16_t) data[index] << 8;
        for (bit = 0U; bit < 8U; bit++) {
            crc = (crc & 0x8000U) != 0U ?
                      (uint16_t) ((crc << 1) ^ 0x1021U) :
                      (uint16_t) (crc << 1);
        }
    }
    return crc;
}

static void Imu_CopyLegacyVector(volatile int16_t output[3], const uint8_t *data)
{
    output[0] = Imu_ReadI16LE(data);
    output[1] = Imu_ReadI16LE(data + 2U);
    output[2] = Imu_ReadI16LE(data + 4U);
}

static uint8_t Imu_ParseLegacy(const uint8_t *payload, uint16_t length)
{
    uint16_t offset = 0U;
    uint8_t parsed = 0U;

    while (offset < length) {
        uint8_t tag = payload[offset];

        switch (tag) {
            case IMU_LEGACY_ID:
                if ((uint16_t) (length - offset) < 2U) {
                    return parsed;
                }
                g_imu.module_id = payload[offset + 1U];
                offset += 2U;
                parsed = 1U;
                break;

            case IMU_LEGACY_ACC_RAW:
            case IMU_LEGACY_ACC_CAL:
            case IMU_LEGACY_ACC_FILTERED:
            case IMU_LEGACY_ACC_LINEAR:
                if ((uint16_t) (length - offset) < 7U) {
                    return parsed;
                }
                Imu_CopyLegacyVector(g_imu.legacy_acc_raw, payload + offset + 1U);
                offset += 7U;
                parsed = 1U;
                break;

            case IMU_LEGACY_GYRO_RAW:
            case IMU_LEGACY_GYRO_CAL:
            case IMU_LEGACY_GYRO_FILTERED:
                if ((uint16_t) (length - offset) < 7U) {
                    return parsed;
                }
                Imu_CopyLegacyVector(g_imu.legacy_gyro_raw, payload + offset + 1U);
                offset += 7U;
                parsed = 1U;
                break;

            case IMU_LEGACY_MAG_RAW:
            case IMU_LEGACY_MAG_CAL:
            case IMU_LEGACY_MAG_FILTERED:
                if ((uint16_t) (length - offset) < 7U) {
                    return parsed;
                }
                Imu_CopyLegacyVector(g_imu.legacy_mag_raw, payload + offset + 1U);
                offset += 7U;
                parsed = 1U;
                break;

            case IMU_LEGACY_EULER_I16:
                if ((uint16_t) (length - offset) < 7U) {
                    return parsed;
                }
                g_imu.euler_deg[0] = (float) Imu_ReadI16LE(payload + offset + 1U) / 100.0f;
                g_imu.euler_deg[1] = (float) Imu_ReadI16LE(payload + offset + 3U) / 100.0f;
                g_imu.euler_deg[2] = (float) Imu_ReadI16LE(payload + offset + 5U) / 10.0f;
                offset += 7U;
                parsed = 1U;
                break;

            case IMU_LEGACY_EULER_F32:
                if ((uint16_t) (length - offset) < 13U) {
                    return parsed;
                }
                g_imu.euler_deg[0] = Imu_ReadF32LE(payload + offset + 1U);
                g_imu.euler_deg[1] = Imu_ReadF32LE(payload + offset + 5U);
                g_imu.euler_deg[2] = Imu_ReadF32LE(payload + offset + 9U);
                offset += 13U;
                parsed = 1U;
                break;

            case IMU_LEGACY_QUAT_F32:
                if ((uint16_t) (length - offset) < 17U) {
                    return parsed;
                }
                g_imu.quat[0] = Imu_ReadF32LE(payload + offset + 1U);
                g_imu.quat[1] = Imu_ReadF32LE(payload + offset + 5U);
                g_imu.quat[2] = Imu_ReadF32LE(payload + offset + 9U);
                g_imu.quat[3] = Imu_ReadF32LE(payload + offset + 13U);
                offset += 17U;
                parsed = 1U;
                break;

            case IMU_LEGACY_TEMPERATURE:
            case IMU_LEGACY_PRESSURE:
                if ((uint16_t) (length - offset) < 5U) {
                    return parsed;
                }
                offset += 5U;
                break;

            default:
                g_imu.unknown_item_count++;
                offset++;
                break;
        }
    }

    if (parsed != 0U) {
        g_imu.protocol = (uint8_t) IMU_PROTOCOL_LEGACY_TLV;
    }
    return parsed;
}

static uint8_t Imu_ParseHi91(const uint8_t *payload, uint16_t length)
{
    uint8_t axis;

    if ((length < IMU_HI91_PAYLOAD_SIZE) || (payload[0] != IMU_HI91_TAG)) {
        return 0U;
    }

    g_imu.temperature_c = (int8_t) payload[3];
    g_imu.pressure_pa = Imu_ReadF32LE(payload + 4U);
    g_imu.system_time_ms = Imu_ReadU32LE(payload + 8U);
    for (axis = 0U; axis < 3U; axis++) {
        g_imu.acc_g[axis] = Imu_ReadF32LE(payload + 12U + ((uint16_t) axis * 4U));
        g_imu.gyro_dps[axis] = Imu_ReadF32LE(payload + 24U + ((uint16_t) axis * 4U));
        g_imu.mag_ut[axis] = Imu_ReadF32LE(payload + 36U + ((uint16_t) axis * 4U));
        g_imu.euler_deg[axis] = Imu_ReadF32LE(payload + 48U + ((uint16_t) axis * 4U));
    }
    for (axis = 0U; axis < 4U; axis++) {
        g_imu.quat[axis] = Imu_ReadF32LE(payload + 60U + ((uint16_t) axis * 4U));
    }
    g_imu.protocol = (uint8_t) IMU_PROTOCOL_HI91;
    return 1U;
}

static void Imu_AcceptFrame(void)
{
    uint8_t parsed;

    g_imu.valid_frame_count++;
    g_imu.last_payload_len = g_imu_receiver.payload_len;
    parsed = Imu_ParseHi91(g_imu_receiver.payload, g_imu_receiver.payload_len);
    if (parsed == 0U) {
        parsed = Imu_ParseLegacy(g_imu_receiver.payload, g_imu_receiver.payload_len);
    }

    if (parsed != 0U) {
        g_imu.data_frame_count++;
        g_imu.frame_updated = 1U;
    } else {
        g_imu.unknown_frame_count++;
    }
}

void Protocol_Imu_Init(void)
{
    memset((void *) &g_imu, 0, sizeof(g_imu));
    memset(&g_imu_receiver, 0, sizeof(g_imu_receiver));
    g_imu_receiver.state = IMU_RX_SYNC_1;
}

void Protocol_Imu_FeedByte(uint8_t byte)
{
    uint16_t calculated_crc;
    uint8_t crc_header[4];

    g_imu.byte_count++;
    g_imu.last_byte = byte;
    g_imu.raw_tail[g_imu.raw_tail_index & 0x0FU] = byte;
    g_imu.raw_tail_index++;

    switch (g_imu_receiver.state) {
        case IMU_RX_SYNC_1:
            if (byte == IMU_SYNC_1) {
                g_imu_receiver.state = IMU_RX_SYNC_2;
            }
            break;

        case IMU_RX_SYNC_2:
            if (byte == IMU_SYNC_2) {
                g_imu.sync_count++;
                g_imu_receiver.state = IMU_RX_LEN_LOW;
            } else if (byte != IMU_SYNC_1) {
                g_imu_receiver.state = IMU_RX_SYNC_1;
            }
            break;

        case IMU_RX_LEN_LOW:
            g_imu_receiver.len_low = byte;
            g_imu_receiver.state = IMU_RX_LEN_HIGH;
            break;

        case IMU_RX_LEN_HIGH:
            g_imu_receiver.payload_len =
                (uint16_t) g_imu_receiver.len_low | ((uint16_t) byte << 8);
            if ((g_imu_receiver.payload_len == 0U) ||
                (g_imu_receiver.payload_len > IMU_MAX_PAYLOAD_SIZE)) {
                g_imu.length_error_count++;
                g_imu_receiver.state = IMU_RX_SYNC_1;
            } else {
                g_imu_receiver.state = IMU_RX_CRC_LOW;
            }
            break;

        case IMU_RX_CRC_LOW:
            g_imu_receiver.received_crc = byte;
            g_imu_receiver.state = IMU_RX_CRC_HIGH;
            break;

        case IMU_RX_CRC_HIGH:
            g_imu_receiver.received_crc |= (uint16_t) byte << 8;
            g_imu_receiver.payload_offset = 0U;
            g_imu_receiver.state = IMU_RX_PAYLOAD;
            break;

        case IMU_RX_PAYLOAD:
            g_imu_receiver.payload[g_imu_receiver.payload_offset++] = byte;
            if (g_imu_receiver.payload_offset >= g_imu_receiver.payload_len) {
                crc_header[0] = IMU_SYNC_1;
                crc_header[1] = IMU_SYNC_2;
                crc_header[2] = g_imu_receiver.len_low;
                crc_header[3] = (uint8_t) (g_imu_receiver.payload_len >> 8);
                calculated_crc = Imu_Crc16Update(0U, crc_header, 4U);
                calculated_crc = Imu_Crc16Update(
                    calculated_crc, g_imu_receiver.payload, g_imu_receiver.payload_len);
                if (calculated_crc == g_imu_receiver.received_crc) {
                    Imu_AcceptFrame();
                } else {
                    g_imu.crc_error_count++;
                }
                g_imu_receiver.state = IMU_RX_SYNC_1;
            }
            break;

        default:
            g_imu_receiver.state = IMU_RX_SYNC_1;
            break;
    }

    g_imu.rx_state = (uint8_t) g_imu_receiver.state;
}

uint8_t Protocol_Imu_ConsumeUpdated(void)
{
    uint8_t updated = g_imu.frame_updated;
    g_imu.frame_updated = 0U;
    return updated;
}
