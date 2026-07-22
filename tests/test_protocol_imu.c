#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "protocol_imu.h"

#define CHECK(condition)                                                     \
    do {                                                                     \
        if (!(condition)) {                                                  \
            fprintf(stderr, "CHECK failed at line %d: %s\n", __LINE__,     \
                    #condition);                                             \
            return 0;                                                        \
        }                                                                    \
    } while (0)

static uint16_t test_crc16(uint16_t crc, const uint8_t *data, size_t length)
{
    size_t index;

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

static size_t make_frame(uint8_t *frame, const uint8_t *payload, uint16_t length)
{
    uint16_t crc;

    frame[0] = 0x5A;
    frame[1] = 0xA5;
    frame[2] = (uint8_t) length;
    frame[3] = (uint8_t) (length >> 8);
    memcpy(frame + 6, payload, length);
    crc = test_crc16(0U, frame, 4U);
    crc = test_crc16(crc, payload, length);
    frame[4] = (uint8_t) crc;
    frame[5] = (uint8_t) (crc >> 8);
    return (size_t) length + 6U;
}

static void feed(const uint8_t *data, size_t length)
{
    size_t index;
    for (index = 0U; index < length; index++) {
        Protocol_Imu_FeedByte(data[index]);
    }
}

static int test_official_hi91_frame(void)
{
    static const uint8_t frame[] = {
        0x5A, 0xA5, 0x4C, 0x00, 0x14, 0xBB, 0x91, 0x08, 0x15, 0x23,
        0x09, 0xA2, 0xC4, 0x47, 0x08, 0x15, 0x1C, 0x00, 0xCC, 0xE8,
        0x61, 0xBE, 0x9A, 0x35, 0x56, 0x3E, 0x65, 0xEA, 0x72, 0x3F,
        0x31, 0xD0, 0x7C, 0xBD, 0x75, 0xDD, 0xC5, 0xBB, 0x6B, 0xD7,
        0x24, 0xBC, 0x89, 0x88, 0xFC, 0x40, 0x01, 0x00, 0x6A, 0x41,
        0xAB, 0x2A, 0x70, 0xC2, 0x96, 0xD4, 0x50, 0x41, 0xED, 0x03,
        0x43, 0x41, 0x41, 0xF4, 0xF4, 0xC2, 0xCC, 0xCA, 0xF8, 0xBE,
        0x73, 0x6A, 0x19, 0xBE, 0xF0, 0x00, 0x1C, 0x3D, 0x8D, 0x37,
        0x5C, 0x3F
    };

    Protocol_Imu_Init();
    feed(frame, sizeof(frame));
    CHECK(g_imu.protocol == IMU_PROTOCOL_HI91);
    CHECK(g_imu.valid_frame_count == 1U);
    CHECK(g_imu.data_frame_count == 1U);
    CHECK(g_imu.crc_error_count == 0U);
    CHECK(fabsf(g_imu.euler_deg[0] - 13.0519f) < 0.001f);
    CHECK(fabsf(g_imu.euler_deg[1] - 12.1885f) < 0.001f);
    CHECK(fabsf(g_imu.euler_deg[2] - (-122.477f)) < 0.001f);
    CHECK(Protocol_Imu_ConsumeUpdated() == 1U);
    CHECK(Protocol_Imu_ConsumeUpdated() == 0U);
    return 1;
}

static int test_legacy_frame_and_resync(void)
{
    static const uint8_t payload[] = {
        0x90, 0x2A,
        0xA0, 0x64, 0x00, 0x38, 0xFF, 0x2C, 0x01,
        0xB0, 0x01, 0x00, 0xFE, 0xFF, 0x03, 0x00,
        0xD0, 0xD2, 0x04, 0xD7, 0xF6, 0x84, 0x03
    };
    uint8_t frame[64];
    static const uint8_t noise[] = {0x00, 0x5A, 0x5A, 0x01, 0x7F};
    size_t frame_length = make_frame(frame, payload, (uint16_t) sizeof(payload));

    Protocol_Imu_Init();
    feed(noise, sizeof(noise));
    feed(frame, frame_length);
    CHECK(g_imu.protocol == IMU_PROTOCOL_LEGACY_TLV);
    CHECK(g_imu.module_id == 0x2AU);
    CHECK(g_imu.legacy_acc_raw[0] == 100);
    CHECK(g_imu.legacy_acc_raw[1] == -200);
    CHECK(g_imu.legacy_acc_raw[2] == 300);
    CHECK(g_imu.legacy_gyro_raw[0] == 1);
    CHECK(g_imu.legacy_gyro_raw[1] == -2);
    CHECK(g_imu.legacy_gyro_raw[2] == 3);
    CHECK(fabsf(g_imu.euler_deg[0] - 12.34f) < 0.001f);
    CHECK(fabsf(g_imu.euler_deg[1] - (-23.45f)) < 0.001f);
    CHECK(fabsf(g_imu.euler_deg[2] - 90.0f) < 0.001f);

    frame[6] ^= 0x01U;
    feed(frame, frame_length);
    CHECK(g_imu.valid_frame_count == 1U);
    CHECK(g_imu.crc_error_count == 1U);
    return 1;
}

int main(void)
{
    if (!test_official_hi91_frame() || !test_legacy_frame_and_resync()) {
        return 1;
    }
    puts("protocol_imu tests passed");
    return 0;
}
