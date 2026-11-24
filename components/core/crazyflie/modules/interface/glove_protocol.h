/**
 * LiteWing Custom Firmware - Glove Communication Protocol
 *
 * Defines the binary packet structure for direct Wi-Fi control
 * from an ESP32-based gesture control glove.
 */

#ifndef __GLOVE_PROTOCOL_H__
#define __GLOVE_PROTOCOL_H__

#include <stdint.h>
#include <stdbool.h>

#define GLOVE_UDP_PORT           2391
#define GLOVE_MAGIC_BYTE_0       0xAA
#define GLOVE_MAGIC_BYTE_1       0x55
#define GLOVE_PROTOCOL_VERSION   1

#pragma pack(push, 1)
typedef struct {
    uint8_t  magic0;        /* 0xAA */
    uint8_t  magic1;        /* 0x55 */
    uint8_t  version;       /* 1 */
    uint8_t  sequence;      /* 0..255 rolling counter */
    uint32_t timestamp_ms;  /* Glove sender timestamp in ms */
    int16_t  roll;          /* -1800 to +1800 (representing -18.0 to +18.0 deg, 0.01 deg units) */
    int16_t  pitch;         /* -1800 to +1800 (representing -18.0 to +18.0 deg, 0.01 deg units) */
    int16_t  yaw;           /* -1800 to +1800 (representing -180.0 to +180.0 deg/s) */
    uint16_t thrust;        /* 0 to 60000 (0 = idle/cut, 32000-38000 = hover, 60000 = max) */
    uint8_t  deadman;       /* 1 = Hand active / flex engaged, 0 = Hand open (failsafe cutoff) */
    uint8_t  emergency;     /* 1 = E-stop button pressed on glove */
    uint8_t  mode;          /* 0 = manual attitude, 1 = althold assist */
    uint8_t  crc;           /* XOR checksum of all preceding bytes */
} glovePacket_t;
#pragma pack(pop)

#define GLOVE_PACKET_SIZE sizeof(glovePacket_t)

static inline uint8_t gloveCalculateCRC(const glovePacket_t *pkt)
{
    const uint8_t *bytes = (const uint8_t *)pkt;
    uint8_t crc = 0;
    /* Compute XOR sum over all bytes up to crc field */
    for (size_t i = 0; i < (sizeof(glovePacket_t) - 1); i++) {
        crc ^= bytes[i];
    }
    return crc;
}

static inline bool gloveValidatePacket(const glovePacket_t *pkt, size_t len)
{
    if (len != sizeof(glovePacket_t)) {
        return false;
    }
    if (pkt->magic0 != GLOVE_MAGIC_BYTE_0 || pkt->magic1 != GLOVE_MAGIC_BYTE_1) {
        return false;
    }
    if (pkt->version != GLOVE_PROTOCOL_VERSION) {
        return false;
    }
    if (gloveCalculateCRC(pkt) != pkt->crc) {
        return false;
    }
    return true;
}

#endif /* __GLOVE_PROTOCOL_H__ */
