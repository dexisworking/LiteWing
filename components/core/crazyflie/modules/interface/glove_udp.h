/**
 * LiteWing Custom Firmware - Glove UDP Server
 *
 * Dedicated UDP socket listener on port 2391 for gesture glove telemetry
 * and control packets. Completely decoupled from official app port 2390.
 */

#ifndef __GLOVE_UDP_H__
#define __GLOVE_UDP_H__

#include <stdbool.h>

void gloveUdpInit(void);
bool gloveUdpTest(void);

#endif /* __GLOVE_UDP_H__ */
