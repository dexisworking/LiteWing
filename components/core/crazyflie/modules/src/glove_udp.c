/**
 * LiteWing Custom Firmware - Glove UDP Server Implementation
 *
 * Dedicated UDP socket listener on port 2391 for gesture glove telemetry
 * and control packets. Completely decoupled from official app port 2390.
 */

#include <string.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "glove_udp.h"
#include "glove_protocol.h"
#include "control_mux.h"
#include "log.h"

#define DEBUG_MODULE "GLOVE_UDP"
#include "debug_cf.h"

#define GLOVE_TASK_STACKSIZE 2048
#define GLOVE_TASK_PRI       2

static int gloveSock = -1;
static bool isInit = false;
static uint32_t packetsReceived = 0;
static uint32_t packetsInvalid = 0;

static void gloveUdpServerTask(void *pvParameters)
{
    struct sockaddr_in dest_addr = {0};
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(GLOVE_UDP_PORT);

    /* Wait a moment for Wi-Fi stack to be ready */
    vTaskDelay(pdMS_TO_TICKS(1500));

    gloveSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (gloveSock < 0) {
        DEBUG_PRINTW("Unable to create glove UDP socket: errno %d\n", errno);
        vTaskDelete(NULL);
        return;
    }

    int err = bind(gloveSock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        DEBUG_PRINTW("Unable to bind glove UDP socket to port %d: errno %d\n", GLOVE_UDP_PORT, errno);
        close(gloveSock);
        gloveSock = -1;
        vTaskDelete(NULL);
        return;
    }

    DEBUG_PRINTI("Glove UDP Server bound to port %d [READY].\n", GLOVE_UDP_PORT);

    struct sockaddr_storage source_addr;
    socklen_t socklen = sizeof(source_addr);
    uint8_t rx_buffer[128];

    while (1) {
        int len = recvfrom(gloveSock, rx_buffer, sizeof(rx_buffer), 0,
                           (struct sockaddr *)&source_addr, &socklen);

        if (len < 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (gloveValidatePacket((const glovePacket_t *)rx_buffer, len)) {
            packetsReceived++;
            controlMuxNotifyGlovePacket((const glovePacket_t *)rx_buffer);
        } else {
            packetsInvalid++;
        }
    }
}

void gloveUdpInit(void)
{
    if (isInit) {
        return;
    }

    controlMuxInit();

    xTaskCreate(gloveUdpServerTask, "GLOVE_UDP", GLOVE_TASK_STACKSIZE, NULL, GLOVE_TASK_PRI, NULL);

    isInit = true;
    DEBUG_PRINTI("Glove UDP module initialized.\n");
}

bool gloveUdpTest(void)
{
    return isInit;
}

LOG_GROUP_START(glove)
LOG_ADD(LOG_UINT32, rxPkt, &packetsReceived)
LOG_ADD(LOG_UINT32, badPkt, &packetsInvalid)
LOG_GROUP_STOP(glove)
