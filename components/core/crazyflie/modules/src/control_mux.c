/**
 * LiteWing Custom Firmware - Control Mux Implementation
 *
 * Implements APP-first control-source arbitration between official
 * LiteWing mobile application (CRTP:2390) and gesture glove (UDP:2391).
 */

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "control_mux.h"
#include "commander.h"
#include "stabilizer.h"
#include "crash_manager.h"
#include "log.h"
#include "param.h"

#define DEBUG_MODULE "MUX"
#include "debug_cf.h"

#define APP_ACTIVITY_TIMEOUT_MS    600  /* App is considered active if setpoint within 600ms */
#define GLOVE_ACTIVITY_TIMEOUT_MS  250  /* Glove packets expire if older than 250ms */

static controlOwner_t currentOwner = CONTROL_OWNER_APP;

static uint32_t lastAppPacketTick = 0;
static uint32_t lastGlovePacketTick = 0;

static glovePacket_t latestGlovePacket = {0};
static bool hasGlovePacket = false;

static uint8_t allowGloveParam = 1;     /* Set 1 to allow glove when app is idle, 0 to lock to app only */
static uint8_t appActiveStatus = 0;
static uint8_t gloveActiveStatus = 0;

static bool isInit = false;

void controlMuxInit(void)
{
    if (isInit) {
        return;
    }

    currentOwner = CONTROL_OWNER_APP;
    lastAppPacketTick = 0;
    lastGlovePacketTick = 0;
    hasGlovePacket = false;
    allowGloveParam = 1;
    appActiveStatus = 0;
    gloveActiveStatus = 0;

    isInit = true;
    DEBUG_PRINTI("Control Mux initialized (APP-First Arbitration).\n");
}

void controlMuxNotifyAppSetpoint(const setpoint_t *setpoint)
{
    lastAppPacketTick = xTaskGetTickCount();
}

void controlMuxNotifyGlovePacket(const glovePacket_t *pkt)
{
    if (!pkt) {
        return;
    }

    latestGlovePacket = *pkt;
    lastGlovePacketTick = xTaskGetTickCount();
    hasGlovePacket = true;

    /* Instant glove emergency stop handling */
    if (pkt->emergency != 0) {
        DEBUG_PRINTW("Glove emergency stop received!\n");
        stabilizerSetEmergencyStop();
    }
}

controlOwner_t controlMuxGetOwner(void)
{
    return currentOwner;
}

bool controlMuxIsAppActive(void)
{
    return ((xTaskGetTickCount() - lastAppPacketTick) < pdMS_TO_TICKS(APP_ACTIVITY_TIMEOUT_MS));
}

bool controlMuxIsGloveActive(void)
{
    return (hasGlovePacket && ((xTaskGetTickCount() - lastGlovePacketTick) < pdMS_TO_TICKS(GLOVE_ACTIVITY_TIMEOUT_MS)));
}

void controlMuxUpdate(void)
{
    if (!isInit) {
        return;
    }

    uint32_t now = xTaskGetTickCount();
    bool appActive = (lastAppPacketTick > 0) && ((now - lastAppPacketTick) < pdMS_TO_TICKS(APP_ACTIVITY_TIMEOUT_MS));
    bool gloveActive = hasGlovePacket && ((now - lastGlovePacketTick) < pdMS_TO_TICKS(GLOVE_ACTIVITY_TIMEOUT_MS));

    appActiveStatus = appActive ? 1 : 0;
    gloveActiveStatus = gloveActive ? 1 : 0;

    /* RULE 1: Official mobile application ALWAYS takes priority */
    if (appActive || (allowGloveParam == 0)) {
        currentOwner = CONTROL_OWNER_APP;
        /* App setpoint is already fed into commander directly via CRTP */
        return;
    }

    /* RULE 2: Glove takes control ONLY when app is idle, glove is enabled, and glove packets are fresh */
    if (gloveActive) {
        /* Check dead-man switch */
        if (latestGlovePacket.deadman == 0) {
            /* Hand open / released -> Failsafe cutoff */
            currentOwner = CONTROL_OWNER_FAILSAFE;
            setpoint_t failsafeSetpoint = {0};
            failsafeSetpoint.timestamp = now;
            failsafeSetpoint.mode.x = modeDisable;
            failsafeSetpoint.mode.y = modeDisable;
            failsafeSetpoint.mode.z = modeDisable;
            failsafeSetpoint.mode.roll = modeAbs;
            failsafeSetpoint.mode.pitch = modeAbs;
            failsafeSetpoint.mode.yaw = modeVelocity;
            failsafeSetpoint.attitude.roll = 0.0f;
            failsafeSetpoint.attitude.pitch = 0.0f;
            failsafeSetpoint.attitudeRate.yaw = 0.0f;
            failsafeSetpoint.thrust = 0;

            commanderSetSetpoint(&failsafeSetpoint, COMMANDER_PRIORITY_CRTP);
            return;
        }

        /* Check glove emergency button */
        if (latestGlovePacket.emergency != 0) {
            currentOwner = CONTROL_OWNER_FAILSAFE;
            stabilizerSetEmergencyStop();
            return;
        }

        /* Glove is active and safe */
        currentOwner = CONTROL_OWNER_GLOVE;

        setpoint_t gloveSetpoint = {0};
        gloveSetpoint.timestamp = now;

        /* Convert raw glove angles: unit is 0.01 deg */
        float rollDeg = (float)latestGlovePacket.roll * 0.01f;
        float pitchDeg = (float)latestGlovePacket.pitch * 0.01f;
        /* Clamp safe tilt limit: ±25 deg */
        if (rollDeg > 25.0f) rollDeg = 25.0f;
        if (rollDeg < -25.0f) rollDeg = -25.0f;
        if (pitchDeg > 25.0f) pitchDeg = 25.0f;
        if (pitchDeg < -25.0f) pitchDeg = -25.0f;

        /* Yaw rate: unit is 0.1 deg/s */
        float yawRate = (float)latestGlovePacket.yaw * 0.1f;
        if (yawRate > 150.0f) yawRate = 150.0f;
        if (yawRate < -150.0f) yawRate = -150.0f;

        /* Thrust: clamp to max 60000 */
        uint16_t thrust = latestGlovePacket.thrust;
        if (thrust > 60000) thrust = 60000;

        gloveSetpoint.mode.x = modeDisable;
        gloveSetpoint.mode.y = modeDisable;
        gloveSetpoint.mode.roll = modeAbs;
        gloveSetpoint.mode.pitch = modeAbs;
        gloveSetpoint.mode.yaw = modeVelocity;

        gloveSetpoint.attitude.roll = rollDeg;
        gloveSetpoint.attitude.pitch = pitchDeg;
        gloveSetpoint.attitudeRate.yaw = yawRate;
        gloveSetpoint.thrust = (float)thrust;

        if (latestGlovePacket.mode == 1) {
            /* Altitude assist mode */
            gloveSetpoint.mode.z = modeVelocity;
            gloveSetpoint.velocity.z = ((float)thrust - 32767.0f) / 32767.0f;
        } else {
            gloveSetpoint.mode.z = modeDisable;
        }

        /* Inject glove setpoint into commander queue */
        commanderSetSetpoint(&gloveSetpoint, COMMANDER_PRIORITY_CRTP);

    } else {
        /* Neither App nor Glove is sending */
        currentOwner = CONTROL_OWNER_APP;
    }
}

PARAM_GROUP_START(mux)
PARAM_ADD(PARAM_UINT8, allowGlove, &allowGloveParam)
PARAM_GROUP_STOP(mux)

LOG_GROUP_START(mux)
LOG_ADD(LOG_UINT8, owner, &currentOwner)
LOG_ADD(LOG_UINT8, appActive, &appActiveStatus)
LOG_ADD(LOG_UINT8, gloveActive, &gloveActiveStatus)
LOG_ADD(LOG_UINT8, seq, &latestGlovePacket.sequence)
LOG_ADD(LOG_UINT16, thrust, &latestGlovePacket.thrust)
LOG_ADD(LOG_UINT8, deadman, &latestGlovePacket.deadman)
LOG_GROUP_STOP(mux)
