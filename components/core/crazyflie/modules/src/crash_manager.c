/**
 * LiteWing Custom Firmware - Crash Manager Implementation
 *
 * Implements deterministic crash detection, motor cutoff,
 * controlled software-level recovery diagnostics, and safe explicit rearming
 * without requiring ESP32 / battery power cycling.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "crash_manager.h"
#include "stabilizer.h"
#include "sitaw.h"
#include "attitude_controller.h"
#include "system.h"
#include "pm_esplane.h"
#include "log.h"
#include "param.h"

#define DEBUG_MODULE "CRASH"
#include "debug_cf.h"

#define IMPACT_G_THRESHOLD        4.5f   /* Total accel magnitude spike (Gs) */
#define TUMBLE_Z_THRESHOLD       -0.5f   /* Inverted / tumble threshold */
#define SUSPECTED_TICKS_LIMIT     10     /* 10ms of sustained fault */
#define DIAG_STEADY_TICKS_REQUIRED 250   /* 250ms of stable upright rest required for recovery */

#define GYRO_REST_THRESHOLD       25.0f  /* deg/s max rate while resting */
#define ACC_REST_Z_MIN            0.75f  /* min vertical Gs at rest upright */
#define ACC_REST_Z_MAX            1.25f  /* max vertical Gs at rest upright */
#define ACC_REST_HORIZ_MAX        0.40f  /* max horizontal tilt Gs at rest */

static crashState_t crashState = CRASH_STATE_NORMAL;
static crashReason_t crashReason = CRASH_REASON_NONE;
static uint32_t suspectedCounter = 0;
static uint32_t steadyCounter = 0;
static uint32_t recoveryCount = 0;

static uint8_t triggerRecoveryParam = 0;
static uint8_t triggerRearmParam = 0;
static float minVbatToArm = 3.20f;
static uint8_t armInhibit = 0;

static bool isInit = false;

void crashManagerInit(void)
{
    if (isInit) {
        return;
    }

    crashState = CRASH_STATE_NORMAL;
    crashReason = CRASH_REASON_NONE;
    suspectedCounter = 0;
    steadyCounter = 0;
    recoveryCount = 0;
    triggerRecoveryParam = 0;
    triggerRearmParam = 0;
    armInhibit = 0;

    isInit = true;
    DEBUG_PRINTI("Crash Manager initialized.\n");
}

crashState_t crashManagerGetState(void)
{
    return crashState;
}

bool crashManagerIsSafeToFly(void)
{
    return (crashState == CRASH_STATE_NORMAL && !armInhibit);
}

bool crashManagerIsArmingInhibited(void)
{
    return (armInhibit != 0);
}

static bool checkSensorsAtRest(const sensorData_t *sensorData)
{
    if (fabsf(sensorData->acc.z - 1.0f) > (ACC_REST_Z_MAX - 1.0f)) {
        return false;
    }
    if (fabsf(sensorData->acc.x) > ACC_REST_HORIZ_MAX || fabsf(sensorData->acc.y) > ACC_REST_HORIZ_MAX) {
        return false;
    }
    if (fabsf(sensorData->gyro.x) > GYRO_REST_THRESHOLD ||
        fabsf(sensorData->gyro.y) > GYRO_REST_THRESHOLD ||
        fabsf(sensorData->gyro.z) > GYRO_REST_THRESHOLD) {
        return false;
    }
    return true;
}

bool crashManagerRequestRecovery(void)
{
    if (crashState == CRASH_STATE_MOTOR_SHUTDOWN || crashState == CRASH_STATE_LATCHED) {
        DEBUG_PRINTI("Recovery requested. Entering diagnostics...\n");
        crashState = CRASH_STATE_RECOVERY_DIAGNOSTICS;
        steadyCounter = 0;
        return true;
    }
    return false;
}

bool crashManagerRequestRearm(void)
{
    if (crashState == CRASH_STATE_RECOVERY_READY) {
        float vbat = pmGetBatteryVoltage();
        if (vbat < minVbatToArm) {
            DEBUG_PRINTW("Rearm rejected: Low battery (%0.2fV < %0.2fV)\n", (double)vbat, (double)minVbatToArm);
            armInhibit = 1;
            return false;
        }

        DEBUG_PRINTI("Rearm accepted. Aircraft operational.\n");
        systemSetArmed(true);
        crashState = CRASH_STATE_NORMAL;
        crashReason = CRASH_REASON_NONE;
        armInhibit = 0;
        return true;
    }
    return false;
}

void crashManagerUpdate(const sensorData_t *sensorData, const state_t *state)
{
    if (!isInit) {
        return;
    }

    /* Check parameter triggers from mobile app / ground station */
    if (triggerRecoveryParam != 0) {
        triggerRecoveryParam = 0;
        crashManagerRequestRecovery();
    }
    if (triggerRearmParam != 0) {
        triggerRearmParam = 0;
        crashManagerRequestRearm();
    }

    /* Monitor battery for arming inhibition */
    float currentVbat = pmGetBatteryVoltage();
    if (currentVbat < minVbatToArm && currentVbat > 1.0f) {
        armInhibit = 1;
    } else {
        armInhibit = 0;
    }

    switch (crashState) {
        case CRASH_STATE_NORMAL: {
            bool faultDetected = false;
            crashReason_t currentReason = CRASH_REASON_NONE;

            /* Check situation awareness tumble detector */
#ifdef SITAW_TU_ENABLED
            if (sitAwTuDetected()) {
                faultDetected = true;
                currentReason = CRASH_REASON_TUMBLE;
            }
#endif
            /* Check high-G impact or inverted flight during active thrust */
            float accMag = sqrtf(sensorData->acc.x * sensorData->acc.x +
                                 sensorData->acc.y * sensorData->acc.y +
                                 sensorData->acc.z * sensorData->acc.z);

            if (accMag > IMPACT_G_THRESHOLD) {
                faultDetected = true;
                currentReason = CRASH_REASON_HIGH_G;
            } else if (sensorData->acc.z < TUMBLE_Z_THRESHOLD) {
                faultDetected = true;
                currentReason = CRASH_REASON_TUMBLE;
            }

            /* Check if stabilizer emergency stop was externally raised */
            if (stabilizerIsEmergencyStop()) {
                faultDetected = true;
                if (currentReason == CRASH_REASON_NONE) {
                    currentReason = CRASH_REASON_ESTOP_PARAM;
                }
            }

            if (faultDetected) {
                suspectedCounter++;
                if (suspectedCounter >= SUSPECTED_TICKS_LIMIT || currentReason == CRASH_REASON_TUMBLE) {
                    crashState = CRASH_STATE_LATCHED;
                    crashReason = currentReason;
                    DEBUG_PRINTW("CRASH DETECTED (reason=%d)! Latched e-stop.\n", (int)crashReason);

                    /* Cut motors immediately */
                    stabilizerSetEmergencyStop();
                    systemSetArmed(false);
                    powerStop();

                    crashState = CRASH_STATE_MOTOR_SHUTDOWN;
                    suspectedCounter = 0;
                }
            } else {
                if (suspectedCounter > 0) {
                    suspectedCounter--;
                }
            }
            break;
        }

        case CRASH_STATE_LATCHED:
        case CRASH_STATE_MOTOR_SHUTDOWN: {
            /* Enforce motor shutdown */
            stabilizerSetEmergencyStop();
            powerStop();

            /* If aircraft has settled upright at rest for > 1 second, automatically transition to diagnostics */
            if (checkSensorsAtRest(sensorData)) {
                steadyCounter++;
                if (steadyCounter > 1000) { /* 1000 ticks = 1 second at rest */
                    crashManagerRequestRecovery();
                }
            } else {
                steadyCounter = 0;
            }
            break;
        }

        case CRASH_STATE_RECOVERY_DIAGNOSTICS: {
            /* Keep motors stopped during diagnostic check */
            stabilizerSetEmergencyStop();
            powerStop();

            if (checkSensorsAtRest(sensorData)) {
                steadyCounter++;
                if (steadyCounter >= DIAG_STEADY_TICKS_REQUIRED) {
                    /* Aircraft is verified stationary and upright */
                    /* Step 1: Reset PID integral terms */
                    attitudeControllerResetAllPID();

                    /* Step 2: Reset situation awareness triggers */
                    sitAwReset();

                    /* Step 3: Clear emergency stop */
                    stabilizerResetEmergencyStop();

                    /* Step 4: Keep system DISARMED (requires explicit rearm) */
                    systemSetArmed(false);

                    recoveryCount++;
                    crashState = CRASH_STATE_RECOVERY_READY;
                    steadyCounter = 0;
                    DEBUG_PRINTI("Crash recovery complete (#%"PRIu32"). Ready for manual rearm.\n", recoveryCount);
                }
            } else {
                /* Aircraft disturbed while in diagnostics, restart steady counter */
                steadyCounter = 0;
            }
            break;
        }

        case CRASH_STATE_RECOVERY_READY: {
            /* System is recovered, but DISARMED until explicit user rearm command */
            if (systemIsArmed()) {
                /* App armed the drone */
                crashManagerRequestRearm();
            }
            break;
        }

        case CRASH_STATE_DISARMED:
        default:
            break;
    }
}

PARAM_GROUP_START(crash)
PARAM_ADD(PARAM_UINT8, recover, &triggerRecoveryParam)
PARAM_ADD(PARAM_UINT8, rearm, &triggerRearmParam)
PARAM_ADD(PARAM_FLOAT, minVbat, &minVbatToArm)
PARAM_GROUP_STOP(crash)

LOG_GROUP_START(crash)
LOG_ADD(LOG_UINT8, state, &crashState)
LOG_ADD(LOG_UINT8, reason, &crashReason)
LOG_ADD(LOG_UINT32, recCount, &recoveryCount)
LOG_ADD(LOG_UINT8, armInhibit, &armInhibit)
LOG_ADD(LOG_UINT32, steadyTime, &steadyCounter)
LOG_GROUP_STOP(crash)
