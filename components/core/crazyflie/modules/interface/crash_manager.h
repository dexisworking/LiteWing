/**
 * LiteWing Custom Firmware - Crash Manager
 *
 * Implements deterministic crash detection, motor cutoff,
 * controlled software-level recovery diagnostics, and safe explicit rearming
 * without requiring ESP32 / battery power cycling.
 */

#ifndef __CRASH_MANAGER_H__
#define __CRASH_MANAGER_H__

#include <stdbool.h>
#include <stdint.h>
#include "stabilizer_types.h"

typedef enum {
    CRASH_STATE_NORMAL = 0,             /* Flying or ready at rest */
    CRASH_STATE_SUSPECTED = 1,          /* Transient impact detected */
    CRASH_STATE_LATCHED = 2,            /* Crash confirmed, e-stop asserted */
    CRASH_STATE_MOTOR_SHUTDOWN = 3,     /* Motors killed, waiting for rest */
    CRASH_STATE_RECOVERY_DIAGNOSTICS = 4, /* Verifying sensors & attitude */
    CRASH_STATE_RECOVERY_READY = 5,     /* Cleared & disarmed, ready for rearm */
    CRASH_STATE_DISARMED = 6            /* Safe disarmed state */
} crashState_t;

typedef enum {
    CRASH_REASON_NONE = 0,
    CRASH_REASON_TUMBLE = 1,
    CRASH_REASON_HIGH_G = 2,
    CRASH_REASON_ESTOP_PARAM = 3,
    CRASH_REASON_FREE_FALL = 4,
} crashReason_t;

/**
 * Initialize crash manager state machine
 */
void crashManagerInit(void);

/**
 * Update crash manager at stabilizer loop rate (1 kHz)
 */
void crashManagerUpdate(const sensorData_t *sensorData, const state_t *state);

/**
 * Request software crash recovery (resets PIDs, sitAw, clears e-stop)
 * @return true if recovery request was accepted
 */
bool crashManagerRequestRecovery(void);

/**
 * Request manual rearm after recovery
 * @return true if rearm conditions are met and drone is armed
 */
bool crashManagerRequestRearm(void);

/**
 * Get current crash manager state
 */
crashState_t crashManagerGetState(void);

/**
 * Check if aircraft is in a safe condition to run motors
 */
bool crashManagerIsSafeToFly(void);

/**
 * Check if arming is inhibited (e.g. low battery or unrecovered crash)
 */
bool crashManagerIsArmingInhibited(void);

#endif /* __CRASH_MANAGER_H__ */
