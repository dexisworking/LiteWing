/**
 * LiteWing Custom Firmware - Control Mux
 *
 * Implements APP-first control-source arbitration between official
 * LiteWing mobile application (CRTP:2390) and gesture glove (UDP:2391).
 */

#ifndef __CONTROL_MUX_H__
#define __CONTROL_MUX_H__

#include <stdbool.h>
#include <stdint.h>
#include "stabilizer_types.h"
#include "glove_protocol.h"

typedef enum {
    CONTROL_OWNER_APP = 0,      /* Official mobile app has active priority */
    CONTROL_OWNER_GLOVE = 1,    /* Gesture glove has active control */
    CONTROL_OWNER_FAILSAFE = 2  /* Stale / deadman failsafe shutdown */
} controlOwner_t;

/**
 * Initialize control mux arbitrator
 */
void controlMuxInit(void);

/**
 * Called whenever official mobile app delivers a setpoint on port 2390
 */
void controlMuxNotifyAppSetpoint(const setpoint_t *setpoint);

/**
 * Called whenever gesture glove delivers a verified packet on port 2391
 */
void controlMuxNotifyGlovePacket(const glovePacket_t *pkt);

/**
 * Periodic update hook in stabilizer or commander loop
 */
void controlMuxUpdate(void);

/**
 * Get current owner
 */
controlOwner_t controlMuxGetOwner(void);

/**
 * Status checks
 */
bool controlMuxIsGloveActive(void);
bool controlMuxIsAppActive(void);

#endif /* __CONTROL_MUX_H__ */
