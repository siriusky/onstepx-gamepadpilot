// GamepadPILOT_functions.cpp
// Contains file-local helper functions and class method implementations.

// ============================================================================
// Includes and external references
// ============================================================================
#include "GamepadPILOT.h"
#include <Bluepad32.h>
#include "../../Common.h"
#include "../../telescope/mount/Mount.h"
#include "../../lib/serial/Serial_Local.h"
#include "../../lib/axis/Axis.h"
#include "../../libApp/commands/CommandBroker.h"
#include "../../telescope/mount/limits/Limits.h"
#include "../../telescope/mount/park/Park.h"
#include <esp_heap_caps.h>   // heap_caps_get_largest_free_block() for VERBOSE_SYSTEM_STATS

extern CommandBroker commandBroker;

// ============================================================================
// File-local compile-time constants and math helpers
// ============================================================================
namespace {
// used in normalizeStick()
constexpr float RANGE_RECIPROCAL = 1.0f / static_cast<float>(STICK_MAX - DEADZONE);
constexpr float DEG_TO_RAD_F = 0.017453292519943f;

// Temporary acceleration used while slewing and upper bound for the
// acceleration derived from the mount configuration (rad/s^2).
constexpr float HIGH_SLEW_ACCEL_RAD_S2 = 10.0f;

// Conservative acceleration used when the derived value is above the bound.
constexpr float SAFE_SLEW_ACCEL_RAD_S2 = 0.1f;
}

// Smoothly approach target speed using a fixed rate over delta time.
// Snaps to target when close enough to avoid micro-oscillation.
static inline float applyAcceleration(float target, float current, float rate, float dt) {
    float step = rate * dt;
    if (fabsf(current - target) < step) return target;
    return (current < target) ? (current + step) : (current - step);
}

// True on the rising edge of any bit in "bits" (0 disables the trigger).
static inline bool risingEdge(uint32_t current, uint32_t previous, uint32_t bits) {
    return bits != 0 && (current & bits) != 0 && (previous & bits) == 0;
}

// Convert raw Bluepad32 stick values (-512..511) to normalized range -1.0..1.0.
// Applies deadzone and manual clamping.
static inline float normalizeStick(int val, int deadzone, float reciprocal) {
    if (val > deadzone) {
        float res = static_cast<float>(val - deadzone) * reciprocal;
        return (res > 1.0f) ? 1.0f : res;
    }
    if (val < -deadzone) {
        float res = static_cast<float>(val + deadzone) * reciprocal;
        return (res < -1.0f) ? -1.0f : res;
    }
    return 0.0f;
}

// ============================================================================
// FreeRTOS task function (file-local)
// ============================================================================

// Main task that runs on Core 0.
// It casts the void pointer back to GamepadPILOT and repeatedly calls run().
static void bluepadTask(void* pvParameters) {
    GamepadPILOT* myPlugin = static_cast<GamepadPILOT*>(pvParameters);

    // Initial pause to avoid competing for resources during OnStepX startup.
    vTaskDelay(pdMS_TO_TICKS(200));

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        myPlugin->run();

        // Keep loop frequency constant for smooth acceleration ramps.
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(POLL_INTERVAL));
    }
}

// ============================================================================
// Bluepad32 connection callbacks and state reset
// ============================================================================

void GamepadPILOT::resetButtonEdges() {
    lastBtns = 0;
    lastDpad = 0;
}

// Build the controller's Bluetooth MAC as an uppercase "AA:BB:CC:DD:EE:FF" string.
// getProperties() returns a copy, so it is stored locally before reading btaddr
static String macToString(ControllerPtr ctl) {
    ControllerProperties props = ctl->getProperties();
    char buf[18];
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             props.btaddr[0], props.btaddr[1], props.btaddr[2],
             props.btaddr[3], props.btaddr[4], props.btaddr[5]);
    return String(buf);
}

// Called by Bluepad32 when a new controller connects.
// Only one active controller is supported (slot 0).
// Rejected controllers are disconnected right away
// so they are free to connect to another device and do not
// look connected while being ignored.
void GamepadPILOT::onConnected(ControllerPtr ctl) {
    String macStr = macToString(ctl);
    String modelName = ctl->getModelName();

    // Limit log spam from a rejected controller that keeps retrying:
    // the same MAC is logged at most every 2 s.
    static String lastLoggedMac = "";
    static uint32_t lastLoggedMs = 0;
    bool logAttempt = (macStr != lastLoggedMac) || (millis() - lastLoggedMs > 2000);
    if (logAttempt) {
        lastLoggedMac = macStr;
        lastLoggedMs = millis();
    }

#if MAC_WHITELIST_ENABLE == 1 || PARK_PAIRING_ONLY == 1
        // Show every controller that tries to connect, to find out its MAC.
        if (logAttempt) {
            VLF(("MSG: Controller connecting: " + modelName + ", MAC " + macStr).c_str());
        }
#endif

    const char* rejectReason = nullptr;

    // Only one controller is supported.
    if (gamepadpilot.myControllers[0] != nullptr) {
        rejectReason = "another controller is already connected";
    }

#if MAC_WHITELIST_ENABLE == 1
        // Static whitelist check
        if (rejectReason == nullptr) {
            String whitelistStr = MAC_WHITELIST_ADDRESS;
            whitelistStr.toUpperCase();
            if (macStr != whitelistStr) rejectReason = "MAC not whitelisted";
        }
#endif

#if PARK_PAIRING_ONLY == 1
        // Park-pairing-only: the first controller must connect while the mount is parked,
        // afterwards only that MAC may (re)connect for the rest of the session.
        bool newSessionPairing = false;
        if (rejectReason == nullptr) {
            const String& sessionMac = gamepadpilot.sessionWhitelistedMac;
            if (sessionMac.length() > 0) {
                if (macStr != sessionMac) rejectReason = "MAC differs from session-paired controller";
            } else if (park.state != PS_PARKED) {
                rejectReason = "mount not parked (park pairing only)";
            } else {
                newSessionPairing = true;
            }
        }
#endif

    if (rejectReason != nullptr) {
        if (logAttempt) {
            VLF(("MSG: Controller rejected (" + String(rejectReason) + "), disconnecting MAC " + macStr).c_str());
        }
        // todo: test ctl->disconnect()  for error "Failed to disconnect HIDS client", 
        // works fine with Bluepad32 4.1.0 and Xbox One
        ctl->disconnect();
        return;
    }

    // Accept: store the controller in slot 0.
    gamepadpilot.myControllers[0] = ctl;
    gamepadpilot.lastDataMillis = millis();  // initialise watchdog timer
    gamepadpilot.resetButtonEdges();

#if PARK_PAIRING_ONLY == 1
        if (newSessionPairing) {
            // Remember the MAC only once the slot is actually assigned.
            gamepadpilot.sessionWhitelistedMac = macStr;
            VLF(("MSG: Park pairing accepted. Session-paired MAC: " + macStr).c_str());
        } else {
            VLF("MSG: Session-paired controller reconnected.");
        }
#endif

    VLF(("MSG: Connected to: " + modelName + ", MAC " + macStr).c_str());
    VLF("MSG: Gamepad connected");
}

// Called by Bluepad32 when a controller disconnects.
// Clears the stored pointer.
void GamepadPILOT::onDisconnected(ControllerPtr ctl) {
    // Clear slot 0 if the disconnected controller matches the stored one.
    if (gamepadpilot.myControllers[0] == ctl) {
        gamepadpilot.myControllers[0] = nullptr;
        gamepadpilot.resetButtonEdges();
        VLF("MSG: Gamepad disconnected");
    }
}

// ============================================================================
// Max slew speed management (async broker request)
// ============================================================================

// Request the current max slew speed from the mount asynchronously.
// The broker handle is stored and later processed in updateMaxSlewSpeedFromBroker().
void GamepadPILOT::requestMaxSlewSpeed() {
    if (brokerHandle == 0) {
        brokerHandle = commandBroker.request(":GX97#", 200);
    }
}

// Check and process the asynchronous :GX97# request, updating max slew and acceleration.
void GamepadPILOT::updateMaxSlewSpeedFromBroker() {
    char reply[20];
    // result() returns CB_DONE when the core has answered the :GX97# command
    CommandBrokerStatus status = commandBroker.result(brokerHandle, reply, sizeof(reply));
    if (status == CB_DONE) {
        if (strchr(reply, '#')) {            // replies without # rejected
            float maxSpeedDeg = atof(reply); // reply format is "n.n#"
            if (maxSpeedDeg > 0.1f && maxSpeedDeg < 360.0f) {   // sanity bounds
                activeMaxSlewSpeedRad = maxSpeedDeg * DEG_TO_RAD_F; // Degrees/s to Radians/s

                // Recalculate original acceleration to maintain consistency with the new max speed
                originalAccel = accelerationFromMaxSpeed(activeMaxSlewSpeedRad);

                VLF(("MSG: GamepadPILOT updated MaxSlew from :GX97#: " + String(maxSpeedDeg) + " deg/s (Accel: " + String(originalAccel) + ")").c_str());
            }
            brokerHandle = 0; // Reset handle after success to allow the next stick movement to trigger a fresh poll
        }
    } else if (status == CB_TIMEOUT || status == CB_FREE) {
        brokerHandle = 0; // Reset handle on failure so can try again on next movement
    }
}

// Derive the mount's original slew acceleration from a maximum slew speed.
// a = v^2 / (2 * d), with a conservative fallback and safety cap.
float GamepadPILOT::accelerationFromMaxSpeed(float maxSlewRad) const {
    const float dRad = static_cast<float>(SLEW_ACCELERATION_DIST) * DEG_TO_RAD_F;
    if (dRad <= 0.0f) return SAFE_SLEW_ACCEL_RAD_S2;

    const float accel = (maxSlewRad * maxSlewRad) / (2.0f * dRad);
    if (accel > HIGH_SLEW_ACCEL_RAD_S2) return SAFE_SLEW_ACCEL_RAD_S2;
    return accel;
}

// ============================================================================
// Input reading and normalization
// ============================================================================

// Read raw stick, button and dpad values from the controller into member variables.
void GamepadPILOT::readStickValues(ControllerPtr myCtl) {
    xVal = myCtl->axisX();
    yVal = myCtl->axisY();
    rxVal = myCtl->axisRX();
    ryVal = myCtl->axisRY();
    btns = myCtl->buttons();
    dpad = myCtl->dpad();
}

// Normalize the raw stick values using the file-local normalizeStick() helper.
void GamepadPILOT::processStickNormalization() {
    stickPercentX  = normalizeStick(xVal,  DEADZONE, RANGE_RECIPROCAL);
    stickPercentY  = normalizeStick(yVal,  DEADZONE, RANGE_RECIPROCAL);
    stickPercentrX = normalizeStick(rxVal, DEADZONE, RANGE_RECIPROCAL);
    stickPercentrY = normalizeStick(ryVal, DEADZONE, RANGE_RECIPROCAL);
}

// ============================================================================
// Speed mapping and smoothing
// ============================================================================

// Apply the configured stick response curve to a single axis value.
float GamepadPILOT::applyStickCurve(float value) {
#if STICK_CURVE_EXPONENT == 2
        return value * fabsf(value);
#elif STICK_CURVE_EXPONENT == 3
        return value * value * value;
#else
        return value;
#endif
}

// Convert normalized stick values to target speeds using gear, swap, and max slew.
void GamepadPILOT::updateTargetSpeeds(float deltaTime) {
    float xScaled = applyStickCurve(mainStickX());
    float yScaled = applyStickCurve(mainStickY());

    float speedMultiplier = SPEED_MULTIPLIERS[currentGearIndex];

#if USE_DPAD_SLEWING == 1
    if (dpad != 0) {
        // Digital D-pad:
        // dpad up:1, down:2, right:4, left:8
        // Add bit values for diagonals.
        xScaled = ((dpad & 4) ? 1.0f : 0.0f) - ((dpad & 8) ? 1.0f : 0.0f); // right - left
        yScaled = ((dpad & 2) ? 1.0f : 0.0f) - ((dpad & 1) ? 1.0f : 0.0f); // down - up

        speedMultiplier = DPAD_SPEED_MULTIPLIERS[currentGearIndex];
    }
#endif

    float invX = activeSwapEW ? -1.0f : 1.0f;
    float invY = activeSwapNS ? -1.0f : 1.0f;

    targetSpeedXNorm = xScaled * speedMultiplier * invX; //(-1..1)
    targetSpeedYNorm = yScaled * speedMultiplier * invY;

#if ENABLE_SAT_TRACKING_FEATURE == 1
    if (satelliteTracker.isActive() || satelliteTracker.isRecording()) {
        // Control step (getPosition, orbit prediction, PI) only on the send tick,
        // so every computed speed is actually sent. Orbit steering: alt-az, GEM and FORK
        // (ALTALT not yet supported -> simple integrator only).
        bool controlTick = (pollCounter + 1 >= COMMAND_INTERVAL_MULTIPLE);
        bool orbitSteerAllowed = (mountType == ALTAZM || mountType == GEM || mountType == FORK);
        satelliteTracker.update(deltaTime, targetSpeedXNorm, targetSpeedYNorm,
                                activeMaxSlewSpeedRad,
                                targetSpeedX, targetSpeedY,
                                controlTick, orbitSteerAllowed);
        if (satelliteTracker.consumeHorizonStop()) {
            VLF("MSG: Satellite tracking ended (below horizon)");
            ControllerPtr ctl = myControllers[0];
            if (ctl && ctl->isConnected()) ctl->playDualRumble(0, 150, 0x80, 0x40);
        }
    } else
#endif
    {
    targetSpeedX = targetSpeedXNorm * activeMaxSlewSpeedRad;
    targetSpeedY = targetSpeedYNorm * activeMaxSlewSpeedRad;
    }

    // detect limits and stop if outside limits
    applyLimits(targetSpeedX, targetSpeedY);
}

// Check for limit error. When error occurs only allow movement for a short amount of time, 
// then stick and D-pad input must be released again.
void GamepadPILOT::applyLimits(float &speedX, float &speedY) {
    // If tracking is active and outside limits, stop tracking immediately
#if ENABLE_SAT_TRACKING_FEATURE == 1
    if (limits.isError() && satelliteTracker.isActive()) {
        satelliteTracker.disable();
        VLF("MSG: satellite tracking disabled due to limit error.");
    }
#endif

    // Normal operation: mount is inside limits.
    if (!limits.isError()) {
        recoveryState = RecoveryState::IDLE;
        return;
    }

    // State Machine for Limit Recovery
    if (recoveryState == RecoveryState::IDLE) {
        recoveryState = RecoveryState::WAITING_FOR_CENTER;
        ControllerPtr ctl = myControllers[0];
        if (ctl && ctl->isConnected()) ctl->playDualRumble(0, 100, 0x80, 0x40);
    }

    if (recoveryState == RecoveryState::WAITING_FOR_CENTER && !slewInputActive()) {
        recoveryState = RecoveryState::ARMED;
    }

    if (recoveryState == RecoveryState::ARMED && slewInputActive()) {
        recoveryState = RecoveryState::MOVING;
        limitRecoveryWindowStart = millis();
    }

    // re-arm when all slew input is released while in MOVING state
    if (recoveryState == RecoveryState::MOVING && !slewInputActive()) {
        recoveryState = RecoveryState::ARMED;
    }

    // Calculate if the recovery window is still active
    bool windowOpen = (recoveryState == RecoveryState::MOVING) &&
                      (millis() - limitRecoveryWindowStart < LIMIT_RECOVERY_WINDOW_MS);

    // ENFORCE SAFETY RULES:
    // Force a full stop if:
    // 1. No slew input is active (blocks other background processes from moving it)
    // 2. OR not armed yet, and the moving window has already expired.
    if (!slewInputActive() || (recoveryState != RecoveryState::ARMED && !windowOpen)) {
        speedX = 0.0f;
        speedY = 0.0f;
    }
}


// Smoothly accelerate towards the current target speeds using delta time.
void GamepadPILOT::calculateSmoothSpeeds(float deltaTime) {
    smoothTargetSpeedX = applyAcceleration(targetSpeedX, smoothTargetSpeedX, ACCEL_RATE, deltaTime);
    smoothTargetSpeedY = applyAcceleration(targetSpeedY, smoothTargetSpeedY, ACCEL_RATE, deltaTime);
}


// ============================================================================
// Slew command, acceleration, and cleanup helpers
// ============================================================================

// Temporarily raise axis acceleration while slewing to be able to set custom acceleration 
// and have decelleration (OnStepX does not apply deceleration to direct autoslew).
void GamepadPILOT::maybeEnableHighAcceleration() {
    if (!accelIsHigh && (smoothTargetSpeedX != 0.0f || smoothTargetSpeedY != 0.0f)) {
        axis1.setSlewAccelerationRate(HIGH_SLEW_ACCEL_RAD_S2);
        axis2.setSlewAccelerationRate(HIGH_SLEW_ACCEL_RAD_S2);
        accelIsHigh = true;
    }
}

// Run every poll tick. setSlewAccelerationRate() is silently ignored unless
// autoRate == AR_NONE, so keep trying until both axes are idle.
void GamepadPILOT::restoreAccelerationWhenIdle() {
    if (!accelIsHigh) return;
    if (axis1.isSlewing() || axis2.isSlewing()) return;

    axis1.setSlewAccelerationRate(originalAccel);
    axis2.setSlewAccelerationRate(originalAccel);
    accelIsHigh = false;
    VLF("MSG: Original acceleration restored");
}

// Send slew commands to both axes if speeds changed, respecting COMMAND_INTERVAL.
void GamepadPILOT::sendSlewCommands() {
    // control step on the same tick the command is sent
    if (++pollCounter >= COMMAND_INTERVAL_MULTIPLE) {
        pollCounter = 0;

#if PLOTTER_DEBUG == 1
        // Serial Plotter Output with Labels
        handlePlotterDebug();
#endif

        if (smoothTargetSpeedX != lastSentSpeedX || smoothTargetSpeedY != lastSentSpeedY) {
            maybeEnableHighAcceleration();

            // -----------------------------------------------------------------
            // Axis 2 speed inversion depending on mount type and pier side
            //   smoothTargetSpeedY < 0  ->  North / up
            //   smoothTargetSpeedY > 0  ->  South / down
            // -----------------------------------------------------------------
            float pierSideCorrectedY = smoothTargetSpeedY;

            // for GEM on pier side west, invert Y
            if (mountType == GEM && mount.getPosition().pierSide == PIER_SIDE_WEST) {
                pierSideCorrectedY = -pierSideCorrectedY;
            }

            // East/West (Axis 1)
            if (smoothTargetSpeedX == 0.0f) {
                if (axis1.isSlewing()) axis1.autoSlew(DIR_FORWARD, 0.0f);
            } else {
                Direction dirX = (smoothTargetSpeedX > 0.0f) ? DIR_FORWARD : DIR_REVERSE;
                axis1.autoSlew(dirX, fabsf(smoothTargetSpeedX));
            }

            // North/South (Axis 2)
            if (pierSideCorrectedY == 0.0f) {
                if (axis2.isSlewing()) axis2.autoSlew(DIR_FORWARD, 0.0f);
            } else {
                Direction dirY = (pierSideCorrectedY > 0.0f) ? DIR_REVERSE : DIR_FORWARD;
                axis2.autoSlew(dirY, fabsf(pierSideCorrectedY));
            }

            lastSentSpeedX = smoothTargetSpeedX;
            lastSentSpeedY = smoothTargetSpeedY;

            // Clear isSlewing flag, only when targetSpeed == 0 so quick direction reversal does not trigger it
            if (lastSentSpeedX == 0.0f && lastSentSpeedY == 0.0f &&
                targetSpeedX == 0.0f && targetSpeedY == 0.0f) {
                SERIAL_LOCAL.transmit(":Qe#");
                SERIAL_LOCAL.transmit(":Qn#");
            }
        }

        restoreAccelerationWhenIdle();
    }
}

// ============================================================================
// Safety stop handler
// Called when connection is lost or watchdog is triggered.
// ============================================================================
void GamepadPILOT::handleSafetyStop(bool connectionLost) {
    // Safety State: Always force target speeds to zero while watchdog or connection loss is active
    targetSpeedX = 0.0f;
    targetSpeedY = 0.0f;

    // Relock the mount if the connection is physically lost
    if (connectionLost) {
        isUnlocked = (USE_UNLOCK_SEQUENCE == 0);
        currentUnlockStep = UnlockStep::LOCKED;
        holdStartTime = 0;

#if ENABLE_SAT_TRACKING_FEATURE == 1
        // Immediately stop satellite tracking if the controller disconnects
        satelliteTracker.disable();
#endif
    }

    // Notify once per contiguous safety-stop event.
    if (!safetyStopNoticeSent) {
        if (connectionLost) {
            VLF("MSG: Connection Lost! Sending Safety Stop...");
        } else {
            VLF("MSG: Watchdog Triggered (Stale Data)! Sending Safety Stop...");
        }
        safetyStopNoticeSent = true;
    }
}

// ============================================================================
// Input processing helpers (unlock sequence and button events)
// ============================================================================

// Handles the unlock sequence and returns whether the mount is currently unlocked.
// When USE_UNLOCK_SEQUENCE == 0, this function always returns true.
bool GamepadPILOT::handleUnlockSequence(uint8_t currentDpad) {
#if USE_UNLOCK_SEQUENCE == 1

    if (!isUnlocked) {
        // dpad values: up:1 ; down:2 ; right:4 ; left:8
        if (currentDpad == 2) { // Holding DOWN
            if (currentUnlockStep == UnlockStep::LOCKED) {
                if (holdStartTime == 0) holdStartTime = millis();
                if (millis() - holdStartTime >= UNLOCK_HOLD_MS) {
                    currentUnlockStep = UnlockStep::WAIT_FOR_UP;
                    holdStartTime = 0;
                    waitForUpStartTime = 0; // Window starts only after DOWN is released
                    ControllerPtr ctl = myControllers[0];
                    if (ctl && ctl->isConnected()) ctl->playDualRumble(0, 100, 0x80, 0x40);
                    VLF("MSG: Unlock Stage 1 complete. Now hold UP.");
                }
            }
        } else if (currentDpad == 1) { // Holding UP
            if (currentUnlockStep == UnlockStep::WAIT_FOR_UP) {
                if (holdStartTime == 0) {
                    // First UP press after DOWN release: check the 2s window.
                    if (waitForUpStartTime == 0 ||
                        (millis() - waitForUpStartTime) > 2000) {
                        currentUnlockStep = UnlockStep::LOCKED;
                        waitForUpStartTime = 0;
                        VLF("MSG: Unlock sequence reset: UP started too late.");
                    } else {
                        holdStartTime = millis();
                    }
                }

                if (currentUnlockStep == UnlockStep::WAIT_FOR_UP && holdStartTime != 0) {
                    if (millis() - holdStartTime >= UNLOCK_HOLD_MS) {
                        isUnlocked = true;
                        waitForUpStartTime = 0;
                        VLF("MSG: GamepadPILOT UNLOCKED.");
                        ControllerPtr ctl = myControllers[0];
                        if (ctl && ctl->isConnected()) ctl->playDualRumble(0, 150, 0x80, 0x40);
                    }
                }
            }
        } else if (currentDpad != 0) { // Pressed something else (Left/Right)
            currentUnlockStep = UnlockStep::LOCKED;
            holdStartTime = 0;
            waitForUpStartTime = 0;
        } else { // D-pad released
            holdStartTime = 0;

            if (currentUnlockStep == UnlockStep::WAIT_FOR_UP) {
                if (waitForUpStartTime == 0) {
                    // DOWN was just released; start the 2 second window.
                    waitForUpStartTime = millis();
                } else if ((millis() - waitForUpStartTime) > 2000) {
                    currentUnlockStep = UnlockStep::LOCKED;
                    waitForUpStartTime = 0;
                    VLF("MSG: Unlock sequence reset: UP not started within 2s.");
                }
            }
        }
    }

    return isUnlocked;
#else
    // Unlock sequence disabled: always unlocked
    return true;
#endif
}

// Handle rising-edge button events and update lastBtns/lastDpad afterwards.
void GamepadPILOT::handleButtonEvents(uint32_t currentBtns, uint8_t currentDpad) {
    // Detect "Rising Edge" of the gear cycle button (pressed now, but wasn't before)
    if (risingEdge(currentBtns, lastBtns, CYCLE_GEAR_BUTTON)) {
        currentGearIndex = (currentGearIndex + 1) % GEAR_COUNT;
        VLF(("MSG: Gear Changed! Current Multiplier: " + String(SPEED_MULTIPLIERS[currentGearIndex]) + "x").c_str());

        // Rumble when the slowest gear (index 0) is selected
        if (currentGearIndex == 0) {
            ControllerPtr myCtl = myControllers[0];
            if (myCtl && myCtl->isConnected()) {
                // Parameters: (delayedStartMs, durationMs, weakMagnitude, strongMagnitude)
                myCtl->playDualRumble(0, 150, 0x80, 0x40);
            }
        }
    }

    // Detect "Rising Edge" of the swap toggle button
    if (risingEdge(currentBtns, lastBtns, SWAP_TOGGLE_BUTTON)) {
#if TOGGLE_SWAP_NS == 1
        activeSwapNS = !activeSwapNS;
#endif

#if TOGGLE_SWAP_EW == 1
        activeSwapEW = !activeSwapEW;
#endif
        String msg = "MSG: Orientation Swapped! ";
        msg += "N/S: " + String(activeSwapNS ? "Inv" : "Norm");
        msg += " E/W: " + String(activeSwapEW ? "Inv" : "Norm");
        VLF(msg.c_str());
    }

    // Detect "Rising Edge" of the tracking toggle button
    if (risingEdge(currentBtns, lastBtns, TRACKING_TOGGLE_BUTTON)) {
        if (mount.isTracking()) {
            commandBroker.send(":Td#"); // turn tracking off
            VLF("MSG: tracking OFF");
        } else {
            commandBroker.send(":Te#"); // turn tracking on
            VLF("MSG: tracking ON");
        }
    }

    // Satellite tracking button state machine
#if ENABLE_SAT_TRACKING_FEATURE == 1
    if (currentBtns & SATELLITE_TRACKING_BUTTON) {
        // Button is currently held down.
        if (!(lastBtns & SATELLITE_TRACKING_BUTTON)) {
            // Rising edge: button was just pressed.
            if (satelliteTracker.isActive()) {
                // If tracking is already active, pressing the button disables it immediately.
                satelliteTracker.disable();
                VLF("MSG: Satellite tracking OFF");
            } else {
                // Not active: start recording while the button is held.
                satelliteTracker.startRecording();
                satelliteTrackingPressStart = millis();
                VLF("MSG: Satellite tracking recording...");
            }
        }
    } else if (lastBtns & SATELLITE_TRACKING_BUTTON) {
        // Falling edge: button was just released.
        if (!satelliteTracker.isActive() && satelliteTrackingPressStart != 0) {
            uint32_t holdTime = millis() - satelliteTrackingPressStart;

            satelliteTracker.stopRecording();

            if (holdTime >= SATELLITE_TRACKING_BUTTON_HOLD_TIME) {
                satelliteTracker.enable(mountType);
                VLF("MSG: Satellite tracking ON");
            } else {
                VLF("MSG: Satellite tracking hold (recording) too short");
            }

            satelliteTrackingPressStart = 0;
        }
    }
#endif

    // Custom LX200 commands, sent once per rising edge.
#define CUSTOM_COMMAND(buttonTrigger, dpadTrigger, command) \
        if (risingEdge(currentBtns, lastBtns, (buttonTrigger)) || \
            risingEdge(currentDpad, lastDpad, (dpadTrigger))) { \
            SERIAL_LOCAL.transmit(command); \
            VL("MSG: Custom command sent: "); \
            VLF(command); \
        }

#if ENABLE_SAT_TRACKING_FEATURE == 1
    if (!satelliteTracker.isActive()) {
#endif
        CUSTOM_COMMANDS
#if ENABLE_SAT_TRACKING_FEATURE == 1
    }
#endif
#undef CUSTOM_COMMAND

    lastBtns = currentBtns;
    lastDpad = currentDpad;
}

// ============================================================================
// Debug helpers
// ============================================================================

// Verbose system stats: print heap diagnostics and task stack high-water mark every 3s.
//   free heap      : total free internal heap right now
//   min free heap  : lowest free heap since boot (the real worst case, catches short dips
//                    between prints, e.g. WiFi/website load)
//   largest block  : biggest single block malloc can return right now. Much smaller than
//                    free heap = fragmented: WiFi/web allocations can fail although
//                    "free heap" looks fine.
void GamepadPILOT::handleVerboseSystemStats() {
    static uint32_t lastStatsTime = 0;
    if (millis() - lastStatsTime > 3000) {                       // print stats every 3 sec
        // Internal RAM heap (same pool as ESP.getFreeHeap(), excludes PSRAM;
        // WiFi/lwIP/Bluetooth buffers must come from internal RAM)
        uint32_t freeHeap     = ESP.getFreeHeap();
        uint32_t minFreeHeap  = ESP.getMinFreeHeap();
        uint32_t largestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

        // Minimum free stack space of THIS plugin task since it started (in bytes)
        uint32_t unusedStack = uxTaskGetStackHighWaterMark(NULL);

        // Direct serial output (no macro dependency)
        Serial.println("--- plugin memory stats ---");
        Serial.printf("MSG: free heap: %u bytes | min free since boot: %u bytes | largest free block: %u bytes\n",
                      static_cast<unsigned>(freeHeap), static_cast<unsigned>(minFreeHeap), static_cast<unsigned>(largestBlock));
        Serial.printf("MSG: Unused Plugin Stack: %u / 4096 bytes\n", static_cast<unsigned>(unusedStack));

        TaskHandle_t web = xTaskGetHandle("WebSvrTask");   // NULL if the website plugin is off
        if (web) Serial.printf("MSG: Unused WebSvrTask stack: %u / 10000 bytes\n",
                               static_cast<unsigned>(uxTaskGetStackHighWaterMark(web)));

        lastStatsTime = millis();
    }
}

// Raw debug mode: read raw Bluepad32 values and print them at the configured interval.
void GamepadPILOT::handleRawDebugMode(ControllerPtr myCtl) {
    static uint32_t lastLogTime = 0;

    int32_t rawX = myCtl->axisX();     // left stick X
    int32_t rawY = myCtl->axisY();     // left stick Y
    int32_t rawRX = myCtl->axisRX();   // right stick X
    int32_t rawRY = myCtl->axisRY();   // right stick Y
    uint32_t rawBtns = myCtl->buttons(); // button state
    uint8_t rawDpad = myCtl->dpad();   // D pad

    if (millis() - lastLogTime > RAW_DEBUG_PRINT_INTERVAL) {
        bool hasData = myCtl->hasData();
        Serial.printf("MSG: RAW - Data: %s | LX: %4d | LY: %4d | RX: %4d | RY: %4d | BTN: %u | DPD: %u\n",
        hasData ? "YES" : "NO ",
            static_cast<int>(rawX),
            static_cast<int>(rawY),
            static_cast<int>(rawRX),
            static_cast<int>(rawRY),
            static_cast<unsigned>(rawBtns),
            static_cast<unsigned>(rawDpad));

        lastLogTime = millis();
    }
}

// Serial Plotter output: one line containing target, smooth, and acceleration state.
void GamepadPILOT::handlePlotterDebug() {
#if PLOTTER_DEBUG_PRINT_INTERVAL > 0
    static uint32_t lastPrintMs = 0;
    uint32_t now = millis();
    if (now - lastPrintMs < PLOTTER_DEBUG_PRINT_INTERVAL) return;
    lastPrintMs = now;
#endif
    Serial.printf("TargetX:%.3f,SmoothX:%.3f,TargetY:%.3f,SmoothY:%.3f,AccelHigh:%d\n",
                targetSpeedX, smoothTargetSpeedX, targetSpeedY, smoothTargetSpeedY, accelIsHigh);
}

// Axis state debug: print axis frequencies, slewing state, and limit status.
void GamepadPILOT::handleAxisStateDebug() {
    static uint32_t lastStatusPrint = 0;
    if (millis() - lastStatusPrint > 500) {
        Serial.printf("DEBUG: Ax1 Freq: %.4f | Slew: %s, Ax2 Freq: %.4f | Slew: %s | Limits: %s (%s)\n",
            axis1.getFrequency(), axis1.isSlewing() ? "YES" : "NO",
            axis2.getFrequency(), axis2.isSlewing() ? "YES" : "NO",
            limits.isError() ? "OUTSIDE" : "INSIDE",
            limits.isEnabled() ? "Enforced" : "Disabled");

        // print the individual limit flags
        Serial.printf("DEBUG: Limit flags | BelowHorizon:%d | AboveOverhead:%d | MeridianW:%d | MeridianE:%d | Axis1Min:%d | Axis1Max:%d\n",
            limits.isBelowHorizon() ? 1 : 0,
            limits.isAboveOverhead() ? 1 : 0,
            limits.isPastMeridianW() ? 1 : 0,
            limits.isPastMeridianE() ? 1 : 0,
            limits.isPastAxis1Min() ? 1 : 0,
            limits.isPastAxis1Max() ? 1 : 0);

        lastStatusPrint = millis();
    }
}

// Tick timing debug: print the duration of the last completed tick
void GamepadPILOT::handleTimingDebug(uint32_t startUs, uint32_t endUs) {
    uint32_t duration = endUs - startUs;               // Safe even if micros() wraps
    static uint32_t lastPrintMs = 0;                   // Static local avoids a class member
    if (millis() - lastPrintMs >= TIMING_DEBUG_PRINT_INTERVAL) {
        Serial.printf("Tick_Duration_us:%u\n", duration);
        lastPrintMs = millis();
    }
}

// ============================================================================
// Initialization
// ============================================================================

// Initialise Bluepad32, fetch config values, set initial acceleration and
// start the FreeRTOS task pinned to Core 0.
void GamepadPILOT::init() {
    BP32.setup(&GamepadPILOT::onConnected, &GamepadPILOT::onDisconnected);

    VLF(("MSG: Plugins, starting: " + String(GP_NAME_STR) + " v" + GP_VERSION_STR).c_str());
    VLF("MSG: Bluetooth searching for gamepads...");

#if RAW_DEBUG_MODE != 0
    VLF("MSG: Plugin: raw debug mode enabled");
#endif

#if USE_DATETIME_OVERRIDE != 0
    commandBroker.send(":SC01/01/1970#");
    commandBroker.send(":SL00:00:01#");
    VLF("MSG: Date set to 01/01/1970 to unlock mount movement");
#endif

    // Fetch base slew speed from configuration and convert to rad/s.
    activeMaxSlewSpeedRad = static_cast<float>(SLEW_RATE_BASE_DESIRED) * DEG_TO_RAD_F;

    // Calculate original acceleration: a = v^2 / (2 * d)
    originalAccel = accelerationFromMaxSpeed(activeMaxSlewSpeedRad);

    // Set initial acceleration on both axes.
    axis1.setSlewAccelerationRate(originalAccel);
    axis2.setSlewAccelerationRate(originalAccel);
    accelIsHigh = false;

    // Read swap and unlock settings.
    activeSwapNS = (SWAP_N_S != 0);
    activeSwapEW = (SWAP_E_W != 0);
    isUnlocked = (USE_UNLOCK_SEQUENCE == 0);

    // Store mount type for later use
    mountType = mount.settings.mountType;

#if MAC_WHITELIST_ENABLE == 1
    VLF(("MSG: Whitelist enabled. Only allowing MAC: " + String(MAC_WHITELIST_ADDRESS)).c_str());
#endif

#if ENABLE_SAT_TRACKING_FEATURE == 1
    // Satellite tracking sample buffers: one heap block, allocated once (not in static DRAM)
    if (satelliteTracker.allocateBuffers()) {
        VF("MSG: GamepadPILOT satellite tracking buffers allocated, bytes: ");
        VL(static_cast<unsigned>(satelliteTracker.bufferBytes()));
    } else {
        DF("WRN: GamepadPILOT satellite tracking disabled, could not allocate bytes: ");
        DL(static_cast<unsigned>(satelliteTracker.bufferBytes()));
    }
#endif

    // Create the Bluepad32 task on Core 0.
    xTaskCreatePinnedToCore(
        bluepadTask,
        "GamepadPILOT",
        4096,
        this,
        1,
        NULL,
        0
    );

    VLF("MSG: Plugin: GamepadPILOT pinned to core 0 using FreeRTOS scheduler");
}
