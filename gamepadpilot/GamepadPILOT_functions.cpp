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

extern CommandBroker commandBroker;

// ============================================================================
// File-local compile-time constants and math helpers
// ============================================================================
namespace {
// used in normalizestick()
    constexpr float RANGE_RECIPROCAL = 1.0f / (float)(STICK_MAX - DEADZONE);
}

// Smoothly approach target speed using a fixed rate over delta time.
// Snaps to target when close enough to avoid micro-oscillation.
static inline float applyAcceleration(float target, float current, float rate, float dt) {
    float step = rate * dt;
    if (abs(current - target) < step) return target;
    return (current < target) ? (current + step) : (current - step);
}

// True on the rising edge of any bit in "bits" (0 disables the trigger).
static inline bool risingEdge(uint32_t current, uint32_t previous, uint32_t bits) {
    return bits != 0 && (current & bits) != 0 && (previous & bits) == 0;
}

// Convert raw Bluepad32 stick values (-512..511) to normalized range -1.0..1.0.
// Applies deadzone and manual clamping.
static inline float normalizeStick(int val, int deadzone, float reciprocal) {
    if (abs(val) <= deadzone) return 0.0f;

    //this can be micro optimized to use esp32 internal fpu, maybe later
    int sign = (val > 0) ? 1 : -1;
    float result = (float)(abs(val) - deadzone) * reciprocal * sign;

    if (result > 1.0f) return 1.0f;
    if (result < -1.0f) return -1.0f;
    return result;
}

// ============================================================================
// FreeRTOS task function (file-local)
// ============================================================================

// This is the main task that runs on Core 0.
// It casts the void pointer back to GamepadPILOT and repeatedly calls run().
static void bluepadTask(void* pvParameters) {
    GamepadPILOT* myPlugin = (GamepadPILOT*)pvParameters;

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

void GamepadPILOT::resetControllerState() {
  //  lastxVal = 0;
  //  lastyVal = 0;
  //  lastrxVal = 0;
  //  lastryVal = 0;
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
    String modelName = ctl->getModelName().c_str();

    // Limit log spam from a rejected controller that keeps retrying:
    // the same MAC is logged at most every 5 s.
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
        // todo: test ctl->disconnect()  for error "Failed to disconnect HIDS client", works fine with Bluepad32 4.1.0 and Xbox One
        ctl->disconnect();
        return;
    }

    // Accept: store the controller in slot 0.
    gamepadpilot.myControllers[0] = ctl;
    gamepadpilot.lastDataMillis = millis();  // initialise watchdog timer
    gamepadpilot.resetControllerState();

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
        gamepadpilot.resetControllerState();
        VLF("MSG: Gamepad disconnected");
    }
}

// ============================================================================
// Max slew speed management (async broker request)
// ============================================================================

// Request the current max slew speed from the mount asynchronously.
// The broker handle is stored and later processed in checkBrokerResult().
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
           float maxSpeedDeg = atof(reply); // reply format is "n.n#"
           if (maxSpeedDeg > 0.1f && maxSpeedDeg < 360.0f) {   // sanity bounds
               activeMaxSlewSpeedRad = maxSpeedDeg * 0.01745329f; // Degrees/s to Radians/s
               
               // Recalculate original acceleration to maintain consistency with the new max speed
               float d_rad = (float)SLEW_ACCELERATION_DIST * 0.01745329f;
               originalAccel = (d_rad > 0) ? (activeMaxSlewSpeedRad * activeMaxSlewSpeedRad) / (2.0f * d_rad) : 10.0f;
               if (originalAccel > 10.0f) {
                   originalAccel = 0.1f;
                   VLF("MSG: Safety feature: Set acceleration to safe 0.1f rad/s^2 value as calculated originalaccel was higher than 10.0f");
                }   
               VLF(("MSG: GamepadPILOT updated MaxSlew from :GX97#: " + String(maxSpeedDeg) + " deg/s (Accel: " + String(originalAccel) + ")").c_str());
           }
           brokerHandle = 0; // Reset handle after success to allow the next stick movement to trigger a fresh poll
       } else if (status == CB_TIMEOUT || status == CB_FREE) {
        brokerHandle = 0; // Reset handle on failure so can try again on next movement
    }
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
        return value * abs(value);
    #elif STICK_CURVE_EXPONENT == 3
        return value * value * value;
    #else
        return value;
    #endif
}

// Convert normalized stick values to target speeds using gear, swap, and max slew.
void GamepadPILOT::updateTargetSpeeds(float dt) {
    float xScaled = applyStickCurve(MAIN_STICK_X);
    float yScaled = applyStickCurve(MAIN_STICK_Y);

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
        satelliteTracker.update(dt, targetSpeedXNorm, targetSpeedYNorm,
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
    targetSpeedX = targetSpeedXNorm * activeMaxSlewSpeedRad; //(rad)
    targetSpeedY = targetSpeedYNorm * activeMaxSlewSpeedRad;
    }

    // detect limits and stop if outside limits
    applyLimits(targetSpeedX, targetSpeedY);
}

// Check for limit error. When error occurs ony allow movement for a short amount of time, then sticks need to be centered again.
void GamepadPILOT::applyLimits(float &targetSpeedX, float &targetSpeedY) {
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

    ControllerPtr myCtl = myControllers[0];


    bool sticksCentered = IS_MAIN_STICK_CENTERED;


    // State Machine for Limit Recovery
    if (recoveryState == RecoveryState::IDLE) {
        recoveryState = RecoveryState::WAITING_FOR_CENTER;
        myCtl->playDualRumble(0, 100, 0x80, 0x40);
    }

    if (recoveryState == RecoveryState::WAITING_FOR_CENTER && sticksCentered) {
        recoveryState = RecoveryState::ARMED;
    }

    if (recoveryState == RecoveryState::ARMED && !sticksCentered) {
        recoveryState = RecoveryState::MOVING;
        limitRecoveryWindowStart = millis();
    }

    // re-arm when sticks are centered while in MOVING state
    if (recoveryState == RecoveryState::MOVING && sticksCentered) {
        recoveryState = RecoveryState::ARMED;
    }

    // Calculate if the recovery window is still active
    bool windowOpen = (recoveryState == RecoveryState::MOVING) && 
                      (millis() - limitRecoveryWindowStart < LIMIT_RECOVERY_WINDOW_MS);

    // ENFORCE SAFETY RULES:
    // Force a full stop if:
    // 1. The sticks are dead-centered (blocks other background processes from moving it)
    // 2. OR not armed yet, and the moving window has already expired.
    if (sticksCentered || (recoveryState != RecoveryState::ARMED && !windowOpen)) {
        targetSpeedX = 0.0f;
        targetSpeedY = 0.0f;
        //smoothTargetSpeedX = 0.0f;
        //smoothTargetSpeedY = 0.0f;
    }
}





// Smoothly accelerate towards the current target speeds using delta time.
void GamepadPILOT::calculateSmoothSpeeds(float dt) {
    smoothTargetSpeedX = applyAcceleration(targetSpeedX, smoothTargetSpeedX, ACCEL_RATE, dt);
    smoothTargetSpeedY = applyAcceleration(targetSpeedY, smoothTargetSpeedY, ACCEL_RATE, dt);
}


// ============================================================================
// Slew command, acceleration, and cleanup helpers
// ============================================================================

// Temporarily raise axis acceleration while slewing to be able to set custom acceleration and have decelleration (Onstepx does not do decelleration with autoslew).
void GamepadPILOT::maybeEnableHighAcceleration() {
    if (!accelIsHigh && (smoothTargetSpeedX != 0.0f || smoothTargetSpeedY != 0.0f)) {
        axis1.setSlewAccelerationRate(10.0f);
        axis2.setSlewAccelerationRate(10.0f);
        accelIsHigh = true;
    }
}

// Store the last sent speeds and start cleanup timer when movement stops.
void GamepadPILOT::updateLastSentSpeeds() {
    lastSentSpeedX = smoothTargetSpeedX;
    lastSentSpeedY = smoothTargetSpeedY;

    // If movement has just come to a halt, start the cleanup timer
    if (lastSentSpeedX == 0.0f && lastSentSpeedY == 0.0f) {
        if (!cleanupRequired) {
            zeroInputTime = millis();
            cleanupRequired = true;
        }
    }
}

// Run cleanup routine 1 second after movement has fully stopped.
void GamepadPILOT::checkCleanupAndPerform() {     //todo: make this better //todo: this does not work cleanly on some axis limits (disabled then), 
                                                    //the real fix is better slewflag cleanup //todo: the REAL fix is making autoslew.stop threadsafe TODO

        if (cleanupRequired && limits.isError()) {
            axis1.setSlewAccelerationRate(originalAccel);
            axis2.setSlewAccelerationRate(originalAccel);
            accelIsHigh = false;
            cleanupRequired = false;
            VLF("MSG: Slew flag cleanup skipped due to limit error, acceleration restored.");
            cleanupRequired = false;
            return;
        }

    if (cleanupRequired && (millis() - zeroInputTime > 1000)) {
        if (axis1.isSlewing() || axis2.isSlewing()) {
            // Transmit LX200 commands to force state machine cleanup
           // SERIAL_LOCAL.transmit(":RC#"); // Set rate to Center
            SERIAL_LOCAL.transmit(":Me#"); // move east
            SERIAL_LOCAL.transmit(":Mn#"); // move north
            SERIAL_LOCAL.transmit(":Q#");  // Final stop (clears isSlewing flag)
            VLF("MSG: Trying to execute Slew flag cleanup routine");
        } else {
            // Once isSlewing is false, restore original mount acceleration physics
            axis1.setSlewAccelerationRate(originalAccel);
            axis2.setSlewAccelerationRate(originalAccel);
            accelIsHigh = false;
            cleanupRequired = false;
            VLF("MSG: Slew flag cleanup routine executed");
        }
    }
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

            // Directional Logic and OnStepX Commands
            Direction dirX, dirY;

            // -----------------------------------------------------------------
            // Axis 2 speed inversion depending on mount type and pier side
            //   smoothTargetSpeedY < 0  ->  North / up
            //   smoothTargetSpeedY > 0  ->  South / down
            // -----------------------------------------------------------------
            float pierSideCorrectedY = smoothTargetSpeedY;

            // for GEM on pier side west, invert Y 
            if (mountType == GEM && mount.getPosition().pierSide == PIER_SIDE_WEST) {
                // PIER_SIDE_WEST
                pierSideCorrectedY = -pierSideCorrectedY;
            }

            // 1. Logic for North/South (Axis 2)
            if (pierSideCorrectedY == 0.0f) {
                axis2.autoSlew(DIR_FORWARD, 0.0f);
                //axis2.autoSlewStop(); //this would be ideal, but is not threadsafe
            } else {
                // Swap is handled in the TargetSpeed
                dirY = (pierSideCorrectedY > 0.0f) ? DIR_REVERSE : DIR_FORWARD;
                axis2.autoSlew(dirY, abs(pierSideCorrectedY));
            }

            // 2. Logic for East/West (Axis 1)
            if (smoothTargetSpeedX == 0.0f) {
                axis1.autoSlew(DIR_FORWARD, 0.0f);
                //axis1.autoSlewStop(); //this would be ideal, but is not threadsafe
            } else {
                dirX = (smoothTargetSpeedX > 0.0f) ? DIR_FORWARD : DIR_REVERSE;
                axis1.autoSlew(dirX, abs(smoothTargetSpeedX));
            }
            
            updateLastSentSpeeds();
        }

        // Reset cleanup trigger if movement starts again
        if (smoothTargetSpeedX != 0.0f || smoothTargetSpeedY != 0.0f) {
            cleanupRequired = false;
        }

        checkCleanupAndPerform();
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
        currentUnlockStep = STEP_LOCKED;
        holdStartTime = 0;
    
        #if ENABLE_SAT_TRACKING_FEATURE == 1
        // Immediately stop satellite tracking if the controller disconnects
        satelliteTracker.disable();
        #endif
    }

    // Only run the notification and immediate-stop logic for a few cycles to avoid flooding
    if (stopRetries < 3) {
        if (connectionLost) {
            VLF("MSG: Connection Lost! Sending Safety Stop...");
        } else {
            VLF("MSG: Watchdog Triggered (Stale Data)! Sending Safety Stop...");
        }

        // Set lastSentSpeed to something non-zero to force the command timer to send the stop commands // not elegant but it works
        lastSentSpeedX = 1.0f;  //test if this is required
        lastSentSpeedY = 1.0f;
        stopRetries++;
    }
}

// ============================================================================
// Input processing helpers (unlock sequence and button events)
// ============================================================================

// Handles the unlock sequence and returns whether the mount is currently unlocked.
// When USE_UNLOCK_SEQUENCE == 0, this function always returns true.
bool GamepadPILOT::handleUnlockSequence(uint8_t dpad) {
#if USE_UNLOCK_SEQUENCE == 1

    if (!isUnlocked) {
        ControllerPtr myCtl = myControllers[0];

        // dpad values: up:1 ; down:2 ; right:4 ; left:8
        if (dpad == 2) { // Holding DOWN
            if (currentUnlockStep == STEP_LOCKED) {
                if (holdStartTime == 0) holdStartTime = millis();
                if (millis() - holdStartTime >= UNLOCK_HOLD_MS) {
                    currentUnlockStep = STEP_WAIT_FOR_UP;
                    holdStartTime = 0;
                    waitForUpStartTime = 0; // Window starts only after DOWN is released
                    myCtl->playDualRumble(0, 100, 0x80, 0x40);
                    VLF("MSG: Unlock Stage 1 complete. Now hold UP.");
                }
            }
        } else if (dpad == 1) { // Holding UP
            if (currentUnlockStep == STEP_WAIT_FOR_UP) {
                if (holdStartTime == 0) {
                    // First UP press after DOWN release: check the 2s window.
                    if (waitForUpStartTime == 0 ||
                        (millis() - waitForUpStartTime) > 2000) {
                        currentUnlockStep = STEP_LOCKED;
                        waitForUpStartTime = 0;
                        VLF("MSG: Unlock sequence reset: UP started too late.");
                    } else {
                        holdStartTime = millis();
                    }
                }

                if (currentUnlockStep == STEP_WAIT_FOR_UP && holdStartTime != 0) {
                    if (millis() - holdStartTime >= UNLOCK_HOLD_MS) {
                        isUnlocked = true;
                        waitForUpStartTime = 0;
                        VLF("MSG: GamepadPILOT UNLOCKED.");
                        myCtl->playDualRumble(0, 150, 0x80, 0x40);
                    }
                }
            }
        } else if (dpad != 0) { // Pressed something else (Left/Right)
            currentUnlockStep = STEP_LOCKED;
            holdStartTime = 0;
            waitForUpStartTime = 0;
        } else { // D-pad released
            holdStartTime = 0;

            if (currentUnlockStep == STEP_WAIT_FOR_UP) {
                if (waitForUpStartTime == 0) {
                    // DOWN was just released; start the 2 second window.
                    waitForUpStartTime = millis();
                } else if ((millis() - waitForUpStartTime) > 2000) {
                    currentUnlockStep = STEP_LOCKED;
                    waitForUpStartTime = 0;
                    VLF("MSG: Unlock sequence reset: UP not started within 3s.");
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
void GamepadPILOT::handleButtonEvents(uint32_t btns, uint8_t dpad) {
    // Detect "Rising Edge" of the gear cycle button (pressed now, but wasn't before)
    if ((btns & CYCLE_GEAR_BUTTON) && !(lastBtns & CYCLE_GEAR_BUTTON)) {
        // Calculate number of gears available
        size_t numGears = sizeof(SPEED_MULTIPLIERS) / sizeof(SPEED_MULTIPLIERS[0]);
        
        // Cycle index and wrap back to 0
        currentGearIndex = (currentGearIndex + 1) % numGears;
        
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
    if ((btns & SWAP_TOGGLE_BUTTON) && !(lastBtns & SWAP_TOGGLE_BUTTON)) {
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
    if ((btns & TRACKING_TOGGLE_BUTTON) && !(lastBtns & TRACKING_TOGGLE_BUTTON)) {
        if (mount.isTracking()) {
            SERIAL_LOCAL.transmit(":Td#"); // turn tracking off
            VLF("MSG: tracking OFF");
        } else {
            SERIAL_LOCAL.transmit(":Te#"); // turn tracking on
            VLF("MSG: tracking ON");
        }
    }

    // Satellite tracking button state machine
    #if ENABLE_SAT_TRACKING_FEATURE == 1
    if (btns & SATELLITE_TRACKING_BUTTON) {
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
        if (risingEdge(btns, lastBtns, (buttonTrigger)) || \
            risingEdge(dpad, lastDpad, (dpadTrigger))) { \
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

    lastBtns = btns;
    lastDpad = dpad;
}

// ============================================================================
// Debug helpers
// ============================================================================

// Verbose system stats: print global heap and task stack high-water mark every 10s.
void GamepadPILOT::handleVerboseSystemStats() {
    static uint32_t lastStatsTime = 0;
    if (millis() - lastStatsTime > 3000) {                       // print stats every 3 sec
        // Check global RAM
        uint32_t freeHeap = ESP.getFreeHeap();
        
        // Check specific Stack for THIS plugin task (the 8192 bytes)
        // Returns the minimum free stack space seen since the task started (in bytes)
        uint32_t unusedStack = uxTaskGetStackHighWaterMark(NULL); 

        // Direct serial output (no macro dependency)
        Serial.println("--- plugin memory stats ---");
        Serial.printf("MSG: free global heap: %u bytes\n", freeHeap);
        Serial.printf("MSG: Unused Plugin Stack: %u / 8192 bytes\n", unusedStack);
        
        lastStatsTime = millis();
    }
}

// Raw debug mode: read raw Bluepad32 values and print them at the configured interval.
void GamepadPILOT::handleRawDebugMode(ControllerPtr myCtl) {
    static uint32_t lastLogTime = 0; 

    int32_t xVal = myCtl->axisX();   // left stick X
    int32_t yVal = myCtl->axisY();   // left stick Y
    int32_t rxVal = myCtl->axisRX(); // right stick X
    int32_t ryVal = myCtl->axisRY(); // right stick Y
    uint32_t btns = myCtl->buttons();  // button state
    uint8_t dpad = myCtl->dpad();  // D pad

    if (millis() - lastLogTime > RAW_DEBUG_PRINT_INTERVAL) {
        bool hasData = myCtl->hasData();
        Serial.printf("MSG: RAW - Data: %s | LX: %4d | LY: %4d | RX: %4d | RY: %4d | BTN: %u | DPD: %u\n",
        hasData ? "YES" : "NO ", (int)xVal, (int)yVal, (int)rxVal, (int)ryVal, (unsigned int)btns, (unsigned int)dpad);

        lastLogTime = millis();
    }
}

// Serial Plotter output: one line containing target, smooth, and acceleration state.
void GamepadPILOT::handlePlotterDebug() {
    Serial.printf("TargetX:%.3f,SmoothX:%.3f,TargetY:%.3f,SmoothY:%.3f,AccelHigh:%d\n", targetSpeedX, smoothTargetSpeedX, targetSpeedY, smoothTargetSpeedY, accelIsHigh);
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
    activeMaxSlewSpeedRad = (float)SLEW_RATE_BASE_DESIRED * 0.01745329f;

    // Calculate original acceleration: a = v^2 / (2 * d)
    float v_rad = activeMaxSlewSpeedRad;
    float d_rad = (float)SLEW_ACCELERATION_DIST * 0.01745329f;
    originalAccel = (d_rad > 0) ? (v_rad * v_rad) / (2.0f * d_rad) : 10.0f;

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

    // Create the Bluepad32 task on Core 0.
    xTaskCreatePinnedToCore(
        bluepadTask,               // free function now file-local
        "GamepadPILOT",
        4096,
        this,
        1,
        NULL,
        0
    );

    VLF("MSG: Plugin: GamepadPILOT pinned to core 0 using FreeRTOS scheduler");
}
