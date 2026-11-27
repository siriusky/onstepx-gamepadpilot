#pragma once

// ============================================================================
// Includes
// ============================================================================
#include <Bluepad32.h>
#include <Arduino.h>
#include "../../lib/commands/CommandErrors.h"
#include "../../libApp/commands/CommandBroker.h"
#include "GamepadPILOT_config.h"

#if ENABLE_SAT_TRACKING_FEATURE == 1
#include "SatelliteTrackingEngine.h"
#endif

// ============================================================================
// Plugin versions
// ============================================================================
#define GP_VERSION_MAJOR           0
#define GP_VERSION_MINOR           1
#define GP_VERSION_PATCH           2
#define GP_VERSION_STR             "0.1.2"
#define GP_NAME_STR                "GamepadPILOT"

// Check OnStepX version and throw error if not supported. 
// GamepadPILOT needs Bug fix from 10.29b, otherwise fails without error.
#if defined(FirmwareVersionMajor) && defined(FirmwareVersionMinor) && defined(FirmwareVersionPatch)
  constexpr bool fwAtLeast10_29b =
      (FirmwareVersionMajor > 10) ||
      (FirmwareVersionMajor == 10 && FirmwareVersionMinor > 29) ||
      (FirmwareVersionMajor == 10 && FirmwareVersionMinor == 29 &&
       FirmwareVersionPatch[0] >= 'b');

  static_assert(fwAtLeast10_29b, "GamepadPILOT requires OnStepX 10.29b or newer");
#endif

// ============================================================================
// GamepadPILOT class - Bluepad32 joystick interface for OnStepX
// ============================================================================
class GamepadPILOT {
public:
    // --- Public API ---------------------------------------------------------
    void init();                      // Initialize Bluepad32 and start task
    void run();                       // Main task loop (called from FreeRTOS task)

    // External command handler, OnStepX expects this from plugins (unused; always returns false)
    bool command(char *reply, char *command, char *parameter, bool *suppressFrame, bool *numericReply, CommandError *commandError) {
        return false;                 // No serial commands processed by this plugin
    }

    // --- Static Bluepad32 callbacks ----------------------------------------
    static void onConnected(ControllerPtr ctl);    // Handle controller connect
    static void onDisconnected(ControllerPtr ctl); // Handle controller disconnect

    // --- Active controllers (slot 0 is the only supported one) --------------
    ControllerPtr myControllers[BP32_MAX_GAMEPADS];

private:
    // --- Satellite tracking -------------------------------------------------
    #if ENABLE_SAT_TRACKING_FEATURE == 1
    SatelliteTrackingEngine satelliteTracker;
    uint32_t satelliteTrackingPressStart = 0;
    #endif

    // --- Mount type --------------------------------------------------------
    uint8_t mountType = 0;            // OnStepX mount type (1..4) (GEM, FORK, ALTAZM, ALTALT)

    // --- Raw input state ----------------------------------------------------
    uint32_t lastDataMillis = 0;      // Timestamp of last fresh data for watchdog
    uint8_t pollCounter = 0;          // Send tick counter (sendSlewCommands, satellite control step)
    int32_t xVal = 0;                 // Current left stick X raw value
    int32_t yVal = 0;                 // Current left stick Y raw value
    int32_t rxVal = 0;                // Current right stick X raw value
    int32_t ryVal = 0;                // Current right stick Y raw value
    uint32_t btns = 0;                // Current button bitmap
    uint8_t dpad = 0;                 // Current D-pad state

    // --- Loop timing and watchdog state -------------------------------------
    float dt = (POLL_INTERVAL * 0.001f); // Constant delta time in seconds
    bool watchdogActive = false;      // Sticky flag when watchdog triggered
    bool wasUserSlewInputActive = false; // Previous stick or dpad slewing activity (for rising edge)
    uint8_t brokerHandle = 0;         // Handle for async slew speed request
    uint32_t lastBtns = 0;            // Previous button bitmap
    uint8_t lastDpad = 0;             // Previous D-pad state

    // --- Compile-time selection of the main stick --------------------------
    static constexpr bool USE_RIGHT_STICK_SELECTED = USE_RIGHT_STICK != 0;
    float mainStickX() const { return USE_RIGHT_STICK_SELECTED ? stickPercentrX : stickPercentX; }
    float mainStickY() const { return USE_RIGHT_STICK_SELECTED ? stickPercentrY : stickPercentY; }
    bool  mainStickCentered() const { return mainStickX() == 0.0f && mainStickY() == 0.0f; }

    // True while trying to slew (main stick deflected, or D-pad pressed if enabled)
    bool slewInputActive() const {
        return !mainStickCentered()
#if USE_DPAD_SLEWING == 1
            || dpad != 0
#endif
        ;
    }

    // --- Normalized stick values (-1.0 to 1.0) ------------------------------
    float stickPercentX = 0;          // Left stick X normalized
    float stickPercentY = 0;          // Left stick Y normalized
    float stickPercentrX = 0;         // Right stick X normalized
    float stickPercentrY = 0;         // Right stick Y normalized

    // --- Target and smoothed speeds (rad/s) ---------------------------------
    float targetSpeedXNorm = 0;       // Desired axis1 speed normalized to -1..1
    float targetSpeedYNorm = 0;       // Desired axis2 speed normalized to -1..1
    float targetSpeedX = 0;           // Desired axis1 speed in rad/s
    float targetSpeedY = 0;           // Desired axis2 speed in rad/s
    float smoothTargetSpeedX = 0;     // Smoothed axis1 speed sent to mount
    float smoothTargetSpeedY = 0;     // Smoothed axis2 speed sent to mount
    float lastSentSpeedX = 0;         // Last commanded axis1 speed
    float lastSentSpeedY = 0;         // Last commanded axis2 speed

    // --- Mount limits -------------------------------------------------------
    enum class RecoveryState : uint8_t {
        IDLE,               // Normal operation (inside safe boundaries)
        WAITING_FOR_CENTER, // Breached a limit; waiting for user to release stick and D-pad
        ARMED,              // All slew input released; ready for the next movement nudge
        MOVING              // actively backing away; countdown timer is running
    };

    RecoveryState recoveryState = RecoveryState::IDLE;
    uint32_t limitRecoveryWindowStart = 0;

    // --- Cleanup and acceleration -------------------------------------------
    float originalAccel = 0;          // Mount's original slew acceleration
    float activeMaxSlewSpeedRad = 0;  // Current max slew speed (rad/s)
    bool accelIsHigh = false;         // Flag for temporary high acceleration
    bool activeSwapNS = false;        // Invert North/South direction
    bool activeSwapEW = false;        // Invert East/West direction
    bool isUnlocked = false;          // Unlock sequence completed

    // --- Unlock sequence state and connection safety ------------------------
    enum class UnlockStep : uint8_t { LOCKED, WAIT_FOR_UP };
    UnlockStep currentUnlockStep = UnlockStep::LOCKED;  // Current step of unlock sequence
    uint32_t holdStartTime = 0;       // Timestamp when D-pad hold started
    uint32_t waitForUpStartTime = 0;  // DOWN-release timestamp for the UP window
    bool safetyStopNoticeSent = false; // Verbose message in handleSafetyStop()

    #if PARK_PAIRING_ONLY == 1
    String sessionWhitelistedMac = "";              // MAC saved after first accepted park pairing (uppercase)
    #endif

    // --- Gear selection -----------------------------------------------------
    uint8_t currentGearIndex = DEFAULT_GEAR_INDEX;  // Current speed multiplier index

    // --- Private helper methods ---------------------------------------------
    void resetButtonEdges();                              // Set last dpad to 0
    void requestMaxSlewSpeed();                           // Send async :GX97# request
    void updateMaxSlewSpeedFromBroker();                  // Process async max speed reply
    float accelerationFromMaxSpeed(float maxSlewRad) const; // Derive accel from max speed
    void readStickValues(ControllerPtr myCtl);            // Read raw sticks/buttons/dpad
    void processStickNormalization();                     // Normalize raw sticks to -1..1
    float applyStickCurve(float value);                   // Apply response curve to stick
    void updateTargetSpeeds(float deltaTime);             // Convert sticks to target speeds
    void applyLimits(float &speedX, float &speedY);       // Clamp target speeds
    void calculateSmoothSpeeds(float deltaTime);          // Smooth speeds using acceleration
    void maybeEnableHighAcceleration();                   // Temporarily raise acceleration
    void restoreAccelerationWhenIdle();                   // Restore acceleration when idle
    void sendSlewCommands();                              // Send autoSlew commands to axes
    void handleSafetyStop(bool connectionLost);           // Execute safety stop
    bool handleUnlockSequence(uint8_t currentDpad);       // Process unlock button sequence
    void handleButtonEvents(uint32_t currentBtns, uint8_t currentDpad); // Handle button rising edges
    void handleVerboseSystemStats();                      // Print memory stats (debug)
    void handleRawDebugMode(ControllerPtr myCtl);         // Print raw controller data
    void handlePlotterDebug();                            // Print data for serial plotter
    void handleAxisStateDebug();                          // Print axis state (debug)
    void handleTimingDebug(uint32_t startUs, uint32_t endUs); // Print tick duration
    // ------------------------------------------------------------------------

};

extern GamepadPILOT gamepadpilot;
