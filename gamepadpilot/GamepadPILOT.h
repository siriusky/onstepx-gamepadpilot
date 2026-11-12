#pragma once

// ============================================================================
// Includes
// ============================================================================
#include <Bluepad32.h>
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
#define GP_VERSION_PATCH           0
#define GP_VERSION_STR             "0.1.0"
#define GP_NAME_STR                "GamepadPILOT" // plugin name

// ============================================================================
// GamepadPILOT class - Bluepad32 joystick interface for OnStepX
// ============================================================================
class GamepadPILOT {
public:
    // --- Public API ---------------------------------------------------------
    void init();                      // Initialize Bluepad32 and start task
    void run();                       // Main task loop (called from FreeRTOS task)

    // External command handler, Onstepx expects this from plugins (unused; always returns false)
    bool command(char *reply, char *command, char *parameter, bool *supressFrame, bool *numericReply, CommandError *commandError) {
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
    uint8_t mountType = 0;            // OnStepX mount type (1..4) (GEM; FORK; ALTAZM; ALTALT)


    // --- Raw input state ----------------------------------------------------
    int32_t stopRetries = 3;          // Safety stop attempt counter (prevents immediate stop)
    uint32_t lastDataMillis = 0;      // Timestamp of last fresh data for watchdog
    uint8_t pollCounter = 0;          // Send tick counter (sendSlewCommands, satellite control step)
    //int32_t lastxVal = 0;             // Previous left stick X raw value
    //int32_t lastyVal = 0;             // Previous left stick Y raw value
    //int32_t lastrxVal = 0;            // Previous right stick X raw value
    //int32_t lastryVal = 0;            // Previous right stick Y raw value

    int32_t xVal = 0;                 // Current left stick X raw value
    int32_t yVal = 0;                 // Current left stick Y raw value
    int32_t rxVal = 0;                // Current right stick X raw value
    int32_t ryVal = 0;                // Current right stick Y raw value
    uint32_t btns = 0;                // Current button bitmap
    uint8_t dpad = 0;                 // Current D-pad state

    // --- Loop timing and watchdog state -------------------------------------
    float dt = (POLL_INTERVAL * 0.001f); //constant delta time in seconds
    bool watchdogActive = false;      // Sticky flag when watchdog triggered
    bool wasUserSlewInputActive = false; // Previous stick or dpad slewing activity (for rising edge)
    uint8_t brokerHandle = 0;         // Handle for async slew speed request
    uint32_t lastBtns = 0;            // Previous button bitmap
    uint8_t lastDpad = 0;             // Previous D-pad state

    // Compile-time selection of the main stick
    #if USE_RIGHT_STICK
        #define MAIN_STICK_X stickPercentrX
        #define MAIN_STICK_Y stickPercentrY
        #define IS_MAIN_STICK_CENTERED (stickPercentrX == 0.0f && stickPercentrY == 0.0f)
    #else
        #define MAIN_STICK_X stickPercentX
        #define MAIN_STICK_Y stickPercentY
        #define IS_MAIN_STICK_CENTERED (stickPercentX == 0.0f && stickPercentY == 0.0f)
    #endif

    // --- Normalized stick values (-1.0 to 1.0) ------------------------------
    float stickPercentX = 0;          // Left stick X normalized
    float stickPercentY = 0;          // Left stick Y normalized
    float stickPercentrX = 0;         // Right stick X normalized
    float stickPercentrY = 0;         // Right stick Y normalized

    // --- Target and smoothed speeds (rad/s) ---------------------------------
    float targetSpeedXNorm = 0;       // Desired axis1 speed normalized to -1..1
    float targetSpeedYNorm = 0;       // Desired axis2 speed normalized to -1..1
    float targetSpeedX = 0;           // Desired axis1 speed in rad
    float targetSpeedY = 0;           // Desired axis2 speed in rad
    float smoothTargetSpeedX = 0;     // Smoothed axis1 speed sent to mount
    float smoothTargetSpeedY = 0;     // Smoothed axis2 speed sent to mount
    float lastSentSpeedX = 0;         // Last commanded axis1 speed
    float lastSentSpeedY = 0;         // Last commanded axis2 speed

    // --- Mount limits -------------------------------------------------------
    // State machine tracking for safety limit recovery
    enum class RecoveryState {
        IDLE,               // Normal operation (inside safe boundaries)
        WAITING_FOR_CENTER, // Breached a limit; waiting for user to let go of sticks
        ARMED,              // Sticks were centered; ready for the next movement nudge
        MOVING              // actively backing away; countdown timer is running
    };

    RecoveryState recoveryState = RecoveryState::IDLE;
    unsigned long limitRecoveryWindowStart = 0;

    // --- Cleanup and acceleration -------------------------------------------
    uint32_t zeroInputTime = 0;       // Timestamp when movement stopped
    bool cleanupRequired = false;     // Flag to run slew cleanup after stop
    float originalAccel = 0;          // Mount's original slew acceleration
    float activeMaxSlewSpeedRad = 0;  // Current max slew speed (rad/s)
    bool accelIsHigh = false;         // Flag for temporary high acceleration
    bool activeSwapNS = false;        // Invert North/South direction
    bool activeSwapEW = false;        // Invert East/West direction
    bool isUnlocked = false;          // Unlock sequence completed

    // --- Unlock sequence state and connection safety ---------------------------
    enum UnlockStep { STEP_LOCKED, STEP_WAIT_FOR_UP };
    UnlockStep currentUnlockStep = STEP_LOCKED;  // Current step of unlock sequence
    uint32_t holdStartTime = 0;       // Timestamp when D-pad hold started
    uint32_t waitForUpStartTime = 0;  // DOWN-release timestamp for the UP window

    #if PARK_PAIRING_ONLY == 1
    String sessionWhitelistedMac = "";              // MAC saved after first accepted park pairing (uppercase)
    #endif

    // --- Gear selection -----------------------------------------------------
    uint8_t currentGearIndex = DEFAULT_GEAR_INDEX;  // Current speed multiplier index

    // --- Private helper methods ---------------------------------------------
    void requestMaxSlewSpeed();                          // Send async :GX97# request

    // ---- Extracted helper methods from run() ----
    void handleVerboseSystemStats();                     // Print memory stats (debug)
    void handleRawDebugMode(ControllerPtr myCtl);        // Print raw controller data
    void readStickValues(ControllerPtr myCtl);           // Read raw sticks/buttons/dpad
    bool handleUnlockSequence(uint8_t dpad);             // Process unlock button sequence
    void processStickNormalization();                    // Normalize raw sticks to -1..1
    void updateMaxSlewSpeedFromBroker();                 // Process async max speed reply
    void handleButtonEvents(uint32_t btns, uint8_t dpad); // Handle button rising edges
    float applyStickCurve(float value);                  // Apply response curve to stick
    void updateTargetSpeeds(float dt);                   // Convert sticks to target speeds
    void applyLimits(float &targetSpeedX, float &targetSpeedY);   // check for limit error and limit targetSpeeds
    void resetControllerState();                         // set btns and dpad to 0

    void handleSafetyStop(bool connectionLost);          // Execute safety stop
    void calculateSmoothSpeeds(float dt);                // Smooth target speeds with acceleration
    void maybeEnableHighAcceleration();                  // Temporarily raise acceleration
    void sendSlewCommands();                             // Send autoSlew commands to axes
    void updateLastSentSpeeds();                         // Store last sent speeds, start cleanup timer
    void checkCleanupAndPerform();                       // Perform cleanup after stopping
    void handleAxisStateDebug();                         // Print axis state (debug)
    void handlePlotterDebug();                           // Print data for serial plotter
    void handleTimingDebug(uint32_t startUs, uint32_t endUs);  // Print tick duration every 500ms
    // ---------------------------------------------
};

extern GamepadPILOT gamepadpilot;