// Configurations for GamepadPILOT plugin
#ifndef GAMEPADPILOT_CONFIG_H
#define GAMEPADPILOT_CONFIG_H


//=============================================================================================================================
// Safety 
//=============================================================================================================================
#define USE_UNLOCK_SEQUENCE 1              // 1,    Set to 1 to require a button combo before the mount moves: dpad down 2 sec dpad up 2sec
#define UNLOCK_HOLD_MS 2000                // 2000, Duration in ms to hold each direction
#define WATCHDOGTIMEOUT 4000               // 4000, if this long no new data packet has been received, the mount stops. Bluepad32 disconnect might take too long to register (ghosting).
                                           //       Some controllers stop sending data when holding stick very still. Set very high to disable.

#define MAC_WHITELIST_ENABLE 0             // 0,    Set to 1 to only allow a specific controller. Other controllers are disconnected.
                                           //       With DEBUG VERBOSE, the MAC of every controller trying to connect is logged.
#define MAC_WHITELIST_ADDRESS "XX:XX:XX:XX:XX:XX"   // Format: XX:XX:XX:XX:XX:XX

#define PARK_PAIRING_ONLY 0                // 0,    Only allow pairing while the mount is parked. Rejected controllers are disconnected.
                                           //       The first accepted MAC is whitelisted for the whole session (until reboot).
                                           //       Disconnect/reconnect of that controller is allowed without being parked.
                                           //       Note: with TRACK_AUTOSTART ON, OnStepX unparks automatically right after boot,
                                           //       so park the mount (app/handbox) before switching on the controller.
                                           //       After pairing, unpark (e.g. :hR# custom command) to move the mount.

#define LIMIT_RECOVERY_WINDOW_MS 500       // 500,  When mount runs outside limit, it can be unlocked this long to move back inside limts. To unlock center all sticks.

//=============================================================================================================================
// Mount control and movement
//=============================================================================================================================
#define USE_RIGHT_STICK 0                  // 0 = left stick, 1 = right stick for normal mount slewing
#define SWAP_N_S  0                        // 0,    change to swap the direction the sticks move the mount
#define SWAP_E_W  0                        // 0,    change to swap the direction the sticks move the mount
#define STICK_CURVE_EXPONENT 2             // 2,    Choose speed curve for slewing with gamepad sticks: 1 (Linear), 2 (Square), 3 (Cubic)
#define USE_DPAD_SLEWING 0                 // 0,    Also use the D-pad for slewing. Speeds are fixed at DPAD_SPEED_MULTIPLIERS as fractions of max speed.


// Button functions ---------------------------------------------------------------------------------------------------------------------

//left trigger         64
//right trigger        128
//left bumper          16
//right bumper         32
//A                    1
//B                    2
//X                    4
//Y                    8
//dpad                 up:1 ; down:2 ; right:4 ; left:8
// for combined buttons add values, e.g. left trigger + X --> 68

#define CYCLE_GEAR_BUTTON 32               // 32,   Cycles through speed multipliers. Controller rumbles at slowest speed.
static const float SPEED_MULTIPLIERS[] = {0.1f, 0.3f, 1.0f};   // adjust speed multipliers here
static const float DPAD_SPEED_MULTIPLIERS[] = {0.03f, 0.2f, 1.0f}; // adjust D-pad speed multipliers here
#define DEFAULT_GEAR_INDEX 2               // 2,    Starts at 100% speed, values 0 ; 1 ; 2

#define SWAP_TOGGLE_BUTTON 16              // 16,   Swaps which way the stick moves the mount, for looking through the eyepiece.
#define TOGGLE_SWAP_NS 1                   // 1,    Set to 1 to have the button toggle N/S swap
#define TOGGLE_SWAP_EW 1                   // 1,    Set to 1 to have the button toggle E/W swap

#define TRACKING_TOGGLE_BUTTON 4           // 4,    toggle regular (sidereal etc.) tracking, 0 to disable

#define SATELLITE_TRACKING_BUTTON 64       // 64,   when enabled, starts satellite tracking. Hold for at least SATELLITE_TRACKING_BUTTON_HOLD_TIME while following satellite to start tracking.
                                           // Push again while tracking to stop.
// Custom LX200 commands 
// One action per line:
//   CUSTOM_COMMAND(button_bitmap, dpad_bitmap, "LX200 command")
// Set the component you don't need to 0. See commands in OnstepX/docs/COMMAND_REFERENCE.md

#define CUSTOM_COMMANDS \
    CUSTOM_COMMAND(1, 0, ":hP#") \
    CUSTOM_COMMAND(2, 0, ":hR#") \
    CUSTOM_COMMAND(0, 0, "") \
    CUSTOM_COMMAND(0, 0, "") \
    /* Add more lines here as needed */
//:hP# --> Park
//:hR# --> Unpark


// Advanced -----------------------------------------------------------------------------------------------------------------------------
#define ACCEL_RATE 0.4f                    // 0.4f, Acceleration in rad/s^2. Independent of loop frequency.
#define DEADZONE  30                       // 30,   Deadzone of the sticks, to avoid drift. In Bluepad32 values -512 to 511. You can use RAW_DEBUG_MODE to test the precision of your controller.
#define STICK_MAX 512                      // 512,  just in case not all gamepads have max at 512
#define USE_DATETIME_OVERRIDE 0            // 0,    Sets the Datetime to an arbitrary value to unlock movement on altaz mounts without rtc without using phone. Leave 0 when you dont know what it does.

//=============================================================================================================================
// Satellite tracking
//=============================================================================================================================
#define ENABLE_SAT_TRACKING_FEATURE 1         // 1,   enables satellite tracking after after holding SATELLITE_TRACKING_BUTTON while following satellite. 0 to disable everything related.
#define SATELLITE_TRACKING_BUTTON_HOLD_TIME 2500  // 2500,  mount starts tracking the satellite after holding button for this long while following satellite

#define SATELLITE_STATS_PRINT 0               // 0, Prints: time since start, tracking mode, mount alt, satellite pass highest elevation, 
                                              //    time to closest approach, orbit altitude, range (distance), angular velocity on sky
                                              //    Might interfere when simultaneously using INDI/ASCOM over USB.
#define SATELLITE_STATS_INTERVAL_MS 1000      // 1000,  Satellite Stats Print interval in milliseconds

// Advanced -----------------------------------------------------------------------------------------------------------------------------
#define SATELLITE_TRACKING_MANUAL_GAIN 0.2f // 0.2f,    manual corrections are multiplied with this gain. Stick response is finer when satellite tracking than during normal slewing
// Simple tracking (speed integrator)
#define SATELLITE_TRACKING_INTEGRATION_GAIN 0.4f     //higher values give manual corrections more wheight when calculating new baseline speed
// Advanced tracking (orbit model)
#define SATELLITE_ORBIT_ENABLE 1               // 1,    Calculate orbit from record coordinates and fit orbit. Needed for Satellite Stats and orbit following
#define SATELLITE_ORBIT_SAMPLE_INTERVAL_MS 500 // 500,  coordinate sample interval
#define SATELLITE_ORBIT_MAX_SAMPLES 600        // 600, stored samples for the whole pass (600 * 500ms = 5 min, 12 bytes each)
#define SATELLITE_ORBIT_FIT_INTERVAL_MS 1000   // 1000, recalculate orbit every x ms
// Two-point fit: circular orbit through anchor A (old, smoothed) and anchor B (current mount position)
#define SATELLITE_ORBIT_DISCARD_START_S 5      // 5,    ignore the first x seconds of tracking for anchor A (least accurate)
#define SATELLITE_ORBIT_MIN_ANCHOR_ALT_DEG 15  // 15,   anchors only above this elevation; below it the last orbit is kept
#define SATELLITE_ORBIT_MIN_BASELINE_S 6       // 6,    min. seconds between anchor A centre and now; first orbit ~ DISCARD + A_WINDOW/2 + this
#define SATELLITE_ORBIT_MAX_BASELINE_S 60      // 60,   max. seconds between anchors; anchor A moves forward on long passes
#define SATELLITE_ORBIT_ANCHOR_A_WINDOW_S 4    // 4,    anchor A: quadratic smoothing over x seconds (centred)
#define SATELLITE_ORBIT_MAX_DEV_DEG 0.25f      // 0.25f,    orbit misses the sample halfway between the anchors by more than x deg -> back to simple tracking, new orbit with shorter baseline
// Speed limit hold: no orbit updates while an axis is (or the orbit would need to be) at max slew speed
#define SATELLITE_ORBIT_SPEED_LIMIT_FRACTION 0.95f // 0.95f,    limit = fraction of the current max slew speed (commanded speed or orbit alt/az rate)
#define SATELLITE_ORBIT_SPEED_RELEASE_S 3      // 3,    release the hold x seconds after the last limit detection
// Orbit steering: orbit rates (feedforward) + PI on the position error
#define SATELLITE_ORBIT_STEER_ENABLE 1         // 1,    The orbit model steers the mount and takes over from the simple integrator once an orbit exists, 0 = observe only
#define SATELLITE_ORBIT_KP 1.0f                // 1.0f, 1/s: error correction rate (1.0 -> error halves in ~0.7 s)
#define SATELLITE_ORBIT_KI 0.05f               // 0.05f,    1/s^2: removes constant offsets

#define SATELLITE_ORBIT_DEBUG 0                // 1 = print orbit fit results and extrapolation error (direct Serial output, independent of OnStepX DEBUG level)

//=============================================================================================================================
// Performance settings
//=============================================================================================================================
#define POLL_INTERVAL 20                   // 20,   Logic tick rate (Bluetooth poll & main logic). 20ms (50Hz) is ideal for HID.
#define COMMAND_INTERVAL_MULTIPLE  3       // 3,    Send slew commands every x poll ticks

//=============================================================================================================================
// Debug settings
//=============================================================================================================================
// Better to turn off when not needed. Might interfere with INDI/ASCOM over USB. Uses Serial.print.
#define RAW_DEBUG_MODE 0                   // 0,    Shows all bluepad32 raw values in serial monitor. All other functions are turned off
#define RAW_DEBUG_PRINT_INTERVAL 250       // 250,  in ms
#define AXIS_STATE_DEBUG 0                 // 0,    Shows frequency and slew status for both axes
#define PLOTTER_DEBUG 0                    // 0,    Enables output formatted for the Arduino Serial Plotter
#define VERBOSE_SYSTEM_STATS 0             // 0,    Shows system stats while running the regular tasks. 
#define TIMING_DEBUG 0                     // 0,    Shows tick duration of the entire plugin
#define TIMING_DEBUG_PRINT_INTERVAL 10     // 10,   if shorter than POLL_INTERVAL, sends every tick, if higher, might not catch every spike


//in case of compiler issues, check:
//use bluepad32 board manager for non arduino boards
//windows: move Arduino15 folder to shorter path name
//usb cdc on boot enable
//use "upload using programmer"
//use gpio pin numbering 

#endif