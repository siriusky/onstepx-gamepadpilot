//comments:

//plugin runs on core 0, onstep motor timing runs on core 1
//Tick time of plugin tested on Esp32S3 and Esp32-WROOM-32U (Terrans V5 Pro)
//~ Idle: 35us Idle, connected: 70us Slewing: 80us Orbit tracking Command Tick: 1ms Orbit fit tick: 3.2ms


// Watchdog Logic:
// 1. If sticks are at zero (idle), only care if the Bluetooth link is physically connected.
// 2. If the mount is moving (slew > 0), require a fresh data packet at least every WATCHDOGTIMEOUT.
// 3. If slewing and data remains stale beyond WATCHDOGTIMEOUT, an emergency stop is triggered.
// 4. The watchdog is "sticky": once triggered, it stays active until fresh data arrives.

//Bluepad32 button states:
//
//for simultaneous button press states add numbers
//
//left stick           yVal; xVal; //-511 to 512
//right stick          ryVal; rxVal; //-511 to 512
//left trigger         digital: myCtl->buttons() //64   analog: ctl->brake()
//right trigger        digital: myCtl->buttons() //128  analog: ctl->throttle()
//left bumper          myCtl->buttons() //16
//right bumper         myCtl->buttons() //32
//A                    myCtl->buttons() //1
//B                    myCtl->buttons() //2
//X                    myCtl->buttons() //4
//Y                    myCtl->buttons() //8
//dpad                 myCtl->dpad() // up:1 ; down:2 ; right:4 ; left:8
//
//sticks: right, down is positive

//Axis mapping: 
//pier side east means telescope is pointing at target in the west

//targetspeedY negative --> north
//targetspeedY positive --> south
//targetspeedX negative --> axis 1 DIR_REVERSE (left on altazm mount from observer)
//targetspeedY positive --> axis 2 DIR_FORWARD (right on altazm mount from observer)

// autoslew commands are in rad/s


//todo:

//Failed to disconnect HIDS client for hids_cid=0, status=2 ?
//single 5ms tick when starting manual movement, normal while manually moving: 80 us, probably :GX97#
//real fix: get max speed and get acceleration public getters
//improve slew flag cleanup
//ALTALT support for those 3 people in the world who actually have an altalt mount
//clean up RAW_DEBUG_MODE conditinal compile part?

// things that could be changed in OnstepX sourcecode

// make axisn.autoslew threadsafe (seems fine, but autoslew.stop is not?)
// enable getacceleration, getmaxslewspeed,
// maybe a different method of bypassing the slow acceleration- fast deceleration behaviour other than setting acceleration=10 (probably not)
// maybe ardu nano esp32 /generic esp32 example pinmap?


//==============================================================
//includes

#include "GamepadPILOT.h"
#include <Bluepad32.h>
#include "../../Common.h"                              

//================================
//Bluepad32 init in functions file

// Initialize the Bluepad32 GamepadPILOT object
GamepadPILOT gamepadpilot; 


void GamepadPILOT::run() {

  // --- debug: plugin tick duration ---------------------------------------
  #if TIMING_DEBUG == 1
    uint32_t tickStartUs = micros();
  #endif

  // --- debug: periodic Freertos system stats ----------------------------
  #if VERBOSE_SYSTEM_STATS == 1
    handleVerboseSystemStats();
  #endif

  // --- update Bluepad32 -------------------------------------------------
  BP32.update();
  ControllerPtr myCtl = myControllers[0];

  // --- watchdog: refresh on new data, detect stale/connection loss --------
  if (myCtl && myCtl->isConnected() && myCtl->hasData()) {
    lastDataMillis = millis();
    watchdogActive = false;
}

  bool isMoving = (smoothTargetSpeedX != 0.0f || smoothTargetSpeedY != 0.0f);
  bool connectionLost = !(myCtl && myCtl->isConnected());
  bool dataStale = (lastDataMillis > 0 && (millis() - lastDataMillis > WATCHDOGTIMEOUT));

  // Activate sticky watchdog if moving and data is stale, unless satellite tracking is active
#if ENABLE_SAT_TRACKING_FEATURE == 1
  if (isMoving && dataStale && !satelliteTracker.isActive()) {
    watchdogActive = true;
  }
#else
  if (isMoving && dataStale) {
    watchdogActive = true;
  }
#endif

bool shouldStop = connectionLost || watchdogActive;

// Main loop running the slewing =========================================================
  if (!shouldStop) {
    stopRetries = 0;

    // --- debug: raw controller output -------------------------------------
    #if RAW_DEBUG_MODE != 0
      handleRawDebugMode(myCtl);
    #endif

    // --- normal operation -------------------------------------------------
    #if RAW_DEBUG_MODE == 0
      readStickValues(myCtl);

      bool buttonsChanged = (btns != lastBtns || dpad != lastDpad);

      // --- unlock sequence (if enabled) ----------------------------------
      #if USE_UNLOCK_SEQUENCE == 1
        if (!handleUnlockSequence(dpad)) {
          stickPercentX = 0;
          stickPercentY = 0;
          stickPercentrX = 0;
          stickPercentrY = 0;
          #if TIMING_DEBUG == 1
            uint32_t tickEndUs = micros();
            handleTimingDebug(tickStartUs, tickEndUs);
          #endif
          return; // Skip the rest of the processing for this tick
        }
      #endif

      // --- normalize sticks to -1..1 -------------------------------------
      processStickNormalization();

      bool userSlewInputActive = (MAIN_STICK_X != 0.0f || MAIN_STICK_Y != 0.0f);
      #if USE_DPAD_SLEWING == 1
        userSlewInputActive = userSlewInputActive || (dpad != 0);
      #endif
      
      // request max slew speed only on rising edge of stick activity
      if (userSlewInputActive && !wasUserSlewInputActive) {
        requestMaxSlewSpeed();
      }
      wasUserSlewInputActive = userSlewInputActive;

      // check async max slew speed response
      if (brokerHandle != 0) {
        updateMaxSlewSpeedFromBroker();
      }

      // --- handle button events -------------------------------------------
      if (buttonsChanged) {
        handleButtonEvents(btns, dpad);
      }

      // --- map sticks to target slew speeds with checking limits -------------------------------
      updateTargetSpeeds(dt);
    #endif // RAW_DEBUG_MODE == 0
  } else {
    // safety stop when connection lost or watchdog active
    handleSafetyStop(connectionLost);
  }

  // --- post-processing (always unless raw debug) --------------------------
  #if RAW_DEBUG_MODE == 0
    // smooth target speeds using acceleration
    calculateSmoothSpeeds(dt);

    // send slew commands, handle acceleration and cleanup
    sendSlewCommands();

    #if AXIS_STATE_DEBUG == 1
      handleAxisStateDebug();
    #endif //AXIS_STATE_DEBUG == 1
  #endif // RAW_DEBUG_MODE == 0

  #if TIMING_DEBUG == 1
    uint32_t tickEndUs = micros();
    handleTimingDebug(tickStartUs, tickEndUs);
  #endif
}

