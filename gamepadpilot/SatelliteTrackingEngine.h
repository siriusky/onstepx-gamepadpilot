#pragma once

#include <Arduino.h>
#include "../../Common.h"   // for VLF() and millis()
#include "../../telescope/mount/Mount.h"   // for Coordinate and extern mount
#include "../../telescope/mount/site/Site.h" // for latitude


#ifndef SATELLITE_TRACKING_START_MAX_SAMPLES
#define SATELLITE_TRACKING_START_MAX_SAMPLES 200 //4s at default settings 20ms tick
#endif

#ifndef SATELLITE_TRACKING_MANUAL_GAIN
#define SATELLITE_TRACKING_MANUAL_GAIN 0.2f
#endif

#ifndef SATELLITE_TRACKING_INTEGRATION_GAIN
#define SATELLITE_TRACKING_INTEGRATION_GAIN 0.4f
#endif

// --- Advanced tracking (orbit model) defaults --------------------------------
#ifndef SATELLITE_ORBIT_ENABLE
#define SATELLITE_ORBIT_ENABLE 1
#endif

#ifndef SATELLITE_ORBIT_SAMPLE_INTERVAL_MS
#define SATELLITE_ORBIT_SAMPLE_INTERVAL_MS 500
#endif

#ifndef SATELLITE_ORBIT_MAX_SAMPLES
#define SATELLITE_ORBIT_MAX_SAMPLES 600
#endif

#ifndef SATELLITE_ORBIT_FIT_INTERVAL_MS
#define SATELLITE_ORBIT_FIT_INTERVAL_MS 1000
#endif

#ifndef SATELLITE_ORBIT_DEBUG
#define SATELLITE_ORBIT_DEBUG 0
#endif

// --- Two-point fit defaults ------------------------------------------------------
#ifndef SATELLITE_ORBIT_DISCARD_START_S
#define SATELLITE_ORBIT_DISCARD_START_S 5
#endif

#ifndef SATELLITE_ORBIT_MIN_ANCHOR_ALT_DEG
#define SATELLITE_ORBIT_MIN_ANCHOR_ALT_DEG 15
#endif

#ifndef SATELLITE_ORBIT_MIN_BASELINE_S
#define SATELLITE_ORBIT_MIN_BASELINE_S 6
#endif

#ifndef SATELLITE_ORBIT_MAX_BASELINE_S
#define SATELLITE_ORBIT_MAX_BASELINE_S 60
#endif

#ifndef SATELLITE_ORBIT_ANCHOR_A_WINDOW_S
#define SATELLITE_ORBIT_ANCHOR_A_WINDOW_S 4
#endif

// Quality / fallback: if the new orbit misses the sample halfway between the anchors
// by more than this (deg, absolute), the orbit is dropped (-> simple integrator) and
// anchor A moves forward to that midpoint for the next fit.
#ifndef SATELLITE_ORBIT_MAX_DEV_DEG
#define SATELLITE_ORBIT_MAX_DEV_DEG 0.25f
#endif

// --- Speed limit hold defaults --------------------------------------------------
#ifndef SATELLITE_ORBIT_SPEED_LIMIT_FRACTION
#define SATELLITE_ORBIT_SPEED_LIMIT_FRACTION 0.95f
#endif

#ifndef SATELLITE_ORBIT_SPEED_RELEASE_S
#define SATELLITE_ORBIT_SPEED_RELEASE_S 3
#endif

// --- Orbit steering defaults ----------------------------------------------------
#ifndef SATELLITE_ORBIT_STEER_ENABLE
#define SATELLITE_ORBIT_STEER_ENABLE 1
#endif

#ifndef SATELLITE_ORBIT_KP
#define SATELLITE_ORBIT_KP 1.0f
#endif

#ifndef SATELLITE_ORBIT_KI
#define SATELLITE_ORBIT_KI 0.05f
#endif

#if SATELLITE_ORBIT_ENABLE == 1
#include "SatelliteOrbitModel.h"
#endif


//todo:

//ALTALT support
//if necessary (cpu time) float version of fit / root bisection algorithm

//mount.getPosition() unit: radians
//r     ra 0..2pi 0h..24h in absolute sky coordinates
//h     local hour angle
//d     dec -0.5pi..0.5pi -90deg..90deg
//a     altitude -0.5pi..0.5pi -90deg..90deg
//z     azimuth 0 at north, east is positive west is negative, south is pi
//aa1   ?
//aa2   ?
//a1    axis1 physical
//a2    axis2 physical, -0.5pi to 0.5pi
//pier  pierside

// ============================================================================
// SATELLITE TRACKING - OVERVIEW (times with default config)
// ============================================================================
// Button held (>= SATELLITE_TRACKING_BUTTON_HOLD_TIME):
//   RECORDING: stick speeds are recorded, mount moves normally by stick.
//
// Button released -> enable(mountType), session time t = 0:
//   SIMPLE INTEGRATOR (steers the mount, always active while tracking):
//     baseline speed = average recorded stick speed
//     manual stick input * MANUAL_GAIN is added and slowly integrated
//     into the baseline (INTEGRATION_GAIN), axis1 scaled by cos(axis2)
//     -> targetSpeed (rad/s) per axis
//
//   ORBIT MODEL (two-point fit, every FIT_INTERVAL_MS):
//     mount alt/az sampled every SAMPLE_INTERVAL_MS from t = 0, and right before each fit
//     anchor A: quadratic smoothing over ANCHOR_A_WINDOW_S, first clean window
//               >= DISCARD_START_S and >= MIN_ANCHOR_ALT_DEG, max MAX_BASELINE_S old
//     anchor B: newest raw sample = current mount position
//     arc A->B must match Kepler speed -> orbit, passes exactly through B
//       -> no jump on a new orbit, manual corrections are kept
//     first orbit: A centre + MIN_BASELINE_S (default ~t = 18 s)
//     quality: orbit vs. the sample halfway between A and B, > MAX_DEV_DEG -> FALLBACK
//     now below MIN_ANCHOR_ALT_DEG: hold, keep last orbit
//     failed / rejected fit: keep last orbit
//     SPEED LIMIT HOLD: an axis in CATCH-UP (orbit mode) or commanded speed
//       >= SPEED_LIMIT_FRACTION of max (simple mode) -> no orbit updates, samples
//       flagged and excluded from fits; released SPEED_RELEASE_S after the last limit
//
//   Orbit output: predict(t) -> target alt/az (+ optional alt/az rates) for any t of the pass
//   (incl. Earth rotation). Math: SatelliteOrbitModel.h/.cpp
//
//   TIMING: control step only on the send tick (every COMMAND_INTERVAL_MULTIPLE ticks):
//     1x getPosition, 1x predict(with rates) -> shared by steering, speed limit,
//     fallback, sampling and debug. Fit tick: +1 fit (double) +1 predict (quality).
//     Manual stick correction: every tick.
//
//   ORBIT STEERING (ALTAZ, GEM, FORK, STEER_ENABLE): takes over from the simple
//   integrator as soon as a valid orbit exists (falls back if it becomes invalid)
//     works per AXIS (index 0 = axis1, 1 = axis2)
//       ALTAZ: axis1 = az, axis2 = alt
//       GEM / FORK: axis1 = hour angle, axis2 = dec (fit stays in alt/az, the
//       predicted line of sight is rotated by the colatitude, no sidereal time needed;
//       mount position from getPosition().h/.d, pier side already removed by OnStepX)
//     (mapping only in predictAxes() / mountToAxes())
//     Meridian / axis limits: not handled here, applyLimits() stops tracking.
//     speed = orbit axis rate (feedforward) + KP*err + KI*integral(err) + manual
//     err   = orbit + manualOffset - mount (wrapped to +-pi)
//     manualOffset integrates the manual correction (PI does not pull it back),
//       reset to 0 when a new orbit is accepted
//     X = axis1 rate, Y = -axis2 rate (Y < 0 = up), integral frozen while clamped / catch-up
//     CATCH-UP (per axis): requested speed (ff + PI) >= SPEED_LIMIT_FRACTION of max
//       -> direction latched, axis runs at max speed in that direction (manual input
//       ignored on that axis) until it reaches/passes its target and the target rate
//       is below the limit again. The other axis keeps tracking its own target.
//     FALLBACK: fit dev > MAX_DEV_DEG (e.g. plane turning) -> orbit dropped,
//       simple integrator continues with the last speed, anchor A moved to the
//       failed midpoint; the next accepted fit returns to orbit steering
//
//   HORIZON STOP: orbit prediction or mount below alt 0
//     -> zero speed, disable(), plugin reports it via consumeHorizonStop()
// ============================================================================


class SatelliteTrackingEngine {
public:

// ---------------------------------------------------------------------
// update() – main satellite tracking math
// ---------------------------------------------------------------------
//   targetXNorm, targetYNorm : normalized stick input (-1 .. 1)
//   maxSlewSpeedRad          : current maximum slew speed in rad/s
//   targetXRad, targetYRad   : output references, final speed in rad/s
// ---------------------------------------------------------------------
void update(float dt, float targetXNorm, float targetYNorm,
            float maxSlewSpeedRad, float &targetXRad, float &targetYRad,
            bool controlTick = true, bool orbitSteerAllowed = false) {

    // If recording, just store the sample and keep the stick command unchanged
    if (recording) {
        recordSample(targetXNorm, targetYNorm);
        targetXRad = targetXNorm * maxSlewSpeedRad;
        targetYRad = targetYNorm * maxSlewSpeedRad;
        return;
    }

    if (!active) return;

    // Control step (send tick only): 1x getPosition, 1x predict, PI, limits, fit.
    // Returns false when tracking stopped (below horizon): send zero speed.
    if (controlTick && !controlStep(maxSlewSpeedRad, orbitSteerAllowed)) {
        targetXRad = 0.0f;
        targetYRad = 0.0f;
        return;
    }

    // Manual stick correction (every tick, normalized)
    float manualXNorm = targetXNorm * SATELLITE_TRACKING_MANUAL_GAIN;
    float manualYNorm = targetYNorm * SATELLITE_TRACKING_MANUAL_GAIN;

    #if SATELLITE_ORBIT_ENABLE == 1
    if (steering) {
        // ORBIT MODE: commanded axis rates from the control step + manual correction.
        // X = axis1 rate, Y = axis2 with Y < 0 = up -> Y = -axis2 rate.
        // An axis in catch-up runs at max speed: manual input ignored on that axis.
        if (catchDir[0] != 0) manualXNorm = 0.0f;
        if (catchDir[1] != 0) manualYNorm = 0.0f;
        float inv = (maxSlewSpeedRad > 0.0f) ? 1.0f / maxSlewSpeedRad : 0.0f;
        targetXNorm =  axCmd[0] * inv + manualXNorm;
        targetYNorm = -axCmd[1] * inv + manualYNorm;

        // Manual corrections move the reference too, so the PI does not pull them back.
        // Reset to 0 when a new orbit is accepted.
        manualOffset[0] +=  manualXNorm * maxSlewSpeedRad * dt;
        manualOffset[1] += -manualYNorm * maxSlewSpeedRad * dt;
    } else
    #endif
    {
        // SIMPLE MODE: integrated baseline + manual correction
        trackingRateX += SATELLITE_TRACKING_INTEGRATION_GAIN * dt * manualXNorm;
        trackingRateY += SATELLITE_TRACKING_INTEGRATION_GAIN * dt * manualYNorm;

        // Clamp baseline to normalized range
        if (trackingRateX > 1.0f)  trackingRateX = 1.0f;
        else if (trackingRateX < -1.0f) trackingRateX = -1.0f;

        if (trackingRateY > 1.0f)  trackingRateY = 1.0f;
        else if (trackingRateY < -1.0f) trackingRateY = -1.0f;

        // Final normalized speed = baseline + scaled manual correction
        targetXNorm = trackingRateX + manualXNorm;
        targetYNorm = trackingRateY + manualYNorm;

        // Altitude/axis2 correction for axis1 (X), cos(axis2) from the control step
        targetXNorm *= referenceCos2 / cachedCos2;
    }

    #if SATELLITE_ORBIT_ENABLE == 1
    // Speed limit (simple mode): commanded speed (before clamping) at the limit on any axis.
    // Sticky until the next control step evaluates it. Orbit mode uses the catch-up state.
    if (!steering &&
        (fabsf(targetXNorm) >= SATELLITE_ORBIT_SPEED_LIMIT_FRACTION ||
         fabsf(targetYNorm) >= SATELLITE_ORBIT_SPEED_LIMIT_FRACTION)) mountAtLimit = true;
    axClamped[0] = fabsf(targetXNorm) > 1.0f;   // per-axis anti-windup
    axClamped[1] = fabsf(targetYNorm) > 1.0f;
    #endif

    // Clamp final normalized values
    if (targetXNorm > 1.0f)  targetXNorm = 1.0f;
    else if (targetXNorm < -1.0f) targetXNorm = -1.0f;

    if (targetYNorm > 1.0f)  targetYNorm = 1.0f;
    else if (targetYNorm < -1.0f) targetYNorm = -1.0f;

    lastOutXNorm = targetXNorm;
    lastOutYNorm = targetYNorm;

    // Always convert normalized to rad/s and write to output references
    targetXRad = targetXNorm * maxSlewSpeedRad;
    targetYRad = targetYNorm * maxSlewSpeedRad;

} //void (update)

    ///////////////////////////////////////////////////////////////////////////////

    // =====================================================================
    // State query helpers
    // =====================================================================

    bool isActive() const { return active; }
    bool isRecording() const { return recording; }

    // True once after tracking stopped because the target went below the horizon
    // (flag is cleared by this call).
    bool consumeHorizonStop() {
        bool s = horizonStop;
        horizonStop = false;
        return s;
    }

    // =====================================================================
    // Tracking state control
    // =====================================================================

    // mountType: OnStepX mount type (GEM, FORK, ALTAZM, ALTALT), selects the axis frame
    void enable(uint8_t mountType = ALTAZM) {
        if (!active) {
            active = true;
            horizonStop = false;
            equatorial = (mountType == GEM || mountType == FORK);
            cachedPos = readPosition();
            referenceAxis2 = cachedPos.a2;                 // record current axis2 angle
            referenceCos2 = fabsf(cosf(referenceAxis2));
            cachedCos2 = referenceCos2;
            lastOutXNorm = trackingRateX;
            lastOutYNorm = trackingRateY;
            
            #if SATELLITE_ORBIT_ENABLE == 1
            // New session: coordinate recording starts with the simple tracker
            sessionStartMillis = millis();
            trajectorySampleCount = 0;
            trajectoryRingIndex = 0;
            lastTrajectorySampleTime = sessionStartMillis - SATELLITE_ORBIT_SAMPLE_INTERVAL_MS; // sample immediately
            lastFitTime = sessionStartMillis;
            orbitModel.reset();
            candidateModel.reset();
            candidateModel.setLatitude((float)site.location.latitude);
            mountAtLimit = false;
            speedHold = false;
            lastLimitMs = 0;
            holdReason = HoldReason::NONE;
            steering = false;
            predValid = false;
            anchorAEarliestMs = 0;
            lastDevDeg = -1.0f;
            resetSteeringState();
            #if SATELLITE_STATS_PRINT == 1
            lastStatsPrintMs = sessionStartMillis;   // first print after interval from start
            lastClosestTca = 0.0f;
            lastClosestMaxElevDeg = 0.0f;
            hasClosestInfo = false;
            #endif
            #endif
        }
    }

    void disable() {
        if (active) {
            active = false;
            #if SATELLITE_ORBIT_ENABLE == 1
            steering = false;
            #endif
        }
    }

    // =====================================================================
    // Recording state control
    // =====================================================================

    // Start coordinate recording.
    void startRecording() {
        recording = true;

        // Clear any pending tracking rate from a previous session
        trackingRateX = 0.0f;
        trackingRateY = 0.0f;

        // Clear previous recording data.
        recordingCount = 0;
        ringIndex = 0;
    }

    // Stop coordinate recording.
    void stopRecording() {
        // Compute average of recorded samples and set as initial baseline
        if (recordingCount > 0) {
            float sumX = 0.0f;
            float sumY = 0.0f;
            for (size_t i = 0; i < recordingCount; i++) {
                sumX += recordedX[i];
                sumY += recordedY[i];
            }
            trackingRateX = sumX / recordingCount;
            trackingRateY = sumY / recordingCount;
        }
        recording = false;
    }


private:

    // =====================================================================
    // simple tracking and stuff
    // =====================================================================

    // ---------------------------------------------------------------------
    // Fixed-size ring buffer: stores up to SATELLITE_TRACKING_START_MAX_SAMPLES
    // samples. When full, the oldest sample is overwritten.
    // ---------------------------------------------------------------------
    static constexpr size_t MAX_SAMPLES = SATELLITE_TRACKING_START_MAX_SAMPLES;

    float recordedX[MAX_SAMPLES];
    float recordedY[MAX_SAMPLES];
    size_t recordingCount = 0;
    size_t ringIndex = 0;

    // Baseline tracking rates adjusted
    float trackingRateX = 0.0f;
    float trackingRateY = 0.0f;

    // Store one target-speed sample.
    void recordSample(float targetX, float targetY) {
        if (recordingCount < MAX_SAMPLES) {
            // Buffer has room.
            recordedX[recordingCount] = targetX;
            recordedY[recordingCount] = targetY;
            recordingCount++;
        } else {
            // Ring buffer full: overwrite the oldest sample.
            recordedX[ringIndex] = targetX;
            recordedY[ringIndex] = targetY;
            ringIndex = (ringIndex + 1) % MAX_SAMPLES;
        }
    }

    bool active = false;
    bool recording = false;
    bool equatorial = false;       // GEM / FORK: axis1 = HA, axis2 = Dec (set in enable())
    bool horizonStop = false;      // set when tracking stopped below the horizon
    //uint32_t lastPrintMillis = 0;
    float referenceAxis2 = 0.0f;
    float referenceCos2 = 0.0f;

    // Cached mount position: read once per control step (also used by sampling,
    // cos(axis2) correction and PI)
    Coordinate cachedPos;
    float cachedCos2 = 1.0f;
    float lastOutXNorm = 0.0f;     // last normalized output (for fallback to simple mode)
    float lastOutYNorm = 0.0f;

    // ---------------------------------------------------------------------
    // Control step: runs on the send tick only (controlTick from the caller)
    // ---------------------------------------------------------------------
    // Mount position with alt/az (.a/.z) AND HA/Dec (.h/.d) filled on every mount type.
    //   ALTAZ: getPosition() (CR_MOUNT_EQU) converts horToEqu -> h/d also valid.
    //   eq + TOPOCENTRIC: the refraction step (equToHor, refrac, horToEqu) leaves
    //   a/z filled (refracted, matching h/d) at no extra cost.
    //   eq + OBSERVED_PLACE: no refraction step -> request the horizon coordinates.
    Coordinate readPosition() const {
        #if MOUNT_COORDS == OBSERVED_PLACE
        if (equatorial) return mount.getPosition(CR_MOUNT_ALL);
        #endif
        return mount.getPosition();
    }

    // Returns false if tracking was stopped (target below the horizon).
    bool controlStep(float maxSlewSpeedRad, bool orbitSteerAllowed) {
        cachedPos = readPosition();                    // the only getPosition per control step
        float c2 = fabsf(cosf(cachedPos.a2));
        cachedCos2 = (c2 > 0.01f) ? c2 : 0.01f;        // avoid division by ~0 near the pole

        #if SATELLITE_ORBIT_ENABLE == 1
        uint32_t now = millis();
        float tNow = (now - sessionStartMillis) * 0.001f;

        //unsigned long predictStartUs = micros();//predict debug

        // One prediction per control step, shared by steering, trigger B, catch-up, debug.
        // Directly in axis coordinates (axTarget / axRate) + altitude for the horizon check.
        predValid = orbitModel.isValid() && predictOrbitAxes(tNow);

        //unsigned long predictElapsedUs = micros() - predictStartUs;//predict debug
        //Serial.printf("Predict_call_us:%u\n", predictElapsedUs);


        // Horizon stop: orbit prediction or mount below the horizon (cached values only)
        if (cachedPos.a < 0.0f || (predValid && predAlt < 0.0f)) {
            stopBelowHorizon(tNow);
            return false;
        }

        // Per-axis position error: (orbit + manual offset) - mount, wrapped to +-pi
        // (no effect on a non-wrapping axis, its error is always far below pi)
        if (predValid) {
            mountToAxes();
            for (int i = 0; i < 2; i++)
                axErr[i] = wrapPi(axTarget[i] + manualOffset[i] - axPos[i]);
        }

        // Catch-up state per axis (orbit mode), from the requested axis speeds.
        // Evaluated before the hold, so fits/samples are blocked in the same step.
        if (steering && predValid) updateCatchup(maxSlewSpeedRad, tNow);
        else catchDir[0] = catchDir[1] = 0;
        updateSpeedHold(now);
        mountAtLimit = false;                          // re-armed for the next ticks

        // Sampling and orbit fit (may accept a new orbit -> manual offset reset).
        // Also sample right before a fit, so the newest sample (the point the new orbit
        // passes through) is the current position and recent manual corrections are kept.
        if (now - lastTrajectorySampleTime >= SATELLITE_ORBIT_SAMPLE_INTERVAL_MS || fitDue(now))
            sampleTrajectory(now);
        updateOrbitFit(now);

        // Mode selection: orbit steering as soon as a valid orbit exists (ALTAZ only).
        // A rejected fit resets the orbit (predValid false) -> simple integrator.
        bool wantSteer = SATELLITE_ORBIT_STEER_ENABLE == 1 && orbitSteerAllowed && predValid;
        if (wantSteer != steering) switchMode(wantSteer, tNow);

        if (steering) piStep(maxSlewSpeedRad, now);
        
        #if SATELLITE_STATS_PRINT == 1
        if (now - lastStatsPrintMs >= SATELLITE_STATS_INTERVAL_MS) {
            printStats(now, maxSlewSpeedRad);
            lastStatsPrintMs = now;
        }
        #endif
        
        #else
        (void)maxSlewSpeedRad; (void)orbitSteerAllowed;
        if (cachedPos.a < 0.0f) {
            stopBelowHorizon(0.0f);
            return false;
        }
        #endif
        return true;
    }

    // Stop tracking cleanly: caller sends zero speed (ACCEL_RATE ramps down),
    // plugin reports it via consumeHorizonStop().
    void stopBelowHorizon(float tNow) {
        disable();
        horizonStop = true;
        lastOutXNorm = lastOutYNorm = 0.0f;
        #if SATELLITE_ORBIT_DEBUG == 1 && SATELLITE_ORBIT_ENABLE == 1
        char buf[60];
        snprintf(buf, sizeof(buf), "ORBIT: below horizon, stop t=%.1fs", tNow);
        Serial.println(buf);
        #else
        (void)tNow;
        #endif
    }

#if SATELLITE_ORBIT_ENABLE == 1
    // =====================================================================
    // Coordinate trajectory recording (advanced tracking input)
    // Recording starts in enable(), i.e. once the simple tracker runs.
    // =====================================================================
    struct TrajectorySample {
        uint32_t timeMs;      // ms since sessionStartMillis
        float altitude;       // radians
        float azimuth;        // radians
        float dx, dy, dz;     // precomputed direction (ENU unit vector)
        bool limited;         // recorded during a speed limit hold -> not used for fits
    };

    static constexpr float DEG_TO_RAD_F = 0.017453292519943f;
    static constexpr float RAD_TO_DEG_F = 57.29577951308232f;

    // Full pass storage (ring buffer, oldest overwritten when full)
    static constexpr size_t TRAJECTORY_MAX_SAMPLES = SATELLITE_ORBIT_MAX_SAMPLES;
    TrajectorySample trajectorySamples[TRAJECTORY_MAX_SAMPLES];
    size_t trajectorySampleCount = 0;
    size_t trajectoryRingIndex = 0;      // next write position once buffer is full
    uint32_t lastTrajectorySampleTime = 0;
    uint32_t sessionStartMillis = 0;

    // Anchor A window buffer (member, not on the task stack).
    // +1 per sample taken right before a fit (max one extra per second).
    static constexpr size_t ANCHOR_A_MAX_SAMPLES =
        (SATELLITE_ORBIT_ANCHOR_A_WINDOW_S * 1000UL) / SATELLITE_ORBIT_SAMPLE_INTERVAL_MS +
        SATELLITE_ORBIT_ANCHOR_A_WINDOW_S + 2;
    OrbitSample anchorAWindow[ANCHOR_A_MAX_SAMPLES];
    OrbitSample anchorB;                 // newest raw sample ("now")
    OrbitSample midSample;               // quality check sample halfway between A and B

    static_assert(SATELLITE_ORBIT_MAX_BASELINE_S > SATELLITE_ORBIT_MIN_BASELINE_S,
                  "SATELLITE_ORBIT_MAX_BASELINE_S must be larger than SATELLITE_ORBIT_MIN_BASELINE_S");
    static_assert(SATELLITE_ORBIT_ANCHOR_A_WINDOW_S >= 2,
                  "SATELLITE_ORBIT_ANCHOR_A_WINDOW_S must be at least 2 s");

    SatelliteOrbitModel orbitModel;      // last accepted fit (kept when a new fit fails)
    SatelliteOrbitModel candidateModel;  // scratch model for the current fit attempt
    uint32_t lastFitTime = 0;
    float lastDevDeg = -1.0f;            // quality of the last fit attempt (deg), -1 = none
    uint32_t anchorAEarliestMs = 0;      // anchor A floor (session ms), moved forward on rejection

    #if SATELLITE_STATS_PRINT == 1
        uint32_t lastStatsPrintMs = 0;       // Timestamp of last stats print
        float lastClosestTca = 0.0f;         // absolute session time of last displayed TCA
        float lastClosestMaxElevDeg = 0.0f;  // max elevation at that TCA (deg)
        bool  hasClosestInfo = false;        // true once there is a valid closest pass
    #endif

    // ---------------------------------------------------------------------
    // Speed limit hold
    //   Trigger MOUNT (simple mode): commanded speed >= SPEED_LIMIT_FRACTION of max on any axis
    //   Trigger CATCHUP (orbit mode): any axis in catch-up (see updateCatchup())
    //   Hold: no orbit updates, samples flagged 'limited' (excluded from fits).
    //   Release: no trigger for SPEED_RELEASE_S.
    // ---------------------------------------------------------------------
    enum class HoldReason : uint8_t { NONE, MOUNT, CATCHUP };
    bool mountAtLimit = false;
    bool speedHold = false;
    uint32_t lastLimitMs = 0;
    HoldReason holdReason = HoldReason::NONE;

    // ---------------------------------------------------------------------
    // Orbit steering state, per axis (index 0 = axis1, 1 = axis2)
    //   feedforward = orbit axis rates (cached prediction of the control step)
    //   PI on the position error (orbit + manual offset - mount)
    //   manual stick corrections: added to the output every tick and integrated
    //   into manualOffset, so the PI does not pull them back; reset on new orbit
    //   catch-up: axis runs at max speed in the latched direction catchDir
    // ---------------------------------------------------------------------
    static constexpr float PI_INTEGRAL_MAX = 0.002f;   // rad/s, limit of the I term (~0.11 deg/s)
    static constexpr float HALF_PI_F = 1.57079633f;

    bool  steering = false;           // true: orbit mode, false: simple integrator
    bool  predValid = false;          // cached prediction of this control step is valid
    float predAlt = 0.0f;             // rad, orbit prediction altitude (horizon check)
    float axTarget[2] = {0, 0};       // rad, orbit target in axis coordinates
    float axRate[2]   = {0, 0};       // rad/s, orbit target rate (feedforward)
    float axPos[2]    = {0, 0};       // rad, mount position in axis coordinates
    float axErr[2]    = {0, 0};       // rad, target + manual offset - mount (wrapped)
    float axInteg[2]  = {0, 0};       // rad/s, I term
    float axCmd[2]    = {0, 0};       // rad/s, commanded axis rate (ff + PI or catch-up)
    float manualOffset[2] = {0, 0};   // rad
    bool  axClamped[2] = {false, false};   // last output of the axis hit the clamp (anti-windup)
    int8_t catchDir[2] = {0, 0};      // 0 = tracking, +1/-1 = catch-up at max speed
    uint32_t lastPiMs = 0;
    #if SATELLITE_ORBIT_DEBUG == 1
    uint32_t lastPiDebugMs = 0;
    #endif


    static float wrapPi(float a) {
        const float PI_F = 3.14159265f, TWO_PI_F = 6.28318531f;
        while (a >  PI_F) a -= TWO_PI_F;
        while (a < -PI_F) a += TWO_PI_F;
        return a;
    }

    void resetSteeringState() {
        for (int i = 0; i < 2; i++) {
            axInteg[i] = axCmd[i] = manualOffset[i] = axErr[i] = 0.0f;
            axClamped[i] = false;
            catchDir[i] = 0;
        }
        lastPiMs = millis();
    }

    // ---------------------------------------------------------------------
    // Axis mapping: the only mount-type specific part of the steering.
    //   ALTAZ    : axis1 = azimuth, axis2 = altitude
    //   GEM / FORK: axis1 = hour angle (west +), axis2 = declination (north +)
    //     X = +HA rate (positive X = DIR_FORWARD = HA increasing / west),
    //     Y = -Dec rate (Y < 0 = north), pier side is handled in sendSlewCommands().
    //     h/d from getPosition() are sky coordinates on both pier sides and h is
    //     continuous through the meridian (wrap at +-180 below the pole -> wrapPi).
    // ---------------------------------------------------------------------
    bool predictOrbitAxes(float tNow) {
        OrbitAxisFrame f = equatorial ? OrbitAxisFrame::EQUATORIAL : OrbitAxisFrame::ALTAZ;
        if (!orbitModel.predictAxes(tNow, f, predAlt, axTarget[0], axTarget[1],
                                    &axRate[0], &axRate[1])) return false;
        return true;
    }

    void mountToAxes() {
        if (equatorial) {
            axPos[0] = cachedPos.h;
            axPos[1] = cachedPos.d;
        } else {
            axPos[0] = cachedPos.z;
            axPos[1] = cachedPos.a;
        }
    }

    // Requested axis speed of the normal PI (before clamping)
    float requestedRate(int i) const {
        return axRate[i] + SATELLITE_ORBIT_KP * axErr[i] + axInteg[i];
    }

    // Per-axis catch-up state (orbit mode). Uses only cached values.
    //   enter: requested speed >= SPEED_LIMIT_FRACTION of max -> latch its direction
    //   exit : axis reached / passed its target (wrapped error crossed 0, |err| < 90 deg
    //          so a lag beyond 180 deg cannot end it) and the target rate is reachable again
    // The command only depends on catchDir, so the direction never flips during catch-up.
    void updateCatchup(float maxSlewSpeedRad, float tNow) {
        float lim = SATELLITE_ORBIT_SPEED_LIMIT_FRACTION * maxSlewSpeedRad;
        for (int i = 0; i < 2; i++) {
            if (catchDir[i] == 0) {
                float req = requestedRate(i);
                if (maxSlewSpeedRad > 0.0f && fabsf(req) >= lim) {
                    catchDir[i] = (req > 0.0f) ? 1 : -1;
                    #if SATELLITE_ORBIT_DEBUG == 1
                    printCatchupDebug(i, true, tNow);
                    #endif
                }
            } else {
                float e = axErr[i];
                bool reached = catchDir[i] * e <= 0.0f && fabsf(e) < HALF_PI_F;
                if (reached && fabsf(axRate[i]) < lim) {
                    catchDir[i] = 0;
                    #if SATELLITE_ORBIT_DEBUG == 1
                    printCatchupDebug(i, false, tNow);
                    #endif
                }
            }
        }
        if (catchDir[0] != 0 || catchDir[1] != 0) mountAtLimit = true;   // -> speed hold
        #if SATELLITE_ORBIT_DEBUG != 1
        (void)tNow;
        #endif
    }

    // Switch between simple integrator and orbit steering
    void switchMode(bool toOrbit, float tNow) {
        steering = toOrbit;
        if (toOrbit) {
            resetSteeringState();   // PI starts on the current error
        } else {
            // Fallback: simple integrator continues with the last commanded speed
            trackingRateX = lastOutXNorm * cachedCos2 / referenceCos2;
            trackingRateY = lastOutYNorm;
        }
        #if SATELLITE_ORBIT_DEBUG == 1
        char buf[80];
        snprintf(buf, sizeof(buf), "ORBIT: mode=%s t=%.1fs", toOrbit ? "orbit" : "simple", tNow);
        Serial.println(buf);
        #else
        (void)tNow;
        #endif
    }

    // Feedforward + PI. Uses only cached values (no getPosition / predict).
    void piStep(float maxSlewSpeedRad, uint32_t now) {
        float dtc = (now - lastPiMs) * 0.001f;
        lastPiMs = now;
        if (dtc > 0.5f) dtc = 0.5f;                    // after a pause: no big integral jump

        for (int i = 0; i < 2; i++) {
            if (catchDir[i] != 0) {
                // Catch-up: max speed in the latched direction, integral frozen
                axCmd[i] = catchDir[i] * maxSlewSpeedRad;
                continue;
            }
            // Anti-windup: freeze the integral while this axis output is clamped
            if (!axClamped[i]) {
                axInteg[i] += SATELLITE_ORBIT_KI * axErr[i] * dtc;
                if (axInteg[i] >  PI_INTEGRAL_MAX) axInteg[i] =  PI_INTEGRAL_MAX;
                if (axInteg[i] < -PI_INTEGRAL_MAX) axInteg[i] = -PI_INTEGRAL_MAX;
            }
            axCmd[i] = requestedRate(i);
        }

        #if SATELLITE_ORBIT_DEBUG == 1
        if (now - lastPiDebugMs >= 1000) {
            lastPiDebugMs = now;
            char buf[200];
            snprintf(buf, sizeof(buf),
                "ORBIT: pi err ax1=%.3f ax2=%.3f deg | ff ax1=%.3f ax2=%.3f | pi ax1=%.4f ax2=%.4f deg/s | off=%.3f deg%s%s%s",
                axErr[0] * RAD_TO_DEG_F, axErr[1] * RAD_TO_DEG_F,
                axRate[0] * RAD_TO_DEG_F, axRate[1] * RAD_TO_DEG_F,
                (axCmd[0] - axRate[0]) * RAD_TO_DEG_F, (axCmd[1] - axRate[1]) * RAD_TO_DEG_F,
                sqrtf(manualOffsetSq()) * RAD_TO_DEG_F,
                speedHold ? " HOLD" : "",
                catchDir[0] != 0 ? " CU1" : "",
                catchDir[1] != 0 ? " CU2" : "");
            Serial.println(buf);
        }
        #endif
    }

    void updateSpeedHold(uint32_t now) {
        if (mountAtLimit) {
            lastLimitMs = now;
            if (!speedHold) {
                speedHold = true;
                holdReason = steering ? HoldReason::CATCHUP : HoldReason::MOUNT;
                #if SATELLITE_ORBIT_DEBUG == 1
                printSpeedHoldDebug(true, now);
                #endif
            }
            return;
        }
        if (!speedHold) return;
        if (now - lastLimitMs < SATELLITE_ORBIT_SPEED_RELEASE_S * 1000UL) return;
        speedHold = false;
        #if SATELLITE_ORBIT_DEBUG == 1
        printSpeedHoldDebug(false, now);
        #endif
        holdReason = HoldReason::NONE;
    }

    // Accumulated manual correction on the sky (rad^2), axis1 scaled by cos(axis2). Debug only.
    float manualOffsetSq() const {
        float ex = manualOffset[0] * cosf(axPos[1]);
        return ex * ex + manualOffset[1] * manualOffset[1];
    }

    // Sample the current alt/az (caller checks SATELLITE_ORBIT_SAMPLE_INTERVAL_MS)
    void sampleTrajectory(uint32_t now) {
        lastTrajectorySampleTime = now;

        const Coordinate &pos = cachedPos;      // read in controlStep() this tick

        size_t idx;
        if (trajectorySampleCount < TRAJECTORY_MAX_SAMPLES) {
            idx = trajectorySampleCount++;
        } else {
            idx = trajectoryRingIndex;
            trajectoryRingIndex = (trajectoryRingIndex + 1) % TRAJECTORY_MAX_SAMPLES;
        }
        // Precompute direction vector
        float ca = cosf(pos.a);

        trajectorySamples[idx].timeMs   = now - sessionStartMillis;
        trajectorySamples[idx].altitude = pos.a;
        trajectorySamples[idx].azimuth  = pos.z;
        trajectorySamples[idx].dx = ca * sinf(pos.z); //precompute vector
        trajectorySamples[idx].dy = ca * cosf(pos.z);
        trajectorySamples[idx].dz = sinf(pos.a);
        trajectorySamples[idx].limited = speedHold;
    }

    // i-th sample in chronological order (0 = oldest)
    const TrajectorySample& trajectoryAt(size_t i) const {
        size_t oldest = (trajectorySampleCount < TRAJECTORY_MAX_SAMPLES) ? 0 : trajectoryRingIndex;
        return trajectorySamples[(oldest + i) % TRAJECTORY_MAX_SAMPLES];
    }

    // First chronological index with timeMs >= tMs (binary search, samples are time ordered).
    // Returns trajectorySampleCount if no such sample exists.
    size_t indexAtOrAfter(uint32_t tMs) const {
        size_t lo = 0, hi = trajectorySampleCount;
        while (lo < hi) {
            size_t mid = (lo + hi) / 2;
            if (trajectoryAt(mid).timeMs < tMs) lo = mid + 1; else hi = mid;
        }
        return lo;
    }

    // Copy samples with tStartMs <= t <= tEndMs into dst (chronological, max maxN).
    // Returns number of samples copied.
    size_t copyRange(uint32_t tStartMs, uint32_t tEndMs, OrbitSample* dst, size_t maxN) const {
        size_t count = 0;
        for (size_t i = indexAtOrAfter(tStartMs); i < trajectorySampleCount && count < maxN; i++) {
            const TrajectorySample& s = trajectoryAt(i);
            if (s.timeMs > tEndMs) break;
            dst[count].t   = s.timeMs * 0.001f;
            dst[count].alt = s.altitude;
            dst[count].az  = s.azimuth;
            dst[count].dx  = s.dx; //vectors
            dst[count].dy  = s.dy;
            dst[count].dz  = s.dz;
            count++;
        }
        return count;
    }

    // Copy one stored sample (chronological index i) into an OrbitSample
    void copySample(size_t i, OrbitSample &dst) const {
        const TrajectorySample& s = trajectoryAt(i);
        dst.t   = s.timeMs * 0.001f;
        dst.alt = s.altitude;
        dst.az  = s.azimuth;
        dst.dx  = s.dx;
        dst.dy  = s.dy;
        dst.dz  = s.dz;
    }

    // Chronological index of the last 'limited' sample with tStartMs <= t <= tEndMs,
    // or trajectorySampleCount if there is none.
    size_t lastLimitedIndex(uint32_t tStartMs, uint32_t tEndMs) const {
        size_t found = trajectorySampleCount;
        for (size_t i = indexAtOrAfter(tStartMs); i < trajectorySampleCount; i++) {
            const TrajectorySample& s = trajectoryAt(i);
            if (s.timeMs > tEndMs) break;
            if (s.limited) found = i;
        }
        return found;
    }

    // Select anchor A window and copy it into anchorAWindow.
    // Rules: window starts after SATELLITE_ORBIT_DISCARD_START_S, at the first sample with
    // elevation >= SATELLITE_ORBIT_MIN_ANCHOR_ALT_DEG, but not earlier than
    // SATELLITE_ORBIT_MAX_BASELINE_S before now (anchor A moves forward on long passes).
    // Returns number of samples copied (0 = no usable anchor A yet).
    size_t selectAnchorA(uint32_t newestMs) {
        const uint32_t winMs  = SATELLITE_ORBIT_ANCHOR_A_WINDOW_S * 1000UL;
        const uint32_t maxBl  = SATELLITE_ORBIT_MAX_BASELINE_S * 1000UL;
        const uint32_t minBl  = SATELLITE_ORBIT_MIN_BASELINE_S * 1000UL;
        const float    minAlt = SATELLITE_ORBIT_MIN_ANCHOR_ALT_DEG * DEG_TO_RAD_F;

        uint32_t earliest = SATELLITE_ORBIT_DISCARD_START_S * 1000UL;
        // Anchor time is the window centre: centre >= newest - maxBaseline
        if (newestMs > maxBl + winMs / 2 && newestMs - maxBl - winMs / 2 > earliest) {
            earliest = newestMs - maxBl - winMs / 2;
        }
        // After a rejected fit anchor A starts at the failed midpoint or later
        if (anchorAEarliestMs > earliest) earliest = anchorAEarliestMs;

        // Anchor A starts after the last 'limited' sample (recorded while the mount
        // lagged, e.g. catch-up), so neither anchor A nor the quality midpoint
        // between A and B lies inside a speed limit hold
        size_t i = indexAtOrAfter(earliest);
        size_t lim = lastLimitedIndex(earliest, newestMs);
        if (lim < trajectorySampleCount) i = lim + 1;

        // First sample from there that is high enough
        while (i < trajectorySampleCount && trajectoryAt(i).altitude < minAlt) i++;
        if (i >= trajectorySampleCount) return 0;

        uint32_t startMs  = trajectoryAt(i).timeMs;
        uint32_t centreMs = startMs + winMs / 2;
        if (centreMs + minBl > newestMs) return 0;    // baseline too short yet

        return copyRange(startMs, startMs + winMs, anchorAWindow, ANCHOR_A_MAX_SAMPLES);
    }

    // True if updateOrbitFit() will attempt a fit at time 'now'
    // (also used to take a fresh sample right before the fit = anchor B)
    bool fitDue(uint32_t now) const {
        return (now - sessionStartMillis) >=
                   (SATELLITE_ORBIT_DISCARD_START_S + SATELLITE_ORBIT_MIN_BASELINE_S) * 1000UL &&
               (now - lastFitTime) >= SATELLITE_ORBIT_FIT_INTERVAL_MS;
    }

    // Periodic two-point orbit fit (anchor A smoothed, anchor B = newest raw sample).
    //   - hold (keep last orbit): speed limit active, newest sample 'limited',
    //     or newest sample below MIN_ANCHOR_ALT_DEG
    //   - quality: orbit vs. the stored sample halfway between A and B,
    //     > MAX_DEV_DEG -> orbit dropped (simple integrator), anchor A moved forward
    //   - failed fit (no root etc.): last orbit kept
    //   - accepted: orbit passes through the current mount position -> manual offset
    //     reset without jump, fallback cleared
    void updateOrbitFit(uint32_t now) {
        if (!fitDue(now)) return;
        lastFitTime = now;
        if (trajectorySampleCount == 0) return;
        if (speedHold) return;                        // reported once by updateSpeedHold()

        float tNow = (now - sessionStartMillis) * 0.001f;
        (void)tNow;  // suppress unused warning when debug is off
        size_t newestIdx = trajectorySampleCount - 1;
        const TrajectorySample& newest = trajectoryAt(newestIdx);
        if (newest.limited) return;
        if (newest.altitude < SATELLITE_ORBIT_MIN_ANCHOR_ALT_DEG * DEG_TO_RAD_F) {
            #if SATELLITE_ORBIT_DEBUG == 1
            if (orbitModel.isValid()) printLowHoldDebug(tNow);
            #endif
            return;                                   // keep last orbit
        }

        size_t nA = selectAnchorA(newest.timeMs);
        if (nA < 3) return;                           // no usable anchor A yet
        copySample(newestIdx, anchorB);

        // Quality check sample: halfway between anchor A centre and B
        float tA  = 0.5f * (anchorAWindow[0].t + anchorAWindow[nA - 1].t);
        uint32_t midMs = (uint32_t)(0.5f * (tA + anchorB.t) * 1000.0f);
        size_t midIdx = indexAtOrAfter(midMs);
        if (midIdx >= newestIdx) return;
        copySample(midIdx, midSample);

        bool ok = candidateModel.fit(anchorAWindow, nA, anchorB);
        lastDevDeg = -1.0f;
        if (ok) {
            lastDevDeg = candidateModel.deviation(midSample) * RAD_TO_DEG_F;
            if (lastDevDeg < 0.0f || lastDevDeg > SATELLITE_ORBIT_MAX_DEV_DEG) {
                // Track is not this circular orbit (plane turning, bad anchor A, systematic
                // error over a long baseline): drop the orbit -> simple integrator in this
                // control step (predValid false -> switchMode), and move anchor A forward
                // to the failed midpoint so the next fit uses a fresh, shorter baseline.
                orbitModel.reset();
                predValid = false;
                anchorAEarliestMs = trajectoryAt(midIdx).timeMs;
                #if SATELLITE_ORBIT_DEBUG == 1
                char buf[120];
                snprintf(buf, sizeof(buf),
                    "ORBIT: rejected t=%.1fs dev=%.3fdeg base=%.1fs -> simple, anchor A from t=%.1fs",
                    tNow, lastDevDeg, candidateModel.baselineS(), anchorAEarliestMs * 0.001f);
                Serial.println(buf);
                #endif
                return;
            }
        }

        if (ok) {
            orbitModel = candidateModel;
            // New orbit passes through the current mount position (anchor B, sampled this
            // control step) -> manual corrections are contained, error is ~0 now.
            // axErr was computed from the old orbit + offset: clear it for this
            // step's PI instead of a second predict().
            for (int i = 0; i < 2; i++) manualOffset[i] = axErr[i] = 0.0f;
        }

        #if SATELLITE_ORBIT_DEBUG == 1
        printFitDebug(ok, nA, tNow);
        #endif
    }

    #if SATELLITE_ORBIT_DEBUG == 1
    // Offset (deg) between the current orbit and the raw newest mount sample (1x predict)
    float offsetToNewestDeg() const {
        if (!orbitModel.isValid() || trajectorySampleCount == 0) return -1.0f;
        OrbitSample s;
        copySample(trajectorySampleCount - 1, s);
        return orbitModel.deviation(s) * RAD_TO_DEG_F;
    }

    void printFitDebug(bool ok, size_t nA, float tNow) {
        char buf[200];
        const SatelliteOrbitModel &m = candidateModel;
        if (m.isValid()) {
            snprintf(buf, sizeof(buf),
                "ORBIT: fit %s t=%.1fs nA=%u base=%.1fs sky=%.2fdeg alt=%.0fkm range=%.0fkm period=%.1fmin dev=%.3fdeg",
                ok ? "OK" : "FAILED", tNow, (unsigned)nA, m.baselineS(), m.skyAngle() * RAD_TO_DEG_F,
                m.altitudeM() * 0.001f, m.rangeM() * 0.001f, m.periodS() / 60.0f, lastDevDeg);
        } else {
            snprintf(buf, sizeof(buf), "ORBIT: fit FAILED t=%.1fs nA=%u base=%.1fs (keeping last orbit)",
                tNow, (unsigned)nA, m.baselineS());
        }
        Serial.println(buf);
    }

    void printLowHoldDebug(float tNow) {
        char buf[100];
        snprintf(buf, sizeof(buf), "ORBIT: hold t=%.1fs reason=low offNow=%.3fdeg",
            tNow, offsetToNewestDeg());
        Serial.println(buf);
    }

    // Printed once when the speed limit hold starts and once when it ends
    void printSpeedHoldDebug(bool start, uint32_t now) {
        char buf[100];
        float tNow = (now - sessionStartMillis) * 0.001f;
        if (start) {
            snprintf(buf, sizeof(buf), "ORBIT: hold t=%.1fs reason=limit(%s) offNow=%.3fdeg",
                tNow, holdReason == HoldReason::MOUNT ? "mount" : "catchup", offsetToNewestDeg());
        } else {
            snprintf(buf, sizeof(buf), "ORBIT: hold released t=%.1fs offNow=%.3fdeg",
                tNow, offsetToNewestDeg());
        }
        Serial.println(buf);
    }

    // Printed once when an axis enters and once when it leaves catch-up
    void printCatchupDebug(int i, bool start, float tNow) {
        char buf[100];
        if (start) {
            snprintf(buf, sizeof(buf), "ORBIT: catchup ax%d dir=%c t=%.1fs ff=%.3fdeg/s err=%.3fdeg",
                i + 1, catchDir[i] > 0 ? '+' : '-', tNow,
                axRate[i] * RAD_TO_DEG_F, axErr[i] * RAD_TO_DEG_F);
        } else {
            snprintf(buf, sizeof(buf), "ORBIT: catchup ax%d done t=%.1fs ff=%.3fdeg/s err=%.3fdeg",
                i + 1, tNow, axRate[i] * RAD_TO_DEG_F, axErr[i] * RAD_TO_DEG_F);
        }
        Serial.println(buf);
    }
    #endif // SATELLITE_ORBIT_DEBUG

#if SATELLITE_STATS_PRINT == 1
    void printStats(uint32_t now, float maxSlewSpeedRad) {
        float tNow = (now - sessionStartMillis) * 0.001f;

        const char* modeStr = steering ? "orbit" : "simple";

        float altKm = 0.0f, rangeKm = 0.0f;
        if (orbitModel.isValid()) {
            altKm = orbitModel.altitudeM() * 0.001f;
            rangeKm = orbitModel.rangeM() * 0.001f;
        }

        float skySpeedDegS = 0.0f;
        if (predValid) {
            // Orbit axis rates (ALTAZ or HA/Dec), axis1 scaled by cos(axis2)
            float r1 = axRate[0] * cosf(axTarget[1]);
            skySpeedDegS = sqrtf(axRate[1] * axRate[1] + r1 * r1) * RAD_TO_DEG_F;
        } else {
            float tx = lastOutXNorm * maxSlewSpeedRad;
            float ty = lastOutYNorm * maxSlewSpeedRad;
            skySpeedDegS = sqrtf(tx * tx + ty * ty) * RAD_TO_DEG_F;
        }

        // Current mount altitude in degrees (cachedPos was updated in controlStep)
        float currentAltDeg = cachedPos.a * RAD_TO_DEG_F;

        float tca = 0.0f, maxElevDeg = 0.0f;
        bool hasClosest = orbitModel.computeClosestApproach(tNow, tca, maxElevDeg);

        if (hasClosest) {
            // Detect a jump to the next orbit pass.
            // The period is always much larger than 600s , so a jump in TCA
            // greater than this threshold means satellite is now past the real peak.
            if (hasClosestInfo && (tca - lastClosestTca) > 600.0f) {
                // Use the stored last actual pass
                tca = lastClosestTca;
                maxElevDeg = lastClosestMaxElevDeg;
            } else {
                // Still approaching the current pass (or first call)
                lastClosestTca = tca;
                lastClosestMaxElevDeg = maxElevDeg;
                hasClosestInfo = true;
            }
        }

        if (hasClosest) {
            float tcaRel = tca - tNow;    // positive before, negative after
            Serial.printf(
                "SAT: t=%.1fs mode=%s || alt(closest)=%.1f(%.1f)deg tca=%+.0fs || alt=%.0fkm range=%.0fkm sky=%.2f deg/s\n",
                tNow, modeStr, currentAltDeg, maxElevDeg, tcaRel, altKm, rangeKm, skySpeedDegS
            );
        } else {
            Serial.printf(
                "SAT: t=%.1fs mode=%s || alt(closest)=n/a tca=n/a || alt=%.0fkm range=%.0fkm sky=%.2f deg/s\n",
                tNow, modeStr, altKm, rangeKm, skySpeedDegS
            );
        }
    }
#endif // SATELLITE_STATS_PRINT == 1

#endif // SATELLITE_ORBIT_ENABLE

};
