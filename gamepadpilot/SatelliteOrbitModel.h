#pragma once

// ============================================================================
// SatelliteOrbitModel - learns a circular satellite orbit from recorded alt/az
// samples and predicts alt/az for any time of the pass.
//
// Pure math, no OnStepX / Arduino dependencies.
// fit() - circular orbit through two anchors:
//   A: older anchor, quadratic smoothing centred on its window
//   B: current raw sample ("now"). The orbit passes exactly through B.
//   Altitude follows from the arc swept between A and B (Kepler), so it gets
//   more precise with a longer baseline. Root search: linear bisection H_MIN..H_MAX.
// deviation() - quality check against one sample (halfway between A and B)
// predict() - t -> alt/az (+ rates), incl. Earth rotation
//
// Conventions:
//   Times are seconds relative to an arbitrary plugin-internal zero.
//   Angles in radians. Azimuth 0 = north, east positive, west negative
//   (same as OnStepX mount.getPosition().z). Output az is in -pi..pi.
//   Local frame: East-North-Up axes, origin at Earth centre.
// ============================================================================

#include <stddef.h>
#include <stdint.h>

struct OrbitSample {
    float t;     // seconds (relative)
    float alt;   // rad
    float az;    // rad
    // Precomputed direction vector (ENU unit vector).
    // Must be filled by the caller before fit().
    float dx, dy, dz;
};

// Small double-precision vector used inside fit()
struct OrbitVec3 { double x, y, z; };

// Axis frame for predictAxes()
//   ALTAZ      : axis1 = azimuth (0 = north, east positive), axis2 = altitude
//   EQUATORIAL : axis1 = hour angle (0 = meridian, west positive, -pi..pi),
//                axis2 = declination (north positive). Fixed to the Earth like altaz
//                Same convention as OnStepX h/d.
enum class OrbitAxisFrame : uint8_t { ALTAZ = 0, EQUATORIAL = 1 };

class SatelliteOrbitModel {
public:
    // Site latitude (rad). Must be set before fit().
    void setLatitude(float latRad);

    // Invalidate the current orbit.
    void reset();

    // Fit circular orbit through anchor A and anchor B.
    //   a, nA : chronological samples around the old anchor (>= 3), anchor time = window centre
    //   b     : newest raw sample ("now"), reference time of the orbit
    // Returns true on success. On failure the orbit is invalidated.
    bool fit(const OrbitSample* a, size_t nA, const OrbitSample &b);

    // Distance (rad, chord ~ angle) between the orbit at s.t and the sample direction.
    // Returns -1 if no valid orbit. Cost: 1x predict.
    float deviation(const OrbitSample &s) const;

    // Predict alt/az at time t. Returns false if no valid orbit.
    // alt < 0 means the satellite is below the horizon.
    // Optional: altRate / azRate (rad/s) at the same t, exact analytic derivative
    // (incl. Earth rotation). azRate is the azimuth axis rate,
    // it becomes very large (but finite) for a pass through the zenith.
    bool predict(float t, float &alt, float &az,
                 float *altRate = nullptr, float *azRate = nullptr) const;

    // Predict in mount axis coordinates (see OrbitAxisFrame) + optional axis rates (rad/s).
    // alt is always returned (horizon check). EQUATORIAL: the line of sight is rotated
    // once about the East axis by the colatitude (~12 extra multiplies, no extra trig).
    // ax1Rate becomes very large (but finite) near the axis1 pole (zenith resp.
    // celestial pole), like azRate in predict().
    bool predictAxes(float t, OrbitAxisFrame frame, float &alt, float &ax1, float &ax2,
                     float *ax1Rate = nullptr, float *ax2Rate = nullptr) const;

    // Lightweight compromise: fixed-Earth TCA, but elevation computed with Earth
    // rotation at that time. Accurate enough for stats/display.
    bool computeClosestApproach(float tNow,
                                float &tca,
                               float &maxElevationDeg) const;

    // --- State / debug getters ---------------------------------------------
    bool  isValid() const        { return valid; }
    float altitudeM() const      { return orbitRadius - EARTH_RADIUS_F; }
    float periodS() const;
    float rangeM() const         { return fitRange; }      // range at tRef
    float refTime() const        { return tRef; }
    uint8_t rootCount() const    { return fitRootCount; }  // last fit: 1 = root found, 0 = none
    float baselineS() const      { return fitBaseline; }   // time between anchors (s)
    float sweptAngle() const     { return fitSweep; }      // arc around the Earth centre (rad)
    float skyAngle() const       { return fitSkyAngle; }   // anchor separation seen by observer (rad)

    static constexpr float EARTH_RADIUS_F = 6371000.0f;

private:
    // Two-point equation context (inertial frame, aligned with local ENU at tB)
    struct TwoPointCtx {
        OrbitVec3 O;     // observer position at tB
        OrbitVec3 OA;    // observer position at tA (rotated into the tB frame)
        OrbitVec3 dA;    // line of sight at tA (rotated into the tB frame)
        OrbitVec3 dB;    // line of sight at tB
        double dt;       // tB - tA (s)
    };

    static bool smoothDirection(const OrbitSample* s, size_t n, double tr, OrbitVec3 &d);
    static double twoPointFunction(const TwoPointCtx &c, double rhoB,
                                   OrbitVec3 *SA = nullptr, OrbitVec3 *SB = nullptr);
    static double rhoFromAltitude(double h, double sinAlt);
    bool scanRoots(const TwoPointCtx &c, double &rhoOut);

    // Line of sight L (observer -> satellite, local ENU, m) at time t and optionally
    // its time derivative Ld (m/s). Shared by predict() and predictAxes().
    bool predictLos(float t, float *L, float *Ld) const;

    // Angles of a direction vector (x, y, z): a1 = atan2(x, y), a2 = asin(z/|L|),
    // plus optional analytic rates from Ld. Returns false for |L| ~ 0.
    static bool losToAngles(const float *L, const float *Ld, float &a1, float &a2,
                            float *r1, float *r2);

    // Rotate v about the Earth axis k = (0, cosLat, sinLat) by angle (rad)
    OrbitVec3 rotateAboutEarthAxis(const OrbitVec3 &v, double angle) const;

    // Site
    double latitude = 0.0;
    double sinLat = 0.0, cosLat = 1.0;
    float  sinLatF = 0.0f, cosLatF = 1.0f;   // float copies for predict()

    // Orbit state (float is sufficient for prediction)
    bool  valid = false;
    float tRef = 0.0f;           // reference time (s)
    float orbitRadius = 0.0f;    // m, from Earth centre
    float omegaSat = 0.0f;       // rad/s
    float ux = 0, uy = 0, uz = 0;   // in-plane unit vector (sat at tRef)
    float vx = 0, vy = 0, vz = 0;   // in-plane unit vector (direction of motion)

    // Debug info from last fit
    float   fitRange = 0.0f;
    float   fitBaseline = 0.0f;
    float   fitSweep = 0.0f;
    float   fitSkyAngle = 0.0f;
    uint8_t fitRootCount = 0;
};
