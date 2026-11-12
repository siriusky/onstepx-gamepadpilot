// SatelliteOrbitModel.cpp
// Circular orbit determination from two anchors of an alt/az track, and prediction.
// See SatelliteOrbitModel.h for the method overview.

#include "SatelliteOrbitModel.h"
#include <math.h>

// ============================================================================
// Constants and vector helpers (file-local)
// ============================================================================
namespace {
    constexpr double R_EARTH   = 6371000.0;        // m, spherical Earth
    constexpr double MU_EARTH  = 3.986004418e14;   // m^3/s^2
    constexpr double W_EARTH   = 7.2921150e-5;     // rad/s, sidereal rotation rate
    constexpr float  W_EARTH_F = 7.2921150e-5f;
    constexpr double TWO_PI_D  = 6.283185307179586;

    // Root search range (altitude above ground). Linear bisection on the range rho
    // between both ends (single sign change for realistic tracks, no grid).
    constexpr double H_MIN       = 10000.0;        // m
    constexpr double H_MAX       = 15000000.0;     // m
    // Range resolution after BISECT_ITER steps: (rho(H_MAX) - rho(H_MIN)) / 2^(BISECT_ITER + 1)
    //   bracket 14990 km (zenith) .. 20042 km (horizon)
    //   BISECT_ITER 18 -> 29..38 m,  20 -> 7..10 m,  22 -> 1.8..2.4 m
    // Effect on prediction: pointing error ~ 1.5 * res * omegaSat * T / rho
    //   e.g. res 38 m, T 300 s, rho 400 km -> ~0.003 deg (negligible)
    // Cost: 2 + BISECT_ITER evaluations of the root function per fit (double math).
    constexpr int    BISECT_ITER = 18;

    // Fit sanity limits
    constexpr double MIN_BASELINE_S     = 5.0;     // s between anchors (engine uses a larger config value)
    constexpr double MIN_SKY_SEPARATION = 1.0e-3;  // rad (~0.06 deg) between anchors on the sky

    typedef OrbitVec3 V3;
    inline V3 add(V3 a, V3 b)     { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
    inline V3 mul(V3 a, double s) { return { a.x * s, a.y * s, a.z * s }; }
    inline double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    inline V3 cross(V3 a, V3 b)   { return { a.y * b.z - a.z * b.y,
                                             a.z * b.x - a.x * b.z,
                                             a.x * b.y - a.y * b.x }; }
    inline double norm(V3 a)      { return sqrt(dot(a, a)); }
}

// ============================================================================
// Setup / getters
// ============================================================================

void SatelliteOrbitModel::setLatitude(float latRad) {
    latitude = latRad;
    sinLat = sin(latitude);
    cosLat = cos(latitude);
    sinLatF = (float)sinLat;
    cosLatF = (float)cosLat;
}

void SatelliteOrbitModel::reset() {
    valid = false;
    fitRange = 0.0f;
    fitBaseline = 0.0f;
    fitSweep = 0.0f;
    fitSkyAngle = 0.0f;
    fitRootCount = 0;
}

float SatelliteOrbitModel::periodS() const {
    if (!valid || omegaSat <= 0.0f) return 0.0f;
    return (float)(TWO_PI_D / (double)omegaSat);
}

// ============================================================================
// Anchor A smoothing: quadratic least-squares fit of each direction component,
// evaluated at time tr (window centre). Returns the unit direction at tr.
// ============================================================================
bool SatelliteOrbitModel::smoothDirection(const OrbitSample* s, size_t n, double tr,
                                          OrbitVec3 &d) {
    double S0 = 0, S1 = 0, S2 = 0, S3 = 0, S4 = 0;     // sums of tau^k
    V3 B0 = {0, 0, 0}, B1 = {0, 0, 0}, B2 = {0, 0, 0}; // sums of p * tau^k

    for (size_t i = 0; i < n; i++) {
        double tau  = (double)s[i].t - tr;
        double tau2 = tau * tau;
        V3 p = { s[i].dx, s[i].dy, s[i].dz };

        S0 += 1.0;  S1 += tau;  S2 += tau2;  S3 += tau2 * tau;  S4 += tau2 * tau2;
        B0 = add(B0, p);
        B1 = add(B1, mul(p, tau));
        B2 = add(B2, mul(p, tau2));
    }

    // Normal equations M * [a b c]^T = [B0 B1 B2]^T, M symmetric:
    //   | S0 S1 S2 |
    //   | S1 S2 S3 |
    //   | S2 S3 S4 |
    // Inverse via cofactors; only the row for a (value at tau = 0) is needed.
    double c00 = S2 * S4 - S3 * S3;
    double c01 = S2 * S3 - S1 * S4;
    double c02 = S1 * S3 - S2 * S2;
    double det = S0 * c00 + S1 * c01 + S2 * c02;
    if (fabs(det) < 1e-12) return false;

    V3 a = mul(add(add(mul(B0, c00), mul(B1, c01)), mul(B2, c02)), 1.0 / det);
    double la = norm(a);
    if (la < 1e-9) return false;
    d = mul(a, 1.0 / la);
    return true;
}

// Range from observer to a point at altitude h along a line of sight (sinAlt = sin(elevation)).
double SatelliteOrbitModel::rhoFromAltitude(double h, double sinAlt) {
    double rs = R_EARTH * sinAlt;
    double rh = R_EARTH + h;
    return -rs + sqrt(rs * rs + rh * rh - R_EARTH * R_EARTH);
}

// Rotate v about the Earth axis k = (0, cosLat, sinLat) by angle (Rodrigues)
OrbitVec3 SatelliteOrbitModel::rotateAboutEarthAxis(const OrbitVec3 &v, double angle) const {
    V3 k = { 0.0, cosLat, sinLat };
    double c = cos(angle), s = sin(angle);
    return add(add(mul(v, c), mul(cross(k, v), s)), mul(k, dot(k, v) * (1.0 - c)));
}

// Two-point equation. For candidate range rhoB to anchor B:
//   SB = O + rhoB*dB,  r = |SB|
//   SA = OA + rhoA*dA  with |SA| = r   (same circular orbit radius)
//   f  = angle(SA, SB) - sqrt(mu/r^3)*dt   (arc swept must match Kepler)
// Root f = 0 -> true range.
double SatelliteOrbitModel::twoPointFunction(const TwoPointCtx &c, double rhoB,
                                             OrbitVec3 *SAout, OrbitVec3 *SBout) {
    V3 SB = add(c.O, mul(c.dB, rhoB));
    double r = norm(SB);
    double sinAltA = dot(c.OA, c.dA) / R_EARTH;   // rotation keeps local elevation
    double rhoA = rhoFromAltitude(r - R_EARTH, sinAltA);
    V3 SA = add(c.OA, mul(c.dA, rhoA));

    if (SAout) *SAout = SA;
    if (SBout) *SBout = SB;

    double ang = atan2(norm(cross(SA, SB)), dot(SA, SB));
    return ang - sqrt(MU_EARTH / (r * r * r)) * c.dt;
}

// Linear bisection on the range rho between altitude H_MIN and H_MAX along the
// line of sight dB (dB.z = sin(alt)). Same sign at both ends -> no solution -> rejected.
bool SatelliteOrbitModel::scanRoots(const TwoPointCtx &c, double &rhoOut) {
    fitRootCount = 0;

    double lo  = rhoFromAltitude(H_MIN, c.dB.z);
    double hi  = rhoFromAltitude(H_MAX, c.dB.z);
    double fLo = twoPointFunction(c, lo);
    double fHi = twoPointFunction(c, hi);
    if ((fLo < 0.0) == (fHi < 0.0)) return false;

    for (int k = 0; k < BISECT_ITER; k++) {
        double mid  = 0.5 * (lo + hi);
        double fMid = twoPointFunction(c, mid);
        if ((fMid < 0.0) == (fLo < 0.0)) { lo = mid; fLo = fMid; }
        else                             { hi = mid; }
    }

    rhoOut = 0.5 * (lo + hi);
    fitRootCount = 1;
    return true;
}

// ============================================================================
// fit() - circular orbit through anchor A (smoothed) and anchor B (raw sample)
// ============================================================================
bool SatelliteOrbitModel::fit(const OrbitSample* a, size_t nA, const OrbitSample &b) {
    valid = false;
    fitRootCount = 0;
    fitRange = 0.0f;
    fitBaseline = 0.0f;
    fitSweep = 0.0f;
    fitSkyAngle = 0.0f;
    if (a == nullptr || nA < 3) return false;

    // --- Anchor A: quadratic smoothing, evaluated at window centre ----------------
    double tA = 0.5 * ((double)a[0].t + (double)a[nA - 1].t);
    V3 dAloc;
    if (!smoothDirection(a, nA, tA, dAloc)) return false;

    // --- Anchor B: raw sample ("now"), the orbit passes exactly through it ---------
    double tB = b.t;
    V3 dB = { b.dx, b.dy, b.dz };

    double dt = tB - tA;
    if (dt < MIN_BASELINE_S) return false;
    if (dAloc.z <= 0.0 || dB.z <= 0.0) return false;   // both above horizon

    double skyAngle = atan2(norm(cross(dAloc, dB)), dot(dAloc, dB));
    fitSkyAngle  = (float)skyAngle;
    fitBaseline  = (float)dt;
    if (skyAngle < MIN_SKY_SEPARATION) return false;   // anchors too close on the sky

    // --- Express A in the inertial frame aligned with local ENU at tB -------------
    // At tA the observer (and its local axes) was rotated by -wE*dt about the Earth axis.
    TwoPointCtx c;
    c.O  = { 0.0, 0.0, R_EARTH };
    c.OA = rotateAboutEarthAxis(c.O, -W_EARTH * dt);
    c.dA = rotateAboutEarthAxis(dAloc, -W_EARTH * dt);
    c.dB = dB;
    c.dt = dt;

    // --- Range to B: root search ----------------------------------------------------
    double rhoB;
    if (!scanRoots(c, rhoB)) return false;

    // --- Orbit parameters: u at B, plane from A x B, motion A -> B ------------------
    V3 SA, SB;
    twoPointFunction(c, rhoB, &SA, &SB);
    double r = norm(SB);
    V3 h = cross(SA, SB);
    double lh = norm(h);
    if (r < R_EARTH || lh < 1e-3) return false;

    V3 u   = mul(SB, 1.0 / r);
    V3 nrm = mul(h, 1.0 / lh);
    V3 v   = cross(nrm, u);                   // unit, direction of motion (A -> B)

    tRef        = (float)tB;
    orbitRadius = (float)r;
    omegaSat    = (float)sqrt(MU_EARTH / (r * r * r));
    ux = (float)u.x; uy = (float)u.y; uz = (float)u.z;
    vx = (float)v.x; vy = (float)v.y; vz = (float)v.z;
    fitRange = (float)rhoB;
    fitSweep = (float)atan2(lh, dot(SA, SB));
    valid = true;
    return true;
}

// ============================================================================
// Quality check: chord distance (rad, = angle for small values) between the
// orbit and one sample. 1x predict, 1x sqrtf.
// ============================================================================
float SatelliteOrbitModel::deviation(const OrbitSample &s) const {
    float pAlt, pAz;
    if (!predict(s.t, pAlt, pAz)) return -1.0f;
    float ca = cosf(pAlt);
    float ex = ca * sinf(pAz) - s.dx;
    float ey = ca * cosf(pAz) - s.dy;
    float ez = sinf(pAlt)     - s.dz;
    return sqrtf(ex * ex + ey * ey + ez * ez);
}

// ============================================================================
// Step 5 - prediction (float, cheap enough for every tick)
// ============================================================================

// Line of sight L (local ENU, m) at time t, optional derivative Ld (m/s), incl. Earth rotation.
bool SatelliteOrbitModel::predictLos(float t, float *L, float *Ld) const {
    if (!valid) return false;

    float tau   = t - tRef;
    float theta = omegaSat * tau;
    float ct = cosf(theta), st = sinf(theta);

    // Position in the inertial frame (aligned with local ENU at tRef)
    float sx = orbitRadius * (ct * ux + st * vx);
    float sy = orbitRadius * (ct * uy + st * vy);
    float sz = orbitRadius * (ct * uz + st * vz);

    // Earth rotation: the local frame has turned by +wE*tau about k,
    // so the inertial vector appears rotated by -wE*tau (Rodrigues formula).
    // k = (0, cosLat, sinLat)
    float ky = cosLatF, kz = sinLatF;
    float phi = -W_EARTH_F * tau;
    float cp = cosf(phi), sp = sinf(phi);
    float omc = 1.0f - cp;
    float kdots = ky * sy + kz * sz;
    // Earth-centre vector of the satellite in the local frame (k x s = (ky*sz - kz*sy, kz*sx, -ky*sx))
    float px = sx * cp + (ky * sz - kz * sy) * sp;
    float py = sy * cp + (kz * sx) * sp + ky * kdots * omc;
    float pz = sz * cp + (-ky * sx) * sp + kz * kdots * omc;

    // Line of sight from observer at (0, 0, R)
    L[0] = px;
    L[1] = py;
    L[2] = pz - EARTH_RADIUS_F;

    if (Ld == nullptr) return true;

    // --- Analytic rate (no extra trig) -----------------------------------------
    // Inertial velocity: Sdot = r*w*(-sin(theta)*u + cos(theta)*v)
    float rw = orbitRadius * omegaSat;
    float wx = rw * (-st * ux + ct * vx);
    float wy = rw * (-st * uy + ct * vy);
    float wz = rw * (-st * uz + ct * vz);
    // Rotate it like the position, then add the frame rotation term:
    //   Ldot = Rot(Sdot) - wE * (k x P),  P = rotated Earth-centre vector
    float kdotw = ky * wy + kz * wz;
    Ld[0] = wx * cp + (ky * wz - kz * wy) * sp                 - W_EARTH_F * (ky * pz - kz * py);
    Ld[1] = wy * cp + (kz * wx) * sp  + ky * kdotw * omc       - W_EARTH_F * (kz * px);
    Ld[2] = wz * cp + (-ky * wx) * sp + kz * kdotw * omc       - W_EARTH_F * (-ky * px);
    return true;
}

// a1 = atan2(x, y) (-pi..pi), a2 = atan2(z, h) with h = sqrt(x^2 + y^2), + rates.
// Near the a1 pole h -> 0 and the a1 rate grows without bound: h is limited to
// ~0.006 deg from the pole so the result stays finite (and very large).
bool SatelliteOrbitModel::losToAngles(const float *L, const float *Ld, float &a1, float &a2,
                                      float *r1, float *r2) {
    float x = L[0], y = L[1], z = L[2];
    float h2  = x * x + y * y;                // part perpendicular to the pole, squared
    float ln2 = h2 + z * z;
    if (ln2 < 1.0f) return false;
    float ln = sqrtf(ln2);

    float s2 = z / ln;
    if (s2 > 1.0f) s2 = 1.0f; else if (s2 < -1.0f) s2 = -1.0f;
    a2 = asinf(s2);
    a1 = atan2f(x, y);

    if (Ld == nullptr || (r1 == nullptr && r2 == nullptr)) return true;

    float hMin2 = ln2 * 1.0e-8f;
    if (h2 < hMin2) h2 = hMin2;
    float h = sqrtf(h2);
    float hDot = x * Ld[0] + y * Ld[1];       // = h * dh/dt
    if (r2) *r2 = (h2 * Ld[2] - z * hDot) / (h * ln2);
    if (r1) *r1 = (y * Ld[0] - x * Ld[1]) / h2;
    return true;
}

bool SatelliteOrbitModel::predict(float t, float &alt, float &az,
                                  float *altRate, float *azRate) const {
    float L[3], Ld[3];
    bool wantRates = (altRate != nullptr || azRate != nullptr);
    if (!predictLos(t, L, wantRates ? Ld : nullptr)) return false;
    // ENU: az = atan2(east, north), alt = asin(up / |L|)
    return losToAngles(L, wantRates ? Ld : nullptr, az, alt, azRate, altRate);
}

bool SatelliteOrbitModel::predictAxes(float t, OrbitAxisFrame frame, float &alt,
                                      float &ax1, float &ax2,
                                      float *ax1Rate, float *ax2Rate) const {
    if (frame == OrbitAxisFrame::ALTAZ) {
        if (!predict(t, alt, ax1, ax2Rate, ax1Rate)) return false;
        ax2 = alt;
        return true;
    }

    float L[3], Ld[3];
    bool wantRates = (ax1Rate != nullptr || ax2Rate != nullptr);
    if (!predictLos(t, L, wantRates ? Ld : nullptr)) return false;

    // Altitude (horizon check) from the unrotated line of sight
    float ln2 = L[0] * L[0] + L[1] * L[1] + L[2] * L[2];
    if (ln2 < 1.0f) return false;
    float sAlt = L[2] / sqrtf(ln2);
    if (sAlt > 1.0f) sAlt = 1.0f; else if (sAlt < -1.0f) sAlt = -1.0f;
    alt = asinf(sAlt);

    // ENU -> equatorial frame (W, M, P), rotation about the East axis by the colatitude:
    //   W = (-1, 0, 0)                west
    //   M = (0, -sinLat, cosLat)      equator on the meridian (HA = 0, Dec = 0)
    //   P = (0,  cosLat, sinLat)      celestial pole
    // HA = atan2(L.W, L.M) (west positive), Dec = asin(L.P / |L|)
    float E[3], Ed[3];
    E[0] = -L[0];
    E[1] = -sinLatF * L[1] + cosLatF * L[2];
    E[2] =  cosLatF * L[1] + sinLatF * L[2];
    if (wantRates) {
        Ed[0] = -Ld[0];
        Ed[1] = -sinLatF * Ld[1] + cosLatF * Ld[2];
        Ed[2] =  cosLatF * Ld[1] + sinLatF * Ld[2];
    }
    return losToAngles(E, wantRates ? Ed : nullptr, ax1, ax2, ax1Rate, ax2Rate);
}

// ============================================================================
// Additional Info: TCA and Max elevation
// approximates earth rotation
// Condition for closest approach: (s - observer) · satellite_velocity = 0
// For circular orbit this reduces to tan(theta) = vz / uz,
// where theta is the orbital phase measured from the u vector.
// ============================================================================
bool SatelliteOrbitModel::computeClosestApproach(float tNow,
                                                 float &tca,
                                                 float &maxElevationDeg) const {
    if (!valid) return false;

    // ---- Step 1: fixed-Earth TCA (observer fixed in inertial space) ----
    float zProj = sqrtf(uz * uz + vz * vz);
    if (zProj < 1e-6f) return false;   // orbit plane edge case

    // Orbital phase where the satellite is on the +z side (nearest to observer)
    float theta = atan2f(vz, uz);

    float period = 2.0f * 3.14159265358979f / omegaSat;
    float tcaRel = theta / omegaSat;          // seconds relative to tRef

    // Move to the first future closest approach
    while (tcaRel < tNow - tRef) {
        tcaRel += period;
    }

    tca = tRef + tcaRel;

    // ---- Step 2: evaluate elevation at that TCA with Earth rotation ----
    float alt, az;
    if (!predict(tca, alt, az)) return false;   // no rates -> cheap

    maxElevationDeg = alt * 57.29577951308232f; // radians -> degrees
    return true;
}
