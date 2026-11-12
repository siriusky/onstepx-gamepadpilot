# SATELLITE PATH PREDICTION FROM MANUAL TRACKING - METHOD OVERVIEW (v2, two-point fit)

**Files:**
- SatelliteOrbitModel.h/.cpp   (pure math, no OnStepX/Arduino dependency)
- SatelliteTrackingEngine.h    (sampling, anchor selection, steering, PI)

## GOAL
Learn a satellite's orbit from the mount's own movement while the user follows it
with the gamepad (alt/az + time). Then predict alt/az (or HA/Dec) and their rates
for any time of the pass, and steer the mount along that path.

## ASSUMPTIONS
- Circular orbit, spherical Earth, refraction already handled by OnStepX.
- Earth rotation is included.
- Only the site latitude is needed. No longitude, no absolute time, no TLE.
- All times are relative seconds (session start = button release).
- Valid altitude range of the solver: 10 km .. 15000 km (LEO, low MEO).

## CONSTANTS
- $R = 6371000$ m            Earth radius
- $\mu = 3.986004418\times 10^{14}$ m³/s²
- $\omega_E = 7.2921150\times 10^{-5}$ rad/s   Earth rotation rate (sidereal)
- $\varphi$  = site latitude

## FRAME
Local East-North-Up (ENU) axes, origin at the Earth's centre.
- Observer:           $\mathbf{O} = (0, 0, R)$
- Earth axis (unit):  $\mathbf{k} = (0, \cos\varphi, \sin\varphi)$
- Azimuth from north, east positive (same as OnStepX `getPosition().z`).
- Direction from alt/az: $\mathbf{d} = (\cos alt \cdot \sin az, \cos alt \cdot \cos az, \sin alt)$

## SESSION OVERVIEW
1. Button held ≥ `SATELLITE_TRACKING_BUTTON_HOLD_TIME` (2.5 s): stick speeds are
   recorded, the mount still moves normally with the stick.
2. Button released: session $t = 0$. The SIMPLE INTEGRATOR steers right away
   (average recorded stick speed as baseline). The mount alt/az is sampled every
   `SAMPLE_INTERVAL_MS` (500 ms) into a ring buffer (`MAX_SAMPLES = 600` = 5 min).
3. From about $t = 13$ s: two-point ORBIT FIT every `FIT_INTERVAL_MS` (1 s).
   As soon as a valid orbit exists, ORBIT STEERING takes over from the integrator.
4. Tracking stops when the mount or the predicted orbit goes below alt 0.

## TWO-POINT FIT

Idea: two lines of sight, A (older) and B (now), at a known time apart. For a
circular orbit, the arc swept between them must equal the Kepler angular rate
times the elapsed time. Only one range satisfies this.

### ANCHOR A (older point, smoothed)
- Window of `ANCHOR_A_WINDOW_S` (4 s) starting at the first sample that is
    - ≥ `DISCARD_START_S` (5 s) into the session (start is least accurate),
    - ≥ `MIN_ANCHOR_ALT_DEG` (15 deg) above the horizon,
    - after the last sample recorded during a speed limit hold,
    - not before `anchorAEarliest` (moved forward after a rejected fit),
    - with its centre no more than `MAX_BASELINE_S` (60 s) before now
      (anchor A slides forward on long passes).
- Quadratic least squares $x,y,z(\tau) = \mathbf{a} + \mathbf{b}\tau + \mathbf{c}\tau^2$, $\tau = t - t_{centre}$,
  closed-form normal equations; direction $\mathbf{d}_A = \mathbf{a}/|\mathbf{a}|$ at the window centre.
- Needed: ≥ 3 samples and centre + `MIN_BASELINE_S` (6 s) ≤ now.
  → first orbit at about 5 + 4/2 + 6 = 13 s (if the satellite is above 15 deg).

### ANCHOR B (now, raw)
- A fresh sample is taken right before each fit, so B = current mount position.
- No smoothing: the orbit passes exactly through B. Therefore a new orbit causes
  no jump, and manual corrections made up to now are kept in the new orbit.

### EARTH ROTATION
$\Delta t = t_B - t_A$. The observer at $t_A$ is rotated into the frame of $t_B$ about $\mathbf{k}$ by
$-\omega_E \Delta t$ (Rodrigues):  $\mathbf{O}_A = \mathrm{rot}(\mathbf{O}, -\omega_E \Delta t)$,  $\mathbf{d}_A' = \mathrm{rot}(\mathbf{d}_A, -\omega_E \Delta t)$.
Everything below is in this (non-rotating) frame, aligned with ENU at $t_B$.

### ROOT FUNCTION  $f(\rho_B)$    (double precision)
$$\begin{aligned}
\mathbf{S}_B &= \mathbf{O} + \rho_B \mathbf{d}_B, & r &= |\mathbf{S}_B| \\
\rho_A &= \rho(r - R, \sin\alpha_A) & &\text{same orbit radius for A,} \\
& & \sin\alpha_A &= (\mathbf{O}_A \cdot \mathbf{d}_A') / R \\
\mathbf{S}_A &= \mathbf{O}_A + \rho_A \mathbf{d}_A' \\
f &= \angle(\mathbf{S}_A, \mathbf{S}_B) - \sqrt{\frac{\mu}{r^3}} \, \Delta t
\end{aligned}$$
with
$$\rho(h, \sin\alpha) = -R \sin\alpha + \sqrt{(R \sin\alpha)^2 + (R+h)^2 - R^2}.$$

### ROOT SEARCH
- Bracket $\rho(H_{min}=10 \text{ km})$ .. $\rho(H_{max}=15000 \text{ km})$ along $\mathbf{d}_B$.
- Same sign at both ends → no solution → fit failed.
- Linear bisection on $\rho$, `BISECT_ITER = 18` → range resolution about 30–40 m
  (pointing effect ~0.003 deg after 300 s, negligible). 20 evaluations of $f$ per fit.
- Sanity checks: $\Delta t \ge 5$ s, both anchors above the horizon,
  sky separation A–B $\ge 10^{-3}$ rad.

### ORBIT PARAMETERS (at $t_{Ref} = t_B$)
$$\begin{aligned}
r &= |\mathbf{S}_B| & &\text{orbit radius (altitude = } r-R\text{)} \\
\mathbf{u} &= \mathbf{S}_B / r & &\text{in-plane unit vector, satellite at } t_{Ref} \\
\mathbf{n} &= \frac{\mathbf{S}_A \times \mathbf{S}_B}{|\mathbf{S}_A \times \mathbf{S}_B|} & &\text{orbit normal, direction of motion A → B} \\
\mathbf{v} &= \mathbf{n} \times \mathbf{u} & &\text{second in-plane unit vector (direction of motion)} \\
\omega_S &= \sqrt{\frac{\mu}{r^3}} &
\end{aligned}$$
Stored as floats (prediction runs in float).

### QUALITY CHECK
- The stored sample halfway between the centre of A and B is compared with the
  new orbit (chord distance ~ angle, 1× predict).
- $dev > $ `MAX_DEV_DEG` (0.25 deg): the track is not this circular orbit (wrong anchor,
  turning plane/aircraft, systematic error over a long baseline):
    - orbit dropped, SIMPLE INTEGRATOR continues with the last commanded speed,
    - anchor A may not start before that midpoint for the next fits
      (shorter, fresher baseline). The next accepted fit returns to orbit steering.

### FIT DECISIONS (every `FIT_INTERVAL_MS`)
- speed limit hold active or newest sample 'limited'  → skip, keep last orbit
- now below `MIN_ANCHOR_ALT_DEG`                         → skip, keep last orbit
- no usable anchor A / midpoint yet                    → skip
- fit failed (no root, checks)                         → keep last orbit
- $dev > $ `MAX_DEV_DEG`                                  → drop orbit (fallback)
- accepted                                             → new orbit, `manualOffset = 0`

## PREDICTION  $t \to$ line of sight (float, every control step)
$$\begin{aligned}
\tau &= t - t_{Ref}, & \theta &= \omega_S \tau \\
\mathbf{S} &= r(\cos\theta \, \mathbf{u} + \sin\theta \, \mathbf{v}) & &\text{non-rotating frame} \\
\mathbf{P} &= \mathrm{rot}(\mathbf{S}, -\omega_E \tau) \text{ about } \mathbf{k} & &\text{Earth rotation} \\
\mathbf{L} &= \mathbf{P} - \mathbf{O} & &\text{line of sight (not normalized)} \\
\dot{\mathbf{L}} &= \text{analytic: rotated } \dot{\mathbf{S}} - \omega_E (\mathbf{k} \times \mathbf{P}) & &\text{(no extra trig)}
\end{aligned}$$
- ALTAZ:      $az = \mathrm{atan2}(L_x, L_y)$,  $alt = \mathrm{asin}(L_z / |\mathbf{L}|)$
- EQUATORIAL: rotate $\mathbf{L}$ about the East axis by the colatitude:
  $\mathbf{W} = (-1,0,0)$, $\mathbf{M} = (0,-\sin\varphi,\cos\varphi)$, $\mathbf{P}_{EQ} = (0,\cos\varphi,\sin\varphi)$
  $HA = \mathrm{atan2}(\mathbf{L}\cdot\mathbf{W}, \mathbf{L}\cdot\mathbf{M})$ (west positive),  
  $Dec = \mathrm{asin}(\mathbf{L}\cdot\mathbf{P}_{EQ} / |\mathbf{L}|)$
  Fixed to the Earth like alt/az → no sidereal time needed.
- Rates of both angles come analytically from $\mathbf{L}$ and $\dot{\mathbf{L}}$. Near the axis‑1 pole
  (zenith for ALTAZ, celestial pole for EQ) the axis‑1 rate gets very large; it is
  kept finite by limiting the distance from the pole to ~0.006 deg.
- Cost: ~4 sin/cos + asin + atan2 + some multiplies.

## MOUNT STEERING
Control step only on the send tick (every `COMMAND_INTERVAL_MULTIPLE` = 3 poll ticks):
1× `getPosition` + 1× predict (with rates), shared by steering, speed limit, horizon
check, sampling and debug. Fit ticks add 1 fit + 1 predict (quality check).
Manual stick correction runs every tick.

### Simple integrator (before the first orbit / after a fallback)
- Normalized speed = baseline + stick × `MANUAL_GAIN` (0.2).
- Baseline += `INTEGRATION_GAIN` (0.4) × $\Delta t$ × manual: corrections are slowly learned.
- Axis 1 is scaled by $\cos(\mathrm{axis2}_{start})/\cos(\mathrm{axis2}_{now})$  
  (Axis 1 turns faster the closer to Zenith/celestial pole).

### Axis mapping (orbit steering, per axis $i = 1, 2$)
- ALTAZ:      axis1 = azimuth,           axis2 = altitude
- GEM / FORK: axis1 = hour angle (h),    axis2 = declination (d)
              pier side is already removed by OnStepX
- ALTALT:     not supported yet (simple integrator only)

All mount limit errors are detected with limits.isError() and movement is stopped.

### Catch-up (per axis)
- Requested speed (ff + PI) ≥ `SPEED_LIMIT_FRACTION` (0.95) of max slew speed:
  direction is latched, the axis runs at max speed (manual input ignored on this
  axis) until it reaches/passes its target and the target rate is below the
  limit again. The other axis keeps tracking normally.

### Speed limit hold
- Trigger: an axis in catch-up (orbit mode) or commanded speed ≥ 0.95 of max
  (simple mode). During the hold: no orbit updates, samples flagged `'limited'` and
  excluded from fits (the mount lags the satellite there).
- Released `SPEED_RELEASE_S` (3 s) after the last trigger.

### Horizon stop
- Mount or predicted orbit below alt 0 → zero speed, tracking disabled,
  reported to the plugin via `consumeHorizonStop()`.

## PI CORRECTIONS
Per axis:
$$\begin{aligned}
err &= \mathrm{wrapPi}(\mathrm{target} + \mathrm{manualOffset} - \mathrm{mountPos}) \\
\mathrm{integ} &\ += K_I \cdot err \cdot \Delta t \quad \text{(limited to }\pm 0.002 \text{ rad/s } \approx 0.11 \text{ deg/s}, \\
& \quad \text{ frozen while the axis output is clamped or in catch-up)} \\
cmd &= \mathrm{targetRate} \text{ (feedforward)} + K_P \cdot err + \mathrm{integ} \\
out &= cmd + \text{manual stick} \times \mathrm{MANUAL\_GAIN}
\end{aligned}$$
- $K_P = 1.0\ \mathrm{s}^{-1}$ (error halves in ~0.7 s),  
  $K_I = 0.05\ \mathrm{s}^{-2}$ (removes constant offsets)
- Stick X = +axis1 rate, Y = −axis2 rate (Y < 0 = up / north).

- Feedforward does almost all the work; the PI only removes the small remaining
  error between mount and orbit.
- Manual corrections are integrated into `manualOffset`, so the PI does not pull
  them back. Because the next accepted orbit passes through the current mount
  position (anchor B), the correction then becomes part of the orbit and
  `manualOffset` is reset to 0 without a jump.
- Switching to orbit mode resets the PI state (starts on the current error).

## TIMELINE (defaults)
| Time | Event |
|------|-------|
| −2.5 s … 0 | button held, stick speeds recorded |
|  0 s | simple integrator, sampling starts |
|  5 s | earliest anchor A start |
| ~13 s | first orbit → orbit steering (if the satellite is above 15 deg) |
| every 1 s | new fit, baseline grows up to 60 s, then anchor A slides forward |
| alt 0 | stop |

## COST (ESP32)
- Fit: anchor A smoothing (~10 samples) + 20 root function evaluations in double
  (~3 sqrt + atan2 each) → whole plugin tick takes 3-4ms (240MHZ Esp32) on orbit fit tick, once per second.
- Predict: float, fine at the control rate.
- Memory: sample ring buffer ~28 bytes/sample (600 samples ≈ 17 kB) + anchor window.

## HISTORY
- **v1** (direct method, replaced): range from a single smoothed window via the
   angular rate (circular-orbit condition $|\dot{\mathbf{S}}|^2 = \mu/r$). Very sensitive to
   tracking noise, because the rate is a derivative. Removed.
- **v1.5** (direct method, two-point fit, replaced): v1 until two-point fit has long 
   baseline. Complex and confusing. Removed.
- **v2** (current): two-point fit with Kepler arc condition, orbit through "now",
   quality check with fallback, orbit steering with feedforward + PI.

## KNOWN LIMITS
- Circular orbit only; eccentric orbits show up as growing deviation on long
  baselines.
- Low elevations (< 15 deg) are not used for fits (highest manual movement error).
- Very high orbits (low angular rate) need a long baseline to be well determined.