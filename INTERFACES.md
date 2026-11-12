# OnStepX interfaces


Plugin files: `GP_functions.cpp` = `GamepadPILOT_functions.cpp`,
`STE.h` = `SatelliteTrackingEngine.h`.

## Build

| Item | Where | Notes |
|---|---|---|
| `PLUGINn`, `PLUGINn_COMMAND_PROCESSING` | `src/plugins/Plugins.config.h` | |
| `SERIAL_LOCAL_MODE` | `src/Common.h` | must be ON; broker and Serial_Local depend on it, dont think its a problem as not user configured |

## Mount

| Call | Where | Notes |
|---|---|---|
| `mount.getPosition()` | `STE.h` readPosition, `GP_functions.cpp` sendSlewCommands | radians; `CR_MOUNT_ALL` when altaz coords are needed from eq mount with refr corr off with MOUNT_COORDS OBSERVED_PLACE|
| `mount.settings.mountType` | init, sendSlewCommands, STE enable | GEM / FORK / ALTAZM / ALTALT |
| `mount.isTracking()` | handleButtonEvents | selects `:Td#` / `:Te#` |
| `mount.getPosition().pierSide` | sendSlewCommands | GEM west side inverts axis2 |

## Axis

| Call | Where | Notes |
|---|---|---|
| `autoSlew(Direction, float)` | sendSlewCommands | rad/s; `0.0f` is used as stop, `autoSlewStop()` is not threadsafe it seems |
| `setSlewAccelerationRate(float)` | init, maybeEnableHighAcceleration, checkCleanupAndPerform | |
| `isSlewing()` | checkCleanupAndPerform, handleAxisStateDebug | |
| `getFrequency()` | handleAxisStateDebug | debug only |

## Limits

| Call | Where | Notes |
|---|---|---|
| `limits.isError()` | applyLimits, checkCleanupAndPerform | i cant be bothered writing and checking every specific limit for every mount type |
| `isEnabled()`, `isBelowHorizon()`, `isAboveOverhead()`, `isPastMeridianE/W()`, `isPastAxis1Min/Max()` | handleAxisStateDebug | debug only |

## Park

| Call | Where | Notes |
|---|---|---|
| `park.state` | onConnected | `PARK_PAIRING_ONLY` |

## Site

| Call | Where | Notes |
|---|---|---|
| `site.location.latitude` | `STE.h` enable | radians |

## CommandBroker

| Call | Where | Notes |
|---|---|---|
| `request()` | requestMaxSlewSpeed, init | |
| `result()` | updateMaxSlewSpeedFromBroker | |
| `send()` | init | `USE_DATETIME_OVERRIDE` |

## SERIAL_LOCAL

| Call | Where | Notes |
|---|---|---|
| `transmit()` | checkCleanupAndPerform, handleButtonEvents | asynchronous; shared reply buffer |

## LX200 commands

| Command | Implementation | Notes |
|---|---|---|
| `:GX97#` | `src/telescope/mount/goto/Goto.command.cpp` | current step rate; plugin uses it as max slew speed |
| `:SC` / `:SL` | `src/telescope/mount/site/Site.command.cpp` | `USE_DATETIME_OVERRIDE`, set date and time|
| `:Te#` / `:Td#` | `src/telescope/mount/Mount.command.cpp` | tracking on/off |
| `:Me#` / `:Mn#` / `:Q#` | `src/telescope/mount/guide/Guide.command.cpp` | weird slew-flag cleanup: move east, north, stop all |
| `:hP#` / `:hR#` | `src/telescope/mount/park/Park.command.cpp` | park / unpark |
| custom | `CUSTOM_COMMANDS` in `GamepadPILOT_config.h`, handleButtonEvents | sent via `SERIAL_LOCAL` |

## Config

| Macro | Where | Notes |
|---|---|---|
| `SLEW_RATE_BASE_DESIRED`, `SLEW_ACCELERATION_DIST` | `Config.h`; init, updateMaxSlewSpeedFromBroker | initial speed and acceleration estimate |
| `MOUNT_COORDS` | `src/Constants.h`; `STE.h` readPosition | selects `CR_MOUNT_ALL` to get altaz coords on eq mount with refr corr off |


## Known quirks and Todo

- Acceleration handling: Plugin sets acceleration to 10.0 rad/s^2 in
  `maybeEnableHighAcceleration()` to calculate its own acceleration  
  (otherwise Onstepx does not apply acceleration when slowing down)
  the restore path is skipped while `limits.isError()` and runs only once both axes report not slewing.
- Stopping an Autoslew axisn.autoSlew(): `autoSlewStop()` and `autoSlewAbort()` crash OnstepX from inside a plugin. (Thread safety?)  
  But why does autoSlew() work??? Idk, but good thing it does.
- TODO `:Me#`/`:Mn#`/`:Q#` slew flag cleanup is a workaround: the axis
  state machine stays in a slew after `autoSlew(dir, 0.0f)`. The real fix
  is a thread-safe `autoSlewStop()`.
- TODO ALTALT mount support (might still partially work rn)


