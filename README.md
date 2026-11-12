# OnStepX Gamecontroller Plugin

Steer your OnStepX mount with a Bluetooth gamepad, and track satellites by hand until the mount takes over.

> **Status: experimental.** Use at your own risk. A lot of effort went into safety (controller disconnects, ghosting, mount limits), but this is a hobby project, so stay near your mount while it moves.
>
> ☀️ **Never point at or near the Sun without a proper solar filter.** That includes satellite/airplane tracking: a pass can cross the Sun's path.

---

## Features

All options are in `gamecontroller_config.h`.

**Mount control**
- Analog stick slewing with selectable speed curve (linear / square / cubic) and smooth acceleration
- 3 speed gears (controller rumbles at the slowest)
- One-button swap of N/S and E/W, for when the view through the eyepiece is mirrored
- Toggle sidereal tracking on/off
- Custom LX200 commands on any button or d-pad combination (e.g. park/unpark)

**Satellite tracking**
- Follow the satellite with the stick, hold a button, release, and the mount continues on its own
- Gentle stick corrections stay active the whole time
- The orbit is learned from your own movement. No TLE, no internet, no exact time needed
- Starts by repeating your speed, then after about 13 s switches to a fitted circular orbit with closed-loop steering
- Stops automatically at the horizon or at any mount limit
- Optional serial stats: time to closest approach, peak elevation, orbit altitude, range, speed on the sky

**Safety**
- Unlock sequence before the mount moves
- Watchdog: mount stops if no data arrives from the controller (covers slow disconnect detection and ghosting)
- Optional MAC whitelist, or pairing only while the mount is parked
- Mount limits respected; short recovery window to move back inside the limits

---

## Compatibility

| | |
|---|---|
| Controller hardware | ESP32-based OnStepX controllers only |
| Mount types | ALTAZM, GEM, FORK (ALTALT planned) |
| Gamepads | Anything [Bluepad32 supports](https://bluepad32.readthedocs.io/en/latest/supported_gamepads/). Button numbers may need adjusting in the config (`RAW_DEBUG_MODE` shows raw values) |
| OnStepX | last tested on 10.29c |
| Board package | esp32_bluepad32 4.1.0 |

**Bluetooth:** 
- some ESP32 variants (e.g. ESP32-S3, ESP32-C3) only support BLE, not Bluetooth Classic. Check that your mount's ESP32 and your gamepad speak the same protocol.
- You can not use **BluetoothSerial** with this plugin (Bluepad32 Board Manager)

**Tested on**

| Hardware | Mount type | Result |
|---|---|---|
| Custom Arduino Nano ESP32 alt-az mount | ALTAZM | ✔ on the sky |
| TerransIndustry V5 Pro kit (upgraded Meade LXD75) | GEM | ✔ on the sky |
| Arduino Nano ESP32 bench setup (motors + Stellarium) | ALTAZM, GEM, FORK | ✔ on the bench |

Works together with WiFi and the OnStepX website plugin (tested with a BLE controller). Bluetooth Classic controllers are not tested yet.

---

## Installation

1. Copy the `gamecontroller` folder into `OnStepX/src/plugins/`.
2. In `OnStepX/src/plugins/Plugins.config.h`, use any free plugin slot, e.g.:
   ```cpp
   #define PLUGIN3                       gamecontroller
   #include "gamecontroller/Gamecontroller.h"
   #define PLUGIN3_COMMAND_PROCESSING    OFF
   ```
3. Adjust `gamecontroller_config.h` to your liking (the defaults are a sensible start).
4. Install the **esp32_bluepad32** board package in the Arduino Board Manager ([Bluepad32 Arduino setup](https://bluepad32.readthedocs.io/en/latest/plat_arduino/)).
5. Select your board **from the esp32_bluepad32 package** (not the regular ESP32 package).
6. Upload using **Sketch → Upload Using Programmer**.
7. Put your gamepad in pairing mode. It connects automatically.

---

## Controls (defaults)

| Input | Function |
|---|---|
| Left stick | Slew the mount |
| D-pad ↓ 2 s, then ↑ 2 s | Unlock (once after connecting) |
| Right bumper | Cycle speed gear (rumble = slowest) |
| Left bumper | Swap N/S and E/W directions |
| X | Sidereal tracking on/off |
| A | Park |
| B | Unpark |
| Left trigger | Satellite tracking: hold to start, press to stop |

If the mount ended up outside a limit, center all sticks to get a short window to move it back.

### Satellite tracking

1. Follow the satellite with the stick (ideally once it's above ~15°).
2. Keep following and hold the **left trigger** for 2.5 s.
3. Release. The mount now tracks on its own.
4. Correct gently with the stick if needed. Press the trigger again to stop.  
   Corrections improve the orbit model.

Tips:
- The smoother your manual tracking before releasing, the better the start.
- Very high passes on alt-az mounts can exceed the max slew speed near the zenith. The mount runs at full speed and catches up afterwards.

---

## How satellite tracking works

Right after you release the button, the mount simply repeats the average speed you moved it at and slowly learns from your corrections. Meanwhile it records its own position. After a few seconds, a circular orbit is fitted through an older point and the current position, using the fact that the arc between them must match Kepler's law. From then on the mount follows the predicted path (feedforward) with a small PI loop removing the remaining error. If the track doesn't fit a circular orbit, it falls back to the simple mode.

Full details: [ORBIT_MODEL.md](ORBIT_MODEL.md)

---

## Troubleshooting

**Compile issues**
- Windows: move the `Arduino15` folder to a shorter path (path length limit).
- Enable **USB CDC On Boot**.
- Out of flash: choose the **Huge APP** partition scheme.

**Runtime**
- Serial debug and stats output (`SATELLITE_STATS_PRINT`, debug options) can interfere with INDI/ASCOM on the same port. Turn them off for normal use.
- Mount doesn't react: did you do the unlock sequence? Is the mount parked?

Still stuck? Open an [issue](../../issues) with your board, mount type, gamepad, OnStepX and board package versions, and the serial log.

---

## Development

Ideas, features and testing are my own. Most of the code text was written with the help of AI models, step by step with precise prompts. Every change was checked by me and tested on hardware. The orbit model was developed with extended help from Claude.

Models used: Gemini 2.5 – 3.1 Flash, Mistral Medium 3.5, DeepSeek V4-Pro, Claude Opus 5.5.

## Contributing

Feedback, ideas, bug reports and pull requests are welcome.

## License & credits

GPL-3.0, like OnStepX.

- [OnStepX](https://github.com/hjd1964/OnStepX) by Howard Dutton
- [Bluepad32](https://github.com/ricardoquesada/bluepad32) by Ricardo Quesada
