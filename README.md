# FSFPV - FPV drone for Microsoft Flight Simulator 2024

Fly an FPV drone, a DJI Inspire style camera drone or a flying wing anywhere in MSFS 2024.
FSFPV is a Community package: no program to launch next to the sim. Everything happens in a
toolbar panel, and the sticks are the axes you assign in the MSFS controls.

## Features

- **FPV drone (acro)**: real quad physics (thrust, drag, prop inflow) and Betaflight / Actual / KISS
  rates, 3D mode, camera angle and FOV.
- **Inspire mode** (DJI Inspire 3 style): the sticks command speeds, centered sticks brake and hold
  the position. Tripod / Normal / Sport modes, each with its own settings. FPV camera stabilized
  on tilt only, with the short catch-up of the real tilt motor on accelerations and braking.
- **Dual OP**: fly a flight, then replay it while operating a Ronin style gimbal with the same sticks
  (roll = pan, pitch = tilt, yaw = gimbal roll). Focal length, Lock / Pan follow / Pan-Tilt follow /
  FPV modes, Ronin roll drift, recordable gimbal track, Replay FPV, replay speed and loop.
- **Fixed wing** (TBS Mojito style flying wing): same sticks and rates as the drone, no yaw (no
  rudder), hand launch, glides, stalls when too slow, level camera.
- **Sim wind**: the drone, the Inspire and the wing fly through the MSFS wind (strength setting,
  can be turned off at any time for calm air).
- Optional flat floor collision, aircraft frozen while you fly, settings saved automatically.

## Install

1. Download `FSFPV-v0.1.0.zip` from the release.
2. Unzip it into your MSFS 2024 **Community** folder. You must get `Community\FSFPV\manifest.json`
   (not `Community\FSFPV-v0.1.0\FSFPV\...`).
3. Start MSFS 2024. The drone icon appears in the toolbar.

Updating: delete the old `FSFPV` folder first, then unzip the new one. Your settings are kept.

## Assign the controls

Options > Controls, pick your controller or radio, and **create a new profile** for the drone so
your flying profile stays untouched:

| MSFS binding | Stick (mode 2) |
| --- | --- |
| THROTTLE AXIS | Left stick Y |
| RUDDER AXIS | Left stick X |
| AILERONS AXIS | Right stick X |
| ELEVATOR AXIS | Right stick Y |

For each axis: **sensitivity 0** (linear curve) and the **smallest deadzone**. The drone applies its
own rates and has its own deadzone setting.

Optional: in the panel's **Controls** tab pick a "toggle event" (for example `SMOKE_TOGGLE`) and bind
it to a button in the same profile. That button turns the drone on and off.

## First flight

1. Start a flight, open the toolbar and click the drone icon.
2. **Controls** tab: move each stick. Throttle up, roll right, pitch forward and yaw right must all
   move the bar to the right. Tick **Invert** where they do not.
3. Click **TURN ON**. The drone appears behind the aircraft, which is frozen while you fly.
4. If the view is wrong, fix it once in the **Camera** tab (invert pitch / bank, swap latitude and
   longitude, altitude in feet).

Inspire mode: tick it in the Inspire tab and use a self-centering throttle stick.
Fixed wing: tick it in the Fixed wing tab, then raise the throttle above 25% to throw it.

## Known limits

- MSFS takes the camera back in menus and pause. The camera can also be disabled in the MSFS camera
  options (the panel says so).
- The sim offers no scenery hit test: the optional collision is a flat floor. Buildings and trees
  are not solid.
- One add-on camera only: no second picture-in-picture view for the pilot in Dual OP.
- The wind is the one at the aircraft's position.

## Reporting a problem

Post in the FSFPV help channel on the Discord with: what you did, what happened, a screenshot of the
panel (Controls and Camera tabs) and your controller / radio model.

## Source code

The repository holds the full source: the WASM flight module (C++, `wasm/src`), the toolbar panel
(`PackageSources/html_ui`) and the MSFS project files. See [BUILD.md](BUILD.md) to build it.

## License

MIT License, (c) 2026 NoclipFPV. See [LICENSE](LICENSE).
