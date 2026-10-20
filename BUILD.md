# FSFPV - build from source (MSFS 2024 WASM package)

A Community-folder add-on, nothing to launch next to the sim:

- `wasm/src/` – standalone WASM module: drone physics (port of the Assetto Corsa mod), rates, floor collision, add-on camera.
- `PackageSources/html_ui/InGamePanels/FpvDrone/` – toolbar panel (HTML/JS/CSS).
- `PackageSources/InGamePanels/` – toolbar panel definition (compiled to `.spb` by the package tool).

Features: acro FPV flight (Betaflight / Actual / KISS rates), **Inspire mode** (DJI Inspire 3 style
speed flight with Tripod / Normal / Sport modes) and **Dual OP** (fly, then replay the flight while
operating a Ronin-style gimbal: focal length, Ronin roll drift, Lock / Pan follow / Pan-Tilt follow /
FPV modes, recordable gimbal track, Replay FPV).
**Fixed wing** (TBS Mojito style flying wing): same sticks and rates as the drone (roll and pitch
scaled), no yaw (no rudder), hand launch by raising the throttle, glides with centered sticks, stalls
when too slow, level camera. Dual OP is not available with the wing.
**Sim wind** (Physics tab, on by default, can be turned off at any time): the three flight models fly
through the air mass moving with the MSFS wind at the aircraft's position, with a strength setting.

## 1. The WASM module

`PackageSources/modules/FpvDrone.wasm` is already built (MSFS 2024 SDK 1.8.1 headers and libraries, LLVM 15). Nothing to do unless you change the C++.

To rebuild after a code change, either:

- **Visual Studio**: open `wasm/FpvDrone.vcxproj` (needs the MSFS 2024 platform installed by the SDK), select **Release | MSFS**, build. The output goes straight to `PackageSources/modules/`.
- **Script** (Linux/WSL): `wasm/build_wasm.sh`, see the comments at the top of the file.

## 2. Build the package

1. Start MSFS 2024 with Developer Mode on.
2. DevMode > Tools > **Project Editor**, open `FpvDrone.xml` (this folder).
3. **Build All**. The package appears in `Packages/FSFPV/`.
4. Copy `Packages/FSFPV` into your **Community** folder and restart MSFS (or mount it from DevMode).

The panel definition must be compiled to `.spb`, so the package has to go through the package tool. Copying `PackageSources` by hand will not show the panel.

## 3. Assign the controls in MSFS

Options > Controls, pick your controller, and **create a new profile** for the drone so your flying profile stays untouched:

| MSFS binding | Stick (mode 2) |
| --- | --- |
| THROTTLE AXIS | Left stick Y |
| RUDDER AXIS | Left stick X |
| AILERONS AXIS | Right stick X |
| ELEVATOR AXIS | Right stick Y |

For each axis: **sensitivity 0** (linear curve) and the **smallest deadzone**. The drone applies its own rates.

Optional: in the panel's **Controls** tab pick a "toggle event" (for example `SMOKE_TOGGLE`) and bind it to a button in the same profile. That button then turns the drone on and off.

## 4. First flight

1. Start a flight. Open the toolbar, click the drone icon.
2. **Controls** tab: move each stick. Throttle up, roll right, pitch forward and yaw right must all move the bar to the right. Tick **Invert** where it does not.
3. Click **TURN ON**. The drone appears behind the aircraft. The aircraft is frozen by default and no longer receives your axes.
4. If the view is wrong, fix it once in the **Camera** tab:
   - sky shows when you dive → invert pitch
   - horizon rolls the wrong way → invert bank
   - drone far away → swap latitude / longitude
   - drone much too high → altitude in feet

Settings save automatically (in the package's `work` folder).

## Known limits

- **Camera API beta**: MSFS takes the camera back in menus and pause. It can also be disabled in the MSFS camera options (the panel shows it).
- **Collision**: no scenery hit-test is available, so the drone hits a flat floor (terrain under the aircraft at launch + offset, or a fixed altitude). Buildings and trees are not solid.
- **Aircraft**: the axes are read as axis events through SimConnect and still reach the aircraft, so "Freeze the aircraft" is on by default.
- The module updates in `module_late_update` (MSFS 2024, runs every frame before drawing). If the sim does not call it, the module falls back to the SimConnect frame event.

## Debugging

- WASM: DevMode > Debug > **WASM Debug** shows loaded modules and `printf` output.
- Panel: Coherent GT Debugger (`MSFS 2024 SDK/Tools/CoherentGT Debugger`), select the FPV Drone panel.
- The panel says "No answer from the FPV Drone module" when the WASM module is not loaded: check that `modules/FpvDrone.wasm` is inside the package.
