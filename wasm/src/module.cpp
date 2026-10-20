// FPV Drone for Microsoft Flight Simulator 2024 - standalone WASM module.
//
// - Reads the aircraft axes the user assigned in MSFS (throttle, ailerons, elevator, rudder).
//   The aircraft still receives them, so it is frozen while the drone flies (option).
// - Runs the drone physics every frame and drives the add-on camera through the native WASM Camera API.
// - Talks to the toolbar panel over the CommBus:
//     JS -> WASM  "FpvDrone.Set"      "key=value;key=value"
//                 "FpvDrone.Command"  "toggle" | "getSettings" | "resetRates" | "resetAll"
//     WASM -> JS  "FpvDrone.Settings" JSON object with every setting
//                 "FpvDrone.State"    JSON object with live status, ~15 Hz

#include <MSFS/MSFS.h>
#include <MSFS/MSFS_WindowsTypes.h> // must come before SimConnect.h in WASM
#include <MSFS/MSFS_Camera.h>
#include <MSFS/MSFS_CommBus.h>
#include <SimConnect.h>

// MSFS_WindowsTypes.h defines Windows-style min/max macros that break std::min/std::max.
#undef min
#undef max

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "drone.h"
#include "gimbal.h"
#include "recorder.h"
#include "settings.h"

namespace {

const char* kSettingsPath = "\\work\\settings.cfg";

// ---------- SimConnect ids ----------

enum DataDefinition { DEF_AIRCRAFT = 1 };
enum DataRequest { REQ_AIRCRAFT = 1 };

enum EventId {
    EVT_FRAME = 1,
    EVT_PAUSE,
    EVT_SIM_STOP,

    // Axes
    EVT_AXIS_THROTTLE = 10,
    EVT_THROTTLE_AXIS_EX1,
    EVT_AXIS_THROTTLE1,
    EVT_THROTTLE1_AXIS_EX1,
    EVT_AXIS_AILERONS,
    EVT_AXIS_ELEVATOR,
    EVT_AXIS_RUDDER,

    // Optional "toggle drone" bindings, index = Settings::toggleEvent
    EVT_TOGGLE_1 = 31,
    EVT_TOGGLE_2,
    EVT_TOGGLE_3,

    // Sent by the module
    EVT_FREEZE_POSITION = 50,
    EVT_FREEZE_ALTITUDE,
    EVT_FREEZE_ATTITUDE,
};

enum GroupId { GROUP_AXES_LISTEN = 1, GROUP_AXES_MASK = 2, GROUP_TOGGLE = 3 };

enum Axis { AXIS_THROTTLE = 0, AXIS_ROLL = 1, AXIS_PITCH = 2, AXIS_YAW = 3 };

struct AxisEvent {
    EventId id;
    const char* simEvent;
    Axis axis;
};

const AxisEvent kAxisEvents[] = {
    {EVT_AXIS_THROTTLE, "AXIS_THROTTLE_SET", AXIS_THROTTLE},
    {EVT_THROTTLE_AXIS_EX1, "THROTTLE_AXIS_SET_EX1", AXIS_THROTTLE},
    {EVT_AXIS_THROTTLE1, "AXIS_THROTTLE1_SET", AXIS_THROTTLE},
    {EVT_THROTTLE1_AXIS_EX1, "THROTTLE1_AXIS_SET_EX1", AXIS_THROTTLE},
    {EVT_AXIS_AILERONS, "AXIS_AILERONS_SET", AXIS_ROLL},
    {EVT_AXIS_ELEVATOR, "AXIS_ELEVATOR_SET", AXIS_PITCH},
    {EVT_AXIS_RUDDER, "AXIS_RUDDER_SET", AXIS_YAW},
};

const char* kToggleEvents[] = {"SMOKE_TOGGLE", "TOGGLE_TAIL_HOOK_HANDLE", "TOGGLE_WATER_RUDDER"};

struct AircraftData {
    double latitude;
    double longitude;
    double altitude;
    double heading;
    double groundAltitude;
    double velocityX; // east
    double velocityY; // up
    double velocityZ; // north
    double yokeX;     // -1..1, aircraft control positions (input source "aircraft controls")
    double yokeY;
    double rudderPedal;
    double throttleLever; // 0..100 %
    double smokeEnable;   // 0/1, watched for the "toggle drone" button (SMOKE_TOGGLE binding)
    double windSpeed;     // m/s, AMBIENT WIND VELOCITY at the aircraft
    double windFrom;      // degrees true, direction the wind blows FROM
};

struct AircraftControls {
    double yokeX = 0, yokeY = 0, rudderPedal = 0, throttleLever = 0;
};

// ---------- State ----------

HANDLE g_sim = 0;
Settings g_settings;
Drone g_drone;

// Dual OP
enum PlayMode { PlayNone = 0, PlayGimbal = 1, PlayFpv = 2 };
FlightTake g_take;
GimbalTrack g_gtrack;
Gimbal g_gimbal;
int g_play = PlayNone;
double g_simWindSpeed = 0, g_simWindFrom = 0; // from SimConnect
Vec3 g_wind;                                  // applied to the drone (smoothed)
Vec3 simWind();
double g_playTime = 0;
bool g_gimbalRecording = false;
AircraftState g_aircraft;
AircraftControls g_controls;
int g_lastSmoke = -1;


double g_axis[4] = {-1, 0, 0, 0}; // raw axis values -1..1, before inversion
bool g_paused = false;
bool g_axesMasked = false;
bool g_frozen = false;
int g_toggleEventInGroup = 0;
double g_toggleCooldown = 0;

int g_cameraState = FS_ACQUIRED_STATE_NOT_ACQUIRED;
bool g_cameraGameControlled = false;
double g_acquireTimer = 0;
double g_cameraPollTimer = 0;
double g_releasePending = 0; // seconds during which a stopped drone keeps checking that the camera was released

int g_framesWithoutLateUpdate = 100;
double g_stateTimer = 0;
bool g_settingsDirty = false;
double g_saveTimer = 0;
std::string g_error;

// ---------- Helpers ----------

bool useAircraftControls() { return g_settings.inputSource >= 0.5; }

void sendToPanel(const char* event, const std::string& payload) {
    fsCommBusCall(event, payload.c_str(), static_cast<unsigned int>(payload.size() + 1), FsCommBusBroadcast_JS);
}

void sendSettings() { sendToPanel("FpvDrone.Settings", settingsToJson(g_settings)); }

void transmit(EventId id, DWORD data) {
    SimConnect_TransmitClientEvent(g_sim, SIMCONNECT_OBJECT_ID_USER, id, data, SIMCONNECT_GROUP_PRIORITY_HIGHEST,
                                   SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY);
}

void setFreeze(bool freeze) {
    if (g_frozen == freeze) return;
    transmit(EVT_FREEZE_POSITION, freeze ? 1 : 0);
    transmit(EVT_FREEZE_ALTITUDE, freeze ? 1 : 0);
    transmit(EVT_FREEZE_ATTITUDE, freeze ? 1 : 0);
    g_frozen = freeze;
}

void updateToggleEventGroup() {
    const int wanted = static_cast<int>(g_settings.toggleEvent + 0.5);
    if (!g_sim || wanted == g_toggleEventInGroup) return;
    if (g_toggleEventInGroup != 0) SimConnect_ClearNotificationGroup(g_sim, GROUP_TOGGLE);
    if (wanted >= 1 && wanted <= 3) {
        SimConnect_AddClientEventToNotificationGroup(g_sim, GROUP_TOGGLE, EVT_TOGGLE_1 + wanted - 1, TRUE);
        SimConnect_SetNotificationGroupPriority(g_sim, GROUP_TOGGLE, SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE);
    }
    g_toggleEventInGroup = wanted;
}

void pollCameraStatus() {
    FsAcquiredState state = FS_ACQUIRED_STATE_NOT_ACQUIRED;
    bool gameControlled = false;
    if (fsCameraGetStatus(&state, &gameControlled)) {
        g_cameraState = state;
        g_cameraGameControlled = gameControlled;
    }
}

void onCameraStatus(FsCameraEvent, const FsAcquiredState state, const bool gameControlled, void*) {
    g_cameraState = state;
    g_cameraGameControlled = gameControlled;
}

// The cached camera state can lag behind the sim, so acquire/release never trust it to skip the call.
void acquireCamera() {
    pollCameraStatus();
    if (g_cameraState == FS_ACQUIRED_STATE_USER_DISABLED) {
        g_error = "The add-on camera is disabled in the MSFS camera options.";
        return;
    }
    if (g_cameraState != FS_ACQUIRED_STATE_ACQUIRED) fsCameraAcquire("FPV Drone");
    g_acquireTimer = 0;
    pollCameraStatus();
}

void releaseCamera() {
    fsCameraRelease(""); // "" = back to the camera used before the drone
    pollCameraStatus();
}

void startDrone() {
    if (!g_aircraft.valid) {
        g_error = "No aircraft data yet. Start a flight first.";
        return;
    }
    g_error.clear();
    const bool freeze = g_settings.freezeAircraft >= 0.5;
    if (freeze) setFreeze(true);
    g_drone.start(g_aircraft, g_settings, !freeze);
    g_wind = simWind();
    g_drone.setWind(g_wind);
    // No axis masking: MSFS 2024 aircraft do not use the legacy axis events, so there is nothing to block.
    acquireCamera();
}

void stopReplay() {
    g_play = PlayNone;
    g_gimbalRecording = false;
}

void stopDrone() {
    stopReplay();
    g_drone.stop();
    releaseCamera();
    g_releasePending = 1.0;
    setFreeze(false);
}

void toggleDrone() {
    if (g_drone.active) stopDrone();
    else startDrone();
}

Sticks currentSticks() {
    const Settings& s = g_settings;
    double raw[4];
    if (useAircraftControls()) {
        raw[AXIS_THROTTLE] = std::clamp(g_controls.throttleLever / 50.0 - 1.0, -1.0, 1.0);
        raw[AXIS_ROLL] = std::clamp(g_controls.yokeX, -1.0, 1.0);
        raw[AXIS_PITCH] = std::clamp(g_controls.yokeY, -1.0, 1.0);
        raw[AXIS_YAW] = std::clamp(g_controls.rudderPedal, -1.0, 1.0);
    } else {
        for (int i = 0; i < 4; ++i) raw[i] = g_axis[i];
    }
    Sticks st;
    st.throttle = s.invertThrottle >= 0.5 ? -raw[AXIS_THROTTLE] : raw[AXIS_THROTTLE];
    st.roll = s.invertRoll >= 0.5 ? -raw[AXIS_ROLL] : raw[AXIS_ROLL];
    st.pitch = s.invertPitch >= 0.5 ? -raw[AXIS_PITCH] : raw[AXIS_PITCH];
    st.yaw = s.invertYaw >= 0.5 ? -raw[AXIS_YAW] : raw[AXIS_YAW];
    return st;
}

void onAxisValue(Axis axis, int value) {
    if (!useAircraftControls()) g_axis[axis] = std::clamp(value / 16383.0, -1.0, 1.0);
}

void onToggleEvent(int index) {
    if (index != static_cast<int>(g_settings.toggleEvent + 0.5)) return;
    if (g_toggleCooldown > 0) return;
    g_toggleCooldown = 0.3;
    toggleDrone();
}


void applyCamera(const CameraPose& pose) {
    if (!pose.valid || g_cameraState != FS_ACQUIRED_STATE_ACQUIRED || g_cameraGameControlled) return;
    const Settings& s = g_settings;

    const GeoPoint g = localToGeo(pose.origin, pose.local);
    FsCameraData cam{};
    cam.positionReferential = FS_POSITION_REFERENTIAL_WORLD;
    cam.positionReferentialObjectId = 0;
    cam.rotationReferential = FS_POSITION_REFERENTIAL_WORLD;
    cam.rotationReferentialObjectId = 0;
    cam.position.x = s.camSwapLatLon >= 0.5 ? g.lon : g.lat;
    cam.position.y = s.camSwapLatLon >= 0.5 ? g.lat : g.lon;
    cam.position.z = s.camAltitudeFeet >= 0.5 ? g.alt * 3.28084 : g.alt;
    cam.pbh.p = s.camInvertPitch >= 0.5 ? -pose.euler.pitch : pose.euler.pitch;
    cam.pbh.b = s.camInvertBank >= 0.5 ? -pose.euler.bank : pose.euler.bank;
    cam.pbh.h = wrap360(s.camInvertHeading >= 0.5 ? -pose.euler.heading : pose.euler.heading);
    cam.fov = degToRad(pose.fovDeg);

    if (!fsCameraSet(&cam, FS_CAMERA_DATA_MASK_ALL_ROTATION)) pollCameraStatus();
}

double deadzone(double v, double percent) {
    const double dz = std::clamp(percent / 100.0, 0.0, 0.99);
    if (std::abs(v) < dz) return 0.0;
    return std::copysign((std::abs(v) - dz) / (1.0 - dz), v);
}

double lensFov(double focalMm) {
    return g_settings.camFovHorizontal >= 0.5 ? Gimbal::hfovDeg(focalMm) : Gimbal::vfovDeg(focalMm);
}

// Dual OP replay: the drone flies the recorded take by itself; in gimbal mode the sticks operate
// the camera (roll = pan, pitch = tilt, yaw = gimbal roll), or the recorded gimbal track plays.
void stepReplay(double dt) {
    const Settings& s = g_settings;
    const double duration = g_take.duration();
    g_playTime += dt * s.replaySpeed;
    if (g_playTime >= duration) {
        if (s.replayLoop >= 0.5 && duration > 0) {
            g_playTime = std::fmod(g_playTime, duration);
            g_gimbalRecording = false; // a gimbal recording ends with the take
        } else {
            g_playTime = duration;
        }
    }

    const FlightTake::Pose p = g_take.at(g_playTime);
    CameraPose pose;
    pose.valid = true;
    pose.origin = g_take.origin();
    double focal = s.focalLength;

    if (g_play == PlayFpv) {
        pose.local = p.position + p.body.up * 0.1;
        pose.euler = basisToEuler(p.camera.forward, p.camera.up);
    } else {
        const Vec3 camPos = p.position - p.body.up * 0.15; // the gimbal hangs below the frame
        double yaw = 0, pitch = 0, roll = 0;
        if (!g_gimbalRecording && g_gtrack.at(g_playTime, yaw, pitch, roll, focal)) {
            // Recorded gimbal track: the sticks are ignored.
        } else {
            const Sticks st = currentSticks();
            GimbalInput in;
            in.pan = deadzone(st.roll, s.stickDeadzone);
            in.tilt = deadzone(st.pitch, s.stickDeadzone) * (s.gimbalInvertTilt >= 0.5 ? -1.0 : 1.0);
            in.roll = deadzone(st.yaw, s.stickDeadzone);
            g_gimbal.update(in, s, p.body, camPos, dt);
            const GimbalAngles a = g_gimbal.angles();
            yaw = a.yaw;
            pitch = a.pitch;
            roll = a.roll;
            if (g_gimbalRecording) g_gtrack.record(g_playTime, yaw, pitch, roll, focal);
        }
        const Basis cam = eulerToBasis(pitch, roll, yaw); // also handles a tilt past -90
        pose.local = camPos;
        pose.euler = basisToEuler(cam.forward, cam.up);
    }
    pose.fovDeg = lensFov(focal);
    applyCamera(pose);
}

void startReplay(int mode) {
    if (g_settings.wingEnabled >= 0.5) {
        g_error = "Dual OP is not available with the fixed wing.";
        return;
    }
    if (g_take.empty()) {
        g_error = "No flight recorded yet: enable Dual OP and fly first.";
        return;
    }
    if (!g_drone.active) {
        startDrone();
        if (!g_drone.active) return;
    }
    g_error.clear();
    g_take.freeze();
    g_play = mode;
    g_playTime = 0;
    g_gimbalRecording = false;
    if (mode == PlayGimbal) g_gimbal.reset(g_take.at(0).body, static_cast<int>(g_settings.roninMode + 0.5));
}

void sendState() {
    const double windKmh = g_simWindSpeed * 3.6;
    const Sticks st = currentSticks();
    const double speed = g_drone.velocity().length() * 3.6;
    const double height = g_drone.position().z - g_drone.floorZ();
    char buf[1024];
    std::snprintf(buf, sizeof(buf),
                  "{\"active\":%d,\"aircraft\":%d,\"paused\":%d,\"cam\":%d,\"camGame\":%d,\"frozen\":%d,"
                  "\"masked\":%d,\"thr\":%.3f,\"roll\":%.3f,\"pitch\":%.3f,\"yaw\":%.3f,\"speed\":%.1f,"
                  "\"height\":%.1f,\"error\":\"%s\",\"take\":%.1f,\"takeFrozen\":%d,\"recording\":%d,"
                  "\"play\":%d,\"ptime\":%.1f,\"gtrack\":%.1f,\"grec\":%d,\"aoa\":%.1f,\"wingWait\":%d,\"wind\":%.1f,\"windFrom\":%.0f,\"windOn\":%.1f}",
                  g_drone.active ? 1 : 0, g_aircraft.valid ? 1 : 0, g_paused ? 1 : 0, g_cameraState,
                  g_cameraGameControlled ? 1 : 0, g_frozen ? 1 : 0, g_axesMasked ? 1 : 0, st.throttle, st.roll,
                  st.pitch, st.yaw, g_drone.active ? speed : 0.0, g_drone.active ? height : 0.0, g_error.c_str(),
                  g_take.duration(), g_take.frozen() ? 1 : 0,
                  (g_drone.active && g_play == PlayNone && g_settings.dualOpEnabled >= 0.5 && g_settings.wingEnabled < 0.5 &&
                   !g_take.frozen()) ? 1 : 0,
                  g_play, g_playTime, g_gtrack.duration(), g_gimbalRecording ? 1 : 0, g_drone.aoaDeg(),
                  (g_drone.active && g_drone.wingWaiting()) ? 1 : 0, g_aircraft.valid ? windKmh : 0.0,
                  g_simWindFrom, g_wind.length() * 3.6);
    sendToPanel("FpvDrone.State", buf);
}

// Sim wind as an ENU vector (where the air goes), scaled by the Wind strength. Zero when the option is off.
Vec3 simWind() {
    if (g_settings.windEnabled < 0.5 || !g_aircraft.valid) return {};
    const double to = degToRad(g_simWindFrom + 180.0);
    const double v = g_simWindSpeed * g_settings.windStrength / 100.0;
    return {std::sin(to) * v, std::cos(to) * v, 0.0};
}

// One simulation step. Runs from module_late_update, or from the SimConnect frame event as a fallback.
void step(double dt) {
    if (!g_sim) return;
    dt = std::clamp(dt, 0.0, 0.1);

    g_toggleCooldown = std::max(0.0, g_toggleCooldown - dt);

    g_cameraPollTimer += dt > 0 ? dt : 1.0 / 60.0;
    if (g_cameraPollTimer > 1.0) {
        g_cameraPollTimer = 0;
        pollCameraStatus();
    }

    if (!g_drone.active && g_releasePending > 0) {
        g_releasePending -= dt > 0 ? dt : 1.0 / 60.0;
        if (g_cameraState == FS_ACQUIRED_STATE_ACQUIRED) releaseCamera();
    }

    if (g_drone.active && !g_paused && dt > 0) {
        if (g_cameraState != FS_ACQUIRED_STATE_ACQUIRED && g_cameraState != FS_ACQUIRED_STATE_USER_DISABLED) {
            g_acquireTimer += dt;
            if (g_acquireTimer > 0.25) acquireCamera(); // retry quickly, the sim may still be busy
        }
        if (g_play != PlayNone && !g_take.empty()) {
            stepReplay(dt);
        } else {
            g_play = PlayNone;
            // Smoothed so that a change of the weather or of the option never kicks the drone.
            g_wind += (simWind() - g_wind) * smoothAlpha(dt, 0.5);
            g_drone.setWind(g_wind);
            g_drone.update(dt, currentSticks(), g_settings);
            if (g_settings.dualOpEnabled >= 0.5 && g_settings.wingEnabled < 0.5) // no Dual OP on the fixed wing
                g_take.record(dt, g_drone.origin(), g_drone.position(), g_drone.body(), g_drone.camera());
            applyCamera(g_drone.pose);
        }
    }

    g_stateTimer += dt > 0 ? dt : 1.0 / 60.0;
    if (g_stateTimer >= 1.0 / 15.0) {
        g_stateTimer = 0;
        sendState();
    }

    if (g_settingsDirty) {
        g_saveTimer += dt > 0 ? dt : 1.0 / 60.0;
        if (g_saveTimer > 1.0) {
            saveSettingsFile(g_settings, kSettingsPath);
            g_settingsDirty = false;
        }
    }
}

// ---------- SimConnect ----------

void CALLBACK onSimConnect(SIMCONNECT_RECV* recv, DWORD, void*) {
    switch (recv->dwID) {
    case SIMCONNECT_RECV_ID_EXCEPTION: {
        const auto* ex = static_cast<SIMCONNECT_RECV_EXCEPTION*>(recv);
        char buf[96];
        std::snprintf(buf, sizeof(buf), "SimConnect exception %lu (send id %lu)",
                      static_cast<unsigned long>(ex->dwException), static_cast<unsigned long>(ex->dwSendID));
        g_error = buf;
        break;
    }

    case SIMCONNECT_RECV_ID_EVENT_FRAME: {
        const auto* fr = static_cast<SIMCONNECT_RECV_EVENT_FRAME*>(recv);
        // Fallback when module_late_update is not called by this sim version.
        if (++g_framesWithoutLateUpdate > 3) step(fr->fFrameRate > 1.0f ? 1.0 / fr->fFrameRate : 1.0 / 30.0);
        break;
    }

    case SIMCONNECT_RECV_ID_EVENT: {
        const auto* ev = static_cast<SIMCONNECT_RECV_EVENT*>(recv);
        const DWORD id = ev->uEventID;
        if (id == EVT_PAUSE) {
            g_paused = ev->dwData != 0;
        } else if (id == EVT_SIM_STOP) {
            if (g_drone.active) stopDrone();
            g_aircraft.valid = false;
        } else if (id >= EVT_AXIS_THROTTLE && id <= EVT_AXIS_RUDDER) {
            for (const AxisEvent& e : kAxisEvents)
                if (e.id == static_cast<EventId>(id)) {
                    onAxisValue(e.axis, static_cast<int>(ev->dwData));
                }
        } else if (id >= EVT_TOGGLE_1 && id <= EVT_TOGGLE_3) {
            onToggleEvent(static_cast<int>(id - EVT_TOGGLE_1) + 1);
        }
        break;
    }

    case SIMCONNECT_RECV_ID_SIMOBJECT_DATA: {
        const auto* obj = static_cast<SIMCONNECT_RECV_SIMOBJECT_DATA*>(recv);
        if (obj->dwRequestID == REQ_AIRCRAFT) {
            AircraftData d;
            std::memcpy(&d, &obj->dwData, sizeof(d));
            g_aircraft.valid = true;
            g_aircraft.position = {d.latitude, d.longitude, d.altitude};
            g_aircraft.headingDeg = d.heading;
            g_aircraft.groundAlt = d.groundAltitude;
            g_aircraft.velocity = {d.velocityX, d.velocityZ, d.velocityY};
            g_simWindSpeed = d.windSpeed;
            g_simWindFrom = d.windFrom;
            g_controls.yokeX = d.yokeX;
            g_controls.yokeY = d.yokeY;
            g_controls.rudderPedal = d.rudderPedal;
            g_controls.throttleLever = d.throttleLever;
            // MSFS 2024 bindings may not send SMOKE_TOGGLE as a key event, but the smoke state still flips.
            const int smoke = d.smokeEnable >= 0.5 ? 1 : 0;
            if (g_lastSmoke >= 0 && smoke != g_lastSmoke) onToggleEvent(1);
            g_lastSmoke = smoke;
        }
        break;
    }

    default:
        break;
    }
}

void setupSimConnect() {
    HANDLE h = g_sim;

    SimConnect_AddToDataDefinition(h, DEF_AIRCRAFT, "PLANE LATITUDE", "degrees");
    SimConnect_AddToDataDefinition(h, DEF_AIRCRAFT, "PLANE LONGITUDE", "degrees");
    SimConnect_AddToDataDefinition(h, DEF_AIRCRAFT, "PLANE ALTITUDE", "meters");
    SimConnect_AddToDataDefinition(h, DEF_AIRCRAFT, "PLANE HEADING DEGREES TRUE", "degrees");
    SimConnect_AddToDataDefinition(h, DEF_AIRCRAFT, "GROUND ALTITUDE", "meters");
    SimConnect_AddToDataDefinition(h, DEF_AIRCRAFT, "VELOCITY WORLD X", "meters per second");
    SimConnect_AddToDataDefinition(h, DEF_AIRCRAFT, "VELOCITY WORLD Y", "meters per second");
    SimConnect_AddToDataDefinition(h, DEF_AIRCRAFT, "VELOCITY WORLD Z", "meters per second");
    SimConnect_AddToDataDefinition(h, DEF_AIRCRAFT, "YOKE X POSITION", "position");
    SimConnect_AddToDataDefinition(h, DEF_AIRCRAFT, "YOKE Y POSITION", "position");
    SimConnect_AddToDataDefinition(h, DEF_AIRCRAFT, "RUDDER PEDAL POSITION", "position");
    SimConnect_AddToDataDefinition(h, DEF_AIRCRAFT, "GENERAL ENG THROTTLE LEVER POSITION:1", "percent");
    SimConnect_AddToDataDefinition(h, DEF_AIRCRAFT, "SMOKE ENABLE", "bool");
    SimConnect_AddToDataDefinition(h, DEF_AIRCRAFT, "AMBIENT WIND VELOCITY", "meters per second");
    SimConnect_AddToDataDefinition(h, DEF_AIRCRAFT, "AMBIENT WIND DIRECTION", "degrees");
    SimConnect_RequestDataOnSimObject(h, REQ_AIRCRAFT, DEF_AIRCRAFT, SIMCONNECT_OBJECT_ID_USER,
                                      SIMCONNECT_PERIOD_SIM_FRAME);

    SimConnect_SubscribeToSystemEvent(h, EVT_FRAME, "Frame");
    SimConnect_SubscribeToSystemEvent(h, EVT_PAUSE, "Pause");
    SimConnect_SubscribeToSystemEvent(h, EVT_SIM_STOP, "SimStop");

    for (const AxisEvent& e : kAxisEvents) SimConnect_MapClientEventToSimEvent(h, e.id, e.simEvent);
    for (int i = 0; i < 3; ++i) SimConnect_MapClientEventToSimEvent(h, EVT_TOGGLE_1 + i, kToggleEvents[i]);
    SimConnect_MapClientEventToSimEvent(h, EVT_FREEZE_POSITION, "FREEZE_LATITUDE_LONGITUDE_SET");
    SimConnect_MapClientEventToSimEvent(h, EVT_FREEZE_ALTITUDE, "FREEZE_ALTITUDE_SET");
    SimConnect_MapClientEventToSimEvent(h, EVT_FREEZE_ATTITUDE, "FREEZE_ATTITUDE_SET");

    // Start in listen-only mode (drone off): the aircraft keeps its controls, the panel still sees the sticks.
    for (const AxisEvent& e : kAxisEvents) SimConnect_AddClientEventToNotificationGroup(h, GROUP_AXES_LISTEN, e.id, FALSE);
    SimConnect_SetNotificationGroupPriority(h, GROUP_AXES_LISTEN, SIMCONNECT_GROUP_PRIORITY_HIGHEST);
    g_axesMasked = false;

    g_toggleEventInGroup = 0;
    updateToggleEventGroup();

    SimConnect_CallDispatch(h, onSimConnect, nullptr);
}

// ---------- CommBus (panel) ----------

void onPanelSet(const char* buf, unsigned int size, void*) {
    const double oldFreeze = g_settings.freezeAircraft;
    const double oldRonin = g_settings.roninMode;
    const double oldInspire = g_settings.inspireEnabled;
    const double oldWing = g_settings.wingEnabled;
    if (applySettings(g_settings, buf, size) > 0) {
        // Turning Inspire mode on starts with the camera level.
        if (oldInspire < 0.5 && g_settings.inspireEnabled >= 0.5) {
            g_settings.inspireCamTilt = 0;
            g_settings.wingEnabled = 0; // one flight type at a time
        }
        if (oldWing < 0.5 && g_settings.wingEnabled >= 0.5) {
            g_settings.inspireEnabled = 0;
            if (g_play != PlayNone) stopReplay();
        }
        g_settingsDirty = true;
        g_saveTimer = 0;
        updateToggleEventGroup();
        if (g_drone.active && oldFreeze != g_settings.freezeAircraft) setFreeze(g_settings.freezeAircraft >= 0.5);
        if (oldRonin != g_settings.roninMode) g_gimbal.setMode(static_cast<int>(g_settings.roninMode + 0.5));
    }
    sendSettings();
}

void onPanelCommand(const char* buf, unsigned int size, void*) {
    std::string cmd(buf, strnlen(buf, size));
    while (!cmd.empty() && (cmd.back() == '\0' || cmd.back() == ' ')) cmd.pop_back();

    // "start" / "stop" are idempotent: a command delivered twice cannot switch the drone back.
    if (cmd == "start" || cmd == "stop" || cmd == "toggle") {
        if (cmd == "toggle") toggleDrone();
        else if (cmd == "start" && !g_drone.active) startDrone();
        else if (cmd == "stop" && g_drone.active) stopDrone();
        sendState();
    } else if (cmd == "getSettings") {
        sendSettings();
        sendState();
    } else if (cmd == "resetRates") {
        resetRates(g_settings);
        g_settingsDirty = true;
        sendSettings();
    } else if (cmd == "resetAll") {
        g_settings = Settings{};
        g_settingsDirty = true;
        updateToggleEventGroup();
        sendSettings();
    } else if (cmd == "clearError") {
        g_error.clear();
    } else if (cmd == "replayGimbal") {
        startReplay(PlayGimbal);
        sendState();
    } else if (cmd == "replayFpv") {
        startReplay(PlayFpv);
        sendState();
    } else if (cmd == "stopReplay") {
        stopReplay();
        sendState();
    } else if (cmd == "deleteFlight") {
        stopReplay();
        g_take.clear();
        g_gtrack.clear();
        sendState();
    } else if (cmd == "recordGimbal") {
        if (g_play != PlayGimbal) {
            g_error = "Start Replay gimbal first, then record the gimbal.";
        } else {
            g_gimbalRecording = !g_gimbalRecording;
            if (g_gimbalRecording) g_gtrack.beginAt(g_playTime);
        }
        sendState();
    } else if (cmd == "clearGimbal") {
        g_gtrack.clear();
        g_gimbalRecording = false;
        sendState();
    } else if (cmd == "recenter") {
        g_gimbal.recenter();
    } else if (cmd == "resetWing") {
        resetWing(g_settings);
        g_settingsDirty = true;
        sendSettings();
    } else if (cmd == "resetInspire") {
        resetInspireMode(g_settings);
        g_settingsDirty = true;
        sendSettings();
    } else if (cmd == "resetGimbal") {
        resetGimbal(g_settings);
        g_settingsDirty = true;
        sendSettings();
    }
}

} // namespace

// ---------- Module entry points ----------

extern "C" MSFS_CALLBACK void module_init(void) {
    std::printf("[FPV Drone] module_init\n");
    loadSettingsFile(g_settings, kSettingsPath);

    fsCommBusRegister("FpvDrone.Set", onPanelSet, nullptr);
    fsCommBusRegister("FpvDrone.Command", onPanelCommand, nullptr);

    fsCameraSubscribeToStatusUpdate(onCameraStatus, nullptr);
    pollCameraStatus();

    if (SUCCEEDED(SimConnect_Open(&g_sim, "FPV Drone", (HWND) nullptr, 0, 0, 0))) {
        setupSimConnect();
        std::printf("[FPV Drone] SimConnect ready\n");
    } else {
        g_sim = 0;
        std::printf("[FPV Drone] SimConnect_Open failed\n");
    }
}

// Called every frame just before drawing (MSFS 2024). Best moment to place the camera.
extern "C" MSFS_CALLBACK void module_late_update(float dTime) {
    g_framesWithoutLateUpdate = 0;
    step(dTime);
}

extern "C" MSFS_CALLBACK void module_deinit(void) {
    if (g_settingsDirty) saveSettingsFile(g_settings, kSettingsPath);
    if (g_sim) {
        if (g_drone.active) stopDrone();
        SimConnect_Close(g_sim);
        g_sim = 0;
    }
    fsCameraUnsubscribeToStatusUpdate();
    fsCommBusUnregister("FpvDrone.Set", onPanelSet);
    fsCommBusUnregister("FpvDrone.Command", onPanelCommand);
}
