#pragma once
#include <string>

// All settings are doubles (booleans are 0/1, choices are indices) so they can be
// exchanged with the panel and saved as plain "key=value" lines.
struct Settings {
    // Drone
    double batteryCells = 6, propDiameter = 5, propPitch = 4.3, motorKv = 1700;
    double droneMass = 0.881;          // kg
    double droneSurfaceArea = 0.015;   // m^2
    double minimalSurfaceAreaCoefficient = 0.4;
    double cameraAngle = 25, cameraFov = 90;
    double linearAcceleration = 0, collision = 0; // flat floor collision, off by default

    // Rates. ratesType: 0 = Betaflight, 1 = Actual, 2 = KISS
    double ratesType = 0;
    double rollRate = 1, rollSuper = 0.7, rollExpo = 0, rollCenter = 182, rollMax = 667, rollCurve = 0;
    double pitchRate = 1, pitchSuper = 0.7, pitchExpo = 0, pitchCenter = 182, pitchMax = 667, pitchCurve = 0;
    double yawRate = 1, yawSuper = 0.7, yawExpo = 0, yawCenter = 182, yawMax = 667, yawCurve = 0;

    // Physics. floorMode: 0 = terrain under the aircraft at launch (+ offset), 1 = fixed altitude
    double airDensity = 1, airDrag = 1, timeScale = 1, gravity = 1;
    double floorMode = 0, floorOffset = 0, floorAltitude = 0;
    double groundFriction = 0.1, bounciness = 0.1;
    // Wind from the sim (AMBIENT WIND at the aircraft). Off = calm air, the original calculation.
    double windEnabled = 1, windStrength = 100; // %

    // Controls. toggleEvent: 0 = panel only, 1 = SMOKE_TOGGLE, 2 = TOGGLE_TAIL_HOOK_HANDLE, 3 = TOGGLE_WATER_RUDDER
    double mode3d = 0;
    double invertThrottle = 0, invertRoll = 1, invertPitch = 0, invertYaw = 1; // validated in MSFS 2024
    double stickDeadzone = 0; // %, roll/pitch/yaw, applied after inversion
    double toggleEvent = 0;
    double freezeAircraft = 1;
    // inputSource: 0 = assigned axes (key events), 1 = aircraft control positions (yoke, pedals, throttle lever)
    // Axis events (received through SimConnect) is the mode validated in MSFS 2024 with the inversions below.
    double inputSource = 0;

    // Inspire mode (DJI Inspire 3 style): sticks command speeds, centered sticks hold position.
    // inspireMode: 0 = Tripod, 1 = Normal, 2 = Sport. Each mode keeps its own values.
    double inspireEnabled = 0, inspireMode = 1;
    double inspireCamTilt = 0, inspireCamFov = 80; // shared by the three modes
    double tripodMaxSpeed = 3, tripodClimb = 1.5, tripodYawRate = 30, tripodResponse = 1.0, tripodMaxAccel = 2, tripodMaxTilt = 10, tripodExpo = 0.5;
    double normalMaxSpeed = 18, normalClimb = 6, normalYawRate = 90, normalResponse = 0.6, normalMaxAccel = 6, normalMaxTilt = 35, normalExpo = 0.35;
    double sportMaxSpeed = 26, sportClimb = 8, sportYawRate = 150, sportResponse = 0.35, sportMaxAccel = 12, sportMaxTilt = 40, sportExpo = 0.2;

    // Fixed wing (TBS Mojito style flying wing): same sticks and rates as the drone, no yaw (no rudder).
    double wingEnabled = 0;
    double wingRollScale = 0.6, wingPitchScale = 0.35; // fraction of the drone rates
    double wingThrust = 1.4;                            // thrust to weight
    double wingGlide = 4;                               // trim angle of attack (deg): higher = slower, floatier

    // Dual OP: fly, then replay the flight while operating a gimbal with the same sticks.
    double dualOpEnabled = 0, replaySpeed = 1, replayLoop = 1;
    double focalLength = 20;          // mm, full frame 36 x 20.25 (16:9), replaces the FOV in replays
    // roninMode: 0 = Lock, 1 = Pan follow, 2 = Pan/Tilt follow, 3 = FPV
    double roninMode = 0, followSpeed = 30; // 0 = follows immediately, 100 = very slowly
    double roninDrift = 0, driftStrength = 6, driftRecovery = 0.45; // deg per g, seconds
    double gimbalPanSpeed = 120, gimbalTiltSpeed = 120, gimbalRollSpeed = 120;       // deg/s
    double gimbalPanSmooth = 0.35, gimbalTiltSmooth = 0.35, gimbalRollSmooth = 0.35; // s
    double gimbalPanExpo = 0, gimbalTiltExpo = 0, gimbalRollExpo = 0;
    double gimbalInvertTilt = 0;

    // Camera calibration (the MSFS Camera API conventions are not fully documented)
    double camInvertPitch = 1, camInvertBank = 1, camInvertHeading = 0;
    double camSwapLatLon = 0, camAltitudeFeet = 0;
    double spawnBehind = 10, spawnAbove = 3; // m
    double camFovHorizontal = 0;              // 1 if MSFS treats the camera FOV as horizontal
};

struct SettingDef {
    const char* key;
    double Settings::*field;
    double min, max;
};

extern const SettingDef kSettingDefs[];
extern const int kSettingCount;

// Applies "key=value" lines (separated by '\n' or ';'). Values are clamped. Returns the number of keys applied.
int applySettings(Settings& s, const char* text, unsigned int size);
std::string serializeSettings(const Settings& s, char separator);   // "key=value" lines
std::string settingsToJson(const Settings& s);
void resetRates(Settings& s);
void resetInspireMode(Settings& s); // the selected mode only
void resetWing(Settings& s);
void resetGimbal(Settings& s);

struct InspireParams {
    double maxSpeed, climb, yawRate, response, maxAccel, maxTilt, expo;
};
InspireParams inspireParams(const Settings& s);

bool loadSettingsFile(Settings& s, const char* path);
bool saveSettingsFile(const Settings& s, const char* path);
