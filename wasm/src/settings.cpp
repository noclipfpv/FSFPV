#include "settings.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#define DEF(name, mn, mx) {#name, &Settings::name, mn, mx}

const SettingDef kSettingDefs[] = {
    DEF(batteryCells, 1, 8), DEF(propDiameter, 2, 8), DEF(propPitch, 2, 6), DEF(motorKv, 500, 4000),
    DEF(droneMass, 0.05, 3), DEF(droneSurfaceArea, 0, 0.1), DEF(minimalSurfaceAreaCoefficient, 0, 1),
    DEF(cameraAngle, -30, 90), DEF(cameraFov, 20, 150),
    DEF(linearAcceleration, 0, 1), DEF(collision, 0, 1),

    DEF(ratesType, 0, 2),
    DEF(rollRate, 0, 3), DEF(rollSuper, 0, 0.99), DEF(rollExpo, 0, 1), DEF(rollCenter, 10, 2000), DEF(rollMax, 0, 2000), DEF(rollCurve, 0, 1),
    DEF(pitchRate, 0, 3), DEF(pitchSuper, 0, 0.99), DEF(pitchExpo, 0, 1), DEF(pitchCenter, 10, 2000), DEF(pitchMax, 0, 2000), DEF(pitchCurve, 0, 1),
    DEF(yawRate, 0, 3), DEF(yawSuper, 0, 0.99), DEF(yawExpo, 0, 1), DEF(yawCenter, 10, 2000), DEF(yawMax, 0, 2000), DEF(yawCurve, 0, 1),

    DEF(airDensity, 0, 3), DEF(airDrag, 0, 3), DEF(timeScale, 0.05, 2), DEF(gravity, -1, 3),
    DEF(floorMode, 0, 1), DEF(floorOffset, -200, 200), DEF(floorAltitude, -500, 9000),
    DEF(groundFriction, 0, 1), DEF(bounciness, 0, 1), DEF(windEnabled, 0, 1), DEF(windStrength, 0, 200),

    DEF(mode3d, 0, 1), DEF(invertThrottle, 0, 1), DEF(invertRoll, 0, 1), DEF(invertPitch, 0, 1), DEF(invertYaw, 0, 1),
    DEF(stickDeadzone, 0, 50),
    DEF(toggleEvent, 0, 3), DEF(freezeAircraft, 0, 1), DEF(inputSource, 0, 1),

    DEF(inspireEnabled, 0, 1), DEF(inspireMode, 0, 2), DEF(inspireCamTilt, -110, 60), DEF(inspireCamFov, 20, 150),
    DEF(tripodMaxSpeed, 0.5, 40), DEF(tripodClimb, 0.5, 15), DEF(tripodYawRate, 5, 300), DEF(tripodResponse, 0.05, 3),
    DEF(tripodMaxAccel, 0.5, 30), DEF(tripodMaxTilt, 0, 60), DEF(tripodExpo, 0, 1),
    DEF(normalMaxSpeed, 0.5, 40), DEF(normalClimb, 0.5, 15), DEF(normalYawRate, 5, 300), DEF(normalResponse, 0.05, 3),
    DEF(normalMaxAccel, 0.5, 30), DEF(normalMaxTilt, 0, 60), DEF(normalExpo, 0, 1),
    DEF(sportMaxSpeed, 0.5, 40), DEF(sportClimb, 0.5, 15), DEF(sportYawRate, 5, 300), DEF(sportResponse, 0.05, 3),
    DEF(sportMaxAccel, 0.5, 30), DEF(sportMaxTilt, 0, 60), DEF(sportExpo, 0, 1),

    DEF(wingEnabled, 0, 1), DEF(wingRollScale, 0.1, 1.5), DEF(wingPitchScale, 0.1, 1.5), DEF(wingThrust, 0.3, 3),
    DEF(wingGlide, 1, 10),
    DEF(dualOpEnabled, 0, 1), DEF(replaySpeed, 0.1, 1), DEF(replayLoop, 0, 1), DEF(focalLength, 6, 200),
    DEF(roninMode, 0, 3), DEF(followSpeed, 0, 100), DEF(roninDrift, 0, 1), DEF(driftStrength, 0, 20),
    DEF(driftRecovery, 0.05, 3),
    DEF(gimbalPanSpeed, 5, 360), DEF(gimbalTiltSpeed, 5, 360), DEF(gimbalRollSpeed, 5, 360),
    DEF(gimbalPanSmooth, 0, 2), DEF(gimbalTiltSmooth, 0, 2), DEF(gimbalRollSmooth, 0, 2),
    DEF(gimbalPanExpo, 0, 1), DEF(gimbalTiltExpo, 0, 1), DEF(gimbalRollExpo, 0, 1), DEF(gimbalInvertTilt, 0, 1),

    DEF(camInvertPitch, 0, 1), DEF(camInvertBank, 0, 1), DEF(camInvertHeading, 0, 1),
    DEF(camSwapLatLon, 0, 1), DEF(camAltitudeFeet, 0, 1), DEF(spawnBehind, -50, 300), DEF(spawnAbove, -20, 200), DEF(camFovHorizontal, 0, 1),
};
#undef DEF

const int kSettingCount = static_cast<int>(sizeof(kSettingDefs) / sizeof(kSettingDefs[0]));

static const SettingDef* findDef(const char* key, size_t len) {
    for (const SettingDef& d : kSettingDefs)
        if (std::strlen(d.key) == len && std::strncmp(d.key, key, len) == 0) return &d;
    return nullptr;
}

int applySettings(Settings& s, const char* text, unsigned int size) {
    int applied = 0;
    const char* p = text;
    const char* end = text + size;
    while (p < end && *p) {
        const char* lineEnd = p;
        while (lineEnd < end && *lineEnd && *lineEnd != '\n' && *lineEnd != ';') ++lineEnd;
        const char* eq = static_cast<const char*>(std::memchr(p, '=', static_cast<size_t>(lineEnd - p)));
        if (eq) {
            const char* k = p;
            while (k < eq && (*k == ' ' || *k == '\r')) ++k;
            size_t len = static_cast<size_t>(eq - k);
            while (len > 0 && k[len - 1] == ' ') --len;
            if (const SettingDef* d = findDef(k, len)) {
                char buf[64] = {};
                const size_t vlen = std::min<size_t>(static_cast<size_t>(lineEnd - eq - 1), sizeof(buf) - 1);
                std::memcpy(buf, eq + 1, vlen);
                char* parsedEnd = nullptr;
                const double v = std::strtod(buf, &parsedEnd);
                if (parsedEnd != buf && std::isfinite(v)) {
                    s.*(d->field) = std::clamp(v, d->min, d->max);
                    ++applied;
                }
            }
        }
        p = lineEnd + 1;
    }
    return applied;
}

std::string serializeSettings(const Settings& s, char separator) {
    std::string out;
    char line[96];
    for (const SettingDef& d : kSettingDefs) {
        std::snprintf(line, sizeof(line), "%s=%.6g%c", d.key, s.*(d.field), separator);
        out += line;
    }
    return out;
}

std::string settingsToJson(const Settings& s) {
    std::string out = "{";
    char item[96];
    for (int i = 0; i < kSettingCount; ++i) {
        std::snprintf(item, sizeof(item), "%s\"%s\":%.6g", i ? "," : "", kSettingDefs[i].key, s.*(kSettingDefs[i].field));
        out += item;
    }
    out += "}";
    return out;
}

void resetRates(Settings& s) {
    const Settings d;
    const double type = s.ratesType;
    for (const SettingDef& def : kSettingDefs) {
        const char* k = def.key;
        if (!std::strncmp(k, "roll", 4) || !std::strncmp(k, "pitch", 5) || !std::strncmp(k, "yaw", 3))
            s.*(def.field) = d.*(def.field);
    }
    s.ratesType = type;
    if (type == 1) s.rollExpo = s.pitchExpo = s.yawExpo = 0.5; // Actual rates default expo
}

bool loadSettingsFile(Settings& s, const char* path) {
    FILE* f = std::fopen(path, "rb");
    if (!f) return false;
    std::string text;
    char buf[1024];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) text.append(buf, n);
    std::fclose(f);
    return applySettings(s, text.c_str(), static_cast<unsigned int>(text.size())) > 0;
}

bool saveSettingsFile(const Settings& s, const char* path) {
    FILE* f = std::fopen(path, "wb");
    if (!f) return false;
    const std::string text = serializeSettings(s, '\n');
    const bool ok = std::fwrite(text.data(), 1, text.size(), f) == text.size();
    std::fclose(f);
    return ok;
}

void resetInspireMode(Settings& s) {
    const Settings d;
    const char* prefix = s.inspireMode < 0.5 ? "tripod" : (s.inspireMode < 1.5 ? "normal" : "sport");
    const size_t n = std::strlen(prefix);
    for (const SettingDef& def : kSettingDefs)
        if (!std::strncmp(def.key, prefix, n)) s.*(def.field) = d.*(def.field);
}

void resetWing(Settings& s) {
    const Settings d;
    for (const SettingDef& def : kSettingDefs)
        if (!std::strncmp(def.key, "wing", 4) && std::strcmp(def.key, "wingEnabled")) s.*(def.field) = d.*(def.field);
}

void resetGimbal(Settings& s) {
    const Settings d;
    for (const SettingDef& def : kSettingDefs)
        if (!std::strncmp(def.key, "gimbal", 6) || !std::strcmp(def.key, "followSpeed")) s.*(def.field) = d.*(def.field);
}

InspireParams inspireParams(const Settings& s) {
    if (s.inspireMode < 0.5)
        return {s.tripodMaxSpeed, s.tripodClimb, s.tripodYawRate, s.tripodResponse, s.tripodMaxAccel, s.tripodMaxTilt, s.tripodExpo};
    if (s.inspireMode < 1.5)
        return {s.normalMaxSpeed, s.normalClimb, s.normalYawRate, s.normalResponse, s.normalMaxAccel, s.normalMaxTilt, s.normalExpo};
    return {s.sportMaxSpeed, s.sportClimb, s.sportYawRate, s.sportResponse, s.sportMaxAccel, s.sportMaxTilt, s.sportExpo};
}
