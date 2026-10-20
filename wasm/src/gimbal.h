#pragma once
// 3-axis camera gimbal (DJI Ronin / RS 4 Pro style), ported from the UEVR plugin and the
// Assetto Corsa mod. Angles in degrees, MSFS / ENU convention:
//   yaw   = heading, clockwise from north
//   pitch = + up (tilt can go past -90: gimbal hung under the drone, looking backward)
//   roll  = + right side down

#include "math3d.h"
#include "settings.h"

enum RoninMode { RoninLock = 0, RoninPanFollow = 1, RoninPanTiltFollow = 2, RoninFpv = 3 };

struct GimbalInput {
    double pan = 0, tilt = 0, roll = 0; // -1..1: + right, + up, + clockwise
};

struct GimbalAngles {
    double yaw = 0, pitch = 0, roll = 0;
};

class Gimbal {
public:
    static constexpr double kTiltMin = -110.0, kTiltMax = 85.0;

    // Starts aligned with the drone (heading, plus its pitch / roll in the modes that follow them).
    void reset(const Basis& droneBody, int mode);
    // Changes the follow mode without a jump of the camera.
    void setMode(int mode);
    // Brings the camera back in front of the drone (the stick offsets go back to zero).
    void recenter();

    // cameraPos: camera position this frame (meters, any fixed frame), for the Ronin drift.
    void update(const GimbalInput& in, const Settings& s, const Basis& droneBody, const Vec3& cameraPos, double dt);

    GimbalAngles angles() const; // final camera angles, drift included
    double driftDeg() const { return drift_; }
    int mode() const { return mode_; }

    static double hfovDeg(double focalMm) { return radToDeg(2.0 * std::atan(36.0 / (2.0 * std::max(focalMm, 0.5)))); }
    static double vfovDeg(double focalMm) { return radToDeg(2.0 * std::atan(20.25 / (2.0 * std::max(focalMm, 0.5)))); }

private:
    int mode_ = RoninLock;
    // Followed drone attitude (filtered by the follow speed)
    double fYaw_ = 0, fPitch_ = 0, fRoll_ = 0;
    // Operator offsets on top of the followed attitude (absolute angles in Lock)
    double offYaw_ = 0, offPitch_ = 0, offRoll_ = 0;
    double panSpeed_ = 0, tiltSpeed_ = 0, rollSpeed_ = 0;

    bool hasPrev_ = false;
    Vec3 prevPos_, prevVel_, accel_;
    double drift_ = 0;

    GimbalAngles followed(int mode) const;
    void setFollowed(const Basis& droneBody);
};
