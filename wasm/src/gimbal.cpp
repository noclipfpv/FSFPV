#include "gimbal.h"

#include <algorithm>
#include <cmath>

namespace {

double expo(double x, double e) {
    x = std::clamp(x, -1.0, 1.0);
    return x * (1.0 - e) + x * x * x * e;
}

} // namespace

void Gimbal::setFollowed(const Basis& droneBody) {
    const Euler e = basisToEuler(droneBody.forward, droneBody.up);
    fYaw_ = wrap180(e.heading);
    fPitch_ = e.pitch;
    fRoll_ = e.bank;
}

GimbalAngles Gimbal::followed(int mode) const {
    GimbalAngles a;
    a.yaw = mode >= RoninPanFollow ? fYaw_ : 0.0;
    a.pitch = mode >= RoninPanTiltFollow ? fPitch_ : 0.0;
    a.roll = mode >= RoninFpv ? fRoll_ : 0.0;
    return a;
}

void Gimbal::reset(const Basis& droneBody, int mode) {
    mode_ = std::clamp(mode, 0, 3);
    setFollowed(droneBody);
    panSpeed_ = tiltSpeed_ = rollSpeed_ = 0;
    hasPrev_ = false;
    accel_ = {};
    drift_ = 0;
    recenter();
}

void Gimbal::recenter() {
    if (mode_ == RoninLock) {
        offYaw_ = fYaw_; // in front of the drone, level
        offPitch_ = 0;
        offRoll_ = 0;
    } else {
        offYaw_ = offPitch_ = offRoll_ = 0;
    }
}

void Gimbal::setMode(int mode) {
    mode = std::clamp(mode, 0, 3);
    if (mode == mode_) return;
    const GimbalAngles cur{wrap180(followed(mode_).yaw + offYaw_), followed(mode_).pitch + offPitch_,
                           wrap180(followed(mode_).roll + offRoll_)};
    mode_ = mode;
    const GimbalAngles fo = followed(mode_);
    offYaw_ = wrap180(cur.yaw - fo.yaw);
    offPitch_ = cur.pitch - fo.pitch;
    offRoll_ = wrap180(cur.roll - fo.roll);
}

void Gimbal::update(const GimbalInput& in, const Settings& s, const Basis& droneBody, const Vec3& cameraPos, double dt) {
    if (!(dt > 0.0)) return;

    // Operator speeds with Ronin-like inertia (per axis)
    panSpeed_ += (expo(in.pan, s.gimbalPanExpo) * s.gimbalPanSpeed - panSpeed_) * smoothAlpha(dt, s.gimbalPanSmooth);
    tiltSpeed_ += (expo(in.tilt, s.gimbalTiltExpo) * s.gimbalTiltSpeed - tiltSpeed_) * smoothAlpha(dt, s.gimbalTiltSmooth);
    rollSpeed_ += (expo(in.roll, s.gimbalRollExpo) * s.gimbalRollSpeed - rollSpeed_) * smoothAlpha(dt, s.gimbalRollSmooth);

    // Follow the drone (always tracked, so Recenter and mode changes know where the drone points)
    const Euler e = basisToEuler(droneBody.forward, droneBody.up);
    const double fa = smoothAlpha(dt, std::clamp(s.followSpeed, 0.0, 100.0) / 100.0 * 1.5);
    fYaw_ = wrap180(fYaw_ + wrap180(e.heading - fYaw_) * fa);
    fPitch_ += (e.pitch - fPitch_) * fa;
    fRoll_ = wrap180(fRoll_ + wrap180(e.bank - fRoll_) * fa);

    // Stick offsets
    offYaw_ = wrap180(offYaw_ + panSpeed_ * dt);
    offPitch_ += tiltSpeed_ * dt;
    offRoll_ = wrap180(offRoll_ + rollSpeed_ * dt);
    const double fp = followed(mode_).pitch;
    offPitch_ = std::clamp(offPitch_, kTiltMin - fp, kTiltMax - fp);

    // Ronin roll drift: under lateral g the payload swings toward the apparent gravity, then recovers.
    if (hasPrev_) {
        const Vec3 vel = (cameraPos - prevPos_) / dt;
        const Vec3 acc = (vel - prevVel_) / dt;
        accel_ += (acc - accel_) * smoothAlpha(dt, 0.12); // filter numeric noise
        prevVel_ = vel;
    } else {
        prevVel_ = {};
        hasPrev_ = true;
    }
    prevPos_ = cameraPos;

    double driftTarget = 0.0;
    if (s.roninDrift >= 0.5) {
        const double yr = degToRad(angles().yaw);
        const Vec3 camRight{std::cos(yr), -std::sin(yr), 0.0};
        const double lateralG = accel_.dot(camRight) / 9.81;
        driftTarget = std::clamp(-lateralG * s.driftStrength, -10.0, 10.0);
    }
    drift_ += (driftTarget - drift_) * smoothAlpha(dt, s.driftRecovery);
}

GimbalAngles Gimbal::angles() const {
    const GimbalAngles fo = followed(mode_);
    GimbalAngles a;
    a.yaw = wrap180(fo.yaw + offYaw_);
    a.pitch = std::clamp(fo.pitch + offPitch_, kTiltMin, kTiltMax);
    a.roll = wrap180(fo.roll + offRoll_) + drift_;
    return a;
}
