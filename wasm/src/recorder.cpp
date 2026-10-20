#include "recorder.h"

#include <algorithm>
#include <cmath>

namespace {

Vec3 v(float x, float y, float z) { return {x, y, z}; }

template <typename F>
size_t findFrame(const std::vector<F>& frames, double t, size_t& hint) {
    size_t i = hint;
    if (i >= frames.size() || frames[i].t > t) i = 0;
    while (i + 1 < frames.size() && frames[i + 1].t <= t) ++i;
    hint = i;
    return i;
}

Basis lerpBasis(const Vec3& fa, const Vec3& ua, const Vec3& fb, const Vec3& ub, double k) {
    Basis b;
    b.forward = (fa + (fb - fa) * k).normalized();
    b.up = (ua + (ub - ua) * k).normalized();
    b.orthonormalize();
    return b;
}

} // namespace

void FlightTake::clear() {
    frames_.clear();
    frozen_ = false;
    hint_ = 0;
}

void FlightTake::record(double dt, const GeoPoint& droneOrigin, const Vec3& localPos, const Basis& body,
                        const Basis& camera) {
    if (frozen_ || frames_.size() >= kMaxFrames) return;
    if (frames_.empty()) origin_ = droneOrigin;
    const double t = frames_.empty() ? 0.0 : frames_.back().t + std::max(dt, 0.0);
    if (!frames_.empty() && t <= frames_.back().t) return; // paused frame

    // A relaunch moves the drone origin: keep every frame relative to the take origin.
    const Vec3 p = localPos + geoToLocal(origin_, droneOrigin);
    frames_.push_back({static_cast<float>(t), static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z),
                       static_cast<float>(body.forward.x), static_cast<float>(body.forward.y), static_cast<float>(body.forward.z),
                       static_cast<float>(body.up.x), static_cast<float>(body.up.y), static_cast<float>(body.up.z),
                       static_cast<float>(camera.forward.x), static_cast<float>(camera.forward.y), static_cast<float>(camera.forward.z),
                       static_cast<float>(camera.up.x), static_cast<float>(camera.up.y), static_cast<float>(camera.up.z)});
}

FlightTake::Pose FlightTake::at(double t) const {
    Pose pose;
    if (frames_.empty()) return pose;
    const size_t i = findFrame(frames_, t, hint_);
    const Frame& a = frames_[i];
    const Frame& b = i + 1 < frames_.size() ? frames_[i + 1] : a;
    const double k = b.t > a.t ? std::clamp((t - a.t) / (b.t - a.t), 0.0, 1.0) : 0.0;
    pose.position = v(a.px, a.py, a.pz) + (v(b.px, b.py, b.pz) - v(a.px, a.py, a.pz)) * k;
    pose.body = lerpBasis(v(a.bfx, a.bfy, a.bfz), v(a.bux, a.buy, a.buz), v(b.bfx, b.bfy, b.bfz), v(b.bux, b.buy, b.buz), k);
    pose.camera = lerpBasis(v(a.cfx, a.cfy, a.cfz), v(a.cux, a.cuy, a.cuz), v(b.cfx, b.cfy, b.cfz), v(b.cux, b.cuy, b.cuz), k);
    return pose;
}

void GimbalTrack::beginAt(double t) {
    while (!frames_.empty() && frames_.back().t >= t) frames_.pop_back();
    hint_ = 0;
}

void GimbalTrack::record(double t, double yaw, double pitch, double roll, double focal) {
    const Frame f{static_cast<float>(t), static_cast<float>(yaw), static_cast<float>(pitch), static_cast<float>(roll),
                  static_cast<float>(focal)};
    if (!frames_.empty() && t <= frames_.back().t) {
        const float lastT = frames_.back().t;
        frames_.back() = f; // paused: overwrite the last sample, keep its time
        frames_.back().t = lastT;
        return;
    }
    if (frames_.size() < 120000) frames_.push_back(f);
}

bool GimbalTrack::at(double t, double& yaw, double& pitch, double& roll, double& focal) const {
    if (frames_.empty()) return false;
    const size_t i = findFrame(frames_, t, hint_);
    const Frame& a = frames_[i];
    const Frame& b = i + 1 < frames_.size() ? frames_[i + 1] : a;
    const double k = b.t > a.t ? std::clamp((t - a.t) / (b.t - a.t), 0.0, 1.0) : 0.0;
    yaw = wrap180(a.yaw + wrap180(b.yaw - a.yaw) * k);
    pitch = a.pitch + (b.pitch - a.pitch) * k;
    roll = wrap180(a.roll + wrap180(b.roll - a.roll) * k);
    focal = a.focal + (b.focal - a.focal) * k;
    return true;
}
