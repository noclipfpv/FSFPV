#pragma once
// Dual OP recordings: the flight ("take") and the gimbal track recorded on top of it.

#include <vector>

#include "math3d.h"

class FlightTake {
public:
    struct Frame {
        float t;
        float px, py, pz;         // meters from the take origin (ENU)
        float bfx, bfy, bfz;      // drone body forward
        float bux, buy, buz;      // drone body up
        float cfx, cfy, cfz;      // pilot camera forward (FPV or Inspire camera)
        float cux, cuy, cuz;      // pilot camera up
    };

    struct Pose {
        Vec3 position;
        Basis body, camera;
    };

    void clear();
    // Appends a frame dt seconds after the previous one. Ignored once the take is frozen.
    void record(double dt, const GeoPoint& droneOrigin, const Vec3& localPos, const Basis& body, const Basis& camera);
    void freeze() { frozen_ = true; } // a replay happened: keep this take until it is deleted

    bool empty() const { return frames_.size() < 2; }
    bool frozen() const { return frozen_; }
    double duration() const { return frames_.empty() ? 0.0 : frames_.back().t; }
    const GeoPoint& origin() const { return origin_; }

    Pose at(double t) const; // interpolated

private:
    std::vector<Frame> frames_;
    GeoPoint origin_;
    bool frozen_ = false;
    mutable size_t hint_ = 0;
    static constexpr size_t kMaxFrames = 120000; // ~33 min at 60 fps
};

// Final gimbal angles + focal over the take time. Replays the same whatever the Ronin mode.
class GimbalTrack {
public:
    struct Frame {
        float t, yaw, pitch, roll, focal;
    };

    void clear() { frames_.clear(); hint_ = 0; }
    bool empty() const { return frames_.empty(); }
    double duration() const { return frames_.empty() ? 0.0 : frames_.back().t; }

    // Starting a recording replaces everything from time t on.
    void beginAt(double t);
    // Adds a sample (a paused / repeated time overwrites the last one).
    void record(double t, double yaw, double pitch, double roll, double focal);
    bool at(double t, double& yaw, double& pitch, double& roll, double& focal) const;

private:
    std::vector<Frame> frames_;
    mutable size_t hint_ = 0;
};
