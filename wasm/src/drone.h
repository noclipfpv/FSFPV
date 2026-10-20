#pragma once
#include "math3d.h"
#include "settings.h"

// User aircraft data received from SimConnect every sim frame.
struct AircraftState {
    bool valid = false;
    GeoPoint position;     // lat/lon degrees, altitude meters MSL
    double headingDeg = 0; // true heading
    double groundAlt = 0;  // terrain altitude under the aircraft, meters MSL
    Vec3 velocity;         // ENU m/s
};

// Sticks after inversion, -1..1. Throttle is -1..1 too (bottom..top).
struct Sticks {
    double throttle = -1, roll = 0, pitch = 0, yaw = 0;
};

// Camera pose for this frame: geodetic origin + local ENU offset + aviation angles.
struct CameraPose {
    bool valid = false;
    GeoPoint origin;
    Vec3 local;
    Euler euler;
    double fovDeg = 90;
};

class Drone {
public:
    bool active = false;
    CameraPose pose;

    void start(const AircraftState& aircraft, const Settings& s, bool inheritVelocity);
    void stop();
    void update(double dt, const Sticks& sticks, const Settings& s);

    // deg/s for a stick input in -1..1. type: 0 Betaflight, 1 Actual, 2 KISS
    static double rate(int type, double input, double rate, double super, double expo, double center, double max,
                       double curve);

    const Vec3& position() const { return position_; }
    const Vec3& velocity() const { return velocity_; }
    double floorZ() const { return floorZ_; }
    double altitudeMsl() const { return origin_.alt + position_.z; }
    const Basis& body() const { return rot_; }
    const Basis& camera() const { return camera_; } // what the pilot sees (FPV or Inspire camera)
    const GeoPoint& origin() const { return origin_; }
    double aoaDeg() const { return aoaDeg_; }
    // Wind (ENU m/s) applied from the next update. Zero = calm air (the original calculation).
    void setWind(const Vec3& w) { wind_ = w; }
    bool wingWaiting() const { return wingMode_ && !wingLaunched_; } // fixed wing held in the hand

private:
    GeoPoint origin_;
    double launchGroundAlt_ = 0;
    Vec3 position_, velocity_;
    Basis rot_;
    double floorZ_ = -1e9;
    Basis camera_;
    double inspireYawRate_ = 0;
    Vec3 inspireAccel_;
    double horizonLag1_ = 0, horizonLag2_ = 0; // delayed body pitch seen by the Inspire tilt motor

    void updateAcro(double dt, const Sticks& st, const Settings& s);
    void updateInspire(double dt, const Sticks& st, const Settings& s);
    void updateWing(double dt, const Sticks& st, const Settings& s);
    void wingSubstep(double dt, const Sticks& st, const Settings& s);

    // Fixed wing
    bool wingMode_ = false, wingLaunched_ = false;
    Vec3 wingRates_;        // deg/s: x roll right, y pitch up, z yaw right
    double wingThrottle_ = 0;
    double aoaDeg_ = 0;
    Vec3 wind_;
    void collideWithFloor(const Settings& s);
};
