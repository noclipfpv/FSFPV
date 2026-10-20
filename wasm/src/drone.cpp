#include "drone.h"

#include <algorithm>
#include <cmath>

// ---------- Physics helpers (ported from the Assetto Corsa drone.lua) ----------

static double thrust(bool linear, double airDensity, double propDiameter, double propPitch, double motorKv,
                     double batteryCells, double throttle, double inflowVelocity) {
    if (linear) return throttle * motorKv / 30.0;

    const double a = airDensity * (3.14 * std::pow(0.0254 * propDiameter, 2) / 4.0) *
                     std::pow(propDiameter / (3.29547 * (propPitch + 0.5)), 1.5) * 1.5 * 4.0;
    const double maxRpm = std::min(motorKv * batteryCells * 3.7 * (5.4 / std::pow(propDiameter, 1.1)),
                                   motorKv * batteryCells * 3.7);
    const double rpm = throttle * maxRpm;
    const double maxVe = maxRpm * 0.0254 * (propPitch + 0.5) / 60.0;
    const double ve = rpm * 0.0254 * (propPitch + 0.5) / 60.0;
    const double ic = maxVe > 0 ? std::max(0.2, (maxVe - std::abs(inflowVelocity)) / maxVe) : 0.2;
    return a * (std::abs(ve) * ve) * ic;
}

static Vec3 airDragForce(double density, const Vec3& velocity, double coefficient, double area, double minAreaCoeff,
                         double inflowCoefficient) {
    const double areaCoeff = inflowCoefficient + minAreaCoeff * (1.0 - inflowCoefficient);
    const double speed = velocity.length();
    return velocity.normalized() * (-0.5 * density * speed * speed * (2.5 * coefficient) * (area * areaCoeff));
}

static double applyDeadzone(double value, double deadzonePercent) {
    const double dz = std::clamp(deadzonePercent / 100.0, 0.0, 0.99);
    if (std::abs(value) < dz) return 0.0;
    const double sign = value < 0 ? -1.0 : 1.0;
    return sign * (std::abs(value) - dz) / (1.0 - dz);
}

double Drone::rate(int type, double input, double rate, double super, double expo, double center, double max,
                   double curve) {
    const double inputAbs = std::abs(input);
    if (type == 1) { // Actual
        const double e = inputAbs * (std::pow(input, 5) * expo + input * (1.0 - expo));
        const double stickMovement = std::max(0.0, max - center);
        return input * center + stickMovement * e;
    }
    if (type == 2) { // KISS
        const double useRates = 1.0 / std::clamp(1.0 - inputAbs * super, 0.01, 1.0);
        const double rcCommand = (std::pow(input, 3) * curve + input * (1.0 - curve)) * (rate / 10.0);
        return std::clamp(2000.0 * useRates * rcCommand, -1998.0, 1998.0);
    }
    // Betaflight
    const double rcCommand = input * std::pow(inputAbs, 3) * expo + input * (1.0 - expo);
    double rcRate = rate;
    if (rcRate > 2.0) rcRate += 14.54 * (rcRate - 2.0);
    const double superFactor = 1.0 / std::clamp(1.0 - inputAbs * super, 0.01, 1.0);
    return 200.0 * rcRate * rcCommand * superFactor;
}

// ---------- Lifecycle ----------

void Drone::start(const AircraftState& aircraft, const Settings& s, bool inheritVelocity) {
    origin_ = aircraft.position;
    launchGroundAlt_ = aircraft.groundAlt;

    const double h = degToRad(aircraft.headingDeg);
    const Vec3 flatForward{std::sin(h), std::cos(h), 0.0};
    position_ = flatForward * -s.spawnBehind + Vec3{0, 0, s.spawnAbove};
    velocity_ = inheritVelocity ? aircraft.velocity : Vec3{};
    rot_.forward = flatForward;
    rot_.up = {0, 0, 1};
    rot_.orthonormalize();
    inspireYawRate_ = 0;
    inspireAccel_ = {};
    wingMode_ = wingLaunched_ = false;
    wingRates_ = {};
    wingThrottle_ = 0;
    aoaDeg_ = 0;
    horizonLag1_ = horizonLag2_ = 0;
    camera_ = rot_;
    active = true;
    pose.valid = false;
}

void Drone::stop() {
    active = false;
    pose.valid = false;
}

// ---------- Per frame ----------

void Drone::update(double dt, const Sticks& sticks, const Settings& s) {
    if (!active) return;
    dt *= s.timeScale;
    if (!(dt > 0)) return;

    floorZ_ = s.floorMode >= 0.5 ? s.floorAltitude - origin_.alt : launchGroundAlt_ + s.floorOffset - origin_.alt;

    Sticks st = sticks;
    st.roll = applyDeadzone(sticks.roll, s.stickDeadzone);
    st.pitch = applyDeadzone(sticks.pitch, s.stickDeadzone);
    st.yaw = applyDeadzone(sticks.yaw, s.stickDeadzone);

    const bool wing = s.wingEnabled >= 0.5;
    const bool inspire = !wing && s.inspireEnabled >= 0.5;
    if (wing) updateWing(dt, st, s);
    else if (inspire) updateInspire(dt, st, s);
    else updateAcro(dt, st, s);
    if (!wing) wingMode_ = false;

    collideWithFloor(s);

    // Camera
    Vec3 camForward, camUp;
    if (inspire) {
        // Like the real Inspire 3 FPV camera: only the tilt is stabilized. The camera keeps the
        // Camera tilt against the horizon, but rolls and turns with the aircraft.
        // The tilt motor catches up with a short delay (two cascaded stages, like the AC mod): on
        // accelerations and braking the image dips briefly, then the horizon settles.
        const Euler body = basisToEuler(rot_.forward, rot_.up);
        const double a = smoothAlpha(dt, 0.05);
        horizonLag1_ += (body.pitch - horizonLag1_) * a;
        horizonLag2_ += (horizonLag1_ - horizonLag2_) * a;
        const Basis cam = eulerToBasis(s.inspireCamTilt + (body.pitch - horizonLag2_), body.bank, body.heading);
        camForward = cam.forward;
        camUp = cam.up;
        pose.fovDeg = s.inspireCamFov;
    } else {
        // Camera tilted up by the camera angle, like a real FPV cam. Fixed wing: level camera (0 deg).
        const Vec3 right = rot_.right();
        const double tilt = wing ? 0.0 : degToRad(s.cameraAngle);
        camForward = rotateAround(rot_.forward, right, tilt);
        camUp = rotateAround(rot_.up, right, tilt);
        pose.fovDeg = s.cameraFov;
        horizonLag1_ = horizonLag2_ = basisToEuler(rot_.forward, rot_.up).pitch;
    }
    camera_.forward = camForward;
    camera_.up = camUp;

    pose.valid = true;
    pose.origin = origin_;
    pose.local = position_ + rot_.up * 0.1;
    pose.euler = basisToEuler(camForward, camUp);
}

void Drone::updateAcro(double dt, const Sticks& st, const Settings& s) {
    // Forces
    const Vec3 air = velocity_ - wind_; // velocity through the air (= velocity_ in calm air)
    const double inflowCoefficient = 1.0 - std::sin(vectorAngle(rot_.up, air));
    const double inflowVelocity = air.length() * inflowCoefficient;
    const double throttle = s.mode3d >= 0.5 ? st.throttle : (st.throttle + 1.0) / 2.0;

    Vec3 force = rot_.up * thrust(s.linearAcceleration >= 0.5, s.airDensity, s.propDiameter, s.propPitch, s.motorKv,
                                  s.batteryCells, throttle, inflowVelocity);
    force += airDragForce(1.2 * s.airDensity, air, s.airDrag, s.droneSurfaceArea, s.minimalSurfaceAreaCoefficient,
                          inflowCoefficient);

    const double mass = std::max(0.01, s.droneMass);
    velocity_ += (force / mass + Vec3{0, 0, -9.8 * s.gravity}) * dt;
    position_ += velocity_ * dt;

    // Rotation. Conventions: +pitch = stick forward = nose down, +roll = roll right, +yaw = yaw right.
    const int type = static_cast<int>(s.ratesType + 0.5);
    const double rollAngle = degToRad(rate(type, st.roll, s.rollRate, s.rollSuper, s.rollExpo, s.rollCenter, s.rollMax, s.rollCurve)) * dt;
    const double pitchAngle = degToRad(rate(type, st.pitch, s.pitchRate, s.pitchSuper, s.pitchExpo, s.pitchCenter, s.pitchMax, s.pitchCurve)) * dt;
    const double yawAngle = degToRad(rate(type, st.yaw, s.yawRate, s.yawSuper, s.yawExpo, s.yawCenter, s.yawMax, s.yawCurve)) * dt;

    const Vec3 right = rot_.right();
    rot_.forward = rotateAround(rot_.forward, right, -pitchAngle);
    rot_.up = rotateAround(rot_.up, right, -pitchAngle);
    rot_.up = rotateAround(rot_.up, rot_.forward.normalized(), rollAngle);
    rot_.forward = rotateAround(rot_.forward, rot_.up.normalized(), -yawAngle);
    rot_.orthonormalize();
}

// DJI Inspire 3 style flight (port of the UEVR plugin's cinema mode). Pitch stick = forward speed,
// roll = sideways speed, throttle (centered = hold altitude) = climb / descent, yaw = turn rate.
// Centered sticks brake to a standstill and hold the position.
void Drone::updateInspire(double dt, const Sticks& st, const Settings& s) {
    const InspireParams c = inspireParams(s);
    const double g = 9.81 * std::max(s.gravity, 0.1);
    const auto expo = [&](double x) {
        x = std::clamp(x, -1.0, 1.0);
        return x * (1.0 - c.expo) + x * x * x * c.expo;
    };

    // Heading
    const double yawTarget = expo(st.yaw) * c.yawRate;
    inspireYawRate_ += (yawTarget - inspireYawRate_) * smoothAlpha(dt, std::max(c.response * 0.6, 0.01));
    const double heading = basisToEuler(rot_.forward, rot_.up).heading + inspireYawRate_ * dt;
    const double h = degToRad(heading);
    const Vec3 fwd{std::sin(h), std::cos(h), 0.0};
    const Vec3 right{std::cos(h), -std::sin(h), 0.0};

    // Desired velocity
    double climb = std::clamp(st.throttle, -1.0, 1.0);
    climb = std::abs(climb) < 0.06 ? 0.0 : (climb - std::copysign(0.06, climb)) / 0.94;
    const double vz = climb * c.climb;
    Vec3 vTarget = fwd * (expo(st.pitch) * c.maxSpeed) + right * (expo(st.roll) * c.maxSpeed) + Vec3{0, 0, vz};
    // GPS hold: the target is a ground speed. The airframe cannot fly faster than its own top speed
    // through the air, so into a headwind it is slower, and a wind stronger than that pushes it away.
    {
        constexpr double kMaxAirspeed = 26.0; // Inspire 3, m/s
        const Vec3 wh{wind_.x, wind_.y, 0.0};
        Vec3 airH = Vec3{vTarget.x, vTarget.y, 0.0} - wh;
        if (airH.length() > kMaxAirspeed) airH = airH * (kMaxAirspeed / airH.length());
        vTarget = {wh.x + airH.x, wh.y + airH.y, vTarget.z};
    }

    // First-order velocity response with an acceleration limit (smooth starts and stops).
    Vec3 accel = (vTarget - velocity_) / std::max(c.response, 0.05);
    Vec3 ah{accel.x, accel.y, 0.0};
    const double ahLen = ah.length();
    if (ahLen > c.maxAccel) ah = ah * (c.maxAccel / ahLen);
    accel = {ah.x, ah.y, std::clamp(accel.z, -4.0, 4.0)};
    velocity_ += accel * dt;
    position_ += velocity_ * dt;
    inspireAccel_ += (accel - inspireAccel_) * smoothAlpha(dt, 0.25);

    // Body attitude (visual): the thrust vector leans into the acceleration and against drag.
    // Leans against the drag of the air it moves through: into the wind when holding position.
    const Vec3 vh{velocity_.x - wind_.x, velocity_.y - wind_.y, 0.0};
    Vec3 lean = Vec3{inspireAccel_.x, inspireAccel_.y, 0.0} + vh * 0.25;
    const double maxLean = std::tan(degToRad(c.maxTilt)) * g;
    if (lean.length() > maxLean) lean = lean * (maxLean / std::max(lean.length(), 1e-9));
    const Vec3 up = (lean + Vec3{0, 0, g}).normalized();
    // Forward keeps the heading exactly (same horizontal direction), tilted into the disc plane.
    const Vec3 f = (fwd - Vec3{0, 0, 1} * (fwd.dot(up) / std::max(up.z, 0.2))).normalized();
    rot_.forward = f;
    rot_.up = up;
    rot_.orthonormalize();
}

// ---------- Fixed wing (port of the UEVR plugin's flying wing, TBS Mojito class) ----------
// ~1.1 m span, ~1.1 kg, pusher prop. The sticks work like acro on a flight controller in rate mode,
// but the elevons lose authority at low airspeed and the airframe weathervanes into the airflow, so
// with centered sticks it glides along its flight path. No rudder: the yaw stick does nothing.

namespace {
constexpr double kWingMass = 1.1, kWingArea = 0.21, kPropSpeed = 34.0;
constexpr double kClAlpha = 3.8, kCl0 = 0.05, kStallDeg = 13.0, kCd0 = 0.025, kInducedK = 0.07, kSideForce = 0.8;
constexpr double kGyroHold = 0.6, kPitchStability = 3.0, kYawStability = 4.0, kControlSpeed = 12.0;
constexpr double kLaunchSpeed = 12.0;
} // namespace

void Drone::updateWing(double dt, const Sticks& st, const Settings& s) {
    const double throttle = std::clamp((st.throttle + 1.0) / 2.0, 0.0, 1.0); // no reverse on a wing
    if (!wingMode_) {
        // Switched to the wing: already flying fast enough, or held in the hand for a launch.
        wingMode_ = true;
        wingLaunched_ = (velocity_ - wind_).length() > 8.0;
        wingRates_ = {};
        wingThrottle_ = throttle;
    }

    if (!wingLaunched_) {
        // Held in the hand until the throttle comes up, then thrown forward, slightly nose up.
        const double heading = basisToEuler(rot_.forward, rot_.up).heading;
        velocity_ = {};
        wingRates_ = {};
        aoaDeg_ = 0;
        if (throttle < 0.25) {
            rot_ = eulerToBasis(0, 0, heading);
            return;
        }
        wingLaunched_ = true;
        rot_ = eulerToBasis(8, 0, heading);
        velocity_ = wind_ + rot_.forward * kLaunchSpeed + Vec3{0, 0, 1.5}; // thrown into the air mass
        wingThrottle_ = throttle;
    }

    const int steps = std::clamp(static_cast<int>(std::ceil(dt / (1.0 / 500.0))), 1, 100);
    for (int i = 0; i < steps; ++i) wingSubstep(dt / steps, st, s);

    // Landed and stopped with the throttle down: back in the hand for a new launch.
    if (s.collision >= 0.5 && position_.z <= floorZ_ + 0.15 && velocity_.length() < 1.0 && throttle < 0.1)
        wingLaunched_ = false;
}

void Drone::wingSubstep(double dt, const Sticks& st, const Settings& s) {
    const double g = 9.8 * s.gravity;
    const double rho = 1.225 * std::max(s.airDensity, 0.0);
    const double throttle = std::clamp((st.throttle + 1.0) / 2.0, 0.0, 1.0);
    wingThrottle_ += (throttle - wingThrottle_) * smoothAlpha(dt, 0.08);

    // Airflow in the body frame (x forward, y right, z up)
    const Vec3 right = rot_.right();
    const Vec3 air = velocity_ - wind_; // airflow: everything aerodynamic uses the airspeed
    const double speed = air.length();
    const double vx = air.dot(rot_.forward), vy = air.dot(right), vz = air.dot(rot_.up);
    const double alpha = speed > 0.5 ? std::atan2(-vz, vx) : 0.0; // + = air from below
    const double beta = speed > 0.5 ? std::atan2(vy, std::max(std::abs(vx), 0.1)) : 0.0;
    aoaDeg_ = radToDeg(alpha);
    const double q = 0.5 * rho * speed * speed;

    // Rotation: flight controller in rate mode + aerodynamic weathervane
    const double authority = std::clamp(speed * speed / (kControlSpeed * kControlSpeed), 0.03, 1.0);
    const int type = static_cast<int>(s.ratesType + 0.5);
    Vec3 target;
    target.x = rate(type, st.roll, s.rollRate, s.rollSuper, s.rollExpo, s.rollCenter, s.rollMax, s.rollCurve) *
               s.wingRollScale * authority;
    target.y = -rate(type, st.pitch, s.pitchRate, s.pitchSuper, s.pitchExpo, s.pitchCenter, s.pitchMax, s.pitchCurve) *
               s.wingPitchScale * authority;
    target.z = 0; // no rudder, no yaw stick
    // Weathervane (deg/s): the nose follows the airflow, toward the trim angle of attack.
    const double v10 = std::min(speed, 40.0) / 10.0 * (1.0 - kGyroHold);
    const double alphaErr = std::clamp(radToDeg(alpha) - s.wingGlide, -60.0, 60.0);
    target.y -= alphaErr * kPitchStability * v10;
    target.z += std::clamp(radToDeg(beta), -60.0, 60.0) * kYawStability * v10;
    wingRates_ += (target - wingRates_) * smoothAlpha(dt, 0.04);

    const double rollA = degToRad(wingRates_.x) * dt, pitchA = degToRad(wingRates_.y) * dt, yawA = degToRad(wingRates_.z) * dt;
    rot_.forward = rotateAround(rot_.forward, right, pitchA);
    rot_.up = rotateAround(rot_.up, right, pitchA);
    rot_.up = rotateAround(rot_.up, rot_.forward.normalized(), rollA);
    rot_.forward = rotateAround(rot_.forward, rot_.up.normalized(), -yawA);
    rot_.orthonormalize();

    // Forces (N)
    const double weight = kWingMass * g;
    Vec3 force{0, 0, -weight};
    // Pusher prop: thrust falls off as the airspeed approaches the prop's pitch speed.
    const double prop = std::max(0.0, 1.0 - std::max(vx, 0.0) / kPropSpeed);
    force += rot_.forward * (wingThrottle_ * s.wingThrust * kWingMass * 9.8 * prop);

    if (speed > 0.5) {
        const Vec3 vdir = air / speed;
        // Lift coefficient: linear up to the stall, then a flat plate.
        const double stall = degToRad(kStallDeg);
        double cl;
        if (std::abs(alpha) <= stall) {
            cl = kCl0 + kClAlpha * alpha;
        } else {
            const double clmax = kCl0 + kClAlpha * std::copysign(stall, alpha);
            const double t = std::clamp((std::abs(alpha) - stall) / degToRad(10.0), 0.0, 1.0);
            cl = clmax * (1.0 - t) + 0.9 * std::sin(2.0 * alpha) * 0.7 * t; // lift breaks down past the stall
        }
        const double sa = std::sin(alpha);
        const double cd = kCd0 + kInducedK * cl * cl + 1.1 * sa * sa;

        Vec3 liftDir = rot_.up - vdir * rot_.up.dot(vdir);
        if (liftDir.length() > 1e-6) force += liftDir.normalized() * (q * kWingArea * cl);
        force -= vdir * (q * kWingArea * cd);
        force -= rot_.right() * (q * kWingArea * kSideForce * beta);
    }

    velocity_ += force * (dt / kWingMass);
    position_ += velocity_ * dt;
}

void Drone::collideWithFloor(const Settings& s) {
    if (s.collision < 0.5) return; // no floor at all: fly through terrain and buildings
    const double minZ = floorZ_ + 0.1;
    if (position_.z >= minZ) return;
    position_.z = minZ;

    {
        if (velocity_.z < 0) velocity_.z = -velocity_.z * s.bounciness;
        Vec3 horizontal{velocity_.x, velocity_.y, 0};
        const double loss = std::min(horizontal.length(), std::min(1.0, velocity_.length() * s.groundFriction * 0.5));
        horizontal -= horizontal.normalized() * loss;
        velocity_.x = horizontal.x;
        velocity_.y = horizontal.y;
    }
}
