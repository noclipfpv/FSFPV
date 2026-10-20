#pragma once
// Minimal double-precision vector math.
// World frame used by the physics: right-handed ENU
//   x = East, y = North, z = Up (meters, relative to the launch point).

#include <algorithm>
#include <cmath>

constexpr double kPi = 3.14159265358979323846;
inline double degToRad(double d) { return d * kPi / 180.0; }
inline double radToDeg(double r) { return r * 180.0 / kPi; }

struct Vec3 {
    double x = 0, y = 0, z = 0;

    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }

    double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
    double length() const { return std::sqrt(dot(*this)); }
    Vec3 normalized() const {
        const double l = length();
        return l > 1e-12 ? *this / l : Vec3{};
    }
};

// Rodrigues rotation of v around unit axis k by angle (radians, right-hand rule).
inline Vec3 rotateAround(const Vec3& v, const Vec3& k, double angle) {
    const double c = std::cos(angle), s = std::sin(angle);
    return v * c + k.cross(v) * s + k * (k.dot(v) * (1.0 - c));
}

inline double vectorAngle(const Vec3& a, const Vec3& b) {
    const double l = a.length() * b.length();
    if (l < 1e-12) return 0.0;
    return std::acos(std::clamp(a.dot(b) / l, -1.0, 1.0));
}

// Orientation stored as forward + up unit vectors. right = forward x up.
struct Basis {
    Vec3 forward{0, 1, 0};
    Vec3 up{0, 0, 1};

    Vec3 right() const { return forward.cross(up).normalized(); }
    void orthonormalize() {
        forward = forward.normalized();
        const Vec3 r = forward.cross(up).normalized();
        up = r.cross(forward).normalized();
    }
};

// Aviation-style angles, degrees.
//   pitch   : + nose up
//   bank    : + right wing down
//   heading : + clockwise from true north, 0..360
struct Euler {
    double pitch = 0, bank = 0, heading = 0;
};

inline double wrap360(double deg) {
    deg = std::fmod(deg, 360.0);
    return deg < 0 ? deg + 360.0 : deg;
}

inline Euler basisToEuler(const Vec3& f, const Vec3& u) {
    const Vec3 worldUp{0, 0, 1};
    Euler e;
    const double fz = std::clamp(f.z, -1.0, 1.0);
    e.pitch = radToDeg(std::asin(fz));
    const Vec3 horizontal{f.x, f.y, 0};
    if (horizontal.length() > 1e-4) {
        e.heading = radToDeg(std::atan2(f.x, f.y));
        const Vec3 levelRight = f.cross(worldUp).normalized();
        const Vec3 levelUp = levelRight.cross(f).normalized();
        const Vec3 r = f.cross(u).normalized();
        e.bank = radToDeg(std::atan2(-r.dot(levelUp), r.dot(levelRight)));
    } else {
        // Looking straight up or down: heading and bank are the same rotation, fold bank into heading.
        e.heading = fz > 0 ? radToDeg(std::atan2(-u.x, -u.y)) : radToDeg(std::atan2(u.x, u.y));
        e.bank = 0;
    }
    e.heading = wrap360(e.heading);
    return e;
}

// Local ENU meters <-> geodetic (flat earth around the origin, fine for a few km).
constexpr double kEarthRadius = 6378137.0;

struct GeoPoint {
    double lat = 0, lon = 0, alt = 0; // degrees, degrees, meters MSL
};

inline GeoPoint localToGeo(const GeoPoint& origin, const Vec3& p) {
    GeoPoint g;
    g.lat = origin.lat + radToDeg(p.y / kEarthRadius);
    g.lon = origin.lon + radToDeg(p.x / (kEarthRadius * std::cos(degToRad(origin.lat))));
    g.alt = origin.alt + p.z;
    return g;
}

inline Vec3 geoToLocal(const GeoPoint& origin, const GeoPoint& g) {
    return {
        degToRad(g.lon - origin.lon) * kEarthRadius * std::cos(degToRad(origin.lat)),
        degToRad(g.lat - origin.lat) * kEarthRadius,
        g.alt - origin.alt,
    };
}

// Inverse of basisToEuler. Pitch beyond +-90 is allowed (inverted gimbal looking backward).
inline Basis eulerToBasis(double pitchDeg, double bankDeg, double headingDeg) {
    const double p = degToRad(pitchDeg), h = degToRad(headingDeg);
    Basis b;
    b.forward = {std::cos(p) * std::sin(h), std::cos(p) * std::cos(h), std::sin(p)};
    const Vec3 levelRight{std::cos(h), -std::sin(h), 0.0};
    b.up = levelRight.cross(b.forward).normalized();
    b.up = rotateAround(b.up, b.forward, degToRad(bankDeg));
    b.orthonormalize();
    return b;
}

inline double wrap180(double a) {
    a = std::fmod(a + 180.0, 360.0);
    if (a < 0.0) a += 360.0;
    return a - 180.0;
}

// Frame-rate independent smoothing factor for a time constant tau (seconds).
inline double smoothAlpha(double dt, double tau) {
    return tau <= 1e-4 ? 1.0 : 1.0 - std::exp(-dt / tau);
}
