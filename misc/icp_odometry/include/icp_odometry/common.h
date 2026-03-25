#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <geometry_msgs/msg/quaternion.hpp>
#include <pointmatcher/PointMatcher.h>

namespace truck {

template <typename T>
inline double squared(const T& x) {
    return x * x;
}

template <typename T>
struct Limits {
    Limits() = default;

    Limits(const T& min, const T& max) : min(min), max(max) {
        assert(min <= max);
    }

    bool isMet(const T& x) const {
        return min <= x && x <= max;
    }

    bool isStrictlyMet(const T& x) const {
        return min < x && x < max;
    }

    T clamp(const T& t) const {
        return std::clamp(t, min, max);
    }

    double ratio(const T& t) const {
        return (clamp(t) - min) / (max - min);
    }

    T min, max;
};

namespace geom {

class Angle {
public:
    constexpr Angle() = default;
    constexpr explicit Angle(double rad) : value_(rad) {}

    static constexpr Angle fromRadians(double rad) noexcept {
        return Angle{rad};
    }

    constexpr double radians() const noexcept {
        return value_;
    }

    friend double sin(Angle angle) noexcept {
        return std::sin(angle.value_);
    }

    friend double cos(Angle angle) noexcept {
        return std::cos(angle.value_);
    }

private:
    double value_{0};
};

constexpr Angle operator-(Angle a, Angle b) noexcept {
    return Angle{a.radians() - b.radians()};
}

inline Angle toAngle(const geometry_msgs::msg::Quaternion& q) {
    return Angle::fromRadians(std::copysign(2.0 * std::acos(q.w), q.z));
}

inline Angle toYawAngle(const geometry_msgs::msg::Quaternion& q) {
    return toAngle(q);
}

}  // namespace geom

namespace icp_odometry {

using Matcher = PointMatcher<float>;
using DataPoints = Matcher::DataPoints;
using Matrix = Matcher::Matrix;
using TransformationParameters = Matcher::TransformationParameters;

}  // namespace icp_odometry
}  // namespace truck

// #pragma once

// #include <pointmatcher/PointMatcher.h>

// namespace truck::icp_odometry {

// using Matcher = PointMatcher<float>;

// using DataPoints = Matcher::DataPoints;
// using Matrix = Matcher::Matrix;
// using TransformationParameters = Matcher::TransformationParameters;

// }  // namespace truck::icp_odometry