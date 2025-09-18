#pragma once
#include "Eigen/Dense"
#include <type_traits>

template <typename T>
requires(std::is_floating_point_v<T>)
class TDegrees;

template <typename T>
requires(std::is_floating_point_v<T>)
class TRadians {
public:
    TRadians() : value_{} {}

    template <typename U>
    requires(std::convertible_to<T, U>)
    constexpr TRadians(U value) : value_{static_cast<T>(value)} {}

    template <typename U>
    requires(std::convertible_to<U, T>)
    constexpr TRadians(TDegrees<U> o)
        : value_{static_cast<T>(static_cast<T>(o) * EIGEN_PI /
                                static_cast<T>(180.0))} {}

    constexpr TRadians& operator+=(const TRadians& o) {
        value_ += o.value_;
        return *this;
    }

    constexpr TRadians& operator-=(const TRadians& o) {
        value_ -= o.value_;
        return *this;
    }

    template <typename U>
    constexpr TRadians& operator*=(U value) {
        value_ *= value;
        return *this;
    }

    template <typename U>
    constexpr TRadians& operator/=(U value) {
        value_ /= value;
        return *this;
    }

    template <typename U>
    constexpr TRadians& operator*=(TRadians<U> value) {
        value_ *= value.value_;
        return *this;
    }

    template <typename U>
    constexpr TRadians& operator/=(TRadians<U> value) {
        value_ /= value.value_;
        return *this;
    }

    constexpr TRadians(const TRadians&) = default;

    constexpr explicit operator T() const noexcept { return value_; }

    constexpr T Value() const noexcept { return value_; }

private:
    T value_;
};

template <typename T>
requires(std::is_floating_point_v<T>)
class TDegrees {
public:
    constexpr TDegrees() : value_{} {}

    template <typename U>
    requires(std::convertible_to<T, U>)
    constexpr TDegrees(U value) : value_{static_cast<T>(value)} {}

    template <typename U>
    requires(std::convertible_to<U, T>)
    constexpr TDegrees(TRadians<U> o)
        : value_{static_cast<T>(static_cast<T>(o) * static_cast<T>(180.0) /
                                EIGEN_PI)} {}

    constexpr TDegrees& operator+=(const TDegrees& o) {
        value_ += o.value_;
        return *this;
    }

    constexpr TDegrees& operator-=(const TDegrees& o) {
        value_ -= o.value_;
        return *this;
    }

    template <typename U>
    constexpr TDegrees& operator*=(U value) {
        value_ *= value;
        return *this;
    }

    template <typename U>
    constexpr TDegrees& operator/=(U value) {
        value_ /= value;
        return *this;
    }

    template <typename U>
    constexpr TDegrees& operator*=(TDegrees<U> value) {
        value_ *= value.value_;
        return *this;
    }

    template <typename U>
    constexpr TDegrees& operator/=(TDegrees<U> value) {
        value_ /= value.value_;
        return *this;
    }

    constexpr TDegrees(const TDegrees&) = default;

    constexpr explicit operator T() const noexcept { return value_; }

    constexpr T Value() const noexcept { return value_; }

private:
    T value_;
};

// mathematics

template <typename T>
TRadians<T> operator+(TRadians<T> r1, TRadians<T> r2) {
    return TRadians<T>(static_cast<T>(r1) + static_cast<T>(r2));
}

template <typename T>
TRadians<T> operator-(TRadians<T> r1, TRadians<T> r2) {
    return TRadians<T>(static_cast<T>(r1) - static_cast<T>(r2));
}

template <typename T>
TRadians<T> operator-(TRadians<T> r) {
    return TRadians<T>(-static_cast<T>(r));
}

template <typename T, typename U>
TRadians<T> operator*(TRadians<T> r1, U value) {
    return TRadians<T>(r1.Value() * value);
}

template <typename T, typename U>
TRadians<T> operator*(U value, TRadians<T> r1) {
    return r1 * value;
}

template <typename T, typename U>
TRadians<T> operator/(TRadians<T> r1, U value) {
    return TRadians<T>(r1.Value() / value);
}

template <typename T>
constexpr TDegrees<T> operator+(TDegrees<T> r1, TDegrees<T> r2) {
    return TDegrees<T>(static_cast<T>(r1) + static_cast<T>(r2));
}

template <typename T>
constexpr TDegrees<T> operator-(TDegrees<T> r1, TDegrees<T> r2) {
    return TDegrees<T>(static_cast<T>(r1) - static_cast<T>(r2));
}

template <typename T>
constexpr TDegrees<T> operator-(TDegrees<T> r) {
    return TDegrees<T>(-static_cast<T>(r));
}

template <typename T, typename U>
constexpr TDegrees<T> operator*(TDegrees<T> r1, U value) {
    return TDegrees<T>(r1.Value() * value);
}

template <typename T, typename U>
constexpr TDegrees<T> operator*(U value, TDegrees<T> r1) {
    return r1 * value;
}

template <typename T, typename U>
constexpr TDegrees<T> operator/(TDegrees<T> r1, U value) {
    return TDegrees<T>(r1.Value() / value);
}

// common type comparison

template <typename T, typename U>
bool operator==(TRadians<T> r1, TRadians<U> r2) {
    return static_cast<T>(r1) == static_cast<U>(r2);
}

template <typename T, typename U>
bool operator!=(TRadians<T> r1, TRadians<U> r2) {
    return !(r1 == r2);
}

template <typename T, typename U>
bool operator>(TRadians<T> r1, TRadians<U> r2) {
    return static_cast<T>(r1) > static_cast<U>(r2);
}

template <typename T, typename U>
bool operator<(TRadians<T> r1, TRadians<U> r2) {
    return static_cast<T>(r1) < static_cast<U>(r2);
}

template <typename T, typename U>
bool operator>=(TRadians<T> r1, TRadians<U> r2) {
    return static_cast<T>(r1) >= static_cast<U>(r2);
}

template <typename T, typename U>
bool operator<=(TRadians<T> r1, TRadians<U> r2) {
    return static_cast<T>(r1) <= static_cast<U>(r2);
}

template <typename T, typename U>
constexpr bool operator==(TDegrees<T> r1, TDegrees<U> r2) {
    return static_cast<T>(r1) == static_cast<U>(r2);
}

template <typename T, typename U>
constexpr bool operator!=(TDegrees<T> r1, TDegrees<U> r2) {
    return !(r1 == r2);
}

template <typename T, typename U>
constexpr bool operator>(TDegrees<T> r1, TDegrees<U> r2) {
    return static_cast<T>(r1) > static_cast<U>(r2);
}

template <typename T, typename U>
constexpr bool operator<(TDegrees<T> r1, TDegrees<U> r2) {
    return static_cast<T>(r1) < static_cast<U>(r2);
}

template <typename T, typename U>
constexpr bool operator>=(TDegrees<T> r1, TDegrees<U> r2) {
    return static_cast<T>(r1) >= static_cast<U>(r2);
}

template <typename T, typename U>
constexpr bool operator<=(TDegrees<T> r1, TDegrees<U> r2) {
    return static_cast<T>(r1) <= static_cast<U>(r2);
}

// radians degree comparision

template <typename T, typename U>
bool operator==(TRadians<T> rad, TDegrees<U> deg) {
    return TDegrees<T>(rad) == deg;
}

template <typename T, typename U>
bool operator!=(TRadians<T> rad, TDegrees<U> deg) {
    return !(rad == deg);
}

template <typename T, typename U>
bool operator>(TRadians<T> rad, TDegrees<U> deg) {
    return TDegrees<T>(rad) > deg;
}

template <typename T, typename U>
bool operator<(TRadians<T> rad, TDegrees<U> deg) {
    return TDegrees<T>(rad) < deg;
}

template <typename T, typename U>
bool operator>=(TRadians<T> rad, TDegrees<U> deg) {
    return TDegrees<T>(rad) >= deg;
}

template <typename T, typename U>
bool operator<=(TRadians<T> rad, TDegrees<U> deg) {
    return TDegrees<T>(rad) <= deg;
}

template <typename T, typename U>
bool operator==(TDegrees<U> deg, TRadians<T> rad) {
    return rad == deg;
}

template <typename T, typename U>
bool operator!=(TDegrees<U> deg, TRadians<T> rad) {
    return rad != deg;
}

template <typename T, typename U>
bool operator>(TDegrees<U> deg, TRadians<T> rad) {
    return rad < deg;
}

template <typename T, typename U>
bool operator<(TDegrees<U> deg, TRadians<T> rad) {
    return rad > deg;
}

template <typename T, typename U>
bool operator>=(TDegrees<U> deg, TRadians<T> rad) {
    return rad <= deg;
}

template <typename T, typename U>
bool operator<=(TDegrees<U> deg, TRadians<T> rad) {
    return rad >= deg;
}

using Radians = TRadians<float>;
using Degrees = TDegrees<float>;

template <typename T>
auto CreatePersp(TRadians<T> fov, T aspect, T n, T f) {
    T focal = 1.0 / std::tan(fov.Value() * 0.5);

    // clang-format off
    return Eigen::Matrix4<T>{
        {focal / aspect,       0,                   0,                       0},
        {             0,  -focal,                   0,                       0},
        {             0,       0,   (f + n) / (n - f), (2.f * n * f) / (n - f)},
        {             0,       0,                  -1,                       0}
    };
    // clang-format on
}

template <typename T>
auto CreateOrtho(T left, T right, T top, T bottom, T n, T f) {
    // clang-format off
    return Eigen::Matrix4<T>{
        {2.0f / (right - left),                  0.0f,               0.0f, (left + right) / (left - right)},
        {                 0.0f,-2.0f / (top - bottom),               0.0f, (bottom + top) / (bottom - top)},
        {                 0.0f,                  0.0f,     1.0f / (n - f),                      f/ (f - n)},
        {                 0.0f,                  0.0f,               0.0f,                            1.0f}
    };
    // clang-format on
}

template <typename T>
auto CreateTranslation(const Eigen::Vector3<T>& position) {
    // clang-format off
    return Eigen::Matrix4<T>{
        {1.0f, 0.0f, 0.0f, position.x()},
        {0.0f, 1.0f, 0.0f, position.y()},
        {0.0f, 0.0f, 1.0f, position.z()},
        {0.0f, 0.0f, 0.0f, 1.0f        }
    };
    // clang-format on
}

template <typename T>
auto CreateRotation(const Eigen::Quaternion<T>& rotation) {
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    mat.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    return mat;
}

template <typename T>
auto LookAt(const Eigen::Vector3<T>& target, const Eigen::Vector3<T>& srcPos,
            const Eigen::Vector3<T>& up) {
    auto zAxis = (srcPos - target).normalized();
    auto xAxis = up.cross(zAxis).normalized();
    auto yAxis = zAxis.cross(xAxis);

    // clang-format off
    return Eigen::Matrix4<T>{
        {xAxis.x(), xAxis.y(), xAxis.z(), srcPos.dot(-xAxis)},
        {yAxis.x(), yAxis.y(), yAxis.z(), srcPos.dot(-yAxis)},
        {zAxis.x(), zAxis.y(), zAxis.z(), srcPos.dot(-zAxis)},
        {        0,      0,       0,                       1}
    };
    // clang-format on
}

template <typename T>
auto CreateXRotation(TRadians<T> radians) {
    float cos = std::cos(radians.Value());
    float sin = std::sin(radians.Value());
    // clang-format off
    return Eigen::Matrix4<T>{
        {1.0f, 0.0f, 0.0f, 0.0f},
        {0.0f,  cos, -sin, 0.0f},
        {0.0f,  sin,  cos, 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f}
    };
    // clang-format on
}

template <typename T>
auto CreateYRotation(TRadians<T> radians) {
    float cos = std::cos(radians.Value());
    float sin = std::sin(radians.Value());
    // clang-format off
    return Eigen::Matrix4<T>{
        { cos, 0.0f,  sin, 0.0f},
        {0.0f, 1.0f,  0.0, 0.0f},
        {-sin, 0.0f,  cos, 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f}
    };
    // clang-format on
}

template <typename T>
auto CreateZRotation(TRadians<T> radians) {
    float cos = std::cos(radians.Value());
    float sin = std::sin(radians.Value());
    // clang-format off
    return Eigen::Matrix4<T>{
        { cos, -sin, 0.0f, 0.0f},
        { sin,  cos, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f}
    };
    // clang-format on
}

template <typename T>
auto CreateXYZRotation(const Eigen::Vector3<TRadians<T>>& r) {
    return CreateXRotation(r.x) * CreateYRotation(r.y) * CreateZRotation(r.z);
}

template <typename T>
Eigen::Matrix4f CreateScale(const Eigen::Vector3<T>& scale) {
    // clang-format off
    return Eigen::Matrix4<T>{
        {scale.x(),       0.0,      0.0f, 0.0f},
        {     0.0f, scale.y(),      0.0f, 0.0f},
        {     0.0f,      0.0f, scale.z(), 0.0f},
        {     0.0f,      0.0f,      0.0f, 1.0f}
    };
    // clang-format on
}