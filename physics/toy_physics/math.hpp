#pragma once
#include "Eigen/Dense"
#include "toy_physics/config.hpp"
#include <array>
#include <span>
#include <type_traits>

namespace toy_physics {
using Vector3 = Eigen::Vector3<real>;
using Vector4 = Eigen::Vector4<real>;
using Matrix33 = Eigen::Matrix<real, 3, 3>;
using Matrix44 = Eigen::Matrix<real, 4, 4>;

template <typename T, size_t N>
using Vector = Eigen::Vector<T, N>;

template <size_t Row, size_t Col>
using MatrixR = Eigen::Matrix<real, Row, Col>;

template <typename T, size_t Row, size_t Col>
using Matrix = Eigen::Matrix<T, Row, Col>;

using Quaternion = Eigen::Quaternion<real>;

static constexpr auto MATH_PI = EIGEN_PI;

template <typename T>
requires(std::is_floating_point_v<T>)
class TDegrees;

template <typename T>
requires(std::is_floating_point_v<T>)
class TRadians {
public:
    TRadians() : m_value{} {}

    template <typename U>
    requires(std::convertible_to<T, U>)
    constexpr TRadians(U value) : m_value{static_cast<T>(value)} {}

    template <typename U>
    requires(std::convertible_to<U, T>)
    constexpr TRadians(TDegrees<U> o)
        : m_value{static_cast<T>(static_cast<T>(o) * EIGEN_PI /
                                 static_cast<T>(180.0))} {}

    constexpr TRadians& operator+=(const TRadians& o) {
        m_value += o.m_value;
        return *this;
    }

    constexpr TRadians& operator-=(const TRadians& o) {
        m_value -= o.m_value;
        return *this;
    }

    template <typename U>
    constexpr TRadians& operator*=(U value) {
        m_value *= value;
        return *this;
    }

    template <typename U>
    constexpr TRadians& operator/=(U value) {
        m_value /= value;
        return *this;
    }

    template <typename U>
    constexpr TRadians& operator*=(TRadians<U> value) {
        m_value *= value.value_;
        return *this;
    }

    template <typename U>
    constexpr TRadians& operator/=(TRadians<U> value) {
        m_value /= value.value_;
        return *this;
    }

    constexpr TRadians(const TRadians&) = default;

    constexpr explicit operator T() const noexcept { return m_value; }

    constexpr T Value() const noexcept { return m_value; }

private:
    T m_value;
};

template <typename T>
requires(std::is_floating_point_v<T>)
class TDegrees {
public:
    constexpr TDegrees() : m_value{} {}

    template <typename U>
    requires(std::convertible_to<T, U>)
    constexpr TDegrees(U value) : m_value{static_cast<T>(value)} {}

    template <typename U>
    requires(std::convertible_to<U, T>)
    constexpr TDegrees(TRadians<U> o)
        : m_value{static_cast<T>(static_cast<T>(o) * static_cast<T>(180.0) /
                                 EIGEN_PI)} {}

    constexpr TDegrees& operator+=(const TDegrees& o) {
        m_value += o.m_value;
        return *this;
    }

    constexpr TDegrees& operator-=(const TDegrees& o) {
        m_value -= o.m_value;
        return *this;
    }

    template <typename U>
    constexpr TDegrees& operator*=(U value) {
        m_value *= value;
        return *this;
    }

    template <typename U>
    constexpr TDegrees& operator/=(U value) {
        m_value /= value;
        return *this;
    }

    template <typename U>
    constexpr TDegrees& operator*=(TDegrees<U> value) {
        m_value *= value.value_;
        return *this;
    }

    template <typename U>
    constexpr TDegrees& operator/=(TDegrees<U> value) {
        m_value /= value.value_;
        return *this;
    }

    constexpr TDegrees(const TDegrees&) = default;

    constexpr explicit operator T() const noexcept { return m_value; }

    constexpr T Value() const noexcept { return m_value; }

private:
    T m_value;
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

using Radians = TRadians<real>;
using Degrees = TDegrees<real>;

enum class BarycentricPolicy {
    None,
    StopWhenNegative,  // stop when any coordinate < 0
};

template <typename T>
struct TBarycentricCoord {
    T m_alpha{};
    T m_beta{};
    T m_gamma{};

    [[nodiscard]] bool IsValid() const {
        return std::abs((m_alpha + m_beta + m_gamma) - T(1)) <= 1e-6;
    }

    [[nodiscard]] bool IsInnerPoint() const {
        return IsValid() && m_alpha >= 0 && m_alpha <= 1 && m_beta >= 0 &&
               m_beta <= 1 && m_gamma >= 0 && m_gamma <= 1;
    }

    TBarycentricCoord() = default;

    TBarycentricCoord(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 q,
                      BarycentricPolicy policy = BarycentricPolicy::None,
                      real tolerance = kDefaultRealTolerance) {
        Vector3 d12 = p2 - p1;
        Vector3 d23 = p3 - p2;
        Vector3 norm = d12.cross(d23);
        real s = norm.dot(norm);

        // fallback to line/point
        if (std::abs(s) <= tolerance) {
            return;
        }

        Vector3 d2q = q - p2;
        Vector3 d3q = q - p3;

        m_alpha = (d2q.cross(d3q)).dot(norm) / s;
        if (m_alpha < 0 && policy == BarycentricPolicy::StopWhenNegative) {
            return;
        }

        Vector3 d1q = q - p1;
        m_beta = (d3q.cross(d1q)).dot(norm) / s;
        if (m_beta < 0 && policy == BarycentricPolicy::StopWhenNegative) {
            return;
        }

        m_gamma = T(1) - m_alpha - m_beta;
    }
};

using BarycentricCoord = TBarycentricCoord<real>;

template <typename T>
struct TTetrahedronBarycentric {
    Vector4 m_coord{Vector4::Zero()};

    [[nodiscard]] bool IsValid() const {
        return std::abs(m_coord.sum() - 1) <= 1e-6;
    }

    [[nodiscard]] bool IsInnerPoint() const {
        return IsValid() && (m_coord.array() >= 0).all() &&
               (m_coord.array() <= 1).all();
    }

    TTetrahedronBarycentric() : m_coord(Vector4::Zero()), m_tolerance{1e-6} {}

    TTetrahedronBarycentric(std::span<const Vector3, 4> pts, Vector3 p,
                            BarycentricPolicy policy = BarycentricPolicy::None,
                            real tolerance = kDefaultRealTolerance)
        : m_tolerance{tolerance} {
        Vector3 d01 = pts[1] - pts[0];
        Vector3 d02 = pts[2] - pts[0];
        Vector3 d03 = pts[3] - pts[0];

        Vector3 cross02_03 = d02.cross(d03);
        real denom = d01.dot(cross02_03);

        if (std::abs(denom) <= tolerance) {
            return;
        }

        Vector3 dp0 = pts[0] - p;
        Vector3 dp1 = pts[1] - p;
        Vector3 dp2 = pts[2] - p;
        Vector3 dp3 = pts[3] - p;

        real inv_denom = 1 / denom;
        Vector3 v = -dp0;

        m_coord[0] = (dp1).dot((dp2).cross(dp3)) * inv_denom;
        if (m_coord[0] < 0 && policy == BarycentricPolicy::StopWhenNegative)
            return;

        m_coord[1] = v.dot(d01.cross(d03)) * inv_denom;
        if (m_coord[1] < 0 && policy == BarycentricPolicy::StopWhenNegative)
            return;

        Vector3 d13 = pts[3] - pts[1];
        m_coord[2] = d01.dot(v.cross(d13)) * inv_denom;
        if (m_coord[2] < 0 && policy == BarycentricPolicy::StopWhenNegative)
            return;

        m_coord[3] = 1 - m_coord[0] - m_coord[1] - m_coord[2];
    }

private:
    real m_tolerance{kDefaultRealTolerance};
};

using TetrahedronBarycentric = TTetrahedronBarycentric<real>;

inline int GetFurthestAxis(Vector3 v) {
    Eigen::Index idx;
    v.cwiseAbs().minCoeff(&idx);
    return static_cast<int>(idx);
}  // namespace toy_physics

inline int GetNearestAxis(Vector3 v) {
    Eigen::Index idx;
    v.cwiseAbs().maxCoeff(&idx);
    return static_cast<int>(idx);
}  // namespace toy_physics

}  // namespace toy_physics
