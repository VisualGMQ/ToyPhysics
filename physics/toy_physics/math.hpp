#pragma once
#include "Eigen/Dense"
#include "toy_physics/config.hpp"
#include <array>
#include <type_traits>

namespace toy_physics {

using Vector3 = Eigen::Vector3<real>;

template <size_t N>
using Vector = Eigen::Vector<real, N>;

using Quaternion = Eigen::Quaternion<real>;

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

    bool IsValid() const {
        return m_alpha >= 0 && m_beta >= 0 && m_gamma >= 0 &&
               std::abs((m_alpha + m_beta + m_gamma) - 1) <= 1e-6;
    }

    TBarycentricCoord() = default;

    TBarycentricCoord(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 q,
                      BarycentricPolicy policy = BarycentricPolicy::None) {
        Vector3 d12 = p2 - p1;
        Vector3 d23 = p3 - p2;
        Vector3 norm = d12.cross(d23);
        real s = norm.dot(norm);

        if (std::abs(s) <= std::numeric_limits<real>::epsilon()) {
            real d12_len = d12.norm();
            if (std::abs(d12_len) <= std::numeric_limits<real>::epsilon()) {
                return;
            }
            m_alpha = (q - p1).norm() / d12.norm();
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

        m_gamma = 1.0 - m_alpha - m_beta;
    }
};

using BarycentricCoord = TBarycentricCoord<real>;

template <typename T>
struct TTetrahedronBarycentric {
    T m_alpha{};
    T m_beta{};
    T m_gamma{};
    T m_delta{};

    bool IsValid() const {
        return m_alpha >= 0 && m_beta >= 0 && m_gamma >= 0 &&
               m_delta >= 0 &&
               std::abs((m_alpha + m_beta + m_gamma + m_delta) - 1) <= 1e-6;
    }

    TTetrahedronBarycentric() = default;

    TTetrahedronBarycentric(Vector3 a, Vector3 b, Vector3 c, Vector3 d,
                            Vector3 p,
                            BarycentricPolicy policy = BarycentricPolicy::None) {
        Vector3 ab = b - a;
        Vector3 ac = c - a;
        Vector3 ad = d - a;

        Vector3 cad = ac.cross(ad);
        real denom = ab.dot(cad);
        if (std::abs(denom) <= std::numeric_limits<real>::epsilon()) {
            return;
        }

        real inv_denom = real{1} / denom;
        Vector3 v = p - a;

        m_alpha = (b - p).dot((c - p).cross(d - p)) * inv_denom;
        if (m_alpha < 0 && policy == BarycentricPolicy::StopWhenNegative) return;

        m_beta = v.dot(cad) * inv_denom;
        if (m_beta < 0 && policy == BarycentricPolicy::StopWhenNegative) return;

        m_gamma = ab.dot(v.cross(ad)) * inv_denom;
        if (m_gamma < 0 && policy == BarycentricPolicy::StopWhenNegative) return;

        m_delta = T{1} - m_alpha - m_beta - m_gamma;
    }
};

using TetrahedronBarycentric = TTetrahedronBarycentric<real>;

}
