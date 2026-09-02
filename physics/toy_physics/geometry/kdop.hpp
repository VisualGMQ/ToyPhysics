#pragma once

#include "toy_physics/common/math.hpp"

namespace toy_physics {

namespace internal {

template <size_t N>
struct KDOPData {
    Vector<real, N> m_mins = Vector<real, N>::Zero();
    Vector<real, N> m_maxs = Vector<real, N>::Zero();
};

}  // namespace internal

template <size_t N>
struct KDOP : public internal::KDOPData<N> {};

template <>
struct KDOP<8> : public internal::KDOPData<8> {};

template <>
struct KDOP<16> : public internal::KDOPData<8> {};

}  // namespace toy_physics
